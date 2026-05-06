#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import threading
from contextlib import asynccontextmanager

import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.rotation import Rotation

FPS = 30
MAX_CARTESIAN_STEP_M = 0.02
GRIPPER_VELOCITY = 0.0
EE_BOUNDS = {"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]}
DEFAULT_TARGET = {"x": 0.20, "y": 0.00, "z": 0.15}
HOST = "0.0.0.0"
PORT = 8000

HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Arm Move UI</title>
  <style>
    body {{ font-family: sans-serif; max-width: 760px; margin: 40px auto; padding: 0 16px; }}
    .card {{ border: 1px solid #ccc; border-radius: 12px; padding: 20px; margin-bottom: 16px; }}
    .row {{ margin-bottom: 18px; }}
    label {{ display: flex; justify-content: space-between; font-weight: 600; margin-bottom: 6px; }}
    input[type=range] {{ width: 100%; }}
    button {{ padding: 10px 18px; font-size: 16px; cursor: pointer; }}
    code, pre {{ background: #f5f5f5; padding: 2px 6px; border-radius: 6px; }}
    .status {{ white-space: pre-wrap; }}
  </style>
</head>
<body>
  <h1>Arm Move UI</h1>
  <div class="card">
    <p>Move the end effector to a target Cartesian position using browser sliders.</p>
    <p>Bounds: x/y/z in <code>[{xmin}, {xmax}]</code>.</p>
  </div>
  <div class="card">
    <div class="row">
      <label for="x">X <span id="x_value"></span></label>
      <input id="x" type="range" min="{xmin}" max="{xmax}" step="0.005" value="{xdefault}">
    </div>
    <div class="row">
      <label for="y">Y <span id="y_value"></span></label>
      <input id="y" type="range" min="{ymin}" max="{ymax}" step="0.005" value="{ydefault}">
    </div>
    <div class="row">
      <label for="z">Z <span id="z_value"></span></label>
      <input id="z" type="range" min="{zmin}" max="{zmax}" step="0.005" value="{zdefault}">
    </div>
    <button id="move_button">Move Arm</button>
  </div>
  <div class="card">
    <h3>Robot Status</h3>
    <pre id="status" class="status">Loading...</pre>
  </div>
  <script>
    const ids = ["x", "y", "z"];
    function syncLabels() {{
      for (const id of ids) {{
        document.getElementById(id + "_value").textContent = Number(document.getElementById(id).value).toFixed(3);
      }}
    }}

    async function refreshState() {{
      const res = await fetch("/api/state");
      const data = await res.json();
      document.getElementById("status").textContent =
        "Current: " + JSON.stringify(data.current_position) + "\\n" +
        "Last target: " + JSON.stringify(data.last_target) + "\\n" +
        "Busy: " + data.busy;
    }}

    document.getElementById("move_button").addEventListener("click", async () => {{
      const button = document.getElementById("move_button");
      const status = document.getElementById("status");
      const payload = {{
        x: Number(document.getElementById("x").value),
        y: Number(document.getElementById("y").value),
        z: Number(document.getElementById("z").value),
      }};

      button.disabled = true;
      status.textContent = "Moving...";
      try {{
        const res = await fetch("/api/move", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify(payload),
        }});
        const data = await res.json();
        if (!res.ok) {{
          throw new Error(data.detail || "Move failed");
        }}
        status.textContent =
          data.message + "\\n" +
          "Current: " + JSON.stringify(data.current_position) + "\\n" +
          "Target: " + JSON.stringify(data.target_position);
      }} catch (err) {{
        status.textContent = "Error: " + err.message;
      }} finally {{
        button.disabled = false;
      }}
    }});

    syncLabels();
    ids.forEach((id) => document.getElementById(id).addEventListener("input", syncLabels));
    refreshState();
  </script>
</body>
</html>
"""


class MoveRequest(BaseModel):
    x: float
    y: float
    z: float


class ArmMover:
    def __init__(self) -> None:
        self._move_lock = threading.Lock()
        self.robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
        self.robot = SO100Follower(self.robot_config)
        self.robot.connect()
        if not self.robot.is_connected:
            raise ValueError("Robot is not connected!")

        self.motor_names = list(self.robot.bus.motors.keys())
        self.kinematics_solver = RobotKinematics(
            urdf_path="./SO101/so101_new_calib.urdf",
            target_frame_name="gripper_frame_link",
            joint_names=self.motor_names,
        )
        self.ee_to_joint_processor = RobotProcessorPipeline[
            tuple[RobotAction, RobotObservation], RobotAction
        ](
            steps=[
                EEBoundsAndSafety(
                    end_effector_bounds=EE_BOUNDS,
                    max_ee_step_m=MAX_CARTESIAN_STEP_M + 1e-6,
                ),
                GripperVelocityToJoint(speed_factor=20.0),
                InverseKinematicsEEToJoints(
                    kinematics=self.kinematics_solver,
                    motor_names=self.motor_names,
                    initial_guess_current_joints=True,
                ),
            ],
            to_transition=robot_action_observation_to_transition,
            to_output=transition_to_robot_action,
        )
        self.last_target = DEFAULT_TARGET.copy()

    def get_current_position(self) -> list[float]:
        robot_obs = self.robot.get_observation()
        current_joints = np.array(
            [float(robot_obs[f"{name}.pos"]) for name in self.motor_names],
            dtype=float,
        )
        current_pose = self.kinematics_solver.forward_kinematics(current_joints)
        return current_pose[:3, 3].astype(float).tolist()

    def move_to(self, x: float, y: float, z: float) -> dict:
        if not self._move_lock.acquire(blocking=False):
            raise RuntimeError("A move is already in progress.")

        try:
            target_pos = np.array([x, y, z], dtype=float)
            robot_obs = self.robot.get_observation()
            current_joints = np.array(
                [float(robot_obs[f"{name}.pos"]) for name in self.motor_names],
                dtype=float,
            )
            current_pose = self.kinematics_solver.forward_kinematics(current_joints)
            current_pos = current_pose[:3, 3].copy()
            current_rotvec = Rotation.from_matrix(current_pose[:3, :3]).as_rotvec()

            distance = float(np.linalg.norm(target_pos - current_pos))
            num_steps = max(1, math.ceil(distance / MAX_CARTESIAN_STEP_M))

            self.ee_to_joint_processor.reset()
            for step_idx in range(1, num_steps + 1):
                alpha = step_idx / num_steps
                waypoint = current_pos + alpha * (target_pos - current_pos)

                robot_obs = self.robot.get_observation()
                ee_action = {
                    "ee.x": float(waypoint[0]),
                    "ee.y": float(waypoint[1]),
                    "ee.z": float(waypoint[2]),
                    "ee.wx": float(current_rotvec[0]),
                    "ee.wy": float(current_rotvec[1]),
                    "ee.wz": float(current_rotvec[2]),
                    "ee.gripper_vel": GRIPPER_VELOCITY,
                }
                joint_action = self.ee_to_joint_processor((ee_action, robot_obs))
                self.robot.send_action(joint_action)
                precise_sleep(1.0 / FPS)

            self.last_target = {"x": float(x), "y": float(y), "z": float(z)}
            return {
                "message": f"Move complete in {num_steps} step(s).",
                "current_position": self.get_current_position(),
                "target_position": target_pos.astype(float).tolist(),
            }
        finally:
            self._move_lock.release()

    def disconnect(self) -> None:
        if self.robot.is_connected:
            self.robot.disconnect()

    @property
    def busy(self) -> bool:
        return self._move_lock.locked()


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.mover = ArmMover()
    try:
        yield
    finally:
        mover = app.state.mover
        if mover is not None:
            mover.disconnect()


app = FastAPI(title="Arm Move UI", lifespan=lifespan)
app.state.mover = None


@app.get("/", response_class=HTMLResponse)
def index() -> str:
    return HTML.format(
        xmin=EE_BOUNDS["min"][0],
        xmax=EE_BOUNDS["max"][0],
        ymin=EE_BOUNDS["min"][1],
        ymax=EE_BOUNDS["max"][1],
        zmin=EE_BOUNDS["min"][2],
        zmax=EE_BOUNDS["max"][2],
        xdefault=DEFAULT_TARGET["x"],
        ydefault=DEFAULT_TARGET["y"],
        zdefault=DEFAULT_TARGET["z"],
    )


@app.get("/api/state")
def state() -> dict:
    mover = app.state.mover
    return {
        "current_position": mover.get_current_position(),
        "last_target": mover.last_target,
        "busy": mover.busy,
    }


@app.post("/api/move")
def move_arm(request: MoveRequest) -> dict:
    mover = app.state.mover
    try:
        return mover.move_to(request.x, request.y, request.z)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


def main() -> None:
    print(f"Starting arm move UI on http://127.0.0.1:{PORT}")
    uvicorn.run(app, host=HOST, port=PORT)


if __name__ == "__main__":
    main()
