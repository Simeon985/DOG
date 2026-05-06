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
import time

import numpy as np

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
MAX_CARTESIAN_STEP_M = 0.01
STEP_DELAY_S = 0.20
GRIPPER_VELOCITY = 0.0
EE_BOUNDS = {"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]}

# Set the target end-effector position here in meters.
TARGET_X = 0.20
TARGET_Y = 0.00
TARGET_Z = 0.15


def main():
    robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
    robot = SO100Follower(robot_config)
    robot.connect()

    if not robot.is_connected:
        raise ValueError("Robot is not connected!")

    motor_names = list(robot.bus.motors.keys())
    kinematics_solver = RobotKinematics(
        urdf_path="./SO101/so101_new_calib.urdf",
        target_frame_name="gripper_frame_link",
        joint_names=motor_names,
    )

    ee_to_joint_processor = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation], RobotAction
    ](
        steps=[
            EEBoundsAndSafety(
                end_effector_bounds=EE_BOUNDS,
                max_ee_step_m=MAX_CARTESIAN_STEP_M + 1e-6,
            ),
            GripperVelocityToJoint(speed_factor=20.0),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=motor_names,
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    robot_obs = robot.get_observation()
    current_joints = np.array([float(robot_obs[f"{name}.pos"]) for name in motor_names], dtype=float)
    print(f"Current joints: {current_joints}")
    current_pose = kinematics_solver.forward_kinematics(current_joints)
    print(f"Current pose: {current_pose}")
    # current_pos = current_pose[:3, 3].copy()
    # current_rotvec = Rotation.from_matrix(current_pose[:3, :3]).as_rotvec()

    # target_pos = np.array([TARGET_X, TARGET_Y, TARGET_Z], dtype=float)
    # distance = float(np.linalg.norm(target_pos - current_pos))
    # num_steps = max(1, math.ceil(distance / MAX_CARTESIAN_STEP_M))

    # print(f"Current EE position: {current_pos}")
    # print(f"Target EE position:  {target_pos}")
    # print(f"Moving in {num_steps} step(s)...")

    # for step_idx in range(1, num_steps + 1):
    #     alpha = step_idx / num_steps
    #     waypoint = current_pos + alpha * (target_pos - current_pos)

    #     robot_obs = robot.get_observation()
    #     ee_action = {
    #         "ee.x": float(waypoint[0]),
    #         "ee.y": float(waypoint[1]),
    #         "ee.z": float(waypoint[2]),
    #         "ee.wx": float(current_rotvec[0]),
    #         "ee.wy": float(current_rotvec[1]),
    #         "ee.wz": float(current_rotvec[2]),
    #         "ee.gripper_vel": GRIPPER_VELOCITY,
    #     }
    #     joint_action = ee_to_joint_processor((ee_action, robot_obs))
    #     robot.send_action(joint_action)
    #     precise_sleep(1.0 / FPS)

    # print("Move complete.")
    # time.sleep(5)


if __name__ == "__main__":
    main()
