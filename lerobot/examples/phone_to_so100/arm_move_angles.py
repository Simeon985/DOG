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

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

from coordinates_from_picture import get_coordinates_from_picture
from coordinates_from_picture import get_M_and_radius_from_picture
from coordinates_from_picture import get_coordinates_from_picture_2
from coordinates_from_picture import get_coordinates_from_frame

import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

FPS = 30
STEP_DELAY_S = 0.05
MAX_BODY_JOINT_STEP_DEG = 2.0
MAX_GRIPPER_STEP = 4.0
BODY_JOINT_SAFETY_MARGIN_DEG = 5.0
GRIPPER_SAFETY_MARGIN = 2.0
DISTANCE_MOTOR_2_3 = 11.6 # length in cm between centers of motors 2 and 3
ANGLE_OFFSET_2_3 = math.asin(3/DISTANCE_MOTOR_2_3) # in radians
DISTANCE_MOTOR_3_4 = 13.6 # length in cm between motors 3 and 4
ANGLE_CAM_WRT_GRIPPER = 35 # degrees
HOR_DISTANCE_MOTOR_1_2 = 3.5 # in cm, meet nog eens deftig via file
VERT_DISTANCE_MOTOR_1_2 = 8.5 # op gevoel, vanaf onderkant van motor 1 gemeten
HOR_DISTANCE_MOTOR_4_CAM = 9.8 + math.sin(math.radians(ANGLE_CAM_WRT_GRIPPER))*2.3
VER_DISTANCE_MOTOR_4_CAM = 6 + math.cos(math.radians(ANGLE_CAM_WRT_GRIPPER))*2.3
DISTANCE_MOTOR_4_GRIPPER = 17
START_HEIGHT_TO_GRIP = 20
PID_HEIGHT = 25


IK_JOINT_TRIM_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
}

# Set the target motor positions here.
# Arm joints use degrees because `use_degrees=True`.
# Gripper uses its native normalized range [0, 100].
TARGET_ANGLES = {
    "shoulder_pan": 90, # ID 1 #-90..90
    "shoulder_lift": 0, # ID 2
    "elbow_flex": 0, # ID 3
    "wrist_flex": 0, # ID 4
    "wrist_roll": 0, # ID 5
    "gripper": 2, # ID 6
}

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink max-buffers=1 drop=true"  # ← changed from just "appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def coordinates_to_angles(x,y,a1,a2):
    """ zie https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/ voor formule """
    cosq2 =(x**2+y**2-a1**2-a2**2)/(2*a1*a2)
    q2 = math.acos(cosq2)
    if (x==0):
        q1 = math.pi/2-math.atan(a2*math.sin(q2)/(a1+a2*cosq2))
    else:
        q1 = math.atan(y/x)%math.pi-math.atan(a2*math.sin(q2)/(a1+a2*cosq2))
    return q1, q2 # in radians
def angles_vertical_movement(height, distance):
    if math.sqrt(height**2+distance**2)<(DISTANCE_MOTOR_2_3+DISTANCE_MOTOR_3_4):
        q1, q2 = coordinates_to_angles(height, distance, DISTANCE_MOTOR_2_3, DISTANCE_MOTOR_3_4)
        angle_motor_2 = q1 - ANGLE_OFFSET_2_3
        angle_motor_3 = q2 - math.pi/2 + ANGLE_OFFSET_2_3
        angle_motor_4 = (math.pi-q1-q2) #- math.radians(30) # -30 WANT SLECHT GECALIBREERD mss best nog hercalibreren zodat die - 30 niet nodig is
        return (math.degrees(angle_motor_2)+180)%360-180, (math.degrees(angle_motor_3)+180)%360-180, (math.degrees(angle_motor_4)+180)%360-180
    else: # dit is een hééél simpele check. zeker niet genoeg
        print("ONMOGELIJK")
        print(height, distance)
        return (0,0,0)
        
def coordinates_3D_to_angles(x,y,z):
    r = math.sqrt(x**2+y**2)
    if x == 0:
        theta = math.pi/2
    else:
        theta = math.atan(y/x)%math.pi
    angle_motor_1 = math.pi/2 - theta
    distance = r - HOR_DISTANCE_MOTOR_1_2
    height = z - VERT_DISTANCE_MOTOR_1_2
    shoulder_pan = (math.degrees(angle_motor_1)+180)%360-180
    shoulder_lift, elbow_flex, wrist_flex = angles_vertical_movement(height,distance)
    return shoulder_pan, shoulder_lift, elbow_flex, wrist_flex

def move_to_xyz(x,y,z,robot,gripping=False,view_mode=False):
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex = coordinates_3D_to_angles(x=x,y=y,z=z)
    shoulder_pan += IK_JOINT_TRIM_DEG["shoulder_pan"]
    shoulder_lift += IK_JOINT_TRIM_DEG["shoulder_lift"]
    elbow_flex += IK_JOINT_TRIM_DEG["elbow_flex"]
    wrist_flex += IK_JOINT_TRIM_DEG["wrist_flex"]
    if gripping == False:
        gripper = 90
    else:
        gripper = 25
    if view_mode==True:
        wrist_flex -= ANGLE_CAM_WRT_GRIPPER

    TARGET_ANGLES = {
        "shoulder_pan": shoulder_pan,
        "shoulder_lift": shoulder_lift,
        "elbow_flex": elbow_flex,
        "wrist_flex": wrist_flex,
        "wrist_roll": 100,
        "gripper": gripper,
    }
    move_to_target_angles(robot, TARGET_ANGLES)

def get_cam_pos(x_motor_4,y_motor_4,z_motor_4,angle_gripper): # angle_gripper = 0 wnr loodrecht naar beneden gericht, 180 wnr naar boven
    if x_motor_4 == 0:
        theta = math.pi/2
    else:
        theta = math.atan(y_motor_4/x_motor_4)%math.pi
    z = z_motor_4 + math.sin(angle_gripper)*VER_DISTANCE_MOTOR_4_CAM - math.cos(angle_gripper)*HOR_DISTANCE_MOTOR_4_CAM
    r_change = math.sin(angle_gripper)*HOR_DISTANCE_MOTOR_4_CAM + math.cos(angle_gripper)*VER_DISTANCE_MOTOR_4_CAM
    print(math.sin(angle_gripper)*VER_DISTANCE_MOTOR_4_CAM)
    print(math.cos(angle_gripper)*HOR_DISTANCE_MOTOR_4_CAM)

    print(r_change)
    x = x_motor_4 + r_change*math.cos(theta)
    y = y_motor_4 + r_change*math.sin(theta)
    cam_angle = angle_gripper + math.radians(90-ANGLE_CAM_WRT_GRIPPER)
    return x,y,z,theta,cam_angle
    #dit kan ik nu gebruiken voor de transformatie van de kalman estimate naar objectief assenstelsel, en dan terug naar nieuwe assenstelsel

def get_trans_matrix_cam(x_motor_4,y_motor_4,z_motor_4,angle_gripper):
    x,y,z,theta,cam_angle = get_cam_pos(x_motor_4,y_motor_4,z_motor_4,math.radians(angle_gripper))
    ca = math.cos(cam_angle-math.pi/2)
    sa = math.sin(cam_angle-math.pi/2)
    ct = math.cos(theta-math.pi/2)
    st = math.sin(theta-math.pi/2)
    M = np.array([
        [ct, st, 0, -(ct * x + st * y)],
        [-ca * st, ca * ct, sa, ca * st * x - ca * ct * y - sa * z],
        [sa * st, -sa * ct, ca, -sa * st * x + sa * ct * y - ca * z],
        [0, 0, 0, 1]
    ])
    return M

def get_calibrated_joint_limits(robot: SO100Follower, motor_names: list[str]) -> dict[str, tuple[float, float]]:
    if not robot.bus.calibration:
        raise ValueError("Robot calibration is required to derive joint limits.")

    limits = {}
    for name in motor_names:
        if name not in robot.bus.calibration:
            raise ValueError(f"Missing calibration for motor: {name}")

        motor_id = robot.bus.motors[name].id
        calibration = robot.bus.calibration[name]
        raw_limits = {
            motor_id: calibration.range_min,
            -motor_id: calibration.range_max,
        }
        norm_min = float(robot.bus._normalize({motor_id: raw_limits[motor_id]})[motor_id])
        norm_max = float(robot.bus._normalize({motor_id: raw_limits[-motor_id]})[motor_id])
        low, high = sorted((norm_min, norm_max))

        margin = GRIPPER_SAFETY_MARGIN if name == "gripper" else BODY_JOINT_SAFETY_MARGIN_DEG
        low += margin
        high -= margin
        if low >= high:
            raise ValueError(f"Calibration range for motor '{name}' is too small after safety margin.")

        limits[name] = (low, high)

    return limits


def validate_target_joints(
    target_joints: dict[str, float], joint_limits: dict[str, tuple[float, float]], motor_names: list[str]
) -> None:
    missing_limits = [name for name in motor_names if name not in joint_limits]
    if missing_limits:
        raise ValueError(f"Missing joint limits for motors: {missing_limits}")

    invalid_targets = []
    for name in motor_names:
        low, high = joint_limits[name]
        target = target_joints[name]
        if not low <= target <= high:
            invalid_targets.append(f"{name}={target} not in [{low}, {high}]")

    if invalid_targets:
        raise ValueError("Unsafe target angles: " + ", ".join(invalid_targets))

def test_coordinates_to_angles(robot: SO100Follower):
    for i in range(3):
        move_to_xyz(x=0,y=20,z=8,robot=robot)
        time.sleep(1)
        move_to_xyz(x=10,y=20,z=8,robot=robot)
        time.sleep(1)
        move_to_xyz(x=20,y=20,z=8,robot=robot)
        time.sleep(1)



def move_vertically(robot: SO100Follower):
    for i in range(10):
        shoulder_lift, elbow_flex, wrist_flex = angles_vertical_movement(height=10,distance=10)
        TARGET_ANGLES = {
            "shoulder_pan": 0,
            "shoulder_lift": shoulder_lift,
            "elbow_flex": elbow_flex,
            "wrist_flex": wrist_flex,
            "wrist_roll": 100,
            "gripper": 2,
        }
        move_to_target_angles(robot, TARGET_ANGLES)

        shoulder_lift, elbow_flex, wrist_flex = angles_vertical_movement(height=0,distance=10)
        TARGET_ANGLES = {
            "shoulder_pan": 0,
            "shoulder_lift": shoulder_lift,
            "elbow_flex": elbow_flex,
            "wrist_flex": wrist_flex,
            "wrist_roll": 100,
            "gripper": 2,
        }
        move_to_target_angles(robot, TARGET_ANGLES)

        shoulder_lift, elbow_flex, wrist_flex = angles_vertical_movement(height=5,distance=10)
        TARGET_ANGLES = {
            "shoulder_pan": 0,
            "shoulder_lift": shoulder_lift,
            "elbow_flex": elbow_flex,
            "wrist_flex": wrist_flex,
            "wrist_roll": 100,
            "gripper": 2,
        }
        move_to_target_angles(robot, TARGET_ANGLES)

        shoulder_lift, elbow_flex, wrist_flex = angles_vertical_movement(height=0,distance=10)
        TARGET_ANGLES = {
            "shoulder_pan": 0,
            "shoulder_lift": shoulder_lift,
            "elbow_flex": elbow_flex,
            "wrist_flex": wrist_flex,
            "wrist_roll": 100,
            "gripper": 2,
        }
        move_to_target_angles(robot, TARGET_ANGLES)

def grab_at_coordinates(robot: SO100Follower, x,y,z):
    move_to_xyz(x,y,START_HEIGHT_TO_GRIP,robot)
    time.sleep(1)
    move_to_xyz(x,y,z+DISTANCE_MOTOR_4_GRIPPER,robot)
    move_to_xyz(x,y,z+DISTANCE_MOTOR_4_GRIPPER,robot,gripping=True)


def inspect_floor_motors(robot: SO100Follower):
    TARGET_ANGLES = {
        "shoulder_pan": -60,
        "shoulder_lift": 0,
        "elbow_flex": 0,
        "wrist_flex": 20,
        "wrist_roll": 100,
        "gripper": 2,
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(2)

    TARGET_ANGLES = {
        "shoulder_pan": 60,
        "shoulder_lift": 0,
        "elbow_flex": 0,
        "wrist_flex": 20,
        "wrist_roll": 100,
        "gripper": 2,
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(2)


def grab_ball(robot: SO100Follower):
    TARGET_ANGLES = {
        "shoulder_pan": 0,
        "shoulder_lift": 0,
        "elbow_flex": 0,
        "wrist_flex": 20,
        "wrist_roll": 100,
        "gripper": 2,
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(2)

    TARGET_ANGLES = {
        "shoulder_pan": 0,
        "shoulder_lift": 60,
        "elbow_flex": -15,
        "wrist_flex": 0,
        "wrist_roll": 100,
        "gripper": 60,
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(2)
    TARGET_ANGLES = {
        "shoulder_pan": 0,
        "shoulder_lift": 60,
        "elbow_flex": -15,
        "wrist_flex": 0,
        "wrist_roll": 100,
        "gripper": 15,
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(2)
    TARGET_ANGLES = {
        "shoulder_pan": 0,
        "shoulder_lift": 0,
        "elbow_flex": 0,
        "wrist_flex": 20,
        "wrist_roll": 100,
        "gripper": 15,
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(2)



def move_to_target_angles(robot: SO100Follower, target_angles: dict[str, float]):
    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    target_joints = {name: float(target_angles[name]) for name in motor_names}
    joint_limits = get_calibrated_joint_limits(robot, motor_names)
    # validate_target_joints(target_joints, joint_limits, motor_names)

    motor_names = list(robot.bus.motors.keys())

    body_deltas = [
        abs(target_joints[name] - current_joints[name]) for name in motor_names if name != "gripper"
    ]
    gripper_delta = abs(target_joints["gripper"] - current_joints["gripper"])
    body_steps = max((math.ceil(delta / MAX_BODY_JOINT_STEP_DEG) for delta in body_deltas), default=1)
    gripper_steps = math.ceil(gripper_delta / MAX_GRIPPER_STEP) if gripper_delta > 0 else 1
    num_steps = max(1, body_steps, gripper_steps)

    for step_idx in range(1, num_steps + 1):
        alpha = step_idx / num_steps
        joint_action = {
            f"{name}.pos": float(current_joints[name] + alpha * (target_joints[name] - current_joints[name]))
            for name in motor_names
        }
        robot.send_action(joint_action)
        precise_sleep(max(1.0 / FPS, STEP_DELAY_S))


    # print(f"Current joints: {current_joints}")
    # print(f"Target joints:  {target_joints}")
    # print(f"Joint limits:   {joint_limits}")
    # print("Move complete.")


def print_current_angles(robot: SO100Follower):
    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    print(f"Current joints: {current_joints}")
    time.sleep(2)

def gradually_get_to_ball(start_x,start_y,start_z,robot):
    arm_x, arm_y, arm_z = start_x,start_y,start_z
    for i in range(1,5):
        cam_x, cam_y, cam_z = get_coordinates_from_picture()
        cam_coordinates = np.array([cam_x, cam_y, cam_z, 1.])
        M = get_trans_matrix_cam(arm_x, arm_y, arm_z,0)
        coordinates_wrt_motor_1 = np.linalg.inv(M) @ cam_coordinates # from_cam_to_robot_perspective
        x,y,z = coordinates_wrt_motor_1[0], coordinates_wrt_motor_1[1], coordinates_wrt_motor_1[2]
        print("COORINDATES WRT MOTOR 1")
        print(coordinates_wrt_motor_1)
        arm_x, arm_y, arm_z = x,y,START_HEIGHT_TO_GRIP - ((START_HEIGHT_TO_GRIP-(z+DISTANCE_MOTOR_4_GRIPPER))*i/5)
        move_to_xyz(arm_x, arm_y, arm_z,robot)

    move_to_xyz(x,y,z+DISTANCE_MOTOR_4_GRIPPER,robot)
    move_to_xyz(x,y,z+DISTANCE_MOTOR_4_GRIPPER,robot,gripping=True)


def reset_arm(robot: SO100Follower):
    TARGET_ANGLES = {
        "shoulder_pan": 0, # ID 1 #-90..90
        "shoulder_lift": -98, # ID 2
        "elbow_flex": 90, # ID 3
        "wrist_flex": 0, # ID 4
        "wrist_roll": 90, # ID 5
        "gripper": 60, # ID 6
    }
    move_to_target_angles(robot, TARGET_ANGLES)

def move_to_aRz(a,R,z,robot,gripping=False,view_mode=False):
    x=R*math.cos(a)
    y=R*math.sin(a)
    move_to_xyz(x,y,z,robot,gripping,view_mode)

def PID_sequentie(robot):
    start_x,start_y,start_z = 0,15,25
    move_to_xyz(x=start_x,y=start_y,z=start_z,robot=robot, view_mode=True)
    cam_x, cam_y, cam_z = get_coordinates_from_picture()
    print("COORINDATES WRT CAM")
    print(cam_x, cam_y, cam_z)
    cam_coordinates = np.array([cam_x, cam_y, cam_z, 1.])
    M = get_trans_matrix_cam(start_x,start_y,start_z,0)
    coordinates_wrt_motor_1 = np.linalg.inv(M) @ cam_coordinates # from_cam_to_robot_perspective
    # omzetten naar poolcoördinaten
    print("COORINDATES WRT MOTOR 1")
    print(coordinates_wrt_motor_1)
    x,y,z,_ = coordinates_wrt_motor_1

    # tijdelijk:
    x,y,z = start_x,start_y,start_z
    R = math.sqrt(x*2+y*2)  +5 # 10 minder zodat grijper erboven staat -> vervang later de 10 nog met magic value MOTOR4_TO_GRIPPER
    a = math.atan2(y,x) % math.pi

    print("R en a")
    print(R, a)

    conditie_PID = False
    C1 = .003
    C2 = .03
    while not conditie_PID:
        print(R)
        move_to_aRz(a,R,z=PID_HEIGHT,robot=robot, view_mode=True)
        Mx,My,_ = get_M_and_radius_from_picture()
        # nu moet je R en a aanpassen om (Mx, My) tot (420,120) te brengen
        if (abs(Mx-420) < 3) and (abs(My-120) < 3):
            conditie_PID = True
            break
        print("errors")
        a += (Mx-420)*C1
        print(Mx-420)
        R += (My-120)*C2

        print(My-120)
        print("a, R")
        print(a, R)
    move_to_aRz(a,R,z=0,robot=robot, view_mode=True)
    move_to_aRz(a,R,z=0,robot=robot, view_mode=True, gripping=True)
    move_to_aRz(a,R,z=20,robot=robot, view_mode=True, gripping=True)

    time.sleep(1)

# van Andreas: afblijven!
# CALIBRATION_FILE = Path("color_calibration.json")

# def load_matrix():
#     if not CALIBRATION_FILE.exists():
#         return None
#     with open(CALIBRATION_FILE, "r") as f:
#         return np.array(json.load(f), dtype=np.float32)

# import time

# class PID:
#     def __init__(self, Kp, Ki, Kd, setpoint=0.0,
#                  integral_limit=None, output_limit=None):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.setpoint = setpoint
#         self.integral_limit = integral_limit  # anti-windup clamp
#         self.output_limit = output_limit      # clamp total output

#         self._integral = 0.0
#         self._prev_error = None
#         self._prev_time = None

#     def reset(self):
#         self._integral = 0.0
#         self._prev_error = None
#         self._prev_time = None

#     def update(self, measurement: float) -> float:
#         now = time.monotonic()
#         error = self.setpoint - measurement

#         # dt
#         if self._prev_time is None:
#             dt = 0.0
#         else:
#             dt = now - self._prev_time

#         # Integral with anti-windup
#         self._integral += error * dt
#         if self.integral_limit is not None:
#             self._integral = max(-self.integral_limit,
#                                   min(self.integral_limit, self._integral))

#         # Derivative (on measurement, not error — avoids "derivative kick")
#         if self._prev_error is None or dt == 0.0:
#             derivative = 0.0
#         else:
#             derivative = (error - self._prev_error) / dt

#         self._prev_error = error
#         self._prev_time = now

#         output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative

#         if self.output_limit is not None:
#             output = max(-self.output_limit, min(self.output_limit, output))

#         return output


def main():
    robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
    robot = SO100Follower(robot_config)
    robot.connect()

    if not robot.is_connected:
        raise ValueError("Robot is not connected!")


    try:
        PID_sequentie(robot)


        # print_current_angles(robot)
        #start_x,start_y,start_z = 0,15,25
        #move_to_xyz(x=start_x,y=start_y,z=start_z,robot=robot, view_mode=True)


        #gradually_get_to_ball(start_x,start_y,start_z,robot)


        # trek foto met camera => coordinaten in camera-assenstelsel
        #cam_x, cam_y, cam_z = get_coordinates_from_picture()
        #cam_x, cam_y, cam_z = 9.6,3.33,-20
        print("COORINDATES WRT CAM")
        #print(cam_x, cam_y, cam_z)

        #cam_coordinates = np.array([cam_x, cam_y, cam_z, 1.])
        #M = get_trans_matrix_cam(start_x,start_y,start_z,0)
        #coordinates_wrt_motor_1 = np.linalg.inv(M) @ cam_coordinates # from_cam_to_robot_perspective
        #coordinates_wrt_motor_1 = np.array([5,10,-15])
        print("COORINDATES WRT MOTOR 1")
        #print(coordinates_wrt_motor_1)


        # van Andreas: afblijven!
        # cap = cv2.VideoCapture(gstreamer_pipeline())
        # arm_x, arm_y, arm_z = start_x, start_y, start_z
        # pid_x = PID(Kp=0.3, Ki=0.01, Kd=0.05, setpoint=0.0,
        #     integral_limit=5.0, output_limit=3.0)
        # pid_y = PID(Kp=0.3, Ki=0.01, Kd=0.05, setpoint=0.0,
        #             integral_limit=5.0, output_limit=3.0)
        # pid_z = PID(Kp=0.3, Ki=0.01, Kd=0.05, setpoint=-20.0,  # 20 cm depth
        #             integral_limit=5.0, output_limit=3.0)
        # while True:
        #     ret, frame = cap.read()
        #     if not ret:
        #         break
        #         frame = capture_frame()

        #     M = load_matrix()
        #     if M is not None:
        #         frame = apply_matrix(frame, M)

        #     try:
        #         cam_x, cam_y, cam_z = get_coordinates_from_frame(frame)
        #         cam_x = -cam_x
        #     except RuntimeError:
        #         # Ball not detected — let integral hold, don't update
        #         continue
        #     cam_dx = pid_x.update(cam_x)   # error = 0 - cam_x
        #     cam_dy = pid_y.update(cam_y)   # error = 0 - cam_y
        #     cam_dz = pid_z.update(cam_z)   # error = -20 - cam_z
        #     cam_coordinates = np.array([cam_dx, cam_dy, cam_dz, 1.])
        #     M = get_trans_matrix_cam(arm_x, arm_y, arm_z,0)
        #     coordinates_wrt_motor_1 = np.linalg.inv(M) @ cam_coordinates # from_cam_to_robot_perspective
        #     arm_x += coordinates_wrt_motor_1[0]
        #     arm_y += coordinates_wrt_motor_1[1]   # moving forward closes depth
        #     arm_z += coordinates_wrt_motor_1[2]
        #     move_to_xyz(arm_x, arm_y, arm_z,robot)
        # cap.release()





        grab_at_coordinates(robot, coordinates_wrt_motor_1[0], coordinates_wrt_motor_1[1], coordinates_wrt_motor_1[2])

    except Exception as e:
        print("ERROR!")
        print(e)
    finally:
        time.sleep(1)
        reset_arm(robot)

    # inspect_floor_motors(robot)
    # grab_ball(robot)
    #test_coordinates_to_angles(robot)
    #move_vertically(robot)
    # print_current_angles(robot)
    time.sleep(1)




if __name__ == "__main__":
    main()
