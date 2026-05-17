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
import sys
import os
import time
from pathlib import Path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from social_functions import neutraal, boos, sad, hart

# This file lives in `DOG/lerobot/`. `main.py` imports it as top-level `arm_move_angles`, and it may
# also be run as `python arm_move_angles.py` from this directory — so parent `DOG/` must be on
# `sys.path` (same idea as `states.py` inserting `lerobot/src` and `lerobot/`).
_this_dir = Path(__file__).resolve().parent
_dog_root = _this_dir.parent
for _p in (_this_dir / "src", _this_dir, _dog_root):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

from coordinates_from_picture import get_coordinates_from_picture, get_M_and_radius_from_picture

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


def coordinates_to_angles(x, y, a1, a2):
    """zie https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/ voor formule"""
    cosq2 = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)
    q2 = math.acos(cosq2)
    if x == 0:
        q1 = math.pi / 2 - math.atan(a2 * math.sin(q2) / (a1 + a2 * math.cos(q2)))
    else:
        q1 = math.atan(y / x) % math.pi - math.atan(a2 * math.sin(q2) / (a1 + a2 * math.cos(q2)))
    return q1, q2


def angles_vertical_movement(height, distance):
    if math.sqrt(height**2 + distance**2) < (DISTANCE_MOTOR_2_3 + DISTANCE_MOTOR_3_4):
        q1, q2 = coordinates_to_angles(height, distance, DISTANCE_MOTOR_2_3, DISTANCE_MOTOR_3_4)
        angle_motor_2 = q1 - ANGLE_OFFSET_2_3
        angle_motor_3 = q2 - math.pi / 2 + ANGLE_OFFSET_2_3
        angle_motor_4 = math.pi - q1 - q2
        return (
            (math.degrees(angle_motor_2) + 180) % 360 - 180,
            (math.degrees(angle_motor_3) + 180) % 360 - 180,
            (math.degrees(angle_motor_4) + 180) % 360 - 180,
        )
    print("ONMOGELIJK")
    print(height, distance)
    return (0, 0, 0)


def coordinates_3D_to_angles(x,y,z):
    print("XYZ")
    print(x)
    print(y)
    print(z)
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

# def get_trans_matrix_cam(x_motor_4,y_motor_4,z_motor_4,angle_gripper):
#     x,y,z,theta,cam_angle = get_cam_pos(x_motor_4,y_motor_4,z_motor_4,math.radians(angle_gripper))
#     ca = math.cos(cam_angle-math.pi/2)
#     sa = math.sin(cam_angle-math.pi/2)
#     ct = math.cos(theta-math.pi/2)
#     st = math.sin(theta-math.pi/2)
#     M = np.array([
#         [ct, st, 0, -(ct * x + st * y)],
#         [-ca * st, ca * ct, sa, ca * st * x - ca * ct * y - sa * z],
#         [sa * st, -sa * ct, ca, -sa * st * x + sa * ct * y - ca * z],
#         [0, 0, 0, 1]
#     ])
#     return M

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



def move_to_target_angles(robot: SO100Follower, target_angles: dict[str, float], step_delay=0.05):
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
        precise_sleep(max(0.01, step_delay))


def print_current_angles(robot: SO100Follower):
    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    print(f"Current joints: {current_joints}")
    time.sleep(2)

def reset_arm(robot: SO100Follower):
    TARGET_ANGLES = {
        "shoulder_pan": 0, # ID 1 #-90..90
        "shoulder_lift": -98, # ID 2
        "elbow_flex": 90, # ID 3
        "wrist_flex": 0, # ID 4
        "wrist_roll": 90, # ID 5
        "gripper": 25, # ID 6
    }
    move_to_target_angles(robot, TARGET_ANGLES)

def move_to_aRz(a,R,z,robot,gripping=False,view_mode=False):
    x=R*math.cos(a)
    y=R*math.sin(a)
    move_to_xyz(x,y,z,robot,gripping,view_mode)

def find_good_scan_pos(robot):
    
    min_distance = np.inf
    positions_to_scan = [(-15,1,25),(-10,5,25),(-5,10,25),(0,15,25),(5,10,25),(10,5,25),(15,1,25)]
    for coordinate_tup in positions_to_scan:
        start_x,start_y,start_z = coordinate_tup
        move_to_xyz(x=start_x,y=start_y,z=start_z,robot=robot, view_mode=True, gripping=True)
        cam_x, cam_y, cam_z = get_coordinates_from_picture()
        if cam_x != None:
            distance_from_middle = cam_x**2+cam_y**2
            if distance_from_middle < min_distance:
                min_distance = distance_from_middle
                best_coordinates = coordinate_tup
    if min_distance == np.inf:
        return None, None, None
    return best_coordinates


def PID_sequentie2(robot):
    opgepakt = False
    neutraal()
    while True:
        
        start_x,start_y,start_z = find_good_scan_pos(robot)
        if start_x == None:
            if opgepakt:
                hart()
            else:
                sad()
            return opgepakt
        elif opgepakt:
            boos()
            opgepakt = False

        move_to_xyz(x=start_x,y=start_y,z=start_z,robot=robot, view_mode=True)
        #print("net voor hij de eerste keer get_coordinates_from_picture oproept")
        #cam_x, cam_y, cam_z = get_coordinates_from_picture()
        #print("net na hij de eerste keer get_coordinates_from_picture oproept")
        #print(cam_x)
        #if cam_x == None:
        #    continue

        #cam_coordinates = np.array([cam_x, cam_y, cam_z, 1.])
        #M = get_trans_matrix_cam(start_x,start_y,start_z,0)
        #coordinates_wrt_motor_1 = np.linalg.inv(M) @ cam_coordinates # from_cam_to_robot_perspective
        # omzetten naar poolcoördinaten

        #x,y,z,_ = coordinates_wrt_motor_1

        #R = math.sqrt(x*2+y*2)  +5 # 10 minder zodat grijper erboven staat -> vervang later de 10 nog met magic value MOTOR4_TO_GRIPPER
        #a = math.atan2(y,x) % math.pi
        R = math.sqrt(start_x*2+start_y*2)
        a = math.atan2(start_y,start_x) % math.pi

        conditie_PID = False
        C1 = .002
        C2 = .04
        Mx_perfect = 440 # 430, 420
        My_perfect = 125 # 115, 120
        Mx = Mx_perfect
        My = My_perfect
        times_failed_PID = 0
        while not conditie_PID:
            a += (Mx-Mx_perfect)*C1
            R += (My-My_perfect)*C2
            print("moving in PID")
            move_to_aRz(a,max(R,5),z=PID_HEIGHT,robot=robot, view_mode=True)
            print("moved in PID")
            time.sleep(.1)
            Mx,My,_ = get_M_and_radius_from_picture()
            if Mx == None:
                Mx = Mx_perfect
                My = My_perfect
                times_failed_PID += 1
                if times_failed_PID > 3:
                    break
                continue
            # nu moet je R en a aanpassen om (Mx, My) tot (420,120) te brengen
            if (abs(Mx-Mx_perfect) < 3) and (abs(My-My_perfect) < 3):
                print("PID geslaagd")
                conditie_PID = True
        if not conditie_PID:
            continue
        print("uit PID")
        move_to_aRz(a,R,z=0,robot=robot, view_mode=True)
        move_to_aRz(a,R,z=0,robot=robot, view_mode=True, gripping=True)
        move_to_xyz(start_x,start_y,start_z,robot=robot, view_mode=True, gripping=True)
        opgepakt = True



def PID_air_tracking(robot):
    while True:
        
        motor_names = list(robot.bus.motors.keys())
        current_obs = robot.get_observation()
        current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
        a = current_obs["shoulder_pan.pos"]
        b = current_obs["wrist_flex.pos"]

        C1 = .002
        C2 = .04
        Mx_perfect = 320
        My_perfect = 180
        Mx = Mx_perfect
        My = My_perfect
        times_failed_PID = 0
        while times_failed_PID < 30:
            a += -(Mx-Mx_perfect)*C2
            b += -(My-My_perfect)*C2
            
            TARGET_ANGLES = {
                "shoulder_pan": a,
                "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
                "elbow_flex": float(current_obs["elbow_flex.pos"]),
                "wrist_flex": b,
                "wrist_roll": float(current_obs["wrist_roll.pos"]),
                "gripper": float(current_obs["gripper.pos"]),
            }
            move_to_target_angles(robot, TARGET_ANGLES)

            time.sleep(.1)
            Mx,My,_ = get_M_and_radius_from_picture()
            if Mx == None:
                Mx = Mx_perfect
                My = My_perfect
                times_failed_PID += 1

        print("uit PID")
        # nu moet hij beginnen zoeken



def main():
    robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
    robot = SO100Follower(robot_config)
    robot.connect()

    if not robot.is_connected:
        raise ValueError("Robot is not connected!")

    try:
        #PID_air_tracking(robot)
        PID_sequentie2(robot)


    except Exception as e:
        print("ERROR!")
        print(e)
    finally:
        time.sleep(1)
        reset_arm(robot)

    time.sleep(1)


if __name__ == "__main__":
    main()
