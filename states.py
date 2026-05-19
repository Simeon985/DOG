import math
import sys
import time
from pathlib import Path
import pygame

# Vendored LeRobot: `examples/` is not inside the installable `lerobot` package on PyPI.
_lerobot_root = Path(__file__).resolve().parent / "lerobot"
for _p in (_lerobot_root / "src", _lerobot_root):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

import torch
from multiprocessing import Array
from arm_move_angles import grab_ball as arm_grab_ball, move_to_target_angles, move_to_xyz
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig
from processes.threads.control_help_commands import init_robot, move_rot_and_straight, rotate_platform

import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import itertools
from multiprocessing.sharedctypes import SynchronizedArray
from processes.threads.mapping import *
from processes.threads.control import *
from processes.threads.control_help_commands import *

from coordinates_from_picture import get_coordinates_from_picture

from merged_camera import initialize_camera, detect_ball, get_video_capture, read_bgr_frame
import cv2


_robot = None
_arm_robot = None
_est = None
_ser = None
_hist_stupid = None

YOLO_MODEL_PATH = "models/balls_ourdata_augmented.pt"
shared_array = Array("d", [0.0] * 11)


def search_loop(
    wrist_angle: float,
    model_path: str | Path | None = None,
) -> tuple[float, float, float] | None:
    print("nu zit hij in search loop!")
    arm_angles = {
        "shoulder_pan": 0,
        "shoulder_lift": -90,
        "elbow_flex": 90,
        "wrist_flex": wrist_angle,
        "wrist_roll": 92,
        "gripper": 60,
    }

    est = _get_est()
    start_angle = est.history[-1][2]

    try:
        mp = model_path if model_path is not None else YOLO_MODEL_PATH
        initialize_camera(model_path=mp)

        print("should start moving arms")
        # Position arm for this subject
        arm = _get_arm_robot()
        target_angles = arm_angles
        move_to_target_angles(arm, target_angles)
        time.sleep(0.5)  # Allow arm to settle

        # Capture and detect
        cap = get_video_capture()
        if cap is None or not cap.isOpened():
            print("Camera not available")
            return None

        coords = None
        while coords is None:
            ret, frame = read_bgr_frame(timeout_sec=3.0)
            cv2.imwrite("image.png", frame)
            
            if not ret or frame is None:
                print("Failed to read frame")
                return None

            # Always use ball detection for now
            coords = detect_ball(frame, wrist_angle=wrist_angle)
            _get_robot().bus.sync_write("Goal_Velocity", rotate_platform(_get_robot().bus, False, 1, "right"))
            print(f"No ball detected")
        else:
            _get_robot().bus.sync_write("Goal_Velocity", rotate_platform(_get_robot().bus, True, 0, "right"))
            print("FINAL CHECK:")
            time.sleep(2)
            coords = detect_ball(frame,wrist_angle=wrist_angle)
            end_angle = est.history[-1][2]
            _get_hist_stupid().append(("rot", end_angle-start_angle))
            print(f"Detected ball at {coords}")

        return coords

    except Exception as e:
        print(f"Error in search_loop: {e}")
        return None


def drive_to_ball(
    x,
    y,
    step_cm: float = 50.0,
    final_stop_distance_cm: float = 20.0,
    step_speed_cm_s: float = 50.0,
    rotation_tolerance_deg: float = 2.0,
    distance_tolerance_cm: float = 5.0
) -> tuple[bool, np.ndarray]:
    """
    Drive the base toward a detected ball in camera coordinates in one chunk.

    Expected coordinate convention:
    - x: horizontal offset in cm (right is positive)
    - y: depth in cm, negative in this codebase

    Returns True when a step was executed, otherwise False if the ball is already
    close enough to stop.
    """
    _get_hist_stupid().append(("drive", (x,y)))
    x =x/100
    y =y/100
    step_m = step_cm / 100.0
    desired_angle = math.degrees(math.atan2(float(x), float(y)))
    print("desired angle:")
    print(desired_angle)
    desired_distance = min(step_m, math.sqrt(float(x) ** 2 + float(y) ** 2))
    robot = _get_robot()
    est = _get_est()
    ser = _get_ser()

    if desired_angle < 0:
        direction = "left"
    else:
        direction = "right"

    start_angle = est.history[-1][2]
    print("start angle")
    print(start_angle)
    start_x = est.history[-1][0]
    start_y = est.history[-1][1]
    # zet om naar world frame ipv robot frame:
    desired_angle = (desired_angle + start_angle) % 360
    #upper_lim_desired_angle= (desired_angle+2) %360
    try:
        while(1):
            current_angle = est.history[-1][2]
            error_rotation = (desired_angle - current_angle) % 360

            if error_rotation > 180:
                error_rotation -= 360
            if error_rotation < -180:
                error_rotation += 360
            print("error rotation")
            print(error_rotation)
            rotation_velocity_normalized=min(20,abs(error_rotation))*0.05
            direction = "left" if error_rotation >0 else "right"


            x,y = est.history[-1][0], est.history[-1][1]
            distance_from_start = np.sqrt((x-start_x)**2 + (y-start_y)**2)
            # print(distance_from_start)
            error_distance = desired_distance - distance_from_start
            straight_velocity_normalized = min(0.1,abs(error_distance))*10

            #print("rotation error: ", error_rotation, " distance error: ", error_distance, " current angle: ", current_angle, " current distance: ", distance_from_start)
            move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, straight_velocity_normalized, error_rotation)
            #vierkant_maken(robot.bus, False, rotation_velocity_normalized, direction, 1, error_rotation)
            # with est.lock:
                # brake = est.brake
                # if brake:
                    # drive_to_ball(x + 100, y)
            if abs(error_rotation) > 2:
                move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, 0, error_rotation)
            elif abs(error_distance) > 0.05:
                move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, straight_velocity_normalized, error_rotation)
            else:
                print("Desired position reached. Stopping the robot.")
                move_rot_and_straight(robot.bus, True, 0, "", 0, 0)
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt in drive_to_ball")
        # print(est.history, " ", np.sqrt(est.history[-1][0]**2 + est.history[-1][1]**2)," meter")
    except Exception as e:
        print(e)
        stop_movement()
    finally:
        print("\nfinally: KeyboardInterrupt in drive_to_ball")
        # print(est.history, " ", np.sqrt(est.history[-1][0]**2 + est.history[-1][1]**2)," meter")

        return (desired_distance == step_m) # pas dit nog aan

def search_locally(robot):
    angles_to_scan = [90, 0, -30]
    for angle in angles_to_scan:
        time.sleep(5)
        motor_names = list(robot.bus.motors.keys())
        current_obs = robot.get_observation()
        current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}

        TARGET_ANGLES = {
            "shoulder_pan": 0,
            "shoulder_lift": -90,
            "elbow_flex": 90,
            "wrist_flex": angle,
            "wrist_roll": 92,
            "gripper": 60,
        }

        move_to_target_angles(robot, TARGET_ANGLES,step_delay=0.05)
        
        ret, frame = read_bgr_frame(timeout_sec=3.0)
        cam_coords = detect_ball(frame, wrist_angle=angle)
        if not cam_coords:
            continue
        cam_x, cam_y, cam_z = cam_coords
        if cam_y and angle >= 25:
            return True, -20
        elif cam_y:
            return True, cam_y
    return False, None

def _get_robot():
    global _robot
    if _robot is None:
        _robot = init_robot()
    return _robot


def _get_arm_robot():
    """Get or initialize the arm (SO100Follower) robot."""
    global _arm_robot
    if _arm_robot is None:
        config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
        _arm_robot = SO100Follower(config)
        _arm_robot.connect()
        if not _arm_robot.is_connected:
            raise RuntimeError("Arm robot failed to connect.")
    return _arm_robot


def _set_est(estimator):
    """Set the global estimator."""
    global _est
    _est = estimator


def _get_est():
    """Get the global estimator."""
    global _est
    if _est is None:
        raise RuntimeError("Estimator not initialized. Call _set_est() first.")
    return _est

def _set_hist_stupid():
    global _hist_stupid
    _hist_stupid = []

def _get_hist_stupid():
    global _hist_stupid
    if _est is None:
        raise RuntimeError("Not initialized. Call _set_hist_stupid first.")
    return _hist_stupid

def _set_ser(serial):
    """Set the global serial connection."""
    global _ser
    _ser = serial


def _get_ser():
    """Get the global serial connection."""
    global _ser
    if _ser is None:
        raise RuntimeError("Serial connection not initialized. Call _set_ser() first.")
    return _ser


def grab_ball() -> None:
    """
    Execute the ball-grabbing sequence with the arm.
    Assumes the arm is positioned such that the ball is directly in front of it.
    """
    arm = _get_arm_robot()
    arm_grab_ball(arm)

def stop_movement():
    robot = _get_robot()
    bus = robot.bus
    move_rot_and_straight(bus, True, 0, "", 0, 0)


def return_with_ball_stupid():
    hist_stupid = _get_hist_stupid()
    for tup in hist_stupid:
        type_of_movement = tup[0]
        if type_of_movement == "rot":
            a = -tup[1]
            x = -0.01*math.sin(a)
            y = 0.01*math.cos(a)
            drive_to_ball(-x,-y)
        elif type_of_movement == "drive":
            x,y = tup[1]
            drive_to_ball(-x,-y)


def return_with_ball():

    est = _get_est()
    hist_len = len(est.history)
    print("hist len")
    print(hist_len)
    print(est.history)
    step_size = 20
    for index in range(hist_len-step_size, 0, -step_size):
        print("index in return")
        print(index)
        print(est.history)
        x_now,y_now = est.history[-1][0], est.history[-1][1]
        x_dest,y_dest = est.history[index][0],est.history[index][1]
        print("x_now, y_now")
        print(x_now, y_now)
        print("x_dest,y_dest")                
        print(x_dest,y_dest)
        x_diff,y_diff = x_dest-x_now,y_dest-y_now
        total_distance = math.sqrt(x_diff**2+y_diff**2)
        if total_distance < .05:
            continue
        angle_now = math.radians(est.history[-1][2])
        angle_dest = math.atan2(y_diff,x_diff)
        angle_diff = angle_dest-angle_now
        x = math.cos(angle_diff)*total_distance
        y = -math.sin(angle_diff)*total_distance
        print("x, y destination van return")
        print(x, y)
        drive_to_ball(x*100,y*100,step_cm=100.)
    print("returned")