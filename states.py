import math
import sys
import time
from pathlib import Path
import pygame

# Vendored LeRobot: `examples/` is not inside the installable `lerobot` package on PyPI.
_lerobot_root = Path(__file__).resolve().parent / "lerobot"
_so100_examples = _lerobot_root / "examples" / "phone_to_so100"
for _p in (_lerobot_root / "src", _so100_examples):
    _s = str(_p)
    if _s not in sys.path:
        sys.path.insert(0, _s)

import torch
from multiprocessing import Array
from arm_move_angles import grab_ball as arm_grab_ball, move_to_target_angles
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




from camera import initialize_camera, detect_ball, get_video_capture, read_bgr_frame


_robot = None
_arm_robot = None

YOLO_MODEL_PATH = "lerobot/examples/phone_to_so100/balls_ourdata_augmented.pt"
shared_array = Array("d", [0.0] * 11)



def opstart():
    sound= "/home/dog/DOG/audio/Eekhoorn3.mp3"
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()

    # Keep the script alive until playback finishes
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

    # Speel geluidje
    # Stuur bepaalde oogjes
    # Draai rondje
    pass


def search_loop(
    subject: str,
    model_path: str | Path | None = None,
) -> tuple[float, float, float] | None:
    """
    Search for an object by positioning the arm and detecting it from the camera feed.

    Args:
        subject: One of "ball_floor", "ball_air", or "person"
        model_path: YOLO .pt weights; defaults to YOLO_MODEL_PATH in this module.

    Returns:
        (x, y, z) coordinates in cm in the camera frame, or None if not detected.
    """
    # Define arm angles for each subject type
    arm_angles = {
        "ball_floor": {
            "shoulder_pan": 0,
            "shoulder_lift": -99,
            "elbow_flex": 90,
            "wrist_flex": 0,
            "wrist_roll": 92,
            "gripper": 60,
        },
        "ball_air": {
            "shoulder_pan": 0,
            "shoulder_lift": 0,
            "elbow_flex": -48,
            "wrist_flex": 0,
            "wrist_roll": 92,
            "gripper": 60,
        },
        "person": {
            "shoulder_pan": 0,
            "shoulder_lift": -99,
            "elbow_flex": 90,
            "wrist_flex": 0,
            "wrist_roll": 92,
            "gripper": 60,
        },
    }

    if subject not in arm_angles:
        print(f"Unknown subject: {subject}")
        return None


    try:
        mp = model_path if model_path is not None else YOLO_MODEL_PATH
        initialize_camera(model_path=mp)

        # Position arm for this subject
        arm = _get_arm_robot()
        target_angles = arm_angles[subject]
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
            if not ret or frame is None:
                print("Failed to read frame")
                return None

            # Always use ball detection for now
            coords = detect_ball(frame)
            _get_robot().bus.sync_write("Goal_Velocity", rotate_platform(_get_robot().bus, False, 1, "right"))
            print(f"No {subject} detected")
        else:
            _get_robot().bus.sync_write("Goal_Velocity", rotate_platform(_get_robot().bus, True, 0, "right"))
            print(f"Detected {subject} at {coords}")

        return coords

    except Exception as e:
        print(f"Error in search_loop({subject}): {e}")
        return None

def drive_to_ball(
    x,
    y,
    step_cm: float = 50.0,
    final_stop_distance_cm: float = 20.0,
    step_speed_cm_s: float = 50.0,
    rotation_tolerance_deg: float = 2.0,
    distance_tolerance_cm: float = 5.0,
) -> bool:
    """
    Drive the base toward a detected ball in camera coordinates in one chunk.

    Expected coordinate convention:
    - x: horizontal offset in cm (right is positive)
    - y: depth in cm, negative in this codebase

    Returns True when a step was executed, otherwise False if the ball is already
    close enough to stop.
    """
    x =x/100
    y =y/100
    step_m = step_cm / 100.0
    desired_angle = math.degrees(math.atan2(float(x), max(1e-6, -float(y))))
    desired_distance = min(step_m, math.sqrt(float(x) ** 2 + float(y) ** 2))
    seperate_movements = True
    robot = _get_robot()
    ser = initialize_esp()
    scale_1, scale_2, angle_1, angle_2 = 5.803293347508479e-05, 5.973355990292423e-05, -43.941917924421915, 120.27700785358547
    data = np.zeros(11)
    global est
    est = PeripheralEstimator(scale_1, scale_2, angle_1, angle_2)
    stop_event = threading.Event()
    test_counter = [0]

    t1 = threading.Thread(target=control, args=(stop_event, test_counter, shared_array), daemon = True)
    print("sensor_mapping_should_begin")
    t2 = threading.Thread(target=est.update, args=(ser, data, stop_event), daemon = True)
    if desired_angle < 0:
        direction = "left"
    else:
        direction = "right"


    # t3 = threading.Thread(target=init_robot, args=(stop_event,direction))
    t1.start()
    t2.start()
    time.sleep(4)  # Ensure the control thread is running before starting the mapping thread
    # t3.start()
    #robot=init_robot()
    start_angle = est.history[10][2]
    x = est.history[10][0]
    y = est.history[10][1]
    # desired_angle = (desired_angle ) % 360
    #upper_lim_desired_angle= (desired_angle+2) %360
    current_angle = est.history[-1][2]
    try:
        while(1):
            current_angle = est.history[-1][2]
            error_rotation = (current_angle- start_angle - desired_angle) % 360
            if error_rotation > 180:
                error_rotation -= 360
            rotation_velocity_normalized=min(20,abs(error_rotation))*0.05
            direction = "left" if error_rotation <0 else "right"


            x,y = est.history[-1][0], est.history[-1][1]
            distance_from_start = np.sqrt(x**2 + y**2)
            error_distance = desired_distance - distance_from_start
            straight_velocity_normalized = min(0.1,abs(error_distance))*10

            #print("rotation error: ", error_rotation, " distance error: ", error_distance, " current angle: ", current_angle, " current distance: ", distance_from_start)
            move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, straight_velocity_normalized, error_rotation)
            #vierkant_maken(robot.bus, False, rotation_velocity_normalized, direction, 1, error_rotation)
            if (seperate_movements):
                if abs(error_rotation) > 2:
                    move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, 0, error_rotation)
                elif abs(error_distance) > 0.05:
                    move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, straight_velocity_normalized, error_rotation)
                else:
                    print("Desired position reached. Stopping the robot.")
                    move_rot_and_straight(robot.bus, True, 0, "", 0, 0)
                    stop_event.set()
                    break
            else:
                if (-1 <error_rotation < 2 and abs(error_distance) < 0.05):
                    print("Desired angle reached. Stopping the robot.")
                    #aanpassen rotate_platform(robot.bus, True)
                    stop_event.set()
                    break


            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt in sensor_control_process")
        print(est.history, " ", np.sqrt(est.history[-1][0]**2 + est.history[-1][1]**2)," meter")
        stop_event.set()
    finally:
        print("\nfinally: KeyboardInterrupt in sensor_control_process")
        stop_event.set()
        print(est.history, " ", np.sqrt(est.history[-1][0]**2 + est.history[-1][1]**2)," meter")
        t1.join()
        t2.join()
    return


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

