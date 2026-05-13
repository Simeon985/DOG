import math
import time

from processes.threads.control_help_commands import init_robot, move_rot_and_straight
from lerobot.examples.phone_to_so100.arm_move_angles import grab_ball as arm_grab_ball
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig


_robot = None
_arm_robot = None


def opstart():
    # Speel geluidje
    # Stuur bepaalde oogjes
    # Draai rondje
    pass


def search_loop(subject):
    pass

def _get_robot():
    global _robot
    if _robot is None:
        _robot = init_robot()
    return _robot


def drive_to_ball(
    x,
    y,
    step_cm: float = 50.0,
    final_stop_distance_cm: float = 20.0,
    step_speed_cm_s: float = 50.0,
) -> bool:
    """
    Drive the base toward a detected ball in camera coordinates in one chunk.

    Expected coordinate convention:
    - x: horizontal offset in cm (right is positive)
    - y: depth in cm, negative in this codebase

    Returns True when a step was executed, otherwise False if the ball is already
    close enough to stop.
    """
    if x is None or y is None:
        return False

    robot = _get_robot()
    bus = robot.bus

    # y is negative for objects in front of the camera in this project.
    depth_cm = max(1e-6, -float(y))
    distance_cm = math.sqrt(float(x) ** 2 + depth_cm ** 2)
    if distance_cm <= final_stop_distance_cm:
        move_rot_and_straight(bus, True, 0, "", 0, 0)
        return False

    error_rotation = math.degrees(math.atan2(float(x), depth_cm))
    rotation_velocity_normalized = min(20.0, abs(error_rotation)) * 0.05
    direction = "left" if error_rotation < 0 else "right"

    # Use the step size to decide how long this chunk should run.
    step_distance = min(max(step_cm, 1.0), distance_cm)
    step_duration_s = step_distance / max(step_speed_cm_s, 1e-6)

    # Match sensor_control.py normalization style, but with cm input.
    straight_velocity_normalized = min(0.1, step_distance / 100.0) * 10.0

    move_rot_and_straight(
        bus,
        False,
        rotation_velocity_normalized,
        direction,
        straight_velocity_normalized,
        error_rotation,
    )
    time.sleep(step_duration_s)
    move_rot_and_straight(bus, True, 0, "", 0, 0)
    return True


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

