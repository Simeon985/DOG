import math
import time

from processes.threads.control_help_commands import init_robot, move_rot_and_straight
from lerobot.examples.phone_to_so100.arm_move_angles import (
    grab_ball as arm_grab_ball,
    move_to_target_angles,
)
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

from camera import (
    initialize_camera,
    detect_ball,
    get_video_capture,
)


_robot = None
_arm_robot = None

# Camera and detection setup
_cap = None
_model = None
_gst_proc = None

# Detection constants
YOLO_MODEL_PATH = "./balls.pt"
YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
YOLO_HALF = torch.cuda.is_available()
YOLO_IMGSZ = 224
CONF_THRESHOLD = 0.2
GREEN_CLASS_INDEX = 2
CAMERA_H_FOV_DEG = 60

# CSI camera config
CSI_SENSOR_ID = 0
CSI_FLIP_METHOD = 0
CSI_WIDTH = 320
CSI_HEIGHT = 240
CSI_FPS = 10
TCP_HOST = "127.0.0.1"
TCP_PORT = 5000
SYSTEM_GST_LAUNCH = "/usr/bin/gst-launch-1.0"
SYSTEM_GST_INSPECT = "/usr/bin/gst-inspect-1.0"
SYSTEM_GST_PLUGIN_SCANNER = "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
SYSTEM_GST_PLUGIN_PATH = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"


def opstart():
    # Speel geluidje
    # Stuur bepaalde oogjes
    # Draai rondje
    pass


def search_loop(subject: str) -> tuple[float, float, float] | None:
    """
    Search for an object by positioning the arm and detecting it from the camera feed.
    
    Args:
        subject: One of "ball_floor", "ball_air", or "person"
    
    Returns:
        (x, y, z) coordinates in cm in the camera frame, or None if not detected.
    """
    # Define arm angles for each subject type
    arm_angles = {
        "ball_floor": {
            "shoulder_pan": 0,
            "shoulder_lift": -99,
            "elbow_flex": 90,
            "wrist_flex": -3,
            "wrist_roll": 107,
            "gripper": 60,
        },
        "ball_air": {
            "shoulder_pan": -6,
            "shoulder_lift": -2,
            "elbow_flex": -48,
            "wrist_flex": -6,
            "wrist_roll": 111,
            "gripper": 60,
        },
        "person": {
            "shoulder_pan": 0,
            "shoulder_lift": -30,
            "elbow_flex": 30,
            "wrist_flex": 0,
            "wrist_roll": 100,
            "gripper": 60,
        },
    }
    
    if subject not in arm_angles:
        print(f"Unknown subject: {subject}")
        return None
    
    try:
        # Initialize camera if needed
        initialize_camera()
        
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
        
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            return None
        
        # Always use ball detection for now
        coords = detect_ball(frame)
        
        if coords is None:
            print(f"No {subject} detected")
        else:
            print(f"Detected {subject} at {coords}")
        
        return coords
        
    except Exception as e:
        print(f"Error in search_loop({subject}): {e}")
        return None


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
    if x is None or y is None:
        return False

    robot = _get_robot()
    bus = robot.bus

    # y is negative for objects in front of the camera in this project.
    depth_cm = max(1e-6, -float(y))
    distance_cm = math.sqrt(float(x) ** 2 + depth_cm ** 2)
    error_distance = distance_cm - final_stop_distance_cm

    if error_distance <= 0:
        move_rot_and_straight(bus, True, 0, "", 0, 0)
        return False

    error_rotation = math.degrees(math.atan2(float(x), depth_cm))
    rotation_velocity_normalized = min(20.0, abs(error_rotation)) * 0.05
    direction = "left" if error_rotation < 0 else "right"

    # Move in bounded chunks so the next `search_loop("ball_floor")` can re-evaluate.
    step_distance = min(max(step_cm, 1.0), error_distance)
    step_duration_s = step_distance / max(step_speed_cm_s, 1e-6)

    # Match sensor_control_process-style control split: first correct heading, then move straight.
    straight_velocity_normalized = min(0.1, step_distance / 100.0) * 10.0

    start_t = time.time()
    while time.time() - start_t < max(step_duration_s, 0.2):
        if abs(error_rotation) > rotation_tolerance_deg:
            move_rot_and_straight(bus, False, rotation_velocity_normalized, direction, 0, error_rotation)
            time.sleep(0.1)
        elif error_distance > distance_tolerance_cm:
            move_rot_and_straight(bus, False, rotation_velocity_normalized, direction, straight_velocity_normalized, error_rotation)
            time.sleep(0.1)
        else:
            move_rot_and_straight(bus, True, 0, "", 0, 0)
            return False

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

