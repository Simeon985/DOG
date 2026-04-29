import time
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig


@dataclass
class VisualServoConfig:
    # Image frame size (pixels)
    width: int
    height: int
    # Proportional gains mapping pixel error to joint deltas (in the robot's native units)
    # Positive ex means ball is to the right of center; positive ey means below center
    k_pan: float = 0.2
    k_flex: float = 0.2
    # Optional approach gain to move forward (lift/elbow) when centered
    k_approach: float = 0.0
    # Deadband (pixels) around image center where no motion is commanded
    deadband_px: int = 8
    # Max absolute delta applied per step (native units)
    max_joint_delta: float = 2.0
    # Rate limiting and loop timing
    step_hz: float = 10.0
    # Safety: clamp joint names to command and optional workspace checks later
    pan_joint: str = "arm_shoulder_pan"
    flex_joint: str = "arm_wrist_flex"
    lift_joint: str = "arm_shoulder_lift"
    elbow_joint: str = "arm_elbow_flex"


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def compute_joint_deltas_from_centroid(
    centroid_uv: Tuple[int, int],
    cfg: VisualServoConfig,
) -> dict:
    """
    Convert a centroid position (u, v) into small joint deltas to re-center the target.
    This uses a simple image-based visual servo heuristic without requiring camera intrinsics.
    """
    u, v = centroid_uv
    cx = cfg.width * 0.5
    cy = cfg.height * 0.5
    ex = u - cx
    ey = v - cy

    # Deadband
    if abs(ex) < cfg.deadband_px:
        ex = 0.0
    if abs(ey) < cfg.deadband_px:
        ey = 0.0

    # Map pixel errors to joint deltas (signs chosen so the camera recenters the target)
    d_pan = _clamp(-cfg.k_pan * (ex / max(1.0, cfg.width)), -cfg.max_joint_delta, cfg.max_joint_delta)
    d_flex = _clamp(cfg.k_flex * (ey / max(1.0, cfg.height)), -cfg.max_joint_delta, cfg.max_joint_delta)

    deltas = {
        cfg.pan_joint: d_pan,
        cfg.flex_joint: d_flex,
    }

    # Optional approach when very close to center (move “forward” using lift/elbow)
    if cfg.k_approach > 0 and ex == 0.0 and ey == 0.0:
        d_lift = _clamp(-cfg.k_approach, -cfg.max_joint_delta, cfg.max_joint_delta)
        deltas[cfg.lift_joint] = d_lift

    return deltas


def apply_joint_deltas(bus, joint_deltas: dict) -> None:
    """
    Reads current joint positions and applies small deltas in the SAME units as read.
    This is unit-safe even if the robot is configured in degrees or normalized ranges.
    """
    if not joint_deltas:
        return
    joint_names = list(joint_deltas.keys())
    present = bus.sync_read("Present_Position", joint_names)
    goals = {name: present[name] + joint_deltas[name] for name in joint_names}
    bus.sync_write("Goal_Position", goals)


def visual_servo_step(
    bus,
    centroid_uv: Optional[Tuple[int, int]],
    cfg: VisualServoConfig,
) -> None:
    """
    One servo step: given the centroid (u, v), compute joint deltas and command the arm.
    If centroid is None, no motion is commanded.
    """
    if centroid_uv is None:
        return
    deltas = compute_joint_deltas_from_centroid(centroid_uv, cfg)
    apply_joint_deltas(bus, deltas)


def run_visual_servo_loop(
    get_centroid: Callable[[], Optional[Tuple[int, int]]],
    cfg: VisualServoConfig,
    robot_id: str = "dog",
    port: str = "/dev/ttyACM0",
    connect_calibrate: bool = False,
    run_seconds: float = 10.0,
) -> None:
    """
    Example loop runner:
      - Initializes LeKiwi
      - Calls get_centroid() repeatedly (you provide this; no detector here)
      - Applies small joint deltas to re-center the target
    """
    robot = LeKiwi(LeKiwiConfig(id=robot_id, port=port))
    bus = robot.bus
    try:
        robot.connect(calibrate=connect_calibrate)
        period = 1.0 / max(1e-3, cfg.step_hz)
        t_end = time.time() + run_seconds if run_seconds > 0 else float("inf")
        while time.time() < t_end:
            centroid = get_centroid()
            visual_servo_step(bus, centroid, cfg)
            time.sleep(period)
    finally:
        if bus.is_connected:
            robot.disconnect()



if __name__ == "__main__":
    cfg = VisualServoConfig(width=640, height=480, step_hz=10.0)
    def get_centroid():
        return (320, 240)
    run_visual_servo_loop(get_centroid, cfg)