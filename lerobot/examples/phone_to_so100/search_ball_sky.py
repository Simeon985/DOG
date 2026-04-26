#!/usr/bin/env python3

import time

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

from coordinates_from_picture import get_coordinates_from_picture
from default.move_platform import (
    LeKiwi,
    LeKiwiConfig,
    PORT as PLATFORM_PORT,
    ROBOT_ID as PLATFORM_ROBOT_ID,
    configure_wheels,
    rotate_platform,
)

from arm_move_angles import move_to_target_angles


# A conservative "look up" pose. Tune if your camera framing is off.
# (All angles are degrees; gripper is normalized 0..100.)
SKY_VIEW_ANGLES: dict[str, float] = {
    "shoulder_pan": 0,
    "shoulder_lift": 55,
    "elbow_flex": -35,
    "wrist_flex": -10,
    "wrist_roll": 90,
    "gripper": 90,
}


def search_ball_sky(
    *,
    arm_port: str = "/dev/ttyACM0",
    arm_id: str = "dog",
    platform_port: str = PLATFORM_PORT,
    platform_id: str = PLATFORM_ROBOT_ID,
    step_deg: float = 15.0,
    max_rotation_deg: float = 360.0,
    settle_s: float = 0.4,
) -> tuple[float, float, float] | None:
    """
    Search for the ball by looking up and rotating the base.

    Returns:
        (x_cm, y_cm, z_cm) in the *camera* coordinate frame if detected, else None.
    """
    if step_deg <= 0:
        raise ValueError("step_deg must be > 0")
    if max_rotation_deg <= 0:
        raise ValueError("max_rotation_deg must be > 0")

    arm = SO100Follower(SO100FollowerConfig(port=arm_port, id=arm_id, use_degrees=True))
    platform = LeKiwi(LeKiwiConfig(id=platform_id, port=platform_port))

    arm.connect()
    if not arm.is_connected:
        raise RuntimeError("Arm is not connected.")

    bus = platform.bus
    bus.connect()
    configure_wheels(bus)

    rotated = 0.0
    try:
        # Put the arm in a stable viewing pose once, then only rotate the base.
        move_to_target_angles(arm, SKY_VIEW_ANGLES)
        time.sleep(settle_s)

        while rotated < max_rotation_deg - 1e-6:
            try:
                # This captures an image and runs detection. If no detection, it raises.
                return get_coordinates_from_picture()
            except Exception:
                # Not found this heading; rotate and try again.
                rotate_platform(bus, angle_deg=step_deg)
                rotated += abs(step_deg)
                time.sleep(settle_s)

        return None
    finally:
        try:
            if bus.is_connected:
                bus.disconnect()
        finally:
            # SO100Follower doesn't expose disconnect consistently across versions; best effort.
            pass


def main() -> None:
    coords = search_ball_sky()
    if coords is None:
        print("Ball not found (full sweep).")
    else:
        x, y, z = coords
        print(f"Ball detected (camera frame): x={x:.1f} cm, y={y:.1f} cm, z={z:.1f} cm")


if __name__ == "__main__":
    main()


