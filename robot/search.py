#!/usr/bin/env python3

import os
import math
import time

from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

import cv2
import numpy as np

from coordinates_from_picture import get_coordinates_from_frame
from arm_camera_stream import ArmCameraFrameClient
from move_platform import (
    LeKiwi,
    LeKiwiConfig,
    PORT as PLATFORM_PORT,
    ROBOT_ID as PLATFORM_ROBOT_ID,
    configure_wheels,
    move_forward,
    WHEEL_MOTORS,
)

from arm_move_angles import move_to_target_angles, reset_arm
from move_platform import move_forward

# Target arm poses. Tune if your camera framing is off.
# (All angles are degrees; gripper is normalized 0..100.)
SKY_VIEW_ANGLES: dict[str, float] = {
    "shoulder_pan": -6,
    "shoulder_lift": -2,
    "elbow_flex": -48,
    "wrist_flex": -6,
    "wrist_roll": 111,
    "gripper": 60,
}

# A more forward/down-ish view for searching on the ground (example defaults).
GROUND_VIEW_ANGLES: dict[str, float] = {
    "shoulder_pan": 0,
    "shoulder_lift": -99,
    "elbow_flex": 90,
    "wrist_flex": -3,
    "wrist_roll": 107,
    "gripper": 60,
}


def _csi_pipeline() -> str:
    # Keep in sync with arm_camera_stream.py.
    sensor_id = int(os.environ.get("ARM_CAMERA_SENSOR_ID", "0"))
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} wbmode=0 awblock=true ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def search_ball(
    *,
    bus,
    sky: bool = True,
    arm_port: str = "/dev/ttyACM0",
    arm_id: str = "dog",
    max_rotation_deg: float = 360.0,
    scan_velocity: int = 600,
    deg_per_s_at_velocity_800: float = 30.0,
    detection_hz: float = 8.0,
    max_scan_extra_s: float = 1.0,
    rotation_time_correction: float = 0.92,
    center_on_detect: bool = True,
    center_gain: float = 0.9,
    center_max_deg: float = 25.0,
    center_verify: bool = True,
    # Closed-loop centering (recommended): keep rotating until x ~= 0
    center_pid: bool = True,
    center_tol_cm: float = 1.5,
    center_stable_frames: int = 3,
    center_timeout_s: float = 4.0,
    pid_kp: float = 35.0,
    pid_ki: float = 0.0,
    pid_kd: float = 8.0,
    pid_i_max: float = 30.0,
    pid_min_velocity: int = 80,
    pid_lost_timeout_s: float = 1.2,
    pid_lost_sweep_vel: int = 140,
) -> tuple[float, float, float] | None:
    """
    Search for the ball by setting an arm pose and rotating the base.

    Returns:
        (x_cm, y_cm, z_cm) in the *camera* coordinate frame if detected, else None.
    """
    if max_rotation_deg <= 0:
        raise ValueError("max_rotation_deg must be > 0")
    if detection_hz <= 0:
        raise ValueError("detection_hz must be > 0")
    if rotation_time_correction <= 0:
        raise ValueError("rotation_time_correction must be > 0")
    if center_gain <= 0:
        raise ValueError("center_gain must be > 0")
    if center_max_deg <= 0:
        raise ValueError("center_max_deg must be > 0")
    if center_tol_cm <= 0:
        raise ValueError("center_tol_cm must be > 0")
    if center_stable_frames <= 0:
        raise ValueError("center_stable_frames must be > 0")
    if center_timeout_s <= 0:
        raise ValueError("center_timeout_s must be > 0")
    if pid_kp <= 0:
        raise ValueError("pid_kp must be > 0")
    if pid_ki < 0:
        raise ValueError("pid_ki must be >= 0")
    if pid_kd < 0:
        raise ValueError("pid_kd must be >= 0")
    if pid_i_max < 0:
        raise ValueError("pid_i_max must be >= 0")
    if pid_min_velocity < 0:
        raise ValueError("pid_min_velocity must be >= 0")
    if pid_lost_timeout_s <= 0:
        raise ValueError("pid_lost_timeout_s must be > 0")
    if pid_lost_sweep_vel < 0:
        raise ValueError("pid_lost_sweep_vel must be >= 0")

    arm = SO100Follower(SO100FollowerConfig(port=arm_port, id=arm_id, use_degrees=True))

    arm.connect()

    if not arm.is_connected:
        raise RuntimeError("Arm is not connected.")

    # Open camera ONCE for realtime reads.
    # In the lerobot conda env, OpenCV often can't open nvarguscamerasrc; use system-python persistent feed.
    cap = cv2.VideoCapture(_csi_pipeline(), cv2.CAP_GSTREAMER)
    cam_client: ArmCameraFrameClient | None = None
    if not cap.isOpened():
        cap.release()
        cap = None
        cam_client = ArmCameraFrameClient()
        cam_client.start()
        # Fail fast if camera feed can't produce frames.
        cam_client.get_jpeg(timeout_s=5.0)

    def _set_base_velocity(raw_velocity: int) -> None:
        bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, int(raw_velocity)), num_retry=5)

    # Convert "one full sweep" into a time budget (open-loop, but smooth).
    deg_per_s = abs(scan_velocity) * (deg_per_s_at_velocity_800 / 800.0)
    max_scan_s = ((max_rotation_deg / max(1e-6, deg_per_s)) + max_scan_extra_s) * rotation_time_correction
    detect_period_s = 1.0 / detection_hz
    started_rotation = False
    scan_vel_sign = 1 if scan_velocity >= 0 else -1

    def _rotate_for(deg: float, *, direction: int) -> None:
        """Rotate base open-loop for a certain number of degrees."""
        deg = abs(float(deg))
        if deg <= 0:
            return
        duration_s = deg / max(1e-6, deg_per_s)
        vel = int(abs(scan_velocity) * int(direction))
        _set_base_velocity(vel)
        time.sleep(duration_s)
        _set_base_velocity(0)

    def _read_frame():
        frame = None
        if cap is not None:
            ret, frame = cap.read()
            if not ret:
                frame = None
        else:
            assert cam_client is not None
            try:
                jpg = cam_client.get_jpeg(timeout_s=2.0)
                arr = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            except Exception:
                frame = None
        return frame

    def _frame_to_coords(frame):
        try:
            return get_coordinates_from_frame(frame)
        except Exception:
            return None

    try:
        # Put the arm in a stable viewing pose once, then only rotate the base.
        target_angles = SKY_VIEW_ANGLES if sky else GROUND_VIEW_ANGLES
        move_to_target_angles(arm, target_angles)
        # Warm up camera a bit (auto exposure / AWB) before scanning.
        if cap is not None:
            for _ in range(10):
                cap.read()

        # Start smooth continuous rotation (much less shock than step rotation).
        _set_base_velocity(scan_velocity)
        started_rotation = True

        t0 = time.time()
        next_detect_t = t0
        while True:
            now = time.time()
            if now - t0 >= max_scan_s:
                return None

            # Read the latest frame as fast as possible, but only run YOLO at detection_hz.
            frame = _read_frame()

            if frame is None:
                continue

            if now < next_detect_t:
                continue
            next_detect_t = now + detect_period_s

            try:
                found = get_coordinates_from_frame(frame)
            except Exception:
                found = None

            if found is not None:
                # Stop immediately on detection.
                _set_base_velocity(0)
                started_rotation = False

                if not center_on_detect:
                    return found

                # 1) PID centering loop (preferred)
                if center_pid:
                    stable = 0
                    e_prev = None
                    t_prev = None
                    i_term = 0.0
                    t_end = time.time() + center_timeout_s
                    last_good_coords = found
                    last_seen_t = time.time()

                    # Direction convention.
                    # Scanning at velocity = scan_velocity made the ball traverse the camera
                    # frame (and almost certainly past the center, due to wheel momentum).
                    # That means rotating in scan direction reduces x when x > 0.
                    # So `vel = sigma * u` with sigma = scan_vel_sign drives x toward 0.
                    sigma = scan_vel_sign

                    # Sigma sanity check: if our commands keep increasing |e| for a while,
                    # the wheel convention is opposite our assumption -> flip sigma once.
                    flips_used = 0
                    e_at_check = None
                    last_check_t = time.time()

                    # Kick the base in the *reverse* of the scan direction to immediately
                    # start undoing the overshoot. This is what "rotate back" means.
                    kick_speed = max(pid_min_velocity, pid_lost_sweep_vel)
                    _set_base_velocity(-scan_vel_sign * kick_speed)

                    while time.time() < t_end:
                        frame_c = _read_frame()
                        coords = _frame_to_coords(frame_c) if frame_c is not None else None
                        if coords is None:
                            # Ball lost. The most likely reason is that we overshot during
                            # the scan -> keep rotating reverse-of-scan to bring it back.
                            sweep_vel = max(
                                pid_min_velocity, min(pid_lost_sweep_vel, abs(scan_velocity))
                            )
                            elapsed_lost = time.time() - last_seen_t
                            if elapsed_lost > pid_lost_timeout_s * 2.0:
                                # Reverse didn't reacquire it; try scan direction.
                                _set_base_velocity(scan_vel_sign * sweep_vel)
                            else:
                                _set_base_velocity(-scan_vel_sign * sweep_vel)
                            continue

                        last_good_coords = coords
                        last_seen_t = time.time()
                        x, y, z = coords

                        if abs(x) <= center_tol_cm:
                            stable += 1
                            _set_base_velocity(0)
                            if stable >= center_stable_frames:
                                return coords
                            continue
                        stable = 0

                        t = time.time()
                        dt = 0.0 if t_prev is None else max(1e-3, t - t_prev)
                        e = float(x)
                        de = 0.0 if e_prev is None else (e - e_prev) / dt

                        if pid_ki > 0:
                            i_term += e * dt
                            if pid_i_max > 0:
                                i_term = max(-pid_i_max, min(pid_i_max, i_term))

                        u = (pid_kp * e) + (pid_ki * i_term) + (pid_kd * de)

                        # vel = sigma * u: drives x toward 0 under our convention.
                        vel = int(sigma * u)
                        vel = max(-abs(scan_velocity), min(abs(scan_velocity), vel))
                        if abs(vel) < pid_min_velocity:
                            vel = int(pid_min_velocity * (1 if vel >= 0 else -1))

                        _set_base_velocity(vel)

                        # Periodically check that |e| is shrinking. If not, our sigma is
                        # backwards -- flip it once and reset the integrator.
                        if e_at_check is None:
                            e_at_check = e
                            last_check_t = t
                        elif (t - last_check_t) > 0.4:
                            if flips_used == 0 and abs(e) > abs(e_at_check) + 0.5:
                                sigma *= -1
                                i_term = 0.0
                                flips_used = 1
                            e_at_check = e
                            last_check_t = t

                        e_prev = e
                        t_prev = t

                    _set_base_velocity(0)
                    return last_good_coords

                # 2) One-shot correction (fallback)
                x, y, z = found
                depth = max(1e-6, -float(z))  # z is negative in this codebase
                yaw_err_deg = math.degrees(math.atan2(float(x), depth))
                corr_deg = min(center_max_deg, abs(yaw_err_deg) * center_gain)

                if corr_deg <= 0.5:
                    return found

                guess_dir = -1 if x > 0 else 1
                direction = guess_dir * scan_vel_sign
                _rotate_for(corr_deg, direction=direction)

                if not center_verify:
                    return found

                frame2 = _read_frame()
                if frame2 is None:
                    return found
                found2 = _frame_to_coords(frame2)
                if found2 is None:
                    return found
                x2, _, _ = found2
                if abs(x2) > abs(x) + 0.5:
                    _rotate_for(corr_deg, direction=-direction)
                    frame3 = _read_frame()
                    found3 = _frame_to_coords(frame3) if frame3 is not None else None
                    return found3 or found

                return found2

    finally:
        # Always stop the base before disconnect to avoid sudden jerks on teardown.
        try:
            _set_base_velocity(0)
        except Exception:
            pass
        if cap is not None:
            cap.release()
        if cam_client is not None:
            cam_client.close()
        # SO100Follower doesn't expose disconnect consistently across versions; best effort.
        pass


def main() -> None:
    platform = LeKiwi(LeKiwiConfig(id=PLATFORM_ROBOT_ID, port=PLATFORM_PORT, cameras={}))
    bus = platform.bus
    try:
        bus.connect()
        configure_wheels(bus)

        coords = search_ball(bus=bus, sky=False)
        if coords is None:
            print("Ball not found (full sweep).")
        else:
            x, y, z = coords
            print(f"Ball detected (camera frame): x={x:.1f} cm, y={y:.1f} cm, z={z:.1f} cm")
            move_forward(bus, 800, 8.0)
    finally:
        try:
            if bus.is_connected:
                bus.disconnect()
        except Exception:
            pass
    # Reset the arm AFTER we returned/printed the result.
    # We reconnect here to keep search_ball() fast and to ensure the result is available immediately.
    try:
        arm = SO100Follower(SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True))
        arm.connect()
        if arm.is_connected:
            reset_arm(arm)
    except Exception:
        pass


if __name__ == "__main__":
    main()


