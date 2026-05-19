# import time

# from lerobot.motors.feetech import OperatingMode
# from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

# ROBOT_ID = "dog"
# PORT = "/dev/ttyACM0"
# WHEEL_MOTORS = [
#     "base_left_wheel",
#     "base_back_wheel",
#     "base_right_wheel",
# ]

# FORWARD_MOTORS = ["base_left_wheel", "base_right_wheel"]


# def configure_wheels(bus) -> None:
#     bus.disable_torque(WHEEL_MOTORS)
#     bus.configure_motors()
#     for name in WHEEL_MOTORS:
#         bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
#     bus.enable_torque(WHEEL_MOTORS)


# def forward(bus, velocity: int = 800, duration_s: float = 10.0) -> dict[str, int]:
#     goal_velocities = {
#         FORWARD_MOTORS[0]: -velocity,   
#         FORWARD_MOTORS[1]: velocity   
#     }
#     bus.sync_write("Goal_Velocity", goal_velocities)
#     time.sleep(duration_s)
#     bus.sync_write("Goal_Velocity", dict.fromkeys(FORWARD_MOTORS, 0), num_retry=5)
#     return goal_velocities


# def backward(bus, velocity: int = 800, duration_s: float = 10.0) -> dict[str, int]:
#     goal_velocities = {
#         FORWARD_MOTORS[0]: velocity,   
#         FORWARD_MOTORS[1]: -velocity   
#     }
#     bus.sync_write("Goal_Velocity", goal_velocities)
#     time.sleep(duration_s)
#     bus.sync_write("Goal_Velocity", dict.fromkeys(FORWARD_MOTORS, 0), num_retry=5)
#     return goal_velocities

# def rotate_platform(
#     bus,
#     angle_deg: float | None = None,
#     velocity: int = 800,
#     duration_s: float = 10.0,
#     deg_per_s_at_velocity_800: float = 30.0,
# ) -> dict[str, int]:
#     """
#     Rotate the base in place.

#     If `angle_deg` is provided, the function computes an open-loop duration based on an approximate
#     calibration constant (deg/s at velocity=800) and scales it with `velocity`.

#     Notes:
#     - This is time-based (open loop). Expect drift; tune `deg_per_s_at_velocity_800` for your floor/battery.
#     - Positive `angle_deg` uses the same direction as positive `velocity` in Goal_Velocity.
#     """
#     if angle_deg is not None:
#         # Simple scaling: assume angular speed is ~ proportional to wheel velocity command.
#         deg_per_s = abs(velocity) * (deg_per_s_at_velocity_800 / 800.0)
#         duration_s = abs(float(angle_deg)) / max(1e-6, deg_per_s)
#         # Flip sign based on desired angle.
#         if angle_deg < 0:
#             velocity = -abs(velocity)
#         else:
#             velocity = abs(velocity)

#     goal_velocities = {
#         WHEEL_MOTORS[0]: velocity,
#         WHEEL_MOTORS[1]: velocity,
#         WHEEL_MOTORS[2]: velocity,
#     }
#     bus.sync_write("Goal_Velocity", goal_velocities)
#     time.sleep(duration_s)
#     bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
#     return goal_velocities


# bus = LeKiwi(LeKiwiConfig(id=ROBOT_ID, port=PORT))
# try:
#     bus.connect()
#     configure_wheels(bus)
#     rotate_platform(bus, 90, 800, 10.0)
# finally:
#     if bus.is_connected:
#         bus.disconnect()