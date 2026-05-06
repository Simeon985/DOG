import time
import threading
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

ROBOT_ID = "dog"
PORT = "/dev/ttyACM0"
WHEEL_MOTORS = [
    "base_left_wheel",
    "base_back_wheel",
    "base_right_wheel",
]

FORWARD_MOTORS = ["base_back_wheel", "base_right_wheel"]


def configure_wheels(bus) -> None:
    bus.disable_torque(WHEEL_MOTORS)
    bus.configure_motors()
    for name in WHEEL_MOTORS:
        bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
    bus.enable_torque(WHEEL_MOTORS)


def move_forward(bus, velocity: int = 800, duration_s: float = 10.0) -> dict[str, int]:
    goal_velocities = {
        FORWARD_MOTORS[0]: velocity,
        FORWARD_MOTORS[1]: -velocity
    }
    bus.sync_write("Goal_Velocity", goal_velocities)
    time.sleep(duration_s)
    bus.sync_write("Goal_Velocity", dict.fromkeys(FORWARD_MOTORS, 0), num_retry=5)
    return goal_velocities


def move_backward(bus, velocity: int = 800, duration_s: float = 10.0) -> dict[str, int]:
    goal_velocities = {
        FORWARD_MOTORS[0]: -velocity,
        FORWARD_MOTORS[1]: velocity
    }
    bus.sync_write("Goal_Velocity", goal_velocities)
    time.sleep(duration_s)
    bus.sync_write("Goal_Velocity", dict.fromkeys(FORWARD_MOTORS, 0), num_retry=5)
    return goal_velocities

def rotate_platform(
    bus,
    stop_event: threading.Event,
    velocity: int = -100,
) -> dict[str, int]:
    """
    Rotate the base in place.

    If `angle_deg` is provided, the function computes an open-loop duration based on an approximate
    calibration constant (deg/s at velocity=800) and scales it with `velocity`.

    Notes:
    - This is time-based (open loop). Expect drift; tune `deg_per_s_at_velocity_800` for your floor/battery.
    - Positive `angle_deg` uses the same direction as positive `velocity` in Goal_Velocity.
    """


    goal_velocities = {
        WHEEL_MOTORS[0]: velocity,
        WHEEL_MOTORS[1]: velocity,
        WHEEL_MOTORS[2]: velocity,
    }
    bus.sync_write("Goal_Velocity", goal_velocities)
    stop_event.wait()
    bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
    return goal_velocities


def init_robot(stop_event,direction) -> None:
    robot = LeKiwi(LeKiwiConfig(id=ROBOT_ID, port=PORT, cameras={}))
    motor_bus = robot.bus
    velocity = -400
    if direction == "left":
        velocity = -velocity
    try:
        motor_bus.connect()
        configure_wheels(motor_bus)
        #rotate_platform(motor_bus, stop_event, velocity)
        move_forward(motor_bus)
    finally:
        if motor_bus.is_connected:
            motor_bus.disconnect()


# if __name__ == "__main__":
#     init_robot()