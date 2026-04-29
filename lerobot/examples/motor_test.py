import time

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

ROBOT_ID = "dog"
PORT = "/dev/ttyACM0"
WHEEL_MOTORS = [
    "base_left_wheel",
    "base_back_wheel",
    "base_right_wheel",
]

FORWARD_MOTORS = ["base_left_wheel", "base_right_wheel"]


def configure_wheels(bus) -> None:
    bus.disable_torque(WHEEL_MOTORS)
    bus.configure_motors()
    for name in WHEEL_MOTORS:
        bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
    bus.enable_torque(WHEEL_MOTORS)


def forward(bus, velocity: int = 800, duration_s: float = 10.0) -> dict[str, int]:
    goal_velocities = {
        FORWARD_MOTORS[0]: -velocity,   
        FORWARD_MOTORS[1]: velocity   
    }
    bus.sync_write("Goal_Velocity", goal_velocities)
    time.sleep(duration_s)
    bus.sync_write("Goal_Velocity", dict.fromkeys(FORWARD_MOTORS, 0), num_retry=5)
    return goal_velocities


def backward(bus, velocity: int = 800, duration_s: float = 10.0) -> dict[str, int]:
    goal_velocities = {
        FORWARD_MOTORS[0]: velocity,   
        FORWARD_MOTORS[1]: -velocity   
    }
    bus.sync_write("Goal_Velocity", goal_velocities)
    time.sleep(duration_s)
    bus.sync_write("Goal_Velocity", dict.fromkeys(FORWARD_MOTORS, 0), num_retry=5)
    return goal_velocities

def rotate(bus, velocity: int = 800, duration_s: float = 10.0) -> dict[str, int]:
    goal_velocities = {
        WHEEL_MOTORS[0]: velocity,
        WHEEL_MOTORS[1]: velocity,
        WHEEL_MOTORS[2]: velocity,
    }
    bus.sync_write("Goal_Velocity", goal_velocities)
    time.sleep(duration_s)
    bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
    return goal_velocities

robot = LeKiwi(LeKiwiConfig(id=ROBOT_ID, port=PORT))
bus = robot.bus

try:
    bus.connect()
    configure_wheels(bus)
    # forward(bus, velocity=8000, duration_s=3.0)
    # time.sleep(1.0)
    rotate(bus, velocity=3500, duration_s=6.0)
    # time.sleep(1.0)
    # backward(bus, velocity=2500, duration_s=4.0)

finally:
    if bus.is_connected:
        bus.disconnect()