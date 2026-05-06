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
    stop: bool,
    velocity_normalized: int = 1,
    direction: str = "left"
) -> dict[str, int]:
    """
    Rotate the base in place.
    Give a normalized velocity between 0 and 1 where 0 is no movement and 1 is maximum speed, and a direction ("left" or "right"), and a stoping event
    """
    #catch if stop is set
    if(stop == True):
        print("Stopping rotation")
        bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
        return dict.fromkeys(WHEEL_MOTORS, 0)
    print("Setting rotation: ", direction, velocity_normalized)
    #Determin actual motor velocity from normalized velocity and direction
    max_velocity = 400
    velocity = max_velocity * velocity_normalized
    if direction == "left":
        velocity = -velocity



    goal_velocities = {
        WHEEL_MOTORS[0]: velocity,
        WHEEL_MOTORS[1]: velocity,
        WHEEL_MOTORS[2]: velocity,
    }

    bus.sync_write("Goal_Velocity", goal_velocities)
    print(f"Rotating {direction} with velocity {velocity} ({velocity_normalized} normalized)")
    # stop_event.wait()
    # bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
    return goal_velocities


def init_robot() -> None:
    print("Initializing robot...")
    robot = LeKiwi(LeKiwiConfig(id=ROBOT_ID, port=PORT, cameras={}))
    motor_bus = robot.bus
    try:
        motor_bus.connect()
        configure_wheels(motor_bus)
        #rotate_platform(motor_bus, stop_event, velocity)
        #move_forward(motor_bus)

    except Exception as e:
        print(f"Error occurred: {e}")
        if motor_bus.is_connected:
            motor_bus.disconnect()
    print("Robot initialized")
    return robot

# if __name__ == "__main__":
#     init_robot()