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
# Faulty naming: back_wheel is the left wheel and vice versa. also right (platform or wheel) turning is always positive
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
    
def move_straight_to_object(bus, velocity_normalized: float, angle: float) -> dict[str, int]:
    if velocity_normalized < 0.05:
        # print("Already at the object")
        return {motor: 0 for motor in WHEEL_MOTORS}
    max_velocity = 800
    velocity = max_velocity * velocity_normalized
    if(-30< angle<= 30):
        return move_in_direction(bus, "base_back_wheel","forward",velocity)
    if(30 < angle <= 90):
        return move_in_direction(bus, "base_right_wheel","forward",velocity)
    if(90 < angle <= 150):
        return move_in_direction(bus, "base_left_wheel","backward",velocity)
    if(150 < angle <=  180 or -180 < angle < -150):
        return move_in_direction(bus, "base_back_wheel","backward",velocity)
    if(-150 < angle <=  -90):
        return move_in_direction(bus, "base_right_wheel","backward",velocity)
    if(-90 < angle <=  -30):
        return move_in_direction(bus, "base_left_wheel","forward",velocity)
def move_in_direction(bus, stationairy_wheel: str,direction: str, velocity: float = 800)-> dict[str, int]:
    #3 omnidirectional wheels, these are harcoded because there was not an elegant way to describe this
    goal_velocities = {}
    factor= -1
    if stationairy_wheel=="base_back_wheel":
        factor=1
    if direction == "backward":
        velocity = -velocity
    if stationairy_wheel == "base_back_wheel":
        goal_velocities = {
            "base_back_wheel": 0,
            "base_right_wheel": velocity,
            "base_left_wheel": -velocity
        }
    if stationairy_wheel == "base_right_wheel":
        goal_velocities = {
            "base_back_wheel": velocity,
            "base_right_wheel": 0,
            "base_left_wheel": -velocity
        }
    if stationairy_wheel == "base_left_wheel":
        goal_velocities = {
            "base_back_wheel": -velocity,
            "base_right_wheel": velocity,
            "base_left_wheel": 0
        }
    goal_velocities = aliassing_wheels(goal_velocities)
    return goal_velocities

def aliassing_wheels(goal_velocities: dict[str, int]) -> dict[str, int]:
    # This function is used to alias the wheel names to the actual motor names, since the control commands use the wheel names and the bus uses the motor names
    aliased_goal_velocities = {}
    for wheel, velocity in goal_velocities.items():
        if wheel == "base_left_wheel":
            aliased_goal_velocities["base_right_wheel"] = velocity
        elif wheel == "base_right_wheel":
            aliased_goal_velocities["base_back_wheel"] = velocity
        elif wheel == "base_back_wheel":
            aliased_goal_velocities["base_left_wheel"] = velocity
    return aliased_goal_velocities



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
    
    if(stop == True):
        print("Stopping rotation")
        bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
        return dict.fromkeys(WHEEL_MOTORS, 0)
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
    return goal_velocities

def move_rot_and_straight(    bus,
    stop: bool,
    velocity_normalized: int,
    direction: str ,
    straight_velocity_normalized: float,
    angle: float
) -> None:
    if(stop == True):
        print("Stopping rotation and straight movement")
        bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
        return dict.fromkeys(WHEEL_MOTORS, 0)
    straight_velocities={}
    rotation_velocities={}
    straight_velocities = move_straight_to_object(bus, straight_velocity_normalized, angle)
    rotation_velocities = rotate_platform(bus, False, velocity_normalized, direction)
    goal_velocities = {motor: straight_velocities.get(motor, 0) + rotation_velocities.get(motor, 0) for motor in WHEEL_MOTORS}
    #print(f"Moving with combined straight and rotational velocities: {goal_velocities}")    bus.sync_write("Goal_Velocity", goal_velocities)

    bus.sync_write("Goal_Velocity", goal_velocities)

def vierkant_maken(    bus,
    stop: bool,
    velocity_normalized: int,
    direction: str ,
    distance: float,
    angle: float
) -> None:
    if(stop == True):
        print("Stopping rotation a")
        bus.sync_write("Goal_Velocity", dict.fromkeys(WHEEL_MOTORS, 0), num_retry=5)
        return dict.fromkeys(WHEEL_MOTORS, 0)
    straight_velocities={}
    rotation_velocities={}
    straight_velocities = move_straight_to_object(bus, distance, 0)
    bus.sync_write("Goal_Velocity", straight_velocities)
    time.sleep(2)
    rotation_velocities = rotate_platform(bus, False, velocity_normalized, direction)
    bus.sync_write("Goal_Velocity", rotation_velocities)

def init_robot() -> None:
    print("Initializing robot...")
    robot = LeKiwi(LeKiwiConfig(id=ROBOT_ID, port=PORT, cameras={}))
    motor_bus = robot.bus
    try:
        motor_bus.connect()
        configure_wheels(motor_bus)

    except Exception as e:
        print(f"Error occurred: {e}")
        if motor_bus.is_connected:
            motor_bus.disconnect()
    print("Robot initialized")
    return robot

# if __name__ == "__main__":
#     init_robot()