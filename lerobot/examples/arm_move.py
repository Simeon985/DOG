import time

from lerobot.motors.feetech import OperatingMode
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

ROBOT_ID = "dog"
PORT = "/dev/ttyACM0"

ARM_MOTORS = [
    "arm_shoulder_pan",
    "arm_shoulder_lift",
    "arm_elbow_flex",
    "arm_wrist_flex",
    "arm_wrist_roll",
    "arm_gripper",
]

def configure_arm(bus) -> None:
    bus.disable_torque(ARM_MOTORS)
    # Assuming global configure sets up registers safely
    bus.configure_motors()
    for name in ARM_MOTORS:
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
        # Optional: match LeKiwi defaults to reduce shakiness
        bus.write("P_Coefficient", name, 16)
        bus.write("I_Coefficient", name, 0)
        bus.write("D_Coefficient", name, 32)
    bus.enable_torque(ARM_MOTORS)

def set_gripper(bus, percent: float) -> None:
    # Gripper uses RANGE_0_100 normalization in LeKiwi
    target = max(0.0, min(100.0, float(percent)))
    bus.sync_write("Goal_Position", {"arm_gripper": target})

def set_wrist_roll(bus, percent: float) -> None:
    percent = max(0.0, min(100.0, float(percent)))
    bus.sync_write("Goal_Position", {"arm_wrist_roll": percent})

def set_wrist_flex(bus, percent: float) -> None:
    percent = max(0.0, min(100.0, float(percent)))
    bus.sync_write("Goal_Position", {"arm_wrist_flex": percent})

def set_elbow_flex(bus, percent: float) -> None:
    percent = max(0.0, min(100.0, float(percent)))
    bus.sync_write("Goal_Position", {"arm_elbow_flex": percent})

def set_shoulder_lift(bus, percent: float) -> None:
    percent = max(0.0, min(100.0, float(percent)))
    bus.sync_write("Goal_Position", {"arm_shoulder_lift": percent})

def set_shoulder_pan(bus, percent: float) -> None:
    percent = max(0.0, min(100.0, float(percent)))
    bus.sync_write("Goal_Position", {"arm_shoulder_pan": percent})

def set_gripper(bus, percent: float) -> None:
    percent = max(0.0, min(100.0, float(percent)))
    bus.sync_write("Goal_Position", {"arm_gripper": percent})


def open_gripper(bus, percent: float = 100.0, hold_s: float | None = None) -> None:
    set_gripper(bus, percent)
    if hold_s is not None:
        time.sleep(hold_s)


robot = LeKiwi(LeKiwiConfig(id=ROBOT_ID, port=PORT))
bus = robot.bus

try:
    bus.connect()
    configure_arm(bus)

    set_wrist_roll(bus, 0.0)
    time.sleep(0.4)
    set_wrist_roll(bus, 30.0)
    time.sleep(0.4)

    open_gripper(bus, 40.0)
    time.sleep(1.0)
    open_gripper(bus, 20.0)
    time.sleep(1.0)


    set_wrist_flex(bus, 0.0)
    time.sleep(0.4)
    set_wrist_flex(bus, 30.0)
    time.sleep(0.4)
    set_wrist_flex(bus, 0.0)
    time.sleep(0.4)
    set_wrist_flex(bus, 30.0)
    time.sleep(0.4)
    set_wrist_flex(bus, 0.0)
    time.sleep(0.4)
    set_wrist_flex(bus, 30.0)
    time.sleep(0.4)
    set_wrist_flex(bus, 0.0)

    time.sleep(1.0)
    open_gripper(bus, 40.0)
    time.sleep(1.0)



finally:
    if bus.is_connected:
        bus.disconnect()