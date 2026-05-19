import pygame
import os
import random
import time


def _get_ser():
    from states import _get_ser as get_ser

    return get_ser()

def move_to_target_angles(robot, target_angles: dict[str, float]):
    from arm_move_angles import move_to_target_angles as mtta
    return mtta(robot, target_angles, step_delay=0.01)

def random_sounds(folder: str = "/home/dog/DOG/audio/random_dog", min_delay: int = 20, max_delay: int = 40):
    while True:
        delay = random.uniform(min_delay, max_delay)
        time.sleep(delay)
        try:
            entries = [f for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]
            files = [f for f in entries if f.lower().endswith((".mp3", ".wav", ".ogg"))]
            if not files:
                print(f"random_sounds: no audio files found in {folder}")
                return None

            sound = os.path.join(folder, random.choice(files))
            try:
                pygame.mixer.init()
            except Exception:
                pass
            try:
                pygame.mixer.music.load(sound)
                pygame.mixer.music.play()
            except Exception as e:
                print(f"random_sounds: failed to play {sound}: {e}")
                return None
            return sound
        except Exception as e:
            print(f"random_sounds: error listing/playing files: {e}")
            return None

def neutraal():
    ser = _get_ser()
    ser.reset_input_buffer()
    ser.write(b'n')
    ser.flush()
def boos():
    ser = _get_ser()
    ser.reset_input_buffer()
    ser.write(b'b')
    ser.flush()
def sad():
    ser = _get_ser()
    ser.reset_input_buffer()
    ser.write(b's')
    ser.flush()
def hart():
    ser = _get_ser()
    ser.reset_input_buffer()
    ser.write(b'h')
    ser.flush()

def grom(robot):
    boos()
    sound= "/home/dog/DOG/audio/Grrrr.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()

    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    degrees_offset = 15
    for _ in range(30):
        degrees_offset = - degrees_offset
        TARGET_ANGLES = {
        "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
        "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
        "elbow_flex": float(current_obs["elbow_flex.pos"]),
        "wrist_flex": float(current_obs["wrist_flex.pos"]),
        "wrist_roll": float(current_obs["wrist_roll.pos"]) + degrees_offset,
        "gripper": float(current_obs["gripper.pos"]),
        }
        move_to_target_angles(robot, TARGET_ANGLES)
    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"]),
    "wrist_roll": float(current_obs["wrist_roll.pos"]),
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    neutraal()

def wenen(robot):
    sound= "/home/dog/DOG/audio/Mimimi.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()

    sad()
    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    print(current_joints)
    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"])+30,
    "wrist_roll": float(current_obs["wrist_roll.pos"]),
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(1.3)

    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"]),
    "wrist_roll": float(current_obs["wrist_roll.pos"]),
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    neutraal()


def eekhoorn(robot):
    print("start eekhoorn")
    sound= "/home/dog/DOG/audio/Eekhoorn3.mp3"
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()
    print("geluidje werkt")
    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    print(current_joints)
    TARGET_ANGLES = {'shoulder_pan': -54.59340659340659, 'shoulder_lift': 71.6043956043956, 'elbow_flex': -93.49450549450549, 'wrist_flex': -14.065934065934066, 'wrist_roll': 85.31868131868131, 'gripper': 25.150501672240804}
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(.5)
    grom(robot)

    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"]),
    "wrist_roll": float(current_obs["wrist_roll.pos"]),
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)

def cute(robot):
    sound= "/home/dog/DOG/audio/sleepy_cute.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()

    hart()
    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    print(current_joints)
    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"]),
    "wrist_roll": float(current_obs["wrist_roll.pos"])+40,
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    time.sleep(1.3)

    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"]),
    "wrist_roll": float(current_obs["wrist_roll.pos"]),
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    neutraal()

def six_seven(robot):

    sound= "/home/dog/DOG/audio/six_seven.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()

    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
    degrees_offset = 30
    for _ in range(6):
        degrees_offset = - degrees_offset
        TARGET_ANGLES = {
        "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
        "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
        "elbow_flex": float(current_obs["elbow_flex.pos"]),
        "wrist_flex": float(current_obs["wrist_flex.pos"]),
        "wrist_roll": float(current_obs["wrist_roll.pos"]) + degrees_offset,
        "gripper": float(current_obs["gripper.pos"]),
        }
        move_to_target_angles(robot, TARGET_ANGLES)
        time.sleep(.15)
    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"]),
    "wrist_roll": float(current_obs["wrist_roll.pos"]),
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)

def celebration(robot):
    hart()
    sound= "/home/dog/DOG/audio/celebration.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()

    motor_names = list(robot.bus.motors.keys())
    current_obs = robot.get_observation()
    current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}

    midden = {'shoulder_pan': -7.824175824175824, 'shoulder_lift': -101.58241758241758, 'elbow_flex': 99.03296703296704, 'wrist_flex': -11.164835164835164, 'wrist_roll': 95.86813186813187, 'gripper': float(current_obs["gripper.pos"])}
    rechts = {'shoulder_pan': 60.48351648351648, 'shoulder_lift': 32.48351648351648, 'elbow_flex': -72.65934065934066, 'wrist_flex': -2.8131868131868134, 'wrist_roll': 85.23076923076923, 'gripper': float(current_obs["gripper.pos"])}
    links = {'shoulder_pan': -74, 'shoulder_lift': 32.48351648351648, 'elbow_flex': -72.65934065934066, 'wrist_flex': -2.8131868131868134, 'wrist_roll': 85.23076923076923, 'gripper': float(current_obs["gripper.pos"])}
    for _ in range(4):
        move_to_target_angles(robot, midden)
        move_to_target_angles(robot, links)
        time.sleep(.05)
        move_to_target_angles(robot, midden)
        move_to_target_angles(robot, rechts)
        time.sleep(.05)

    TARGET_ANGLES = {
    "shoulder_pan": float(current_obs["shoulder_pan.pos"]),
    "shoulder_lift": float(current_obs["shoulder_lift.pos"]),
    "elbow_flex": float(current_obs["elbow_flex.pos"]),
    "wrist_flex": float(current_obs["wrist_flex.pos"]),
    "wrist_roll": float(current_obs["wrist_roll.pos"]),
    "gripper": float(current_obs["gripper.pos"]),
    }
    move_to_target_angles(robot, TARGET_ANGLES)
    neutraal()

def thomas(robot):
    cute(robot)
    hart()
    sound= "/home/dog/DOG/audio/thomas.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()
    time.sleep(7)
    cute(robot)

def august(robot):
    hart()
    sound= "/home/dog/DOG/audio/august.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()
    time.sleep(3)
    cute(robot)

def yente(robot):
    boos()
    sound= "/home/dog/DOG/audio/yente.mp3"
    
    pygame.mixer.init()
    pygame.mixer.music.load(sound)
    pygame.mixer.music.play()
    time.sleep(4)
    grom(robot)