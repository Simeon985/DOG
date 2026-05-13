import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import itertools
from multiprocessing import Array
from multiprocessing.sharedctypes import SynchronizedArray
from processes.threads.mapping import *
from processes.threads.control import *
from processes.threads.control_help_commands import *

# global ani
robot = None
est= None
def sensor_control_process(estimator: str, shared_array: SynchronizedArray, desired_angle: int=0, desired_distance: float=0.06, seperate_movements: bool=True) -> None:
    """Process running the control and sensor threads."""
    ser = initialize_esp()
    #old calibration
    #scale_1, scale_2, angle_1, angle_2 = 5.963691140961605e-05, 5.8920022248807314e-05, -46.31328914736247, 117.43771136252667
    #newer calibration
    # scale_1, scale_2, angle_1, angle_2 = 6.096668242094706e-05, 6.370891224635226e-05, -43.696952942756894, 117.94623106591115
    #calibration on floor airo lab
    scale_1, scale_2, angle_1, angle_2 = 5.803293347508479e-05, 5.973355990292423e-05, -43.941917924421915, 120.27700785358547
    data = np.zeros(11)
    global est

    if estimator == "Peripheral":
        est = PeripheralEstimator(scale_1, scale_2, angle_1, angle_2)
    elif estimator == "Kalman":
        raise RuntimeError("Kalman is not implemented yet")
    else:
        raise RuntimeError("Provided estimator isn't implemented")

    stop_event = threading.Event()
    test_counter = [0]

    t1 = threading.Thread(target=control, args=(stop_event, test_counter, shared_array), daemon = True)
    print("sensor_mapping_should_begin")
    t2 = threading.Thread(target=est.update, args=(ser, data, stop_event), daemon = True)
    if desired_angle < 0:
        direction = "left"
    else:
        direction = "right"


    # t3 = threading.Thread(target=init_robot, args=(stop_event,direction))
    t1.start()
    t2.start()
    time.sleep(4)  # Ensure the control thread is running before starting the mapping thread
    # t3.start()
    robot=init_robot()
    start_angle = est.history[10][2]
    x = est.history[10][0]
    y = est.history[10][1]
    # desired_angle = (desired_angle ) % 360
    #upper_lim_desired_angle= (desired_angle+2) %360
    current_angle = est.history[-1][2]
    try:
        while(1):
            current_angle = est.history[-1][2]
            error_rotation = (current_angle- start_angle - desired_angle) % 360
            if error_rotation > 180:
                error_rotation -= 360
            rotation_velocity_normalized=min(20,abs(error_rotation))*0.05
            direction = "left" if error_rotation <0 else "right"


            x,y = est.history[-1][0], est.history[-1][1]
            distance_from_start = np.sqrt(x**2 + y**2)
            error_distance = desired_distance - distance_from_start
            straight_velocity_normalized = min(0.1,abs(error_distance))*10

            #print("rotation error: ", error_rotation, " distance error: ", error_distance, " current angle: ", current_angle, " current distance: ", distance_from_start)
            move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, straight_velocity_normalized, error_rotation)
            #vierkant_maken(robot.bus, False, rotation_velocity_normalized, direction, 1, error_rotation)
            if (seperate_movements):
                if abs(error_rotation) > 2:
                    move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, 0, error_rotation)
                elif abs(error_distance) > 0.05:
                    move_rot_and_straight(robot.bus, False, rotation_velocity_normalized, direction, straight_velocity_normalized, error_rotation)
                else:
                    print("Desired position reached. Stopping the robot.")
                    move_rot_and_straight(robot.bus, True, 0, "", 0, 0)
                    stop_event.set()
                    break
            else:
                if (-1 <error_rotation < 2 and abs(error_distance) < 0.05):
                    print("Desired angle reached. Stopping the robot.")
                    #aanpassen rotate_platform(robot.bus, True)
                    stop_event.set()
                    break


            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt in sensor_control_process")
        print(est.history, " ", np.sqrt(est.history[-1][0]**2 + est.history[-1][1]**2)," meter")
        move_rot_and_straight(robot.bus, True, 0, "", 0, 0)
        stop_event.set()
        print("Stopping the robot and exiting sensor_control_process...")
    finally:
        print("\nKeyboardInterrupt")
        print(est.history)
        stop_event.set()
        rotate_platform(robot.bus, True)
        t1.join()
        t2.join()
#sensor_control_process("Peripheral", [0.0] * 11)

def end():
    print("Terminating process and stopping the robot.")
    rotate_platform(robot.bus, True)  # Stop the robot
