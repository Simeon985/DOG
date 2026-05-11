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
def sensor_control_process(estimator: str, shared_array: SynchronizedArray, desired_angle: int=120) -> None:
    """Process running the control and sensor threads."""
    ser = initialize_esp()
    scale_1, scale_2, angle_1, angle_2 = 5.963691140961605e-05, 5.8920022248807314e-05, -46.31328914736247, 117.43771136252667
    data = np.zeros(11)


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
    time.sleep(2)  # Ensure the control thread is running before starting the mapping thread
    # t3.start()
    robot=init_robot()
    start_angle = est.history[10][2]
    # desired_angle = (desired_angle ) % 360
    upper_lim_desired_angle= (desired_angle+2) %360
    current_angle = est.history[-1][2]
    try:
        while(1):
            current_angle = est.history[-1][2]
            #print("current position 's and IMu: ", est.history[-1][0]," ",est.history[-1][1]," ",(current_angle-start_angle)%360)
            error = (current_angle- start_angle - desired_angle) % 360
            if error > 180:
                error -= 360
            rotation_velocity_normalized=min(20,abs(error))*0.05
            direction = "left" if error <0 else "right"
            print("IMU: now, error, begin: ", current_angle, error ,start_angle)
            #rotate_platform(robot.bus, False, rotation_velocity_normalized, direction)
            move_straight_to_object(robot.bus, 1, -60)
            print("-------------------------------------------------------------")
            #if ((current_angle-start_angle)%360>desired_angle and (current_angle-start_angle)%360 < upper_lim_desired_angle) or ((current_angle-start_angle)%360<360-desired_angle and (current_angle-start_angle)%360 > 360-upper_lim_desired_angle):
            if (-1 <error < 2):
                rotate_platform(robot.bus, True)
                stop_event.set()
                break


            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")
        stop_event.set()
        rotate_platform(robot.bus, True)
    finally:
        print("\nKeyboardInterrupt")
        stop_event.set()
        rotate_platform(robot.bus, True)
    t1.join()
    t2.join()

    # fig, ax = plt.subplots()
    # line, = ax.plot([], [], 'b-')

    # def update(_):
    #     with lock:
    #         if not est.history:
    #             return line,
    #         xs = [p[0] for p in est.history]
    #         ys = [p[1] for p in est.history]
    #     line.set_data(xs, ys)
    #     ax.relim()
    #     ax.autoscale_view()
    #     return line,

    # ani = FuncAnimation(fig, update, frames=itertools.count(),
    #                     interval=100, blit=False, cache_frame_data=False)
    # plt.show()

    # try:
    #     while True:
    #         time.sleep(0.1)
    # except KeyboardInterrupt:
    #     stop_event.set()

    # t1.join()
    # t2.join()
    # init_robot()

    # print(f"counter  = {test_counter}")
    # print(f"x = {est.pose[0]}, y = {est.pose[1]}")
    # print(f"history  = {est.history[-5]}")
    # print("sensor/control process closing")
sensor_control_process("Peripheral", [0.0] * 11)
