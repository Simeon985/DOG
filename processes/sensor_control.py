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

global ani
def sensor_control_process(estimator: str, shared_array: SynchronizedArray) -> None:
    """Process running the control and sensor threads."""
    ser = initialize_esp()
    scale_1, scale_2, angle_1, angle_2 = 1.0, 1.0, 0.0, 0.0
    data = np.zeros(11)

    if estimator == "Peripheral":
        est = PeripheralEstimator(scale_1, scale_2, angle_1, angle_2)
    elif estimator == "Kalman":
        raise RuntimeError("Kalman is not implemented yet")
    else:
        raise RuntimeError("Provided estimator isn't implemented")

    stop_event = threading.Event()
    test_counter = [0]

    t1 = threading.Thread(target=control, args=(stop_event, test_counter, shared_array))
    print("sensor_mapping_should_begin")
    t2 = threading.Thread(target=est.update, args=(ser, data, stop_event))
    t3 = threading.Thread(target=init_robot, args=(stop_event,))
    t1.start()
    t2.start()
    t3.start()
    while(1):
        print("IMU: ", est.history[-1][2]-est.history[0][2])
        if est.history[-1][2]-est.history[0][2]>30 and est.history[0][2] < 40:
            stop_event.set()
            break
        # if est.history[-1][2] > math.radians(30) and est.history[-1][2] < math.radians(40):
        #     stop_event.set()
        #     break

        time.sleep(0.1)


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
