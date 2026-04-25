import time
import threading
import numpy as np
from multiprocessing import Array
from multiprocessing.sharedctypes import SynchronizedArray
from processes.threads.mapping import *
from processes.threads.control import *


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
    t2 = threading.Thread(target=est.update, args=(ser, data, stop_event))
    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()

    t1.join()
    t2.join()

    print(f"counter  = {test_counter}")
    print(f"x = {est.pose[0]}, y = {est.pose[1]}")
    print(f"history  = {est.history[-5]}")
    print("sensor/control process closing")
