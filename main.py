import time
import threading
from sensor.mapping import *
from control.control import *
from multiprocessing import process, Array

def main(estimator):
    ser = initialize_esp()
    scale_1, scale_2, angle_1, angle_2 = 1.0, 1.0, 0.0, 0.0 #Calculated using specific hardware specifications
    est = None
    data = np.zeros(10)
    if estimator == "Peripheral":
        est = PeripheralEstimator(scale_1, scale_2, angle_1, angle_2)
    elif estimator == "Kalman":
        raise RuntimeError("Kalman is not implemented yet")
    else:
        raise RuntimeError("Provided estimator isn't implemented")
    stop_event = threading.Event()
    test_counter = [0]
    t1 = threading.Thread(target= control, args=(stop_event, test_counter))
    t2 = threading.Thread(target= est.update, args=(ser, data, stop_event))

    t1.start()
    t2.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()  # This affects both threads simultaneously
    
    t1.join()
    t2.join()
    print(f"counter={test_counter}")
    print(f"x = {est.pose[0]}, y = {est.pose[1]}")
    print(f"history = {est.history[-5]}")
    print("main thread closing")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process sensor data from text file')
    parser.add_argument('estimator', 
                       nargs='?',  # Makes it optional
                       default='Peripheral',  # Default value
                       help='selected type of position estimator. options: Peripheral(default), Kalman')
    args = parser.parse_args()
    main(args.estimator)