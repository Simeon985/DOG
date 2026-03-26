import time
import threading
from AI.mapping import *

def main(estimator):
    ser = initialize_esp()
    scale_1, scale_2, angle_1, angle_2 = 0.0, 0.0 #Calculated using specific hardware specifications
    est = None
    data = np.zeros(10)
    if estimator == "Peripheral":
        est = PeripheralEstimator(scale_1, scale_2, angle_1, angle_2)
    elif estimator == "Kalman":
        raise RuntimeError("Kalman is not implemented yet")
    else:
        raise RuntimeError("Provided estimator isn't implemented")
    stop_event = threading.Event()
    


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process sensor data from text file')
    parser.add_argument('estimator', 
                       nargs='?',  # Makes it optional
                       default='Peripheral',  # Default value
                       help='selected type of position estimator. options: Peripheral(default), Kalman')
    args = parser.parse_args()
    main(args.estimator)