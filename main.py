import time
import argparse
from multiprocessing import Process, Array
from processes.sensor_control import sensor_control_process
from processes.camera import camera_process


def main(estimator: str) -> None:
    shared_array = Array('d', [0.0] * 11)

    p_sensor = Process(target=sensor_control_process, args=(estimator, shared_array), name="sensor_control")
    p_camera = Process(target=camera_process,         args=(shared_array,),           name="camera")

    p_sensor.start()
    p_camera.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Shutting down...")

    p_sensor.join()
    p_camera.join()
    print("All processes closed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process sensor data from text file')
    parser.add_argument(
        'estimator',
        nargs='?',
        default='Peripheral',
        help='selected type of position estimator. options: Peripheral (default), Kalman'
    )
    args = parser.parse_args()
    main(args.estimator)