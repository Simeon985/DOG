import time
import argparse
from multiprocessing import Process, Array
from processes.sensor_control import sensor_control_process
from processes.sensor_control import end
from states import opstart, search_loop, drive_to_ball, stop_movement, grab_ball, _set_est, _set_ser
# from processes.camera import camera_process
import numpy as np
from processes.threads.mapping import *

# States =
state = "START"

est = None
ser = None

def main(estimator: str) -> None:
    global est, ser
    shared_array = Array('d', [0.0] * 11)

    start = time.time()
    timeout = 20

    # p_sensor = Process(target=sensor_control_process, args=(estimator, shared_array), name="sensor_control", daemon = True)
    # set terminate target function on process
    #p_camera = Process(target=camera_process,         args=(shared_array,),           name="camera")

    # p_camera.start()
    # print("initializing camera")
    # while np.isclose(shared_array[10],0):
    #     if time.time() - start > timeout:
    #         break
    #     time.sleep(0.1)
    # print("camera initialized")

    #p_sensor.start()

    try:
        #opstart()
        while True:
            scale_1, scale_2, angle_1, angle_2 = 5.803293347508479e-05, 5.973355990292423e-05, -43.941917924421915, 120.27700785358547
            est = PeripheralEstimator(scale_1, scale_2, angle_1, angle_2)
            ser = initialize_esp()
            _set_est(est)
            _set_ser(ser)
            # Main loop
            # # x,y,z in centimeters; x is left/right, y is forward, z is vertical
            # ball = search_loop(subject="ball_floor")
            # print(ball)
            ball = (40,-40,0)
            while ball is not None:
                x, y, z = ball
                moved = drive_to_ball(x, y, step_cm=50.0)
                if not moved:
                    break
                ball = search_loop(subject="ball_floor")

            # grab_ball()
            

            # person = search_loop(subject="person")
            # while person is not None:
            #     x, y, z = ball
            #     moved = drive_to_ball(x, y, step_cm=50.0)
            #     if not moved:
            #         break
            #     person = search_loop(subject="person")

            # x,y,z = search_loop(subject="person")

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Trying to shut down")
        stop_movement()
        #rotate_platform(p_sensor.robot.bus, True)
        print("Shutting down...")

    # p_sensor.join()
    # p_camera.join()
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
