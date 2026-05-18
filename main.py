import time
import argparse
from multiprocessing import Process, Array
from processes.sensor_control import sensor_control_process
from processes.sensor_control import end
from states import *
from states import opstart, search_loop, drive_to_ball, stop_movement, grab_ball, _set_est, _set_ser, _set_hist_stupid
# from processes.camera import camera_process
import numpy as np
from processes.threads.mapping import *
from social_functions import neutraal, boos, sad, hart, eekhoorn, random_sounds
from arm_move_angles import PID_sequentie2

# States =
state = "START"

est = None
ser = None

def main(estimator: str) -> None:
    global est, ser
    shared_array = Array('d', [0.0] * 11)

    start = time.time()
    timeout = 20

    try:
        #opstart()
        while True:
            scale_1, scale_2, angle_1, angle_2 = 5.803293347508479e-05, 5.973355990292423e-05, -43.941917924421915, 120.27700785358547
            est = PeripheralEstimator(scale_1, scale_2, angle_1, angle_2)
            ser = initialize_esp()
            _set_est(est)
            _set_ser(ser)
            _set_hist_stupid()

            stop_event = threading.Event()
            test_counter = [0]
            data = np.zeros(11)

            t1 = threading.Thread(target=control, args=(stop_event, test_counter, shared_array), daemon = True)
            print("sensor_mapping_should_begin")
            time.sleep(0.3)
            t2 = threading.Thread(target=est.update, args=(ser, data, stop_event), daemon = True)

            sound_thread = threading.Thread(target=random_sounds, daemon=True)
            
            t1.start()
            t2.start()
            sound_thread.start()


            print("tot hier gekomen")
            #hart()

            robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
            robot = SO100Follower(robot_config)
            robot.connect()
            # print("robot conected!")
            cute(robot)
            # eekhoorn(robot)
            # print("tot einde geraakt")
            
            # PID_sequentie2(robot)
            break
            # drive_to_ball(1,20)

            # Main loop
            # # x,y,z in centimeters; x is left/right, y is forward, z is vertical
            # ball = search_loop(subject="ball_floor")
            # print(ball)
            print("search loop should start")
            ball = search_loop(subject="ball_floor", wrist_angle=0)
            print(ball)
            drive_to_ball(ball[0], ball[1]-5)
            PID_sequentie2(robot)

            step_size = 50

            # while ball is not None:
            #     # if ball[1] > step_size:
            #     #     print("BIGGER STEP SIZE")
            #     #     drive_to_ball(ball[0],step_size)
            #     #     print("AFTER DRIVE TO BALL")
            #     #     wrist_angle = atan2((ball[1]-50) / 25)
            #     #     print("wrist_angle:")
            #     #     print(wrist_angle)
            #     #     ball = search_loop(subject="ball_floor", wrist_angle=wrist_angle-30)  # OF +30 NOG TESTEN
            #     # else:
            #     print("FINAL DRIVE")
            #     drive_to_ball(ball[0], ball[1])
            #     print("AT BALL!")
            #     ball = None

            # return_with_ball_stupid()
            # grab_ball()

            # ball = (40,-40,0)
            # while ball is not None:
            #     x, y, z = ball
            #     moved = drive_to_ball(x, y, step_cm=50.0)
            #     if not moved:
            #         break
            #     ball = search_loop(subject="ball_floor")

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
    except Exception as e:
        print(e)
        stop_movement()
        print("Shutting down...")
    finally:
        t1.join()
        t2.join()
        sound_thread.join()

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
