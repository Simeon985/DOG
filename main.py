import time
import argparse
from multiprocessing import Process, Array
from processes.sensor_control import sensor_control_process
from processes.sensor_control import end
from states import *
from states import opstart, search_loop, drive_to_ball, stop_movement, grab_ball, _set_est, _set_ser, _set_hist_stupid
import numpy as np
from processes.threads.mapping import *
from social_functions import neutraal, boos, sad, hart, eekhoorn, cute, random_sounds, six_seven, grom, wenen, celebration
from arm_move_angles import PID_sequentie2, PID_air_tracking

# States =
state = "START"

est = None
ser = None

def main() -> None:
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

            #time.sleep(30)
            print("tot hier gekomen")
            # hart()
        

            robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
            robot = SO100Follower(robot_config)
            robot.connect()

            #PID_air_tracking(robot)
            # break
            # motor_names = list(robot.bus.motors.keys())
            # current_obs = robot.get_observation()
            # current_joints = {name: float(current_obs[f"{name}.pos"]) for name in motor_names}
            # print(current_joints)
            # celebration(robot)

            # break
            # eekhoorn(robot)

            # PID_sequentie2(robot)

            # grom(robot)
            # time.sleep(1)
            # wenen(robot)
            # time.sleep(1)
            # six_seven(robot)
            # time.sleep(1)

            # cute(robot)
            
            # break

            # eekhoorn(robot)
            # PID_sequentie2(robot)
            #drive_to_ball(0,20)
            #drive_to_ball(10,0)
            # print("start returning")
            # return_with_ball()
            # print("done returning")
            # break
            
            # drive_to_ball(1,30)
            # break
            # drive_to_ball(-1,50)
            # print("drive1")
            #drive_to_ball(,1)
            # print("drive2")
            # return_with_ball()
            # break

            ball = search_loop(subject="ball_floor", wrist_angle=0)
            drive_to_ball(ball[0]+5, ball[1]-5) # Rijdt naar de bal
            for i in range(3):
                detected = search_locally(robot) # Search on the floor, finds it?
                if detected:
                    PID_sequentie2(robot)
                drive_to_ball(0,-20) # If not detected, drive 20cm back
                ball = search_loop(subject="ball_floor", wrist_angle=20)
                drive_to_ball(ball[0], ball[1]-5)


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
    main()
