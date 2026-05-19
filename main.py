import time
import argparse
from multiprocessing import Process, Array
from processes.sensor_control import sensor_control_process
from processes.sensor_control import end
from states import *
from states import search_loop, drive_to_ball, stop_movement, grab_ball, _set_est, _set_ser, _set_hist_stupid
import numpy as np
from processes.threads.mapping import *
from social_functions import neutraal, boos, sad, hart, eekhoorn, cute, random_sounds, six_seven, grom, wenen, celebration
from arm_move_angles import PID_sequentie2, PID_air_tracking
import random

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
        # VERWIJDER DEZE TIME.SLEEP NIET!!!!
        time.sleep(1)
 
        robot_config = SO100FollowerConfig(port="/dev/ttyACM0", id="dog", use_degrees=True)
        robot = SO100Follower(robot_config)
        robot.connect()
        while True:

            
            PID_air_tracking(robot)
            ball_opgepakt = False
            while not ball_opgepakt:
                while True:
                    print("search loop")
                    ball = search_loop(wrist_angle=0)
                    if ball:
                        break
                    # Random richting rijden
                    x = (random.random() - 0.5) * 15
                    y = (random.random() - 0.5) * 15
                    drive_to_ball(x,y)

                drive_to_ball(ball[0]+0.1*ball[1], ball[1]) # Rijdt naar de bal
                for i in range(3):
                    opgepakt = PID_sequentie2(robot)
                    if opgepakt:
                        ball_opgepakt = True
                        break
                    print("Searching locally")
                    detected, move_y = search_locally(robot) # Search on the floor, finds it in range?
                    if detected:
                        print("bijsturen met move_y")
                        drive_to_ball(0,move_y)
                    else:
                        print("keert 50 cm terug")
                        drive_to_ball(0,-50)
                        break

            print("return")
            return_with_ball()

            print("reset history")            
            est.clear_history_and_reset_pose()
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