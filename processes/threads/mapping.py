import numpy as np
import argparse
import math
from processes.threads.sensor_communication import *
import numpy.typing as npt
import threading

class PeripheralEstimator:
    def __init__(self,scale_1, scale_2, angle_1, angle_2):
        self.pose = np.array([0.0, 0.0])  # x, y
        self.angle_1_cos = math.cos(math.radians(angle_1))
        self.angle_1_sin = math.sin(math.radians(angle_1))
        self.angle_2_cos = math.cos(math.radians(angle_2))
        self.angle_2_sin = math.sin(math.radians(angle_2))
        self.scale_1 = scale_1
        self.scale_2 = scale_2
        self.history: list[tuple[float, float, float, float]] = [(0.0, 0.0, 0.0, 0)]  # (x, y, heading, timestamp)
        self.mission = False
        self.returning = False
        self.brake = False
        self.lock = threading.Lock()

    def update(self, ser : serial.Serial, data : npt.NDArray[np.float64], stop_signal : threading.Event):
        """
        ser = the Serial busdriver
        data = [heading, gyro, lin_acc_x, lin_acc_y, US_1_distance, OFS_1_X, OFS_1_Y, OFS_2_X, OFS_2_Y, elapsed time]
        stop_signal = the threading.Event() of the main thread
        """
        while not stop_signal.is_set():
            #request the sensor data
            get_sensor_data(ser, data)

            #check whether the robot is about to hit a wall
            if data[4] < 10:
                with self.lock:
                    self.brake = True

            #check whether the robot is about to fall of an edge
            #if data[5] > #max height:
            #    with self.lock:
            #        self.brake = True

            # Convert heading: in the original code they use -raw_orientation_x
            p_a = math.radians(data[0])   # this becomes the current orientation angle

            # Calculate velocities from flow (same as _calculate_velocities)
            fx1, fy1, fx2, fy2 = self.scale_1*data[5], self.scale_1*data[6], self.scale_2*data[7], self.scale_2*data[8]
            
            #saving elapsed time
            dt = data[9]
            # sensor 1
            v_x1 = (fx1*self.angle_1_cos + fy1*self.angle_1_sin) / dt # -fy1 / dt
            v_y1 = (- fx1*self.angle_1_sin + fy1*self.angle_1_cos) / dt # fx1 / dt
            # sensor 2
            v_x2 = (fx2*self.angle_2_cos + fy2*self.angle_2_sin) / dt #fy2 / dt
            v_y2 = (- fx2*self.angle_2_sin + fy2*self.angle_2_cos) / dt #-fx2 / dt
            # average
            v_x = (v_x1 + v_x2) / 2.0
            v_y = (v_y1 + v_y2) / 2.0

            # Calculate displacements first
            dx = (v_x * math.cos(p_a) - v_y * math.sin(p_a)) * dt
            dy = (v_x * math.sin(p_a) + v_y * math.cos(p_a)) * dt

            # Update position using current heading (p_a)
            with self.lock:
                self.pose[0] += dx
                self.pose[1] += dy
                self.history.append((self.pose[0], self.pose[1], p_a, self.history[-1][-1] + dt))
            print(f'{data}')
        print("mapping thread closing")
    
    def start_mission(self):
        self.mission = True
    
    def complete_mission(self):
        self.mission = False

    def clear_history(self):
        self.history.clear()