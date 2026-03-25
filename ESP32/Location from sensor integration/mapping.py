import numpy as np
import argparse
import math
from AI.sensor_communication import *

class PeripheralEstimator:
    def __init__(self,scale_1, scale_2, angle_1, angle_2):
        self.pose = np.array([0.0, 0.0])  # x, y
        self.angle_1_cos = math.cos(math.radians(angle_1))
        self.angle_1_sin = math.sin(math.radians(angle_1))
        self.angle_2_cos = math.cos(math.radians(angle_2))
        self.angle_2_sin = math.sin(math.radians(angle_2))
        self.scale_1 = scale_1
        self.scale_2 = scale_2
        self.history: list[tuple[float, float, float, float]] = []  # (x, y, heading, timestamp)
        self.mission = False
        self.returning = False

    def update(self, data):
        """
        data = [heading, gyro, lin_acc_x, lin_acc_y, US_1_distance, OFS_1_X, OFS_1_Y, OFS_2_X, OFS_2_Y, elapsed time]
        """
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

        # Update position using current heading (p_a)
        self.pose[0] += (v_x * math.cos(p_a) - v_y * math.sin(p_a)) * dt
        self.pose[1] += (v_x * math.sin(p_a) + v_y * math.cos(p_a)) * dt
        self.history.append(self.pose[0], self.pose[1], p_a, self.history[-1][-1] + dt)
    
    def start_mission(self):
        self.mission = True
    
    def complete_mission(self):
        self.mission = False

    def clear_history(self):
        self.history.clear()

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
    while True:
        while not est.mission and not est.returning:
            None
        while est.mission:
            get_sensor_data(ser, data)
            est.update(data)
        while not est.mission and est.returning:
            None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process sensor data from text file')
    parser.add_argument('estimator', 
                       nargs='?',  # Makes it optional
                       default='Peripheral',  # Default value
                       help='selected type of position estimator. options: Peripheral(default), Kalman')
    args = parser.parse_args()
    main(args.estimator)