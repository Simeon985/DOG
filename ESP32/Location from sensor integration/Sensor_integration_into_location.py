import numpy as np
import argparse
import math
import matplotlib.pyplot as plt 
from pykalman import UnscentedKalmanFilter  # requires pykalman

def read_sensor_data(filename):
    """
    Read sensor data from text file and return as lists of values
    
    Format: dt heading gyro_x lin_acc_x lin_acc_y deltaX1 deltaY1 deltaX2 deltaY2 distance
    """
    # Initialize empty lists for each variable
    dt = []
    heading = []
    gyro_x = []
    lin_acc_x = []
    lin_acc_y = []
    deltaX1 = []
    deltaY1 = []
    deltaX2 = []
    deltaY2 = []
    distance = []
    
    try:
        with open(filename, 'r') as file:
            for line_num, line in enumerate(file, 1):
                # Skip empty lines
                line = line.strip()
                if not line:
                    continue
                
                # Split the line by spaces and filter out empty strings
                values = line.split()
                
                # Check if we have the expected number of values (10)
                if len(values) != 10:
                    print(f"Warning: Line {line_num} has {len(values)} values, expected 10. Skipping...")
                    continue
                
                try:
                    # Parse each value as float (all values are numeric)
                    dt.append(float(values[0]))
                    heading.append(float(values[1]))
                    gyro_x.append(float(values[2]))
                    lin_acc_x.append(float(values[3]))
                    lin_acc_y.append(float(values[4]))
                    deltaX1.append(float(values[5]))
                    deltaY1.append(float(values[6]))
                    deltaX2.append(float(values[7]))
                    deltaY2.append(float(values[8]))
                    distance.append(float(values[9]))
                    
                except ValueError as e:
                    print(f"Warning: Line {line_num} contains non-numeric data: {e}")
                    continue
                    
        print(f"Successfully read {len(dt)} lines from {filename}")
        return {
            'dt': dt,
            'heading': heading,
            'gyro_x': gyro_x,
            'lin_acc_x': lin_acc_x,
            'lin_acc_y': lin_acc_y,
            'deltaX1': deltaX1,
            'deltaY1': deltaY1,
            'deltaX2': deltaX2,
            'deltaY2': deltaY2,
            'distance': distance,
            'filename': filename
        }
    
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
        return None
    except Exception as e:
        print(f"Error reading file: {e}")
        return None


def locations_Kalman(data, scale_factor_1, scale_factor_2, angle_1, angle_2):
    """
    Estimate robot pose using two methods:
      1) Peripheral (flow + heading) integrator
      2) Fused Kalman filter (UKF) combining flow and heading
    
    Adds the estimated trajectories to the data dictionary.
    """
    # Constants from the original hardware definitions
    T_X = 0.348      # mounting position x of flow sensor
    T_Y = 0.232      # mounting position y of flow sensor
    R = np.sqrt(T_X**2 + T_Y**2)
    alpha = np.arctan2(T_Y, T_X)          # angle of sensor position
    sqrt2_over_2 = np.sqrt(2) / 2.0

    n = len(data['dt'])
    if n == 0:
        print("No data to process.")
        return data

    # ----------------------------------------------------------------------
    # 1. Peripheral estimator (like PlatformPoseEstimatorPeripherals)
    # ----------------------------------------------------------------------
    class PeripheralEstimator:
        def __init__(self, angle_1, angle_2):
            self.pose = np.array([0.0, 0.0, 0.0])  # x, y, heading
            self.angle_1_cos = math.cos(math.radians(angle_1))
            self.angle_1_sin = math.sin(math.radians(angle_1))
            self.angle_2_cos = math.cos(math.radians(angle_2))
            self.angle_2_sin = math.sin(math.radians(angle_2))

        def update(self, dt, flow, heading):
            """
            flow = [deltaX1, deltaY1, deltaX2, deltaY2]
            heading = compass reading (raw)
            """
            # Convert heading: in the original code they use -raw_orientation_x
            p_a = math.radians(heading)   # this becomes the current orientation angle

            # Calculate velocities from flow (same as _calculate_velocities)
            fx1, fy1, fx2, fy2 = flow
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
            self.pose[2] = p_a   # heading is set directly from measurement

            return self.pose.copy()

    # ----------------------------------------------------------------------
    # 2. Fused Kalman filter estimator (like PlatformPoseEstimatorFused)
    # ----------------------------------------------------------------------
    class FusedEstimator:
        def __init__(self):
            angle_1, angle_2 = -84.41242279324928, 80.71742720691299

            self.angle_1_cos = math.cos(math.radians(angle_1))
            self.angle_1_sin = math.sin(math.radians(angle_1))
            self.angle_2_cos = math.cos(math.radians(angle_2))
            self.angle_2_sin = math.sin(math.radians(angle_2))

            T_X_1, T_Y_1 = 0.0916, -.0514
            self.R_1 = np.sqrt(T_X_1**2 + T_Y_1**2)
            alpha_1 = np.arctan2(T_Y_1, T_X_1)
            self.angle_term_1 = np.pi/2+ math.radians(angle_1)+alpha_1 # 5 * np.pi / 4 - alpha_1

            T_X_2, T_Y_2 = -0.0730, 0.0739
            self.R_2 = np.sqrt(T_X_2**2 + T_Y_2**2)
            alpha_2 = np.arctan2(T_Y_2, T_X_2)
            self.angle_term_2 = np.pi/2+math.radians(angle_2)+alpha_2  # 5 * np.pi / 4 - alpha_2

            self.state_mean = np.zeros(6)
            self.state_cov = np.eye(6) * .001

            trans_std = 0.001
            obs_flow_std = 0.01
            obs_heading_std = 0.000001

            # Build covariances and immediately ensure they are SPD
            self.transition_covariance = self._make_spd(np.eye(6) * (trans_std ** 2))
            
            obs_cov = np.eye(5)
            obs_cov[0:4, 0:4] *= (obs_flow_std ** 2)
            obs_cov[4, 4] *= (obs_heading_std ** 2)
            self.observation_covariance = self._make_spd(obs_cov)

            self.kf = UnscentedKalmanFilter(
                transition_functions=self.transition_function,
                observation_functions=self.observation_function,
                transition_covariance=self.transition_covariance,
                observation_covariance=self.observation_covariance,
                initial_state_mean=self.state_mean,
                initial_state_covariance=self.state_cov
            )
            self.dt_val = 0.0

        @staticmethod
        def _make_spd(matrix, jitter=1e-4):
            """Force a matrix to be symmetric positive definite via eigenvalue clamping."""
            matrix = (matrix + matrix.T) / 2.0
            eigvals, eigvecs = np.linalg.eigh(matrix)
            eigvals = np.maximum(eigvals, jitter)   # clamp all eigenvalues >= jitter
            return eigvecs @ np.diag(eigvals) @ eigvecs.T

        def _regularize_cov(self, cov):
            return self._make_spd(cov)



        def transition_function(self, state, noise):
            F = np.eye(6)
            F[0:3, 3:6] = np.eye(3) * self.dt_val
            return F @ state + noise

        def observation_function(self, state, noise):
            p_x, p_y, p_a, v_x, v_y, v_a = state
            sqrt2_over_2 = np.sqrt(2) / 2.0
            v_x_mobi = v_x * np.cos(p_a) + v_y * np.sin(p_a)
            v_y_mobi = -v_x * np.sin(p_a) + v_y * np.cos(p_a)


            flow_x1 = (v_x_mobi*self.angle_1_cos - v_y_mobi*self.angle_1_sin +
                    self.R_1 * v_a * np.cos(self.angle_term_1)) * self.dt_val
            flow_y1 = (v_x_mobi*self.angle_1_sin + v_y_mobi*self.angle_1_cos -
                    self.R_1 * v_a * np.sin(self.angle_term_1)) * self.dt_val
            flow_x2 = (v_x_mobi*self.angle_2_cos - v_y_mobi*self.angle_2_sin +
                    self.R_2 * v_a * np.cos(self.angle_term_2)) * self.dt_val
            flow_y2 = (v_x_mobi*self.angle_2_sin + v_y_mobi*self.angle_2_cos -
                    self.R_2 * v_a * np.sin(self.angle_term_2)) * self.dt_val

            return np.array([flow_x1, flow_y1, flow_x2, flow_y2, p_a]) + noise

        def update(self, dt, flow, heading):
            self.dt_val = dt
            orientation = math.radians(heading)
            observation = np.array([flow[0], flow[1], flow[2], flow[3], orientation])

            # Regularize before each update
            self.state_cov = self._regularize_cov(self.state_cov)
            

            self.state_mean, self.state_cov = self.kf.filter_update(
                self.state_mean,
                self.state_cov,
                observation=observation,
            )

            # Regularize after update too
            self.state_cov = self._regularize_cov(self.state_cov)

            return self.state_mean[0:3].copy()
    # ----------------------------------------------------------------------
    # Run both estimators over the data
    # ----------------------------------------------------------------------
    peripheral = PeripheralEstimator(angle_1, angle_2)
    fused = FusedEstimator()

    peripheral_poses = []   # list of [x, y, heading] at each step
    fused_poses = []

    for i in range(n):
        dt = data['dt'][i]
        heading = data['heading'][i]
        flow = [data['deltaX1'][i]*scale_factor_1, data['deltaY1'][i]*scale_factor_1,
                data['deltaX2'][i]*scale_factor_2, data['deltaY2'][i]*scale_factor_2]

        # Peripheral update
        p_pose = peripheral.update(dt, flow, heading)
        peripheral_poses.append(p_pose)

        # Fused update
        f_pose = fused.update(dt, flow, heading)
        fused_poses.append(f_pose)

    # Store results in the data dictionary
    data['peripheral_pose_x'] = [p[0] for p in peripheral_poses]
    data['peripheral_pose_y'] = [p[1] for p in peripheral_poses]
    data['peripheral_pose_a'] = [p[2] for p in peripheral_poses]

    data['fused_pose_x'] = [f[0] for f in fused_poses]
    data['fused_pose_y'] = [f[1] for f in fused_poses]
    data['fused_pose_a'] = [f[2] for f in fused_poses]

    print(f"Processed {n} steps. Added peripheral and fused pose estimates.")
    return data


def plot_trajectories(data):
    """
    Plot the 2D trajectories from peripheral and fused estimators.
    """
    if 'peripheral_pose_x' not in data or 'fused_pose_x' not in data:
        print("No trajectory data to plot. Run locations_Kalman first.")
        return

    x_peri = data['peripheral_pose_x']
    y_peri = data['peripheral_pose_y']
    x_fused = data['fused_pose_x']
    y_fused = data['fused_pose_y']

    if not x_peri or not x_fused:
        print("Empty trajectory data.")
        return

    plt.figure(figsize=(8, 8))
    plt.plot(x_peri, y_peri, 'b-', label='Peripheral (flow + heading)')
    plt.plot(x_fused, y_fused, 'r-', label='Fused (UKF)')
    plt.scatter(x_peri[0], y_peri[0], c='green', marker='o', label='Start')
    plt.scatter(x_peri[-1], y_peri[-1], c='blue', marker='s', label='End (peripheral)')
    plt.scatter(x_fused[-1], y_fused[-1], c='red', marker='s', label='End (fused)')
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    plt.title('Robot Trajectory Estimation')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()


def plot_trajectories_temp(data): 
    """
    just a temporary function that just plots trajectories for peripheral estimator
    """


    x_peri = data['peripheral_pose_x']
    y_peri = data['peripheral_pose_y']


    plt.figure(figsize=(8, 8))
    plt.plot(x_peri, y_peri, 'b-', label='Location estimation')
    plt.scatter(x_peri[0], y_peri[0], c='green', marker='o', label='Start')
    plt.scatter(x_peri[-1], y_peri[-1], c='blue', marker='s', label='End')
    plt.xlabel('X position (m)')
    plt.ylabel('Y position (m)')
    plt.title('Robot Trajectory Estimation')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()


def recalibrate_flow_sensors():
    file = 'imu_log_2_meter.txt'
    true_dist = 2.
    calibration_data = read_sensor_data(file)
    distX1 = sum(calibration_data['deltaX1'])
    distY1 = sum(calibration_data['deltaY1'])
    distX2 = sum(calibration_data['deltaX2'])
    distY2 = sum(calibration_data['deltaY2'])
    dist1 = math.sqrt(distX1*distX1 + distY1*distY1)
    dist2 = math.sqrt(distX2*distX2 + distY2*distY2)
    print(f"dist1 = {dist1}, dist2 = {dist2}")
    scale_factor_1 = true_dist/dist1
    scale_factor_2 = true_dist/dist2

    angle_1 = (math.degrees(math.atan2(distY1, distX1)) - 180) % 360 - 180
    angle_2 = (math.degrees(math.atan2(distY2, distX2)) - 180) % 360 - 180

    print(f"\nscale_factor_1: {scale_factor_1}\nscale_factor_2: {scale_factor_2}\nangle_1: {angle_1}\nangle_2: {angle_2}\n")

    return scale_factor_1, scale_factor_2, angle_1, angle_2
    

def main(filename, recalibrate):
    """
    Main function that takes filename as parameter
    
    Args:
        filename (str): Path to the sensor data file
    """
    print(f"Processing sensor data from: {filename}")
    print("-" * 50)

    scale_factor_1, scale_factor_2, angle_1, angle_2 = 0,0,0,0
    if recalibrate:
        scale_factor_1, scale_factor_2, angle_1, angle_2 = recalibrate_flow_sensors()
    
    # Read the data
    data = read_sensor_data(filename)
    
    if data is None:
        print("Failed to read data. Exiting.")
        return

    # Estimate poses
    data = locations_Kalman(data, scale_factor_1, scale_factor_2, angle_1, angle_2)

    #plot_trajectories(data)
    plot_trajectories_temp(data)

if __name__ == "__main__":
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description='Process sensor data from text file')
    parser.add_argument('filename', 
                       nargs='?',  # Makes it optional
                       default='imu_log_2_meter.txt',  # Default value
                       help='Path to the sensor data file (default: imu_log_2_meter.txt)')
    parser.add_argument('--recalibrate', 
                       action='store_true',  # This makes it a boolean flag
                       help='Force recalibration of sensor data')
    
    args = parser.parse_args()
    
    # Call main with the filename parameter
    main(args.filename, args.recalibrate)