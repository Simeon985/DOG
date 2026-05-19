import numpy as np
import argparse
import math
from processes.threads.sensor_communication import *
import numpy.typing as npt
import threading
import time
import os
import matplotlib.pyplot as plt
from datetime import datetime

class PeripheralEstimator:
    def __init__(self,scale_1, scale_2, angle_1, angle_2, plot_dir="plots"):
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
        # scale_factor_1: 5.963691140961605e-05
        # scale_factor_2: 5.8920022248807314e-05
        # angle_1: -46.31328914736247
        # angle_2: 117.43771136252667
        # Plotting attributes
        self.plot_dir = plot_dir
        self.plot_counter = 0
        os.makedirs(self.plot_dir, exist_ok=True)
        
        # Initialize plot
        plt.ion()  # Interactive mode on
        self.fig, self.ax = plt.subplots(figsize=(10, 8))


    def update(self, ser : serial.Serial, data : npt.NDArray[np.float64], stop_signal : threading.Event):
        """
        ser = the Serial busdriver
        data = [heading, gyro, lin_acc_x, lin_acc_y, US_1_distance, OFS_1_X, OFS_1_Y, OFS_2_X, OFS_2_Y, elapsed time]
        stop_signal = the threading.Event() of the main thread
        """
        last_plot_time = time.time()
        plot_interval = 10
        while not stop_signal.is_set():
            #print("beginning of mapping thread loop")
            #request the sensor data
            get_sensor_data(ser, data)

            #check whether the robot is about to hit a wall
            if data[4] < 40:
                with self.lock:
                    self.brake = True

            #check whether the robot is about to fall of an edge
            if data[5] > 40: #max height
               with self.lock:
                   self.brake = True
            #print("data: ", data)

            # Convert heading: in the original code they use -raw_orientation_x
            p_a = math.radians(data[0])   # this becomes the current orientation angle

            # Calculate velocities from flow (same as _calculate_velocities)
            fx1, fy1, fx2, fy2 = self.scale_1*data[6], self.scale_1*data[7], self.scale_2*data[8], self.scale_2*data[9]
            #saving elapsed time
            dt = data[10]
            # sensor 1
            v_x1 = (fx1*self.angle_1_cos + fy1*self.angle_1_sin) / dt # -fy1 / dt
            v_y1 = (- fx1*self.angle_1_sin + fy1*self.angle_1_cos) / dt # fx1 / dt
            # sensor 2
            v_x2 = (fx2*self.angle_2_cos + fy2*self.angle_2_sin) / dt #fy2 / dt
            v_y2 = (- fx2*self.angle_2_sin + fy2*self.angle_2_cos) / dt #-fx2 / dt
            # average
            v_x = (v_x1 + v_x2) / 2.0
            v_y = (v_y1 + v_y2) / 2.0

            # zorg dat als heading 0 is, rechte beweging vooruit de y-richting is
            # zodat het in lijn is met de andere 'robot frame' conventies
            v_x, v_y = -v_y, v_x
            # v_x en v_y zijn nu dus correct in het robot-frame,
            # dx en dy zetten dit dan om naar het world frame

            # Calculate displacements first
            dx = (v_x * math.cos(p_a) - v_y * math.sin(p_a)) * dt
            dy = (v_x * math.sin(p_a) + v_y * math.cos(p_a)) * dt

            # Update position using current heading (p_a)
            with self.lock:
                if not math.isnan(dx):
                    self.pose[0] += dx
                if not math.isnan(dy):   
                    self.pose[1] += dy

                #self.history.append((self.pose[0], self.pose[1], data[0], self.history[-1][-1] + dt))
                self.history.append((float(self.pose[0]), float(self.pose[1]), float(data[0]), float(self.history[-1][-1] + dt)))
            #print(f"history: {self.history[-1]}")
            current_time = time.time()
            if current_time - last_plot_time >= plot_interval:  # Uncomment to limit plot frequency
                print(f"us 1: {data[4]}")
                print(f"us 2: {data[5]}")
                self.plot_and_save_history()
                last_plot_time = current_time
            time.sleep(0.01)
        print("mapping thread closing")


    def start_mission(self):
        self.mission = True

    def complete_mission(self):
        self.mission = False

    def clear_history(self):
        self.history.clear()
        self.plot_counter = 0 

    def calculate_total_distance(self):
        """Calculate total distance traveled"""
        with self.lock:
            if len(self.history) < 2:
                return 0.0
            
            total_dist = 0.0
            for i in range(1, len(self.history)):
                dx = self.history[i][0] - self.history[i-1][0]
                dy = self.history[i][1] - self.history[i-1][1]
                total_dist += math.sqrt(dx*dx + dy*dy)
            return total_dist

    def cleanup_old_plots(self, max_plots=1000):
        """Remove old plot files to save disk space"""
        try:
            files = sorted([f for f in os.listdir(self.plot_dir) if f.startswith('trajectory_step_')])
            if len(files) > max_plots:
                for f in files[:-max_plots]:
                    os.remove(os.path.join(self.plot_dir, f))
        except Exception as e:
            print(f"Warning: Could not clean up old plots: {e}")

    def plot_and_save_history(self):
        """Plot the trajectory history and save to an image file"""
        with self.lock:
            if len(self.history) < 2:
                return
            
            # Extract data
            x_vals = [point[0] for point in self.history]
            y_vals = [point[1] for point in self.history]
            headings = [point[2] for point in self.history]
            timestamps = [point[3] for point in self.history]
        
        # Clear and replot
        self.ax.clear()
        
        # Plot trajectory
        self.ax.plot(x_vals, y_vals, 'b-', linewidth=2, label='Trajectory')
        self.ax.plot(x_vals[0], y_vals[0], 'go', markersize=10, label='Start')
        self.ax.plot(x_vals[-1], y_vals[-1], 'ro', markersize=10, label='Current Position')
        
        # Add heading arrows at regular intervals
        num_arrows = min(20, len(self.history))
        arrow_indices = np.linspace(0, len(self.history)-1, num_arrows, dtype=int)
        
        for idx in arrow_indices:
            if idx < len(x_vals):
                # Convert heading from degrees to radians for arrow direction
                heading_rad = math.radians(headings[idx])
                # Arrow length (scaled appropriately)
                arrow_len = 0.1
                dx_arrow = arrow_len * math.cos(heading_rad)
                dy_arrow = arrow_len * math.sin(heading_rad)
                self.ax.arrow(x_vals[idx], y_vals[idx], dx_arrow, dy_arrow, 
                            head_width=0.05, head_length=0.05, fc='gray', ec='gray', alpha=0.5)
        
        # Add color map for time - FIXED: convert timestamps to numpy array
        if len(timestamps) > 1:
            timestamps_np = np.array(timestamps)  # Convert list to numpy array
            time_normalized = (timestamps_np - timestamps_np[0]) / (timestamps_np[-1] - timestamps_np[0])
            colors = plt.cm.viridis(time_normalized)
            self.ax.scatter(x_vals, y_vals, c=colors, s=20, alpha=0.6, label='Path points')
        
        # Labels and title
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title(f'Robot Trajectory - Step {self.plot_counter}', fontsize=14)
        self.ax.grid(True, alpha=0.3)
        self.ax.axis('equal')
        
        # Add info text
        info_text = f"Position: ({x_vals[-1]:.3f}, {y_vals[-1]:.3f})\n"
        info_text += f"Heading: {headings[-1]:.1f}°\n"
        info_text += f"Total distance: {self.calculate_total_distance():.3f} m\n"
        info_text += f"Mission: {'Active' if self.mission else 'Inactive'}"
        self.ax.text(0.02, 0.98, info_text, transform=self.ax.transAxes,
                    fontsize=10, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        self.ax.legend(loc='best')
        
        # Save the plot
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"trajectory.png"#f"trajectory_step_{self.plot_counter:06d}_{timestamp}.png"
        filepath = os.path.join(self.plot_dir, filename)
        
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        
        # If you want to display the plot in real-time (uncomment if needed)
        # self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        
        self.plot_counter += 1
        
        # Optional: Clean up old plots to save disk space (keep last 1000)
        self.cleanup_old_plots()