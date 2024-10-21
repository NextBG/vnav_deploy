import rospy
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import os
import time
import pickle
import numpy as np
from PIL import Image as PILImage
import argparse
import matplotlib.pyplot as plt
import shutil

class DataCollector:
    def __init__(self, interval, n_points):
        rospy.init_node('data_collector')

        # Params
        self.interval = interval
        self.n_points = n_points
        self.seq_idx = 0
        self.data_idx = 0  # For image sequence numbering
        self.traj_data = {"positions": [], "yaws": [], "raw": []}
        self.obs_curr = None
        self.collecting_flag = False
        self.last_change_time = time.time()

        # Subscribers
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self._obs_callback)
        self.odom_sub = rospy.Subscriber("/camera/odom/sample", Odometry, self._odom_callback)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self._joy_callback)
        
        # Dataset folder with date as the name
        self.dataset_dir = os.path.join("datasets", time.strftime("%y%m%d"))
        os.makedirs(self.dataset_dir, exist_ok=True)
        self.current_data_dir = None

        print("Data collector node initialized.")

    def _obs_callback(self, img: Image):
        # Convert ROS Image message to PIL image
        img_pil = PILImage.frombytes("RGB", (img.width, img.height), img.data) # 360 640

        # Update the current image
        self.obs_curr = img_pil

    def _joy_callback(self, joy_msg: Joy):
        if joy_msg.buttons[5] == 1 and time.time() - self.last_change_time > 1.0:
            self.last_change_time = time.time()
            self.collecting_flag = not self.collecting_flag
            if self.collecting_flag:
                print(f"Data collection STARTED, sequence {self.seq_idx:06d} under {self.dataset_dir}")
            else:
                print(f"Data collection STOPPED, sequence {self.seq_idx:06d} under {self.dataset_dir} deleted")

    def _odom_callback(self, odom: Odometry):
        # Extract 2D position (x, y) and yaw from odometry
        self.pos = odom.pose.pose.position
        self.ori = odom.pose.pose.orientation

    def quaternion_to_yaw(self, orientation): # w, x, y, z to rx, ry, rz
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
        # rx = np.arctan2(2*(w*x+y*z), 1-2*(x**2+y**2))
        # ry = np.arcsin(2*(w*y-z*x))
        rz = np.arctan2(2*(w*z+x*y), 1-2*(y**2+z**2))
        return rz

    def save_data(self):
        # Check if data collection is enabled
        if not self.collecting_flag:
            if self.data_idx != 0:
                # Delete the unfinished sequence
                if self.current_data_dir is not None and os.path.exists(self.current_data_dir):
                    shutil.rmtree(self.current_data_dir)
                self.data_idx = 0
            return

        # Create the dataset folder if it doesn't exist
        if self.data_idx == 0:
            self.data_idx = 0
            self.traj_data = {"positions": [], "yaws": [], "raw": []}
            self.current_data_dir = os.path.join(self.dataset_dir, f"{self.seq_idx:06d}")
            os.makedirs(self.current_data_dir, exist_ok=True)

        # Save the image and odometry data
        if self.obs_curr and self.pos and self.ori:
            # Image
            img_path = os.path.join(self.current_data_dir, f"{self.data_idx:06d}.jpg")
            self.obs_curr.save(img_path)

            # Odometry
            yaw = self.quaternion_to_yaw(self.ori)
            self.traj_data["positions"].append([self.pos.x, self.pos.y])
            self.traj_data["yaws"].append(yaw)
            self.traj_data["raw"].append([self.pos.x, self.pos.y, self.pos.z, self.ori.w, self.ori.x, self.ori.y, self.ori.z])

            print(f"Point {self.data_idx:03d} saved: {self.pos.x:.2f}, {self.pos.y:.2f}, Yaw: {yaw:.2f}", end="\r")

            self.data_idx += 1
        else:
            print("Data not received")
            
        # Save the trajectory data after self.n_points images
        if self.data_idx >= self.n_points:
            pkl_path = os.path.join(self.current_data_dir, 'traj_data.pkl')
            with open(pkl_path, 'wb') as f:
                pickle.dump(self.traj_data, f)
            self.plot_data()
            print(f"Saved to {pkl_path}")

            self.data_idx = 0
            self.seq_idx += 1

    def plot_data(self):
        # Plot the trajectory and save the image
        fig, ax = plt.subplots()
        ax.set_title("Trajectory")
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('y')
        ax.set_ylabel('x')
        ax.invert_xaxis()
        ax.grid()

        # Turn to np
        self.traj_data['positions'] = np.array(self.traj_data['positions'])
        self.traj_data['yaws'] = np.array(self.traj_data['yaws'])

        # Plot the trajectory
        ax.plot(self.traj_data['positions'][:,1], self.traj_data['positions'][:,0], color='r', linewidth=0.5)
        # Plot every 100th point
        ax.plot(self.traj_data['positions'][:,1][::100], self.traj_data['positions'][:,0][::100], 'o', color='b', markersize=0.1)
        # Plot the direction vector
        vis_stride = 5
        ax.quiver(
            self.traj_data['positions'][:,1][::vis_stride], 
            self.traj_data['positions'][:,0][::vis_stride], 
            -np.sin(self.traj_data['yaws'][::vis_stride]),
            np.cos(self.traj_data['yaws'][::vis_stride]),
            color='g', 
            width=0.005)
        

        # legend
        ax.legend([f'every {vis_stride} points', 'every 100 points', 'direction vector', 'yaw angle'], loc='lower right', fontsize='xx-small')

        fig.savefig(os.path.join(self.current_data_dir, 'visualization.png'))

        # Close the figure
        plt.close(fig)

    def run(self):
        rate = rospy.Rate(1.0 / self.interval)  # Collect data at specified interval
        while not rospy.is_shutdown():
            self.save_data()

            # Sleep according to the rate
            rate.sleep()

if __name__ == "__main__":
    # Parse interval argument
    parser = argparse.ArgumentParser(description='Data collector for ROS')
    parser.add_argument('--interval', type=float, default=0.5, help='Data collection interval in seconds')
    parser.add_argument('--n_points', type=int, default=1000, help='Number of points to collect before saving the trajectory')
    args = parser.parse_args()

    # ROS
    try:
        data_collector = DataCollector(args.interval, args.n_points)
        data_collector.run()
    except rospy.ROSInterruptException:
        pass
