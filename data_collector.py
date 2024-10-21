import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, Joy
from nav_msgs.msg import Odometry
import os
import time
import pickle
import numpy as np
from PIL import Image as PILImage
import argparse
import matplotlib.pyplot as plt

class DataCollector:
    def __init__(self, interval):
        rospy.init_node('data_collector')

        # Params
        self.interval = interval
        self.seq_idx = 0
        self.data_idx = 0  # For image sequence numbering
        self.traj_data = {"positions": [], "yaws": [], "raw": []}
        self.obs_curr = None

        # Subscribers
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self._obs_callback)
        self.odom_sub = rospy.Subscriber("/camera/odom/sample", Odometry, self._odom_callback)
        
        # Dataset folder with date as the name
        self.dataset_dir = os.path.join("datasets", time.strftime("%y%m%d"))
        os.makedirs(self.dataset_dir, exist_ok=True)

        # Subfolder for current dataset
        self.current_data_dir = os.path.join(self.dataset_dir, f"{self.seq_idx:06d}")
        os.makedirs(self.current_data_dir, exist_ok=True)

        print("Data collector node initialized.")

    def _obs_callback(self, img: Image):
        # Convert ROS Image message to PIL image
        img_pil = PILImage.frombytes("RGB", (img.width, img.height), img.data) # 360 640

        # Update the current image
        self.obs_curr = img_pil

    def _odom_callback(self, odom: Odometry):
        # Extract 2D position (x, y) and yaw from odometry
        self.pos = odom.pose.pose.position
        self.ori = odom.pose.pose.orientation

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw."""
        q_x, q_y, q_z, q_w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def save_data(self):
        # Save the image and odometry data
        if self.obs_curr and self.pos and self.ori:
            # Image
            img_path = os.path.join(self.current_data_dir, f"{self.data_idx:6d}.jpg")
            self.obs_curr.save(img_path)

            # Odometry
            yaw = self.quaternion_to_yaw(self.ori) * 180 / np.pi
            self.traj_data["positions"].append([self.pos.x, self.pos.y])
            self.traj_data["yaws"].append(yaw)
            self.traj_data["raw"].append([self.pos.x, self.pos.y, self.pos.z, self.ori.w, self.ori.x, self.ori.y, self.ori.z])

            print(f"Point {self.data_idx:03d} saved: {self.pos.x:.2f}, {self.pos.y:.2f}, Yaw: {yaw:.2f}", end="\r")

            self.data_idx += 1
            
        # Save the trajectory data after 1000 images
        if self.data_idx >= 1000:
            pkl_path = os.path.join(self.current_data_dir, 'traj_data.pkl')
            with open(pkl_path, 'wb') as f:
                pickle.dump(self.traj_data, f)
            self.plot_data()
            print(f"Trajectory data saved to {pkl_path}")
            
            # Start new dataset after saving the first one
            self.data_idx = 0
            self.traj_data = {"positions": [], "yaws": [], "raw": []}
            self.current_data_dir = os.path.join(self.dataset_dir, f"{self.seq_idx:06d}")
            os.makedirs(self.current_data_dir, exist_ok=True)
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

        # Plot the trajectory
        ax.plot(self.traj_data['positions'][:,1], self.traj_data['positions'][:,0], color='r', linewidth=0.1)
        # Plot every 100th point
        ax.plot(self.traj_data['positions'][:,1][::100], self.traj_data['positions'][:,0][::100], 'o', color='b', markersize=0.1)
        # Plot the direction vector
        vis_stride = 10
        ax.quiver(
            self.traj_data['positions'][:,1][::vis_stride], 
            self.traj_data['positions'][:,0][::vis_stride], 
            -np.sin(self.traj_data['yaws'][::vis_stride]),
            np.cos(self.traj_data['yaws'][::vis_stride]),
            color='g', 
            width=0.001)

        # legend
        ax.legend([f'every {vis_stride} points', 'every 100 points', 'direction vector', 'yaw angle'], loc='lower right', fontsize='xx-small')

        fig.savefig(os.path.join(self.current_data_dir, 'visualization.png'))

        # Close the figure
        plt.close(fig)

    def run(self):
        rate = rospy.Rate(1.0 / self.interval)  # Collect data at specified interval
        while not rospy.is_shutdown():
            # Do the data collection
            self.save_data()

            # Sleep according to the rate
            rate.sleep()

if __name__ == "__main__":
    # Parse interval argument
    parser = argparse.ArgumentParser(description='Data collector for ROS')
    parser.add_argument('--interval', type=float, default=1.0, help='Data collection interval in seconds')
    args = parser.parse_args()

    # ROS
    try:
        data_collector = DataCollector(args.interval)
        data_collector.run()
    except rospy.ROSInterruptException:
        pass
