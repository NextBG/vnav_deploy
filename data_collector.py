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

class DataCollector:
    def __init__(self, interval):
        rospy.init_node('data_collector')

        # Params
        self.interval = interval
        self.seq = 0  # For image sequence numbering
        self.traj_data = {"positions": [], "yaws": [], "raw": []}
        self.obs_curr = None

        # Subscribers
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self._obs_callback)
        self.odom_sub = rospy.Subscriber("/camera/odom/sample", Odometry, self._odom_callback)
        
        # Dataset folder with date as the name
        self.dataset_dir = os.path.join("datasets", time.strftime("%y%m%d"))
        os.makedirs(self.dataset_dir, exist_ok=True)

        # Subfolder for current dataset
        self.current_data_dir = os.path.join(self.dataset_dir, f"{self.seq:06d}")
        os.makedirs(self.current_data_dir, exist_ok=True)

        print("Data collector node initialized.")

    def _obs_callback(self, img: Image):
        # Convert ROS Image message to PIL image
        img_pil = PILImage.frombytes("RGB", (img.width, img.height), img.data) # 360 640

        # Update the current image
        self.obs_curr = img_pil

    def _odom_callback(self, odom: Odometry):
        # Extract 2D position (x, y) and yaw from odometry
        pos = odom.pose.pose.position
        ori = odom.pose.pose.orientation

        # Convert quaternion to yaw
        yaw = self.quaternion_to_yaw(ori)

        # Store the 2D position and yaw
        self.traj_data["positions"].append([pos.x, pos.y])
        self.traj_data["yaws"].append(yaw)
        
        # Store raw pose (position + orientation as quaternion)
        self.traj_data["raw"].append([pos.x, pos.y, pos.z, ori.w, ori.x, ori.y, ori.z])

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw."""
        q_x, q_y, q_z, q_w = orientation.x, orientation.y, orientation.z, orientation.w
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def save_data(self):
        # Save the current image
        if self.obs_curr:
            img_path = os.path.join(self.current_data_dir, f"{self.seq:6d}.jpg")
            self.obs_curr.save(img_path)

        # Save the trajectory data after 1000 images
        if self.seq >= 1000:
            pkl_path = os.path.join(self.current_data_dir, 'traj_data.pkl')
            with open(pkl_path, 'wb') as f:
                pickle.dump(self.traj_data, f)
            
            # Start new dataset after saving the first one
            self.seq = 0
            self.traj_data = {"positions": [], "yaws": [], "raw": []}
            self.current_data_dir = os.path.join(self.dataset_dir, f"{self.seq:06d}")
            os.makedirs(self.current_data_dir, exist_ok=True)

    def run(self):
        rate = rospy.Rate(1.0 / self.interval)  # Collect data at specified interval
        while not rospy.is_shutdown():
            # Do the data collection
            self.save_data()
            self.seq += 1

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
