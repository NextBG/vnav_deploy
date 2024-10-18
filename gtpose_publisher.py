# Get the gps data from the /fix topic 
import rospy
import math
from typing import Tuple
from geometry_msgs.msg import Pose
from gnss import Gnss

# Wit motion SDK
from wit_sdk.device_model import DeviceModel
from wit_sdk.protocol_resolver.wit_protocol_resolver import WitProtocolResolver

# Constants
R_EARTH = 6378137.0 # meters

class GtPosePublisher:
    def __init__(self):
        rospy.init_node('gt_pose_publisher')
        
        # GPS variables
        self.gps = Gnss()
        self.origin = (35.02685186433334, 135.7817819336666) # (lat,lon) Kyoto University, Building 8, 413
        self.position = (0.0, 0.0) # (x,y) in meters

        # Wit imu SDK setup
        self.imu = DeviceModel(
            WitProtocolResolver()
        )
        self.imu.serialConfig.portName = "/dev/wit_imu"
        self.imu.serialConfig.baud = 115200 
        self.imu.openDevice()

        # Pose that includes position xy and orientation z
        self.pose = Pose()
        
        # Publishers
        self.pose_publisher = rospy.Publisher("/rover/gt_pose", Pose, queue_size=10)

        print(f"gt_pose_publisher node initialized, origin set to: {self.origin}")

    def lonlat_to_xy(self, lon: float, lat: float) -> Tuple[float, float]:
        # Convert lon/lat to x/y
        lat0_rad = self.origin[0] * math.pi / 180.0
        lon0_rad = self.origin[1] * math.pi / 180.0
        lat1_rad = lat * math.pi / 180.0
        lon1_rad = lon * math.pi / 180.0
        dlat_rad = lat1_rad - lat0_rad
        dlon_rad = lon1_rad - lon0_rad

        # Convert to meters
        x = R_EARTH * dlon_rad * math.cos((lat0_rad + lat1_rad) / 2.0)
        y = R_EARTH * dlat_rad

        return (y, -x) # ROS x=forward, y=left
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # GPS
            lat, lon, numSv = self.gps.read()
            if (lat == None or lon == None) or (lat == 0.0 and lon == 0.0):
                x, y = 0.0, 0.0
                print(f"GPS not received, numSv: {numSv}")
            else:
                x, y = self.lonlat_to_xy(lon, lat)

            # IMU
            rot_z = self.imu.getDeviceData("angleZ")
            if rot_z == None:
                rot_z = 0.0
                print("IMU not received")
            rot_z_rad = rot_z * math.pi / 180.0

            # Publish pose
            self.pose.position.x = x
            self.pose.position.y = y
            self.pose.orientation.z = rot_z_rad
            self.pose_publisher.publish(self.pose)
            
            print(f"Pose x: {x:.2f}, y: {y:.2f}, z: {rot_z:.2f}")

            rate.sleep()

if __name__ == '__main__':
    try:
        gt_pose_publisher = GtPosePublisher()
        gt_pose_publisher.run()
    except rospy.ROSInterruptException:
        pass