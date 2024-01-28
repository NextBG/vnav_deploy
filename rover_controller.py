'''
Given the waypoint, navigate the rover to the waypoint using PD controller
'''
import numpy as np
from typing import Tuple

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

# Control
FREQ = 10.0
DT = 1.0/FREQ
MAX_V = 0.8
MAX_W = 0.8

# PD controller
KP_DIST = 1.0
KD_DIST = 0.1
KP_THETA = 1.0
KD_THETA = 0.1
DIST_EPS = 0.1
THETA_EPS = 0.02

# Safety
MAX_STEP_DIST = 2.0 # meters
MAX_CMD_X = 2.0 # meters
MAX_CMD_Y = 2.0 # meters

class RoverController:
    def __init__(self):
        rospy.init_node('rover_controller')

        # Params
        self.waypoint = Vector3()
        self.origin_odom = Twist() 
        self.latest_odom = Twist()
        self.origin_initialized = False

        # Subscribers
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self._joy_callback)
        self.odom_subscriber = rospy.Subscriber("/rover/odometry", Twist, self._odom_callback)
        self.waypoint_subscriber = rospy.Subscriber("/waypoint", Vector3, self._waypoint_callback)
        
        # Publishers
        self.rover_publisher = rospy.Publisher("/rover/cmd_vel", Twist, queue_size=10)
        self.busy_flag_publisher = rospy.Publisher("/rover/controller_busy", Bool, queue_size=10)

        # PD controller params
        self.kp_dist = KP_DIST
        self.kd_dist = KD_DIST
        self.kp_theta = KP_THETA
        self.kd_theta = KD_THETA

        # PD controller variables
        self.prev_err_dist = 0.0
        self.prev_err_theta = 0.0
        self.dt = DT
        self.theta_reached = False
        self.busy = False

        # Joystick variables
        self.joy_l_stick_x = 0.0    
        self.joy_l_stick_y = 0.0
        self.user_control = False

        print("Waypoint approacher node initialized")

    def _reset_pd_controller(self):
        # Reset the PD controller
        self.origin_odom = self.latest_odom
        self.prev_err_dist = 0.0
        self.prev_err_theta = 0.0
        self.theta_reached = False

    def _joy_callback(self, joy_msg: Joy):
        # Check if user control is enabled
        user_control_prev = self.user_control
        self.user_control = joy_msg.axes[2] < 0.5 or joy_msg.buttons[4] == 1 # LB or LT pressed

        if user_control_prev != self.user_control:
            # Reset PD controller
            self._reset_pd_controller()
            if self.user_control:
                self.busy = True
                print("User control engaged")
            else:
                self.busy = False
                print("User control disengaged")

        # Get joystick command
        self.joy_l_stick_x = joy_msg.axes[0]    # float -1.0 to 1.0
        self.joy_l_stick_y = joy_msg.axes[1]    # float -1.0 to 1.0

    def _waypoint_callback(self, waypoint_msg: Vector3):
        # Check if the waypoint is within the safety range
        if np.abs(waypoint_msg.x) > MAX_CMD_X or np.abs(waypoint_msg.y) > MAX_CMD_Y:
            print("Waypoint out of safety range, stopped")
            self.rover_publisher.publish(Twist())
            return
        
        print(f"New waypoint received x: {waypoint_msg.x:.3f}, y: {waypoint_msg.y:.3f}")
        self.waypoint = waypoint_msg

        # Reset PD controller
        self._reset_pd_controller()
        self.busy = True

    def _odom_callback(self, odometry: Twist):
        self.latest_odom = odometry

        # Set the first origin to current odom
        if not self.origin_initialized:
            self.origin_odom = self.latest_odom
            self.origin_initialized = True

    def _get_error(self):
        # Global relative position
        x_relv = self.latest_odom.linear.x - self.origin_odom.linear.x
        y_relv = self.latest_odom.linear.y - self.origin_odom.linear.y
        theta_relv = self.latest_odom.angular.z - self.origin_odom.angular.z

        # Wrap around theta
        if theta_relv > np.pi:
            theta_relv -= 2*np.pi
        elif theta_relv < -np.pi:
            theta_relv += 2*np.pi

        # Local relative position
        theta = self.origin_odom.angular.z
        x_relv_loc = x_relv * np.cos(-theta) - y_relv * np.sin(-theta)
        y_relv_loc = x_relv * np.sin(-theta) + y_relv * np.cos(-theta)

        # Open loop distance error
        dist_relv_loc = np.sqrt(x_relv_loc**2 + y_relv_loc**2)
        dist_desired = np.sqrt(self.waypoint.x**2 + self.waypoint.y**2)
        err_dist_open = dist_desired - dist_relv_loc

        # Closed loop distance error 
        err_x = self.waypoint.x - x_relv_loc
        err_y = self.waypoint.y - y_relv_loc
        err_dist_closed = np.sqrt(err_x**2 + err_y**2)

        # Theta error
        desired_theta = np.arctan2(err_y, err_x)
        err_theta = desired_theta - theta_relv

        return (dist_relv_loc, theta_relv), (err_dist_open, err_dist_closed, err_theta)

    def _pd_controller(self) -> Tuple[float]:
        '''
        Position Error
        '''
        # Errors
        (dist_relv_loc, theta_relv), (err_dist_open, err_dist_closed, err_theta) = self._get_error()

        # Theta not reached
        if not self.theta_reached:
            if np.abs(err_theta) < THETA_EPS:
                self.theta_reached = True

        # Dist reached
        elif err_dist_open < DIST_EPS:
            self.busy = False
            print("Goal reached, stopped")
            return 0.0, 0.0

            
        # Safety
        if dist_relv_loc > MAX_STEP_DIST or np.abs(theta_relv) > np.pi/2.0:
            self._reset_pd_controller()
            self.busy = False
            print(f"Safety limit reached, stopped: dist_relv_loc: {dist_relv_loc:.3f}, theta_relv: {theta_relv:.3f}")
            return 0.0, 0.0

        '''
        PD control
        '''
        # Derivative of errors
        d_err_dist = (err_dist_open - self.prev_err_dist)/self.dt
        d_err_theta = (err_theta - self.prev_err_theta)/self.dt

        # Velocity command
        if not self.theta_reached: 
            # Rotate in place
            w = self.kp_theta * err_theta + self.kd_theta * d_err_theta 
            v = 0.0
        else:
            # Move forward
            w = 0.0
            v = self.kp_dist * err_dist_open + self.kd_dist * d_err_dist

        # Update previous error
        self.prev_err_theta = err_theta
        self.prev_err_dist = err_dist_open

        # print(f"err_x: {err_x:.3f}, err_y: {err_y:.3f}, err_dist: {err_dist:.3f}, desired_theta: {desired_theta: .3f}, err_theta: {err_theta:.3f}, v: {v:.3f}, w: {w:.3f}")

        return v, w
    
    def run(self):
        rate = rospy.Rate(FREQ)

        # ROS loop
        while not rospy.is_shutdown():
            vel_msg = Twist()

            # User control
            if self.user_control:
                # Velocity command
                vel_msg.linear.x = self.joy_l_stick_y * MAX_V
                vel_msg.angular.z = self.joy_l_stick_x * MAX_W
            
            # Waypoint approach
            elif self.busy:
                # Velocity command
                v, w = self._pd_controller()
                vel_msg.linear.x = v
                vel_msg.angular.z = w

            # Publish
            self.busy_flag_publisher.publish(self.busy)
            self.rover_publisher.publish(vel_msg)

            # ROS sleep
            rate.sleep()
        
if __name__ == '__main__':
    try:
        rover_controller = RoverController()
        rover_controller.run()
    except rospy.ROSInterruptException:
        pass