import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# Control
FREQ = 30.0
MAX_V = 1.0
MAX_W = 1.0

# Safety
MAX_STEP_DIST = 2.0 # meters
MAX_CMD_X = 2.0 # meters
MAX_CMD_Y = 2.0 # meters

class RoverController:
    def __init__(self):
        rospy.init_node('rover_controller_rconly')

        # Subscribers
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self._joy_callback)
        
        # Publishers
        self.rover_publisher = rospy.Publisher("/rover/cmd_vel", Twist, queue_size=10)

        # Joystick variables
        self.joy_l_stick_x = 0.0    
        self.joy_l_stick_y = 0.0
        self.user_control = False

        # Maximum velocities
        self.max_v = MAX_V
        self.max_w = MAX_W

        print("Rover controller (RC-only) approacher node initialized")

    def _joy_callback(self, joy_msg: Joy):
        # Check if user control is enabled
        self.user_control = joy_msg.axes[2] < 0.5 or joy_msg.buttons[4] == 1 # LB or LT pressed

        # Get joystick command
        self.joy_l_stick_x = joy_msg.axes[0]    # float -1.0 to 1.0
        self.joy_l_stick_y = joy_msg.axes[1]    # float -1.0 to 1.0

        # Max speed
        if joy_msg.buttons[0] == 1:
            self.max_v = 1.6
            self.max_w = 1.6
        else:
            self.max_v = MAX_V
            self.max_w = MAX_W
    
    def run(self):
        rate = rospy.Rate(FREQ)

        # ROS loop
        while not rospy.is_shutdown():
            vel_msg = Twist()

            # User control
            if self.user_control:
                # Velocity command
                vel_msg.linear.x = self.joy_l_stick_y * self.max_v
                vel_msg.angular.z = self.joy_l_stick_x * self.max_w
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
            
            # Publish
            self.rover_publisher.publish(vel_msg)

            # ROS sleep
            rate.sleep()
        
if __name__ == '__main__':
    try:
        rover_controller = RoverController()
        rover_controller.run()
    except rospy.ROSInterruptException:
        pass
