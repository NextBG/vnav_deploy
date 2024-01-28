# Control the rover with joystick
# Joy stick topic: /joy
# rover control topic: /rover/twist
# linear.x: forward/backward, controlled by left stick up/down
# angular.z: turn left/right, controlled by left stick left/right
# control frequency: 50Hz

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickRoverControl:
    def __init__(self):
        rospy.init_node('joystick_rover_control')
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.rover_publisher = rospy.Publisher("/rover/cmd_vel", Twist, queue_size=10)
        self.twist_msg = Twist()
        print("Joystick rover control node initialized")

    def joy_callback(self, joy_msg):

        print(joy_msg)

        if joy_msg.axes[2] < 0.0 or joy_msg.buttons[4] == 1: # If left trigger is pressed or button LB is pressed
            self.twist_msg.linear.x = joy_msg.axes[1] # Linear velocity (forward/backward) controlled by left stick up/down (axis 1)
            self.twist_msg.angular.z = joy_msg.axes[0] # Angular velocity (turn left/right) controlled by left stick left/right
        else:
            self.twist_msg.linear.x = 0
            self.twist_msg.angular.z = 0

    def run(self):
        rate = rospy.Rate(50)  # Control frequency: 50Hz
        while not rospy.is_shutdown():
            self.rover_publisher.publish(self.twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        joystick_control = JoystickRoverControl()
        joystick_control.run()
    except rospy.ROSInterruptException:
        pass
