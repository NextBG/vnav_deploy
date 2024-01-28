# Publish the waypoint to the controller

import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class WaypointPublisher:
    def __init__(self):
        rospy.init_node('waypoint_publisher')
        self.waypoint = Vector3()
        self.wp_reached = Bool()
        self.wp_publisher = rospy.Publisher("/waypoint", Vector3, queue_size=10)
        self.wp_reached_subscriber = rospy.Subscriber("/waypoint_reached", Bool, self.waypoint_reached_callback)
        print("Waypoint publisher node initialized")

    def waypoint_reached_callback(self, wp_reached: Bool):
        self.wp_reached = wp_reached

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            if(self.wp_reached.data == True):

                wp = Vector3()

                # Take input from user
                wp.x = float(input("Enter x: "))
                wp.y = float(input("Enter y: "))
                wp.z = 0.0

                # x should be > 0
                if wp.x < 0:
                    print("x should be > 0, try again")
                    continue

                self.wp_publisher.publish(wp)
                print("new wp published")

            rate.sleep()
        
if __name__ == '__main__':
    try:
        wp_publisher = WaypointPublisher()
        wp_publisher.run()
    except rospy.ROSInterruptException:
        pass