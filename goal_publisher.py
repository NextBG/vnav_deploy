'''
Publish a vector pointing to the goal[x,y] in local frame to /goal_vec according a list of goals in global frame
'''

import rospy
from geometry_msgs.msg import Vector3, Pose

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher')
        
        # Subscribers
        self.gtpose_subscriber = rospy.Subscriber("/rover/gt_pose", Pose, self.gtpose_callback)

        # Publishers
        self.goal_publisher = rospy.Publisher("/goal_vec", Vector3, queue_size=10)

        # Params
        self.gtpose = Pose()

    def _gtpose_callback(self, gtpose: Pose):
        self.gtpose = gtpose

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            goal = Vector3()

            # Dummy goal
            goal.x = 1.0
            goal.y = 0.0
            goal.z = 0.0

            self.goal_publisher.publish(goal)

            rate.sleep()