# Publish the suboal to the controller

import rospy
import numpy as np
from geometry_msgs.msg import Vector3, Pose

IN_FILE = "subgoals/test.txt"

THREADHOLD = 2.5 # meters

class SubgoalPublisher:
    def __init__(self):
        rospy.init_node('subgoal_publisher')

        # Variables
        self.curr_pose = Pose()
        self.subgoal = Vector3()
        self.subgoal_idx = 0

        # Publishers
        self.goalvec_publisher = rospy.Publisher("/goal_vec", Vector3, queue_size=10)
        self.subgoal_publisher = rospy.Publisher("/subgoal", Vector3, queue_size=10)

        # Subscribers
        self.pose_subscriber = rospy.Subscriber("/rover/gt_pose", Pose, self._pose_callback)

        # Read subgoals
        self.subgoals = []
        with open(IN_FILE, "r") as f:
            for line in f:
                x, y = line.split()
                self.subgoals.append((float(x), float(y)))
        print(f"Subgoals loaded, total {len(self.subgoals)}")
        
        print("Subgoal publisher node initialized")

    def _to_local_coords(self, points, origin, yaw0):
        # Rotation matrix
        R = np.array([
            [np.cos(yaw0), -np.sin(yaw0)],
            [np.sin(yaw0), np.cos(yaw0)],
        ])
        points = points - origin
        points = points @ R
        return points

    def _pose_callback(self, pose: Pose):
        # Get pose
        self.curr_pose = pose

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            x, y, yaw = self.curr_pose.position.x, self.curr_pose.position.y, self.curr_pose.orientation.z

            # Distance to subgoal
            g_x, g_y = self.subgoals[self.subgoal_idx]
            dist = ((x-g_x)**2 + (y-g_y)**2)**0.5

            # Check if subgoal reached
            if dist < THREADHOLD:
                print(f"Subgoal reached: {g_x:.3f}, {g_y:.3f}")
                self.subgoal_idx += 1

                # Check if all subgoals reached
                if self.subgoal_idx >= len(self.subgoals):
                    print("All subgoals reached!")
                    return

            # Calculate goal vector
            g_x, g_y = self.subgoals[self.subgoal_idx]
            goal_vec = Vector3()
            goal_vec.x, goal_vec.y = self._to_local_coords(np.array([[g_x, g_y]]), np.array([[x, y]]), yaw).flatten()

            # Publish
            self.goalvec_publisher.publish(goal_vec)
            self.subgoal_publisher.publish(Vector3(g_x, g_y, 0))
            print(f"Subgoal x:{g_x:.2f} y:{g_y:.2f}, idx:{self.subgoal_idx}", end="\r")
            
            rate.sleep()
        
if __name__ == '__main__':
    try:
        subgoal_publisher = SubgoalPublisher()
        subgoal_publisher.run()
    except rospy.ROSInterruptException:
        pass