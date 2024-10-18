# Get the gps data from the /fix topic 
import os
import rospy
from typing import Tuple
from geometry_msgs.msg import Pose

OUT_FILE = "subgoals/test.txt"

class SubgoalCollector:
    def __init__(self):
        rospy.init_node('subgoal_collector')
        
        # GPS variables
        self.position = (0.0, 0.0) # (x,y) in meters
        
        # Publishers
        self.pose_subscriber = rospy.Subscriber("/rover/gt_pose", Pose, self._pose_callback)

        # Delete if exists
        if os.path.exists(OUT_FILE):
            os.remove(OUT_FILE)

        print(f"subgoal_collector node initialized")

    def _pose_callback(self, pose: Pose):
        # Update
        self.position = (pose.position.x, pose.position.y)
        print(f"Pose updated: {self.position[0]:.3f}, {self.position[1]:.3f}", end="\r")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            
            # Check for user input
            print("Press Enter to collect subgoal")
            input()

            # Save
            with open(OUT_FILE, "a") as f:
                f.write(f"{self.position[0]:.3f} {self.position[1]:.3f}\n")
                print(f"Subgoal saved: {self.position[0]:.3f}, {self.position[1]:.3f}")

            rate.sleep()

if __name__ == '__main__':
    try:
        subgoal_collector = SubgoalCollector()
        subgoal_collector.run()
    except rospy.ROSInterruptException:
        pass