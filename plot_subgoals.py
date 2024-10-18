import rospy
import numpy as np
from typing import Tuple
from geometry_msgs.msg import Pose, Vector3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

IN_FILE = "subgoals/test.txt"

class SubgoalPlotter:
    def __init__(self):
        rospy.init_node('subgoal_plotter')

        # Variables
        self.curr_pose = Pose()
        self.curr_goal = Vector3()

        # Subscribers
        self.pose_subscriber = rospy.Subscriber("/rover/gt_pose", Pose, self._pose_callback)
        self.goal_subscriber = rospy.Subscriber("/goal_vec", Vector3, self._goal_callback)

        # Read subgoals
        subgoals = []
        with open(IN_FILE, "r") as f:
            for line in f:
                x, y = line.split()
                subgoals.append((float(x), float(y)))
        self.subgoals = np.array([[p[0] for p in subgoals], [p[1] for p in subgoals]])

        print(f"Subgoal plotter node initialized")

    def _pose_callback(self, pose: Pose):
        self.curr_pose = pose

    def _goal_callback(self, goal: Vector3):
        self.curr_goal = goal

    def _to_local_coords(self, points, yaw0):
        # Rotation matrix
        R = np.array([
            [np.cos(yaw0), -np.sin(yaw0)],
            [np.sin(yaw0), np.cos(yaw0)],
        ])
        points = points @ R
        return points

    def _to_global_coords(self, points, yaw0):
        # Rotation matrix
        R = np.array([
            [np.cos(yaw0), -np.sin(yaw0)],
            [np.sin(yaw0), np.cos(yaw0)],
        ])
        points = points @ R.T
        return points

    def _plot_and_save(self):
        plt.plot(self.subgoals[1], self.subgoals[0], "r-o")
        plt.plot(self.curr_pose.position.y, self.curr_pose.position.x, "bo")

        # Arrow from current pose to goal
        curr_goal_vec = np.array([self.curr_goal.x, self.curr_goal.y])
        goal_glob = self._to_global_coords(curr_goal_vec, self.curr_pose.orientation.z)
        plt.arrow(self.curr_pose.position.y,
                  self.curr_pose.position.x, 
                  goal_glob[1],
                  goal_glob[0],
                  head_width=0.5, 
                  head_length=0.5, 
                  fc='g', 
                  ec='g')
        
        # Unit arrow for orientation
        plt.arrow(self.curr_pose.position.y, 
                  self.curr_pose.position.x, 
                  5*np.sin(self.curr_pose.orientation.z), 
                  5*np.cos(self.curr_pose.orientation.z), 
                  head_width=0.5, 
                  head_length=0.5, 
                  fc='b', 
                  ec='b')

        plt.xlabel("Y (m)")
        plt.ylabel("X (m)")
        plt.gca().invert_xaxis()
        plt.title("Subgoals")
        plt.grid()
        plt.gca().set_aspect('equal', adjustable='box')

        plt.savefig("subgoals.png")

        # Clear plot
        plt.clf()

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self._plot_and_save()
            rate.sleep()

if __name__ == "__main__":
    try:
        subgoal_plotter = SubgoalPlotter()
        subgoal_plotter.run()
    except rospy.ROSInterruptException:
        pass