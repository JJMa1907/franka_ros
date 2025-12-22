#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import sys
import numpy as np

class Move3D:
    def __init__(self, delta=0.01):
        self.current_position = Point()
        self.current_orientation = Quaternion()
        self.delta = delta
        rospy.init_node("pose_pub")

    def go_to_position(self, x, y, z):
        # get initial position and orientation
        data = rospy.wait_for_message("/robot_pose", Pose)
        self.current_position = data.position
        self.current_orientation = data.orientation
        print("The current position of the robot is: ", self.current_position)
        distance = np.sqrt(
            (self.current_position.x - x) ** 2 + (self.current_position.y - y) ** 2 + (self.current_position.z - z) ** 2)
        N = int(np.ceil(distance / self.delta))
        # generate linearly interpolated targets
        x_targets = np.linspace(self.current_position.x, x, N)
        y_targets = np.linspace(self.current_position.y, y, N)
        z_targets = np.linspace(self.current_position.z, z, N)
        # for idx in range(N):
        #     print(x_targets[idx], y_targets[idx], z_targets[idx])
        # set goal using the starting position
        goal = PoseStamped()
        goal_publisher = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=10)
        rate = rospy.Rate(10)
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = ""
        for idx in range(N):
            goal.pose.position.x = x_targets[idx]
            goal.pose.position.y = y_targets[idx]
            goal.pose.position.z = z_targets[idx]

            goal.pose.orientation.x = self.current_orientation.x
            goal.pose.orientation.y = self.current_orientation.y
            goal.pose.orientation.z = self.current_orientation.z
            goal.pose.orientation.w = self.current_orientation.w
            goal_publisher.publish(goal)
            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 4:
        raise RuntimeError('require 3 (x, y, z) float inputs')
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    move = Move3D()
    if not rospy.is_shutdown():
        try:
            move.go_to_position(x, y, z)
        except rospy.ROSInterruptException:
            pass
