#!/usr/bin/env python
import rospy, actionlib
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon

rospy.init_node("grasp_aluminium_plate")
client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
client.wait_for_server()

goal = GraspGoal()
goal.width = 0.0031           # 10 mm —— 根据标定板厚度调整
goal.epsilon = GraspEpsilon(inner=0.0005, outer=0.0005)  # << 关键：无此会 1 s 后松手
goal.speed  = 0.02           # m/s
goal.force  = 60.0            # N，可根据重量调小
client.send_goal(goal)
client.wait_for_result()

rospy.spin()  # 保持节点运行，力会一直存在
-0.6963642, -0.6963642, -0.1227878, -0.1227878
0.6963642, 0.6963642, 0.1227878, 0.1227878
