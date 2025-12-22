#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import sys
import numpy as np
import dynamic_reconfigure.client
from scipy.spatial.transform import Rotation as R
import time

def update_goal(position, orientation, rate=10):
    """
    update the goal pose, please note the position is in the form of numpy array while orientation is ros pose data
    :param position: numpy array
    :param orientation: ros pose.orientation
    :return:
    """
    goal = PoseStamped()
    rate = rospy.Rate(rate)
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = ""
    goal.pose.position.x = position.x
    goal.pose.position.y = position.y
    goal.pose.position.z = position.z
    goal.pose.orientation.x = orientation.x
    goal.pose.orientation.y = orientation.y
    goal.pose.orientation.z = orientation.z
    goal.pose.orientation.w = orientation.w
    # wait to update
    time.sleep(2)
    goal_publisher.publish(goal)
    rate.sleep()

if __name__ == '__main__':
    # cartesian_position_sub()
    # file_name = str(sys.argv[1])
    # For IJRR experiments
    file_name = '/home/jihong/dressing_data_IJRR/Human_dressing_exps/path_' + str(sys.argv[1]) + '.npy'
    print("loading path...\n")
    print("go back to start...\n")
    path = np.load(file_name)
    # For IJRR experiments------------------------
    path = path.T
    #---------------------------------------------
    # delete low velocity components
    velo = np.linalg.norm(np.diff(path, axis=0), axis=1)
    index = np.where(velo > 1e-4)[0]
    path = path[index, :]
    # init a ros node
    rospy.init_node("pose_pub")
    rate = rospy.Rate(10)
    #---------------For Jianyong's IJRR
    # Move the robot to hand
    print("Move the robot to the hand position\n")
    client = dynamic_reconfigure.client.Client("/dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness_X': str(0),
              'translational_stiffness_Y': str(0),
              'translational_stiffness_Z': str(0),
              'rotational_stiffness_Z': str(0)
              }
    config = client.update_configuration(params)
    input("Press <ENTER> to continue...\n")
    #--------------------------------------------
    # check the current position
    data = PoseStamped()
    while not rospy.is_shutdown():
        data = rospy.wait_for_message("/cartesian_pose", PoseStamped)
        current_position = data.pose.position
        current_orientation = data.pose.orientation
        print("The current position of the robot is: ", current_position)
        rate.sleep()
        break
    # For Jianyong's IJRR, implementing rotation-------------------------------
    current_orientation_array = np.array([data.pose.orientation.x,
                                         data.pose.orientation.y,
                                         data.pose.orientation.z,
                                         data.pose.orientation.w])
    init_z_quat = R.from_quat(current_orientation_array)
    z_euler = init_z_quat.as_euler('ZYX')
    z_euler_init = np.copy(z_euler)
    # #----------------------------------------------------------------------------
    goal_publisher = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=10)
    update_goal(current_position, current_orientation,1)
    print("The current position of the robot is: ", current_position)
    # set the stiffness
    # client = dynamic_reconfigure.client.Client("/dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness_X': str(400),
              'translational_stiffness_Y': str(400),
              'translational_stiffness_Z': str(400),
              'rotational_stiffness_Z': str(15)
              }
    config = client.update_configuration(params)
    distance = np.sqrt((current_position.x - path[0, 0]) ** 2 +
                       (current_position.y - path[0, 1]) ** 2 +
                       (current_position.z - path[0, 2]) ** 2)
    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = ""
    # if distance > 0.05:
    #     N = int(np.ceil(distance / 0.03))
    #     # generate linearly interpolated targets
    #     x_targets = np.linspace(current_position.x, path[0, 0], N)
    #     y_targets = np.linspace(current_position.y, path[0, 1], N)
    #     z_targets = np.linspace(current_position.z, path[0, 2], N)
    #     for idx in range(N):
    #         goal.pose.position.x = x_targets[idx]
    #         goal.pose.position.y = y_targets[idx]
    #         goal.pose.position.z = z_targets[idx]
    #
    #         goal.pose.orientation.x = current_orientation.x
    #         goal.pose.orientation.y = current_orientation.y
    #         goal.pose.orientation.z = current_orientation.z
    #         goal.pose.orientation.w = current_orientation.w
    #         goal_publisher.publish(goal)
    #         rate.sleep()
    # else:
    goal.pose.position.x = path[0, 0]
    goal.pose.position.y = path[0, 1]
    goal.pose.position.z = path[0, 2]

    goal.pose.orientation.x = current_orientation.x
    goal.pose.orientation.y = current_orientation.y
    goal.pose.orientation.z = current_orientation.z
    goal.pose.orientation.w = current_orientation.w
    # goal.pose.orientation.x = path[0, 3]
    # goal.pose.orientation.y = path[0, 4]
    # goal.pose.orientation.z = path[0, 5]
    # goal.pose.orientation.w = path[0, 6]
    goal_publisher.publish(goal)
    rate.sleep()
    # run the path
    print('replay the path \n')
    input("Press <ENTER> to continue...\n")
    for i in range(1, path.shape[0]):
        # dummy orientation updates------------------------------
        if abs(z_euler[0] - z_euler_init[0]) < 1.57:
            z_euler = z_euler + np.array([0.02, 0, 0])
            z_ori = R.from_euler('ZYX', z_euler)
            z_quat = z_ori.as_quat()
        #---------------------------------------------------------
        goal.pose.position.x = path[i, 0]
        goal.pose.position.y = path[i, 1]
        goal.pose.position.z = path[i, 2]
        goal.pose.orientation.x = z_quat[0]
        goal.pose.orientation.y = z_quat[1]
        goal.pose.orientation.z = z_quat[2]
        goal.pose.orientation.w = z_quat[3]
        goal_publisher.publish(goal)
        rate.sleep()