#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import sys
import dynamic_reconfigure.client
from functions.bimanual_setup import set_impedance


current_pose = PoseStamped()

def callback_pos_sub(data):
    global current_pose
    current_pose = data

def read_pose(process_rate=10):
    rate = rospy.Rate(process_rate)
    # while loop for getting ONCE the current position of the robot in cartesian space
    while not rospy.is_shutdown():
        current_position_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, callback_pos_sub)
        if current_pose == PoseStamped():
            # skip the initial value
            continue
        else:
            break
    # Update the goal on the robot so when the stiffness is back, the robot is still safe
    #----------------------------------------------------------------------------------
    # goal = PoseStamped()
    # goal_publisher = rospy.Publisher("/equilibrium_pose", PoseStamped, queue_size=10)
    # goal.header.seq = 1
    # goal.header.stamp = rospy.Time.now()
    # goal.header.frame_id = ""
    # goal.pose = current_pose
    # goal_publisher.publish(goal)
    #-----------------------------------------------------------------------------------
    position = (current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
    orientation = (current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w)
    pose = position + orientation
    rate.sleep()
    return pose


if __name__ == '__main__':
    # rospy.init_node("path_record")
    all_pose = []
    process_rate = 50
    if len(sys.argv) <= 1:
        file_name = '/home/jihong/dressing_data_IJRR/path.npy'
    else:
        file_name = '/home/jihong/dressing_data_IJRR/' + str(sys.argv[1])
    rospy.init_node("pose_pub")
    print("file name is: ", file_name)
    rate = rospy.Rate(process_rate)
    # set the stiffness to 0
    client = dynamic_reconfigure.client.Client("/dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness_X': str(0),
              'translational_stiffness_Y': str(0),
              'translational_stiffness_Z': str(0),
              'rotational_stiffness_X': str(0),
              'rotational_stiffness_Y': str(0),
              'rotational_stiffness_Z': str(0)
              }
    config = client.update_configuration(params)
    while not rospy.is_shutdown():
        try:
            # data = rospy.wait_for_message("/cartesian_pose", PoseStamped)
            # pose = (current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z)
            pose = read_pose(process_rate)
            # print("current pose:", pose)
            all_pose.append(pose)
            with open(file_name, 'wb') as f:
                np.save(f, np.array(all_pose))
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
    # np.save(all)