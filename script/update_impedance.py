#!/usr/bin/env python3


"""
Update the impedance with service
"""
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped
import dynamic_reconfigure.client

current_position = Point()
current_wrench = WrenchStamped()

def pos_callback(data):
    global current_position
    current_position = data

def set_translational_impedance(trans_stiffness):
    client = dynamic_reconfigure.client.Client("/dynamic_reconfigure_compliance_param_node")
    params = {'translational_stiffness_X': str(trans_stiffness),
              'translational_stiffness_Y': str(trans_stiffness),
              'translational_stiffness_Z': str(trans_stiffness)}
    config = client.update_configuration(params)

def send_new_position_with_impedance(trans_stiffness = 0):
    set_translational_impedance(trans_stiffness)

if __name__ == '__main__':
    rospy.init_node("stiffness_X")
    try:
        send_new_position_with_impedance()
    except rospy.ROSInterruptException:
        pass
