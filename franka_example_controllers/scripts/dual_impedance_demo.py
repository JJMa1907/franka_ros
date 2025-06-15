#!/usr/bin/env python3

"""
Dual Impedance Controller Demo

This script demonstrates how to use the dual impedance controller by:
1. Starting in cartesian mode
2. Sending cartesian pose commands
3. Switching to joint mode
4. Sending joint position commands
5. Switching back to cartesian mode

Usage:
  rosrun franka_example_controllers dual_impedance_demo.py
"""

import rospy
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
import time

class DualImpedanceDemo:
    def __init__(self):
        rospy.init_node('dual_impedance_demo', anonymous=True)
        
        # Publishers
        self.mode_pub = rospy.Publisher('/impedance_mode', Bool, queue_size=1)
        self.joint_cmd_pub = rospy.Publisher('/joint_command', Float64MultiArray, queue_size=1)
        self.pose_cmd_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=1)
        
        # Wait for services and topics to be available
        rospy.loginfo("Waiting for dual impedance controller...")
        rospy.sleep(2.0)
        
        rospy.loginfo("Dual Impedance Controller Demo Started!")
        
    def switch_mode(self, cartesian=True):
        """Switch between cartesian (True) and joint (False) modes"""
        mode_msg = Bool()
        mode_msg.data = cartesian
        self.mode_pub.publish(mode_msg)
        
        mode_str = "Cartesian" if cartesian else "Joint"
        rospy.loginfo(f"Switched to {mode_str} Impedance Mode")
        rospy.sleep(1.0)  # Give time for mode switch
        
    def send_joint_command(self, joint_positions):
        """Send joint position command (7 DOF)"""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = joint_positions
        self.joint_cmd_pub.publish(cmd_msg)
        rospy.loginfo(f"Sent joint command: {joint_positions}")
        
    def send_cartesian_command(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """Send cartesian pose command"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "panda_link0"
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        
        self.pose_cmd_pub.publish(pose_msg)
        rospy.loginfo(f"Sent cartesian command: pos=({x:.3f}, {y:.3f}, {z:.3f})")
        
    def run_demo(self):
        """Run the complete demo"""
        rospy.loginfo("=== Starting Dual Impedance Controller Demo ===")
        
        # Demo sequence
        try:
            # 1. Start in Cartesian mode
            rospy.loginfo("\n1. Starting in Cartesian Impedance Mode")
            self.switch_mode(cartesian=True)
            
            # Send some cartesian commands
            rospy.loginfo("Sending cartesian pose commands...")
            self.send_cartesian_command(0.4, 0.0, 0.5)  # Forward
            rospy.sleep(3.0)
            
            self.send_cartesian_command(0.4, 0.2, 0.5)  # Right
            rospy.sleep(3.0)
            
            self.send_cartesian_command(0.4, -0.2, 0.5)  # Left
            rospy.sleep(3.0)
            
            # 2. Switch to Joint mode
            rospy.loginfo("\n2. Switching to Joint Impedance Mode")
            self.switch_mode(cartesian=False)
            
            # Send some joint commands (safe positions)
            rospy.loginfo("Sending joint position commands...")
            home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
            self.send_joint_command(home_joints)
            rospy.sleep(4.0)
            
            # Small joint movements
            joints_1 = [0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
            self.send_joint_command(joints_1)
            rospy.sleep(3.0)
            
            joints_2 = [-0.2, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
            self.send_joint_command(joints_2)
            rospy.sleep(3.0)
            
            # Return to home
            self.send_joint_command(home_joints)
            rospy.sleep(3.0)
            
            # 3. Switch back to Cartesian mode
            rospy.loginfo("\n3. Switching back to Cartesian Impedance Mode")
            self.switch_mode(cartesian=True)
            
            # Final cartesian command
            self.send_cartesian_command(0.4, 0.0, 0.4)
            rospy.sleep(3.0)
            
            rospy.loginfo("\n=== Demo completed successfully! ===")
            rospy.loginfo("You can now manually control the robot using:")
            rospy.loginfo("  - /impedance_mode topic to switch modes")
            rospy.loginfo("  - /joint_command topic for joint control") 
            rospy.loginfo("  - /equilibrium_pose topic for cartesian control")
            rospy.loginfo("  - /switch_impedance_mode service for mode switching")
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Demo interrupted by user")
        except Exception as e:
            rospy.logerr(f"Demo failed: {e}")

if __name__ == '__main__':
    try:
        demo = DualImpedanceDemo()
        demo.run_demo()
        
        # Keep node alive for manual control
        rospy.loginfo("Demo node staying alive for manual control...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
