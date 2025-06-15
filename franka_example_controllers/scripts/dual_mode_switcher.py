#!/usr/bin/env python3

"""
Dual Impedance Controller Mode Switcher

This node provides a service and topic interface to switch between 
cartesian and joint impedance control modes.

Services:
  /switch_impedance_mode (std_srvs/SetBool) - Switch control mode
  
Topics:
  /impedance_mode (std_msgs/Bool) - Mode command topic (true=cartesian, false=joint)
  /current_impedance_mode (std_msgs/String) - Current mode status
  
Parameters:
  ~default_mode (string): Initial mode ("cartesian" or "joint")
"""

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse

class DualModeSwitcher:
    def __init__(self):
        rospy.init_node('dual_mode_switcher', anonymous=True)
        
        # Get default mode parameter
        default_mode = rospy.get_param('~default_mode', 'cartesian')
        self.current_mode_cartesian = (default_mode == 'cartesian')
        
        # Publishers
        self.mode_pub = rospy.Publisher('/impedance_mode', Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/current_impedance_mode', String, queue_size=1, latch=True)
        
        # Services
        self.switch_service = rospy.Service('/switch_impedance_mode', SetBool, self.switch_mode_callback)
        
        # Initialize mode
        self.publish_current_mode()
        
        rospy.loginfo(f"Dual Mode Switcher initialized. Current mode: {self.get_mode_string()}")
        rospy.loginfo("Available services:")
        rospy.loginfo("  /switch_impedance_mode (std_srvs/SetBool) - Switch control mode")
        rospy.loginfo("Available topics:")
        rospy.loginfo("  /impedance_mode (std_msgs/Bool) - Mode command (true=cartesian, false=joint)")
        rospy.loginfo("  /current_impedance_mode (std_msgs/String) - Current mode status")
        rospy.loginfo("Usage examples:")
        rospy.loginfo("  rosservice call /switch_impedance_mode \"data: true\"   # Switch to cartesian")
        rospy.loginfo("  rosservice call /switch_impedance_mode \"data: false\"  # Switch to joint")
        rospy.loginfo("  rostopic pub /impedance_mode std_msgs/Bool \"data: true\"  # Command cartesian mode")
    
    def get_mode_string(self):
        return "cartesian" if self.current_mode_cartesian else "joint"
    
    def publish_current_mode(self):
        # Publish mode command
        mode_msg = Bool()
        mode_msg.data = self.current_mode_cartesian
        self.mode_pub.publish(mode_msg)
        
        # Publish status
        status_msg = String()
        status_msg.data = self.get_mode_string()
        self.status_pub.publish(status_msg)
    
    def switch_mode_callback(self, req):
        """
        Service callback to switch impedance control mode
        
        Args:
            req.data (bool): True for cartesian mode, False for joint mode
            
        Returns:
            SetBoolResponse: Success status and message
        """
        old_mode = self.get_mode_string()
        self.current_mode_cartesian = req.data
        new_mode = self.get_mode_string()
        
        self.publish_current_mode()
        
        response = SetBoolResponse()
        response.success = True
        response.message = f"Switched from {old_mode} to {new_mode} impedance mode"
        
        rospy.loginfo(response.message)
        return response
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            # Periodically republish status
            self.publish_current_mode()
            rate.sleep()

if __name__ == '__main__':
    try:
        switcher = DualModeSwitcher()
        switcher.run()
    except rospy.ROSInterruptException:
        pass
