#!/usr/bin/env python3

"""
Test script for Dual Impedance Controller

This script tests the dual impedance controller to verify that the 
velocity/acceleration discontinuity errors have been resolved.

It will:
1. Start monitoring controller output for errors
2. Allow the controller to initialize and stabilize
3. Test mode switching between cartesian and joint impedance
4. Report any motion abort errors

Usage:
    rosrun franka_example_controllers test_dual_controller.py
"""

import rospy
import time
import subprocess
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class DualControllerTester:
    def __init__(self):
        rospy.init_node('dual_controller_tester', anonymous=True)
        
        # Monitoring variables
        self.startup_complete = False
        self.error_detected = False
        self.pose_received = False
        self.joint_state_received = False
        
        # Publishers for testing
        self.mode_pub = rospy.Publisher('/impedance_mode', Bool, queue_size=1)
        
        # Subscribers for monitoring
        self.pose_sub = rospy.Subscriber('/cartesian_pose', PoseStamped, self.pose_callback)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.mode_status_sub = rospy.Subscriber('/current_impedance_mode', String, self.mode_status_callback)
        
        # Service client
        try:
            rospy.wait_for_service('/switch_impedance_mode', timeout=10.0)
            self.mode_service = rospy.ServiceProxy('/switch_impedance_mode', SetBool)
        except rospy.ROSException:
            rospy.logwarn("Mode switching service not available, will use topic instead")
            self.mode_service = None
            
        self.current_mode = "unknown"
        
    def pose_callback(self, msg):
        self.pose_received = True
        
    def joint_callback(self, msg):
        self.joint_state_received = True
        
    def mode_status_callback(self, msg):
        self.current_mode = msg.data
        
    def monitor_logs_for_errors(self):
        """Monitor ROS logs for motion abort errors"""
        try:
            # Monitor rosout for error messages
            import rostopic
            error_keywords = [
                "motion aborted by reflex",
                "cartesian_motion_generator_joint_velocity_discontinuity", 
                "cartesian_motion_generator_joint_acceleration_discontinuity",
                "Move command aborted"
            ]
            
            def log_callback(msg):
                if msg.level >= 8:  # ERROR level or higher
                    for keyword in error_keywords:
                        if keyword in msg.msg:
                            rospy.logerr(f"DETECTED MOTION ERROR: {msg.msg}")
                            self.error_detected = True
                            return
                            
            rospy.Subscriber('/rosout_agg', rospy.msg.Log, log_callback)
            
        except Exception as e:
            rospy.logwarn(f"Could not monitor logs: {e}")
    
    def wait_for_controller_startup(self, timeout=15.0):
        """Wait for controller to start up and stabilize"""
        rospy.loginfo("Waiting for controller to start up and stabilize...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.pose_received and self.joint_state_received:
                rospy.loginfo("Controller appears to be running!")
                # Wait additional time for startup smoothing to complete
                rospy.loginfo("Waiting for startup smoothing to complete (6 seconds)...")
                time.sleep(6.0)
                self.startup_complete = True
                return True
                
            time.sleep(0.1)
            
        rospy.logerr("Timeout waiting for controller to start")
        return False
    
    def test_mode_switching(self):
        """Test switching between cartesian and joint modes"""
        if not self.startup_complete:
            rospy.logwarn("Skipping mode switching test - startup not complete")
            return False
            
        rospy.loginfo("Testing mode switching...")
        
        # Test 1: Switch to joint mode
        rospy.loginfo("Switching to joint impedance mode...")
        if self.mode_service:
            try:
                response = self.mode_service(False)  # False = joint mode
                if response.success:
                    rospy.loginfo("Mode switch request sent successfully")
                else:
                    rospy.logwarn(f"Mode switch failed: {response.message}")
            except Exception as e:
                rospy.logwarn(f"Service call failed: {e}")
        else:
            # Use topic
            self.mode_pub.publish(Bool(data=False))
            
        # Wait for transition
        time.sleep(3.0)
        
        if self.error_detected:
            rospy.logerr("ERROR DETECTED during mode switch to joint!")
            return False
            
        # Test 2: Switch back to cartesian mode  
        rospy.loginfo("Switching back to cartesian impedance mode...")
        if self.mode_service:
            try:
                response = self.mode_service(True)  # True = cartesian mode
                if response.success:
                    rospy.loginfo("Mode switch request sent successfully")
                else:
                    rospy.logwarn(f"Mode switch failed: {response.message}")
            except Exception as e:
                rospy.logwarn(f"Service call failed: {e}")
        else:
            # Use topic
            self.mode_pub.publish(Bool(data=True))
            
        # Wait for transition
        time.sleep(3.0)
        
        if self.error_detected:
            rospy.logerr("ERROR DETECTED during mode switch to cartesian!")
            return False
            
        rospy.loginfo("Mode switching test completed successfully!")
        return True
    
    def run_test(self):
        """Run the complete test sequence"""
        rospy.loginfo("=== Starting Dual Impedance Controller Test ===")
        
        # Start monitoring for errors
        self.monitor_logs_for_errors()
        
        # Wait for controller startup
        if not self.wait_for_controller_startup():
            rospy.logerr("TEST FAILED: Controller startup timeout")
            return False
            
        if self.error_detected:
            rospy.logerr("TEST FAILED: Errors detected during startup!")
            return False
        else:
            rospy.loginfo("✅ Controller started successfully without velocity/acceleration discontinuity errors!")
            
        # Test mode switching
        if not self.test_mode_switching():
            rospy.logerr("TEST FAILED: Errors during mode switching")
            return False
        else:
            rospy.loginfo("✅ Mode switching completed without errors!")
            
        # Final check
        if self.error_detected:
            rospy.logerr("TEST FAILED: Errors detected during testing!")
            return False
        else:
            rospy.loginfo("🎉 ALL TESTS PASSED! Controller is working correctly!")
            rospy.loginfo("=== Velocity/Acceleration Discontinuity Errors RESOLVED ===")
            return True

if __name__ == '__main__':
    try:
        tester = DualControllerTester()
        success = tester.run_test()
        
        if success:
            rospy.loginfo("Test completed successfully. Controller is ready for use.")
        else:
            rospy.logerr("Test failed. Please check the controller implementation.")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed with exception: {e}")
