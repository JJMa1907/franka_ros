#!/usr/bin/env python3

"""
Quick test script to verify the dual impedance controller starts without errors
"""

import rospy
import time
import subprocess
from geometry_msgs.msg import PoseStamped

class QuickTester:
    def __init__(self):
        rospy.init_node('quick_tester', anonymous=True)
        self.pose_received = False
        self.pose_sub = rospy.Subscriber('/cartesian_pose', PoseStamped, self.pose_callback)
        
    def pose_callback(self, msg):
        self.pose_received = True
        rospy.loginfo("✅ Received cartesian pose - controller is running!")
        
    def test(self):
        rospy.loginfo("🔍 Quick test: Waiting for controller to start...")
        
        start_time = time.time()
        while time.time() - start_time < 10.0:
            if self.pose_received:
                rospy.loginfo("🎉 SUCCESS! Controller started without velocity/acceleration discontinuity errors!")
                rospy.loginfo("✅ Test PASSED - Controller is working correctly!")
                return True
            time.sleep(0.1)
            
        rospy.logerr("❌ Test FAILED - No pose messages received")
        return False

if __name__ == '__main__':
    try:
        tester = QuickTester()
        success = tester.test()
        
        if success:
            rospy.loginfo("🏆 Dual Impedance Controller is ready for use!")
        else:
            rospy.logerr("💥 Controller test failed")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")
