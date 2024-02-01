#!/usr/bin/env python3

import rospy
import actionlib
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

def foo():
    rospy.init_node("navigation", anonymous=True)
    rate = rospy.Rate(10) # 10hz
    head_scan = True
    while not rospy.is_shutdown():
        if head_scan:
            rospy.wait_for_service('/funmap/trigger_head_scan')
            rospy.loginfo("trigger head scan")
            trigger_head_scan = rospy.ServiceProxy('/funmap/trigger_head_scan', Trigger)
            result = trigger_head_scan()
            rospy.loginfo(str(result))
            if result.success:
                head_scan = False
                rospy.loginfo("success")
        else:
            rospy.wait_for_service('/funmap/trigger_drive_to_scan')
            rospy.loginfo("trigger drive")
            trigger_drive_to_head_scan = rospy.ServiceProxy('/funmap/trigger_drive_to_scan', Trigger)
            result = trigger_drive_to_head_scan()
            rospy.loginfo(str(result))
            if result.success:
                head_scan = True
                rospy.loginfo("success")
        rate.sleep()

if __name__ == "__main__":
    foo()
