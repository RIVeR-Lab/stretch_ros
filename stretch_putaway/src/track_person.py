#!/usr/bin/env python3

import rospy
import numpy as np
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospkg
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from itertools import compress
from hello_helpers.hello_misc import HelloNode
import math
from std_srvs.srv import Trigger

class PersonTracker(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'person_tracker', 'person_tracker', wait_for_first_pointcloud=False)
        self.rospack = rospkg.RosPack()
        self.tracking_frame = rospy.get_param("~tracking_frame", "hololens")
        self.tracking_frame = "hololens"
        self.tracking = True
        rospy.Service('track_person', Trigger, self.track_person)
        rospy.Service('stop_tracking', Trigger, self.stop_tracking)
        
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

    def track_person(self, req):
        self.tracking = True
        return []
    
    def stop_tracking(self, req):
        self.tracking = False
        return []

    # Move the camera to look at a specific frame
    def look_at(self, look_at_frame_name='hololens'):
        print("Waiting for transform")
        look_at_tf = HelloNode.get_tf(self, 'static_camera_link', look_at_frame_name).transform
        yaw = math.atan2(look_at_tf.translation.x, look_at_tf.translation.y)
        pitch = math.atan2(math.hypot(look_at_tf.translation.x, look_at_tf.translation.y)
                           , look_at_tf.translation.z)

        yaw = -yaw
        pitch = 1.57 - pitch
        print("Look at rotations", yaw, pitch)

        if yaw < -3.54 or yaw > 1.67:
            # Yaw is out of range, don't move the head
            print("Yaw out of range")
            yaw = max(-3.54, min(1.67, yaw))
        if pitch < -1.5 or pitch > 0.4:
            print("Pitch out of range")

            pitch = max(-1.5, min(0.4, pitch))
        HelloNode.move_to_pose(self, {'joint_head_pan': yaw,
                                      'joint_head_tilt': pitch})
        


    def run(self):
        while not rospy.is_shutdown():
            if self.tracking:
                self.look_at(self.tracking_frame)
            rospy.sleep(0.1)



if __name__ == '__main__':
    mediapipe_detector = PersonTracker()
    mediapipe_detector.run()
    rospy.spin()