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


# Keypoint IDs from https://developers.google.com/mediapipe/solutions/vision/pose_landmarker
# 0 - nose
# 1 - left eye (inner)
# 2 - left eye
# 3 - left eye (outer)
# 4 - right eye (inner)
# 5 - right eye
# 6 - right eye (outer)
# 7 - left ear
# 8 - right ear
# 9 - mouth (left)
# 10 - mouth (right)
# 11 - left shoulder
# 12 - right shoulder
# 13 - left elbow
# 14 - right elbow
# 15 - left wrist
# 16 - right wrist
# 17 - left pinky
# 18 - right pinky
# 19 - left index
# 20 - right index
# 21 - left thumb
# 22 - right thumb
# 23 - left hip
# 24 - right hip
# 25 - left knee
# 26 - right knee
# 27 - left ankle
# 28 - right ankle
# 29 - left heel
# 30 - right heel
# 31 - left foot index
# 32 - right foot index


class MediapipeDetector:
    def __init__(self):
        rospy.init_node('mediapipe_detector')
        self.rospack = rospkg.RosPack()
        base_options = python.BaseOptions(model_asset_path=self.rospack.get_path('stretch_putaway') + '/config/pose_landmarker_full.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True
        )
        self.detector = vision.PoseLandmarker.create_from_options(options)
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Keypoint IDs to track
        self.keypoint_ids = [0, 11, 12, 13, 14, 15, 16, 23, 24]
        self.keypoint_mask = np.zeros(33, dtype=bool)
        self.keypoint_mask[self.keypoint_ids] = True



        self.color_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_image_callback)

    def draw_landmarks_on_image(self, rgb_image, detection_result):
        pose_landmarks_list = detection_result.pose_landmarks
        annotated_image = np.copy(rgb_image)

        # Loop through the detected poses to visualize.
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]

            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
            annotated_image,
            pose_landmarks_proto,
            solutions.pose.POSE_CONNECTIONS,
            solutions.drawing_styles.get_default_pose_landmarks_style())
        return annotated_image


    def color_image_callback(self, image_msg):
        color_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        color_image = cv2.rotate(color_image, cv2.ROTATE_90_CLOCKWISE)

        mp_image = mp.Image(image_format = mp.ImageFormat.SRGB, data=color_image)
        results = self.detector.detect(mp_image)

        if results.pose_world_landmarks:
            self.publish_joint_tfs(results.pose_world_landmarks[0])

        annotated_image = self.draw_landmarks_on_image(mp_image.numpy_view(), results)
 
        cv2.imshow('color_image', annotated_image)
        cv2.waitKey(1)

    def publish_joint_tfs(self, world_landmarks):
        for i, pose_landmarks in enumerate(world_landmarks):
            if pose_landmarks.visibility < 0.95 or not self.keypoint_mask[i]:
                continue
            print('Pose', i, pose_landmarks)
            tf = TransformStamped()
            tf.header.frame_id = 'camera_color_optical_frame'
            tf.child_frame_id = f'pose_{i}'
            tf.header.stamp = rospy.Time.now()
            tf.transform.translation.x = pose_landmarks.y
            tf.transform.translation.y = -pose_landmarks.z
            tf.transform.translation.z = pose_landmarks.x
            tf.transform.rotation.w = 1
            self.tf_broadcaster.sendTransform(tf)


if __name__ == '__main__':
    mediapipe_detector = MediapipeDetector()
    rospy.spin()