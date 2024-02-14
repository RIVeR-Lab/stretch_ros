#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
# from hello_helpers import HelloNode
# from util import *
from tf2_ros import Buffer, TransformListener
import yaml
import tf.transformations as tft
from copy import deepcopy
import stretch_body.robot as rb
import std_srvs
    

    
class LocalizeNode():
    def __init__(self):
        rospy.init_node("localize")
        rospy.sleep(2)
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.odom_tf, self.initial_tf = self.get_initial_tf()
        zero_pose = PoseWithCovarianceStamped()
        zero_pose.header.frame_id = 'map'
        zero_pose.pose.pose.position.x = self.odom_tf.transform.translation.x
        zero_pose.pose.pose.position.y = self.odom_tf.transform.translation.y
        zero_pose.pose.pose.position.z = self.odom_tf.transform.translation.z
        zero_pose.pose.pose.orientation.x = self.odom_tf.transform.rotation.x
        zero_pose.pose.pose.orientation.y = self.odom_tf.transform.rotation.y
        zero_pose.pose.pose.orientation.z = self.odom_tf.transform.rotation.z
        zero_pose.pose.pose.orientation.w = self.odom_tf.transform.rotation.w
        
        zero_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        
        zero_pose.header.stamp = rospy.Time.now()
        for i in range(10):
            self.initial_pose_pub.publish(zero_pose)

        # while not rospy.is_shutdown():

        self.odom_tf.header.stamp = rospy.Time.now()
        self.initial_tf.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.odom_tf)
        self.br.sendTransform(self.initial_tf)

        self.rate.sleep()


    def get_initial_tf(self):
        initial_tf_diff = self.tf2_buffer.lookup_transform('original_stretch_head', 'stretch_head', rospy.Time(), rospy.Duration(5.0))
        odom_diff = deepcopy(initial_tf_diff)
        odom_diff.header.stamp = rospy.Time.now()
        odom_diff.child_frame_id = 'odom'
        odom_diff.header.frame_id = 'map'
        odom_diff.transform.translation.z = 0
        initial_tf_diff.child_frame_id = 'initial_stretch_head'
        # self.br.sendTransform(initial_tf_diff)
        print(odom_diff, initial_tf_diff)
        return odom_diff, initial_tf_diff

    

if __name__ == '__main__':
    localize_node = LocalizeNode()
    # print_tf()
    # localize_robot()
    # put_away_node.circle_table([])
    rospy.spin()