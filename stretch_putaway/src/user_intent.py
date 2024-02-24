#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, PolygonStamped, Point32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
# from hello_helpers import HelloNode
# from util import *
from tf2_ros import Buffer, TransformListener
import yaml
import tf.transformations as tft
from copy import deepcopy
import stretch_body.robot as rb
from std_srvs.srv import Empty, Trigger
from threading import Lock, RLock
from stretch_putaway.srv import ObjectRemove, ZonesWithObjects, ZonesWithObjectsResponse, ObjectsInZone, ObjectsInZoneResponse
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from nav_msgs.srv import GetPlan


class UserModel():
    def __init__(self):
        rospy.init_node("user_model")
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.rospack = rospkg.RosPack()

        self.user_frame = 'hololens_head'

        self.obstacle_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        self.obstacle_msg = ObstacleArrayMsg()
        self.obstacle_msg.header.frame_id = 'map'
        self.make_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

    def get_user_path(self, target_name):
        user_tf = self.tf2_buffer.lookup_transform('map', self.user_frame, rospy.Time.now())
        user_pose = self.tf_to_posetamped(user_tf, 'map')

        target_tf = self.tf2_buffer.lookup_transform('map', target_name, rospy.Time.now())
        target_pose = self.tf_to_posetamped(target_tf, 'map')

        user_path = self.make_plan_service(user_pose, target_pose, 0.1)
        user_path_points = user_path.plan.poses

        


    def tf_to_posetamped(self, tf, frame_id):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = tf.translation.x
        pose.pose.position.y = tf.translation.y
        pose.pose.position.z = tf.translation.z
        pose.pose.orientation.x = tf.rotation.x
        pose.pose.orientation.y = tf.rotation.y
        pose.pose.orientation.z = tf.rotation.z
        pose.pose.orientation.w = tf.rotation.w
        return pose


    