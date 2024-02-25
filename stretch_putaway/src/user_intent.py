#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, PolygonStamped, Point32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import PointCloud, PointCloud2
from sensor_msgs import point_cloud2
import sensor_msgs
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
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class UserModel():
    def __init__(self):
        rospy.init_node("user_model")
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.rospack = rospkg.RosPack()

        self.user_frame = 'base_link'

        self.obstacle_pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
        self.path_pub = rospy.Publisher('/user_path', PointCloud2, queue_size=1)
        self.obstacle_msg = ObstacleArrayMsg()
        self.obstacle_msg.header.frame_id = 'map'
        self.make_plan_service = rospy.ServiceProxy('/user/user_move_base/make_plan', GetPlan)
        self.clear_costmap_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        rospy.sleep(2)

        while not rospy.is_shutdown():
            self.get_user_path('zone_top_left')
            # self.clear_costmap_service()
            self.rate.sleep()

    def get_user_path(self, target_name):
        user_tf = self.tf2_buffer.lookup_transform('map', self.user_frame, rospy.Time()).transform
        user_pose = self.tf_to_posetamped(user_tf, 'map')

        target_tf = self.tf2_buffer.lookup_transform('map', target_name, rospy.Time()).transform
        target_pose = self.tf_to_posetamped(target_tf, 'map')

        self.obstacle_msg = ObstacleArrayMsg()
        self.obstacle_msg.header.frame_id = 'map'
        user_path = self.make_plan_service(user_pose, target_pose, 0.1)
        user_path_points = user_path.plan.poses


        if len(user_path_points) <= 1:
            rospy.loginfo('No path found')
            return

        # Create a PointCloud message
        point_cloud = PointCloud()
        point_cloud.header.frame_id = 'map'
        point_cloud.header.stamp = rospy.Time.now()
        point_cloud.points = []
        p = []
        for i in range(len(user_path_points)):
            point = Point32()
            point.x = user_path_points[i].pose.position.x
            point.y = user_path_points[i].pose.position.y
            point.z = user_path_points[i].pose.position.z
            point_cloud.points.append(point)
            p.append([point.x, point.y, point.z])

        point_cloud2_msg = point_cloud2.create_cloud_xyz32(point_cloud.header, p)
    
        # Publish the point cloud
        # self.clear_costmap_service()
        self.path_pub.publish(point_cloud2_msg)

        
        # for i in range(len(user_path_points) - 1):
        #     start_point = user_path_points[i].pose.position
        #     end_point = user_path_points[i+1].pose.position
        #     self.obstacle_msg.obstacles.append(ObstacleMsg())
        #     self.obstacle_msg.obstacles[i].id = 0
        #     line_start = Point32()
        #     line_start.x = start_point.x
        #     line_start.y = start_point.y
        #     line_end = Point32()
        #     line_end.x = end_point.x
        #     line_end.y = end_point.y
        #     self.obstacle_msg.obstacles[i].polygon.points = [line_start, line_end]
        # self.obstacle_pub.publish(self.obstacle_msg)



        map


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


if __name__ == "__main__":
    user_model = UserModel()
    rospy.spin()