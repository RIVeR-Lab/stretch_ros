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
from std_srvs.srv import Empty, Trigger
from threading import Lock, RLock
from stretch_putaway.srv import ObjectRemove
    
class ObjectTFNode():
    def __init__(self):
        rospy.init_node("object_tf_spawner")
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.rospack = rospkg.RosPack()
        self.remove_object_service = rospy.Service('remove_object', ObjectRemove, self.remove_object)

        self.object_list_yaml = rospy.get_param("~object_list_yaml", self.rospack.get_path('stretch_putaway') + '/config/object_locations.yaml')

        self.object_list = {}
        self.dropoff_list = {}
        self.zone_list = {}
        self.initialize_object_list(self.object_list_yaml)
        self.object_list_lock = RLock()


        while not rospy.is_shutdown():
            with self.object_list_lock:
                for object_name, tf in self.object_list.items():
                    tf.header.stamp = rospy.Time.now()
                    self.br.sendTransform(tf)
            for color, tf in self.dropoff_list.items():
                tf.header.stamp = rospy.Time.now()
                self.br.sendTransform(tf)
            for zone, tf in self.zone_list.items():
                tf.header.stamp = rospy.Time.now()
                self.br.sendTransform(tf)

            
            self.rate.sleep()
    
    def initialize_object_list(self, object_list_yaml):
        with open(object_list_yaml, 'r') as f:
            object_list = yaml.safe_load(f)
            # print(object_list)
            self.frame_id = object_list['frame']
            print(len(object_list['objects'].items()))
            self.load_item_list(object_list['objects'].items(), self.object_list)
            self.load_item_list(object_list['dropoffs'].items(), self.dropoff_list)
            self.load_item_list(object_list['zones'].items(), self.zone_list)

    def load_item_list(self, list, my_dict):
        for name, info in list:
            my_dict[name] = TransformStamped()
            my_dict[name].header.frame_id = self.frame_id
            my_dict[name].child_frame_id = name
            my_dict[name].transform.translation.x = info['location'][0]
            my_dict[name].transform.translation.y = info['location'][1]
            my_dict[name].transform.translation.z = info['location'][2]
            
            object_quat = tft.quaternion_from_euler(0.0, 0.0, info['orientation_z'])
            my_dict[name].transform.rotation.x = object_quat[0]
            my_dict[name].transform.rotation.y = object_quat[1]
            my_dict[name].transform.rotation.z = object_quat[2]
            my_dict[name].transform.rotation.w = object_quat[3]



    def remove_object(self, srv):
        try:
            object_name = srv.object_name
            with self.object_list_lock:
                self.object_list.pop(object_name)
            return True
        except:
            rospy.ERROR("Failed to remove object " + object_name)
            return False

        

    

if __name__ == '__main__':
    object_node = ObjectTFNode()
    rospy.spin()