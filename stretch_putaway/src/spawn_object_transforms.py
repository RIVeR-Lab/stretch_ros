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
from stretch_putaway.srv import ObjectRemove, ZonesWithObjects, ZonesWithObjectsResponse, ObjectsInZone, ObjectsInZoneResponse
    
class ObjectTFNode():
    def __init__(self):
        rospy.init_node("object_tf_spawner")
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.rospack = rospkg.RosPack()
        self.remove_object_service = rospy.Service('remove_object', ObjectRemove, self.remove_object)
        self.get_zones_with_objects_service = rospy.Service('get_zones_with_objects', ZonesWithObjects, self.get_zones_with_objects)
        self.get_objects_in_zone_service = rospy.Service('get_objects_in_zone', ObjectsInZone, self.get_objects_in_zone)

        self.object_list_yaml = rospy.get_param("~object_list_yaml", self.rospack.get_path('stretch_putaway') + '/config/object_locations.yaml')

        self.object_list = {}
        self.object_info_list = {}
        self.dropoff_list = {}
        self.zone_list = {}
        self.zone_objects = {}
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

            for object_name, info in object_list['objects'].items():
                self.object_info_list[object_name] = info
                if info['zone'] not in self.zone_objects:
                    self.zone_objects[info['zone']] = []
                self.zone_objects[info['zone']].append(object_name)

            

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

            object_zone = self.object_info_list[object_name]['zone']
            self.zone_objects[object_zone].remove(object_name)

            return True
        except:
            rospy.ERROR("Failed to remove object " + object_name)
            return False

    def get_zones_with_objects(self, srv):
        zone_list = []
        for zone, objects in self.zone_objects.items():
            if len(objects) > 0:
                zone_list.append(zone)
        return ZonesWithObjectsResponse(zone_list)
    
    def get_objects_in_zone(self, srv):
        zone = srv.zone
        return ObjectsInZoneResponse(self.zone_objects[zone])

        

    

if __name__ == '__main__':
    object_node = ObjectTFNode()
    rospy.spin()