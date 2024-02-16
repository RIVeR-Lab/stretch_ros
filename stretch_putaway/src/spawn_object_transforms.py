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

    
class ObjectTFNode():
    def __init__(self):
        rospy.init_node("object_tf_spawner")
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.object_list = {}
        self.initialize_object_list()
        self.object_list_lock = RLock()


        while not rospy.is_shutdown():
            with self.object_list_lock:
                for object_name, tf in self.object_list:
                    tf.header.stamp = rospy.Time.now()
                    self.br.sendTransform(tf)

            
            self.rate.sleep()
    
    def initialize_object_list(self):
        pass

    def remove_object(self, object_name):
        with self.object_list_lock:
            self.object_list.pop(object_name)
        

    

if __name__ == '__main__':
    object_node = ObjectTFNode()
    rospy.spin()