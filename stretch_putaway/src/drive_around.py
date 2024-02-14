#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
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
    

    
class PutAwayNode():
    def __init__(self):
        # super(PutAwayNode, self).__init__('put_away')
        rospy.init_node("drive_around")
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.visual_servo_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.move_base_goal = MoveBaseGoal()
        self.move_base_simple_goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        rospy.wait_for_service('/stow_the_robot')
        self.stow_the_robot = rospy.ServiceProxy('/stow_the_robot', std_srvs.Trigger)
        self.stow_the_robot()

        self.robot = rb.Robot()

        map_pose = Pose()
        self.move_base(map_pose)

    def move_base(self, goal_pose):
        self.move_base_client.wait_for_server()
        self.move_base_goal.target_pose.header.frame_id = "map"
        self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_goal.target_pose.pose = goal_pose
        self.move_base_client.send_goal(self.move_base_goal)
        self.move_base_client.wait_for_result()
        return self.move_base_client.get_result()
    
    

    def circle_table(self, table_poses):
        for pose in table_poses:
            self.move_base(pose)
            rospy.sleep(5)


    

if __name__ == '__main__':
    put_away_node = PutAwayNode()
    # print_tf()
    # localize_robot()
    # put_away_node.circle_table([])
    rospy.spin()