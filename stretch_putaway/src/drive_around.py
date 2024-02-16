#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import tf2_ros
from hello_helpers.hello_misc import HelloNode
# from util import *
from tf2_ros import Buffer, TransformListener
import yaml
import tf.transformations as tft
from copy import deepcopy
import stretch_body.robot as rb
from std_srvs.srv import Trigger
    

    
class PutAwayNode(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'putaway_node', 'putaway_node', wait_for_first_pointcloud=False)
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.visual_servo_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.move_base_goal = MoveBaseGoal()
        self.move_base_simple_goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)


        self.stow_the_robot()
        rospy.sleep(4)

        self.robot = rb.Robot()
        left_table_pose = Pose()
        left_table_pose.position.x = 0.75
        left_table_pose.position.y = 2
        left_table_pose.orientation.w = 1
        self.move_base(left_table_pose)
        self.look_to_side()
        self.grasp_object()

        # self.circle_table()

    def move_base(self, goal_pose):
        self.move_base_client.wait_for_server()
        self.move_base_goal.target_pose.header.frame_id = "map"
        self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_goal.target_pose.pose = goal_pose
        print("Sending goal")
        self.move_base_client.send_goal(self.move_base_goal)
        self.move_base_client.wait_for_result()
        return self.move_base_client.get_result()
    

    def circle_table(self):
        table_poses = [None] * 2
        table_poses[0] = Pose()
        table_poses[0].position.x = 2
        table_poses[0].orientation.w = 1
        table_poses[1] = Pose()
        table_poses[1].position.x = -1
        table_poses[1].position.y = 2
        table_poses[1].orientation.w = 1
        for pose in table_poses:
            print('moving to pose', pose)
            self.move_base(pose)

    def look_to_side(self):
        HelloNode.move_to_pose(self, {'joint_head_pan': -1.5,
                                      'joint_head_tilt': -0.4})

    def grasp_object(self, arm_length=0.4):
        HelloNode.move_to_pose(self, {'joint_wrist_roll': 0.0,
                                      'joint_lift': 1.05})
        
        HelloNode.move_to_pose(self, {'joint_arm': arm_length,
                                      'joint_gripper_finger_left' : 0.25,
                                      'joint_wrist_yaw': 0.0,
                                      'joint_wrist_pitch' : -0.5})
        
        # HelloNode.move_to_pose(self, {'joint_wrist_pitch': -0.5})

        # Close the gripper and wait for grasp
        HelloNode.move_to_pose(self, {'joint_gripper_finger_left' : -0.25})
        rospy.sleep(3)
        # HelloNode.move_to_pose(self, {'joint_wrist_pitch': 0})
        # HelloNode.move_to_pose(self, {'joint_arm' : 0,
        #                               'joint_wrist_yaw': 3.0})


    

if __name__ == '__main__':
    put_away_node = PutAwayNode()
    # print_tf()
    # localize_robot()
    # put_away_node.circle_table([])
    rospy.spin()