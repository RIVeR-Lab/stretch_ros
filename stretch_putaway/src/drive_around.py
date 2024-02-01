#!/usr/bin/env python3

import rospy
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import Pose, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
from hello_helpers import HelloNode

class PutAwayNode(HelloNode):
    def __init__(self):
        super(PutAwayNode, self).__init__('put_away')
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.visual_servo_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.move_base_goal = MoveBaseGoal()
        self.move_base_simple_goal_publisher = rospy.Publisher('move_base_simple/goal', Pose, queue_size=1)



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
    
    put_away_node.circle_table([])
    rospy.spin()