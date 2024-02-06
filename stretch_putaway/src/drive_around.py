#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import Pose, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
# from hello_helpers import HelloNode
# from util import *
from tf2_ros import Buffer, TransformListener
import yaml
import tf.transformations as tft

def localize_robot(map_initial_tf_file='map_starting_tf.yaml'):
    ''' Calculate the different between the initial map transform and the current transform
        Transforms are taken with respect from the Optitrack stretch_head to the Optitrack table
        The initial transform is taken from the initial map transform file, which stores the starting point of the robot when mapping was done
        returns a transform to be set as the position of the robot in the map'''
    rospy.loginfo('Localizing robot')
    tf2_buffer = Buffer()
    tf2_listener = TransformListener(tf2_buffer)
    rospack = rospkg.RosPack()
    with open(rospack.get_path('stretch_putaway') + '/config/' + map_initial_tf_file, 'r') as f:
        map_initial_tf_data = yaml.safe_load(f)
    print(map_initial_tf_data)
    initial_tf = TransformStamped()
    initial_tf.transform.translation.x = map_initial_tf_data['transform']['translation']['x']
    initial_tf.transform.translation.y = map_initial_tf_data['transform']['translation']['y']
    initial_tf.transform.translation.z = map_initial_tf_data['transform']['translation']['z']
    initial_tf.transform.rotation.x = map_initial_tf_data['transform']['rotation']['x']
    initial_tf.transform.rotation.y = map_initial_tf_data['transform']['rotation']['y']
    initial_tf.transform.rotation.z = map_initial_tf_data['transform']['rotation']['z']
    initial_tf.transform.rotation.w = map_initial_tf_data['transform']['rotation']['w']
    

    # Get current tf from stretch_head to table
    current_tf = None
    while current_tf is None:
        try:
            current_tf = tf2_buffer.lookup_transform('table', 'stretch_head', rospy.Time())
        except:
            continue

    tf_diff = TransformStamped()
    tf_diff.transform.translation.x = current_tf.transform.translation.x - initial_tf.transform.translation.x
    tf_diff.transform.translation.y = current_tf.transform.translation.y - initial_tf.transform.translation.y
    tf_diff.transform.translation.z = current_tf.transform.translation.z - initial_tf.transform.translation.z
    quat_diff = tft.quaternion_multiply([current_tf.transform.rotation.x, current_tf.transform.rotation.y, current_tf.transform.rotation.z, current_tf.transform.rotation.w], 
                                        tft.quaternion_inverse([initial_tf.transform.rotation.x, initial_tf.transform.rotation.y, initial_tf.transform.rotation.z, initial_tf.transform.rotation.w]))
    tf_diff.transform.rotation.x = quat_diff[0]
    tf_diff.transform.rotation.y = quat_diff[1]
    tf_diff.transform.rotation.z = quat_diff[2]
    tf_diff.transform.rotation.w = quat_diff[3]

    print(tf_diff)
    return tf_diff

    

def print_tf(table_frame='table', stretch_frame='stretch_head'):
    tf2_buffer = Buffer()
    tf2_listener = TransformListener(tf2_buffer)

    while not rospy.is_shutdown():
        try:
            tf = tf2_buffer.lookup_transform(table_frame, stretch_frame, rospy.Time())
            print(tf)
        except:
            continue
    
class PutAwayNode():
    def __init__(self):
        # super(PutAwayNode, self).__init__('put_away')
        rospy.init_node("drive_around")
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.visual_servo_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.move_base_goal = MoveBaseGoal()
        self.move_base_simple_goal_publisher = rospy.Publisher('move_base_simple/goal', Pose, queue_size=1)

        self.tf_diff = localize_robot()
        self.tf_diff.transform.translation.z = 0
        # self.tf2_buffer.set_transform(self.tf_diff, 'stretch_head', rospy.Time.now(), 'map', 'stretch_head')
        self.br.sendTransform(self.tf_diff)


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
    localize_robot()
    # put_away_node.circle_table([])
    rospy.spin()