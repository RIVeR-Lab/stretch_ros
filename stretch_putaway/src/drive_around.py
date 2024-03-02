#!/usr/bin/env python3

import rospy
import rospkg
import actionlib
import py_trees
from actionlib_msgs.msg import GoalStatus
from math import sqrt, pow
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image
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
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import String
import math    
from tf2_geometry_msgs import PoseStamped as TF2PoseStamped
from stretch_putaway.srv import ObjectRemove, ZonesWithObjects, ObjectsInZone, ZonesWithObjectsResponse, ObjectsInZoneResponse
import time
from image_geometry import PinholeCameraModel
import cv_bridge
import cv2
    
class PutAwayNode(HelloNode):
    def __init__(self):
        HelloNode.__init__(self)
        HelloNode.main(self, 'putaway_node', 'putaway_node', wait_for_first_pointcloud=False)
        self.rate = rospy.Rate(10)
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.rospack = rospkg.RosPack()
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.visual_servo_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.move_base_goal = MoveBaseGoal()
        # self.move_base_simple_goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=1)
        self.navigation_mode_service = rospy.ServiceProxy('switch_to_navigation_mode', Trigger)
        self.position_mode_service = rospy.ServiceProxy('switch_to_position_mode', Trigger)
        self.cam_model = PinholeCameraModel()
        self.cam_model.fromCameraInfo(rospy.wait_for_message('/camera/color/camera_info', CameraInfo))
        self.bridge = cv_bridge.CvBridge()

        # Load in object specific information and set up service to remove objects once picked up
        self.object_list_yaml = rospy.get_param("~object_list_yaml", self.rospack.get_path('stretch_putaway') + '/config/object_locations.yaml')
        self.object_info = {}
        self.initialize_object_list(self.object_list_yaml)
        self.remove_object_service = rospy.ServiceProxy('remove_object', ObjectRemove)
        self.clear_costmap_service = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
        self.get_zones_with_objects_service = rospy.ServiceProxy('get_zones_with_objects', ZonesWithObjects)
        self.get_objects_in_zone_service = rospy.ServiceProxy('get_objects_in_zone', ObjectsInZone)
        self.get_all_objects_service = rospy.ServiceProxy('get_all_objects', ObjectsInZone)
        self.make_plan_service = rospy.ServiceProxy('move_base/GlobalPlanner/make_plan', GetPlan)

        self.high_stow()
        rospy.sleep(2)


        self.robot = rb.Robot()
        # self.pickup_object('object_1')
        # self.get_closest_object()
        blue_tf = HelloNode.get_tf(self, 'map', 'blue').transform
        blue_pose = self.tf_to_posetamped(blue_tf, 'map').pose
        green_tf = HelloNode.get_tf(self, 'map', 'green').transform
        green_pose = self.tf_to_posetamped(green_tf, 'map').pose
        red_tf = HelloNode.get_tf(self, 'map', 'red').transform
        red_pose = self.tf_to_posetamped(red_tf, 'map').pose
        yellow_tf = HelloNode.get_tf(self, 'map', 'yellow').transform
        yellow_pose = self.tf_to_posetamped(yellow_tf, 'map').pose

        
        while not rospy.is_shutdown():
            start_time = time.time()
            target_object_name = self.get_closest_object()
            print("Closest object", target_object_name)
            print("Time to get closest object", time.time() - start_time)
            
            self.pickup_object(target_object_name)
            

    # Load in object information, such as color and arm extension length from the object yaml file
    def initialize_object_list(self, object_list_yaml):
        with open(object_list_yaml, 'r') as f:
            object_list = yaml.safe_load(f)
            for object_name, info in object_list['objects'].items():
                self.object_info[object_name] = {'color': info['color'],
                                                 'arm_length': info['arm_length'],}


    def pickup_object(self, object_name):
        # TODO estimate human intent before picking up object
        print("Picking up object", object_name)
        print("Object info ", self.object_info[object_name])

        # Poses are robot base positions in the map frame. Arm length is part of the object yaml file.
        object_base_tf = HelloNode.get_tf(self, 'map', object_name).transform
        object_base_pose = Pose()
        object_base_pose.position.x = object_base_tf.translation.x
        object_base_pose.position.y = object_base_tf.translation.y
        object_base_pose.orientation.x = object_base_tf.rotation.x
        object_base_pose.orientation.y = object_base_tf.rotation.y
        object_base_pose.orientation.z = object_base_tf.rotation.z
        object_base_pose.orientation.w = object_base_tf.rotation.w
        arm_length = self.object_info[object_name]['arm_length']
        arm_length_diff = self.move_base_accurate(object_base_pose)
        print("Arm length diff", arm_length_diff)
        self.grasp_object(arm_length = arm_length - arm_length_diff)

        self.remove_object_service(object_name)

        # Drive to dropoff location and release
        # dropoff_base_tf = HelloNode.get_tf(self, 'map', self.object_info[object_name]['color']).transform
        # dropoff_base_pose = Pose()
        # dropoff_base_pose.position.x = dropoff_base_tf.translation.x
        # dropoff_base_pose.position.y = dropoff_base_tf.translation.y
        # dropoff_base_pose.orientation.x = dropoff_base_tf.rotation.x
        # dropoff_base_pose.orientation.y = dropoff_base_tf.rotation.y
        # dropoff_base_pose.orientation.z = dropoff_base_tf.rotation.z
        # dropoff_base_pose.orientation.w = dropoff_base_tf.rotation.w
        # self.move_base(dropoff_base_pose)
        # arm_length_diff = self.fine_tune_position(dropoff_base_pose)


        # self.release_object(arm_length_diff)
        self.high_stow()

    # Move the base using the move_base action server  
    def move_base(self, goal_pose):
        self.clear_costmap_service()
        rospy.sleep(1)
        self.move_base_client.wait_for_server()
        self.move_base_goal.target_pose.header.frame_id = "map"
        self.move_base_goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base_goal.target_pose.pose = goal_pose
        print("Sending goal", goal_pose)
        self.move_base_client.send_goal(self.move_base_goal)
        self.move_base_client.wait_for_result()
        return 0

    # Move the base to the goal pose, but use optitrack to fine tune the position
    def move_base_accurate(self, goal_pose):
        self.move_base(goal_pose)

        # Wait for the robot to reach the goal pose, and for the head to stop moving
        rospy.sleep(3)

        rospy.loginfo("Fine tuning with optitrack localization")
        self.position_mode_service()

        stretch_opti_tf = HelloNode.get_tf(self, 'original_stretch_head', 'stretch_head').transform

        # Drive directly at the target
        goal_pose_stamped = TF2PoseStamped()
        goal_pose_stamped.header.frame_id = "original_stretch_head"
        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_stamped.pose = goal_pose
        goal_pose_stretch_opti_frame = self.tf2_buffer.transform(goal_pose_stamped, 'stretch_head', rospy.Duration(3))
        print("Offset = ", goal_pose_stretch_opti_frame)

        face_goal_pose_angle = math.atan2(goal_pose_stretch_opti_frame.pose.position.y, 
                                          goal_pose_stretch_opti_frame.pose.position.x)
        backwards = False
        if face_goal_pose_angle < -1.57:
            face_goal_pose_angle = 3.14 + face_goal_pose_angle
            backwards = True
        elif face_goal_pose_angle > 1.57:
            face_goal_pose_angle = 3.14 - face_goal_pose_angle
            backwards = True
        
        print("Rotating to face mobile base by ", face_goal_pose_angle)
        input("Press Enter to Continue...")
        HelloNode.move_to_pose(self, {'rotate_mobile_base': face_goal_pose_angle})
        rospy.sleep(2)

        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_stretch_opti_frame = self.tf2_buffer.transform(goal_pose_stamped, 'stretch_head', rospy.Duration(3))
        goal_pose_offset = math.hypot(goal_pose_stretch_opti_frame.pose.position.y,
                                      goal_pose_stretch_opti_frame.pose.position.x)
        if backwards:
            goal_pose_offset = -goal_pose_offset
        print("Driving to move towards goal pose ", goal_pose_offset)
        input("Press Enter to Continue...")
        HelloNode.move_to_pose(self, {'translate_mobile_base': goal_pose_offset})
        rospy.sleep(2)

        # After driving directly at the goal pose, rotate to match the orientation of the goal pose
        stretch_opti_tf = HelloNode.get_tf(self, 'original_stretch_head', 'stretch_head').transform
        stretch_opti_quaternion_inv = [stretch_opti_tf.rotation.x, stretch_opti_tf.rotation.y, stretch_opti_tf.rotation.z, -stretch_opti_tf.rotation.w]
        goal_pose_quaternion = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]

        quat_diff = tft.quaternion_multiply(goal_pose_quaternion, stretch_opti_quaternion_inv)
        
        rotation_diff = tft.euler_from_quaternion(quat_diff)
        print("Rotation diff", rotation_diff)
        # return
        HelloNode.move_to_pose(self, {'rotate_mobile_base': rotation_diff[2]})
        rospy.sleep(2)

        # Assume orientation is correct, now move forward or backward to the correct position
        goal_pose_stamped = TF2PoseStamped()
        goal_pose_stamped.header.frame_id = "original_stretch_head"
        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_stamped.pose = goal_pose
        goal_pose_stretch_opti_frame = self.tf2_buffer.transform(goal_pose_stamped, 'stretch_head', rospy.Duration(3))
        translation_diff = goal_pose_stretch_opti_frame.pose.position.x
        print("Translation diff (x)" , translation_diff)
        HelloNode.move_to_pose(self, {'translate_mobile_base': translation_diff})
        stretch_opti_tf = HelloNode.get_tf(self, 'original_stretch_head', 'stretch_head').transform 

        # Output position in difference after translation (Debugging purposes)
        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_stretch_opti_frame_post = self.tf2_buffer.transform(goal_pose_stamped, 'stretch_head', rospy.Duration(3))
        print("Post translation", goal_pose_stretch_opti_frame_post.pose.position.x)

        # Switch back to navigation mode once position is fine tuned
        self.navigation_mode_service()

        # Return Y offset to account for arm length difference. Taken with respect to the robot head
        # If y offset is positive, that means goal is to the left which means the robot is closer to the table
        return goal_pose_stretch_opti_frame.pose.position.y
    
    # Fine tune the position of the robot to the goal pose only using robot localization information
    def fine_tune_position(self, goal_pose):
        self.position_mode_service()

        # Rotate to match the  goal pose orientation
        current_pose = HelloNode.get_tf(self, 'map', 'base_link').transform
        goal_pose_quaternion = [goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w]
        current_pose_quaternion_inv = [current_pose.rotation.x, current_pose.rotation.y, current_pose.rotation.z, -current_pose.rotation.w]
        quat_diff = tft.quaternion_multiply(goal_pose_quaternion, current_pose_quaternion_inv)
        rotation_diff = tft.euler_from_quaternion(quat_diff)
        print("Rotation diff", rotation_diff)
        HelloNode.move_to_pose(self, {'rotate_mobile_base': rotation_diff[2]})
        rospy.sleep(2)
        
        # Assume orientation is correct, now move forward or backward to the correct position
        goal_pose_stamped = TF2PoseStamped()
        goal_pose_stamped.header.frame_id = "map"
        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_stamped.pose = goal_pose
        goal_pose_base_frame = self.tf2_buffer.transform(goal_pose_stamped, 'base_link', rospy.Duration(3))
        translation_diff = goal_pose_base_frame.pose.position.x
        print("Translation diff (x)" , translation_diff)
        HelloNode.move_to_pose(self, {'translate_mobile_base': translation_diff})
        goal_pose_stamped.header.stamp = rospy.Time.now()
        goal_pose_base_frame_post = self.tf2_buffer.transform(goal_pose_stamped, 'base_link', rospy.Duration(3))
        print("Post translation", goal_pose_base_frame_post.pose.position.x)

        self.navigation_mode_service()

        return goal_pose_base_frame.pose.position.y

    # Returns the name of the closest object by euclidian distance from the robot to object and from the object to dropoff
    def get_closest_object(self):
        all_object_names = self.get_all_objects_service('').object_names
        min_object_name = ''
        min_dist = 100000000
        for object_name in all_object_names:
            robot_to_object = HelloNode.get_tf(self, 'base_link', object_name)
            object_to_dropoff = HelloNode.get_tf(self, object_name, self.object_info[object_name]['color'])
            dist = sqrt(pow(robot_to_object.transform.translation.x, 2) + pow(robot_to_object.transform.translation.y, 2)) + \
                     sqrt(pow(object_to_dropoff.transform.translation.x, 2) + pow(object_to_dropoff.transform.translation.y, 2))
            if dist < min_dist:
                min_dist = dist
                min_object_name = object_name
        return min_object_name
        

    
    # Returns the name of the closest tf to the robot in terms of path steps
    # This can either be useds to get the closest object or the closest zone
    # First use closest zone, then get closest object in that zone
    def get_closest_object_by_path(self, target_tf_names):
        # First get the current robot position
        rospy.loginfo("Getting robot position")
        robot_base_tf = HelloNode.get_tf(self, 'map', 'base_link').transform
        robot_base_pose = self.tf_to_posetamped(robot_base_tf, 'map')
        
        min_steps = 100000
        global_costmap = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid)
        closest_zone = None
        print("Costmap", global_costmap.info)
        print("Costmap data", len(global_costmap.data))
        for tf_name in target_tf_names:
            rospy.loginfo("Getting tf position " + tf_name)
            base_tf = HelloNode.get_tf(self, 'map', tf_name).transform
            base_pose = self.tf_to_posetamped(base_tf, 'map')

            # Check if goal pose is occupied
            goal_x = base_tf.translation.x
            goal_y = base_tf.translation.y
            goal_index = int((goal_y - global_costmap.info.origin.position.y) / global_costmap.info.resolution) * global_costmap.info.width \
                        + int((goal_x - global_costmap.info.origin.position.x) / global_costmap.info.resolution)
            print("Goal index", goal_index)
            print(global_costmap.data[goal_index])
            if global_costmap.data[goal_index] > 0:
                print("TF is occupied, continuing")
                continue

            start_time = time.time()

            # Calculate plan to each object and take the shortest one
            tf_plan = self.make_plan_service(robot_base_pose, base_pose, 0.1)

            end_time = time.time()
            print("Time to get plan", end_time - start_time)
            print("Object plan for object", tf_name, len(tf_plan.plan.poses))
            if len(tf_plan.plan.poses) < min_steps:
                min_steps = len(tf_plan.plan.poses)
                closest_tf = tf_name

        if closest_tf is None:
            rospy.loginfo("No objects found, will resort to greedy approach")
            return self.get_closest_object()
        
        return closest_tf
                
    # Look at an object's tf an get the color values at that location
    def check_object_presence(self, object_name):
        object_tf = HelloNode.get_tf(self, 'camera_color_optical_frame', object_name).transform
        if object_tf is None:
            return False
        camera_image = rospy.wait_for_message('/camera/color/image_raw', Image)
        cv_image = self.bridge.imgmsg_to_cv2(camera_image, "bgr8")
        pt_cv = self.cam_model.project3dToPixel((object_tf.translation.x, object_tf.translation.y, object_tf.translation.z))
        print("Pixel location", pt_cv)
        cv2.circle(cv_image, (int(pt_cv[0]), int(pt_cv[1])), 5, (0, 0, 255), -1)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(0)

        

        
    # Move the camera head to look at a specific transform
    def look_at(self, look_at_frame_name='hololens_head'):
        print("Waiting for transform")
        look_at_tf = HelloNode.get_tf(self, 'static_camera_link', look_at_frame_name).transform
        yaw = math.atan2(look_at_tf.translation.x, look_at_tf.translation.y)
        pitch = math.atan2(math.hypot(look_at_tf.translation.x, look_at_tf.translation.y)
                           , look_at_tf.translation.z)

        yaw = -yaw
        pitch = 1.57 - pitch
        print("Look at rotations", yaw, pitch)

        if yaw < -3.54 or yaw > 1.67:
            # Yaw is out of range, don't move the head
            print("Yaw out of range")
            return
        if pitch < -1.5 or pitch > 1.5:
            print("Pitch out of range")
            return
        HelloNode.move_to_pose(self, {'joint_head_pan': yaw,
                                      'joint_head_tilt': pitch})


    # Reach arm to pre-defined height and wrist positions. Arm length is determined based on robot position
    def grasp_object(self, arm_length=0.4):
        HelloNode.move_to_pose(self, {'joint_wrist_roll': 0.0,
                                      'joint_lift': 1.02})
        HelloNode.move_to_pose(self, {'joint_wrist_pitch': -0.5,
                                      'joint_wrist_yaw': 0.0})
        
        HelloNode.move_to_pose(self, {'joint_arm': arm_length,
                                      'joint_gripper_finger_left' : 0.25,
                                      'joint_wrist_yaw': 0.0,
                                      'joint_wrist_pitch' : -0.5})
        
        input("Press Enter to Continue...")
        # Close the gripper and wait for grasp
        HelloNode.move_to_pose(self, {'joint_gripper_finger_left' : -0.25})
        rospy.sleep(2)
        self.high_stow()

    # Release the object
    def release_object(self, arm_length_diff=0.0):
        HelloNode.move_to_pose(self, {'joint_lift': 0.7,
                                      'joint_arm': 0.3 - arm_length_diff,
                                      'joint_wrist_yaw': 0.0})
        HelloNode.move_to_pose(self, {'joint_gripper_finger_left' : 0.25})
        rospy.sleep(2)
        self.high_stow()

    # Stow position, but higher for faster task completion
    def high_stow(self):
        HelloNode.move_to_pose(self, {'joint_wrist_pitch': 0})
        HelloNode.move_to_pose(self, {'joint_arm' : 0,
                                      'joint_lift': 1.05,
                                      'joint_wrist_yaw': 3.0,
                                      'joint_gripper_finger_left' : 0})
        
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


    

if __name__ == '__main__':
    put_away_node = PutAwayNode()
    # print_tf()
    # localize_robot()
    # put_away_node.circle_table([])
    rospy.spin()