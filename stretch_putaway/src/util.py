import rospy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

def localize_robot(map_initial_tf_file, current_tf):
    ''' Calculate the different between the initial map transform and the current transform
        Transforms are taken with respect from the Optitrack stretch_head to the Optitrack table
        The initial transform is taken from the initial map transform file, which stores the starting point of the robot when mapping was done
        returns a transform to be set as the position of the robot in the map'''
    rospy.loginfo('Localizing robot')
    tf_diff = TransformStamped()