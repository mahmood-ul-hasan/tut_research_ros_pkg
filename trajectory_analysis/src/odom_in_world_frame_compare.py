#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_inverse
from geometry_msgs.msg import Pose, Quaternion

initial_orientation = None
initial_position = None
global roll, pitch, yaw
roll =0
pitch = 0
yaw = 0



def gt_odom_callback(odom):
    global roll, pitch, yaw
     
    original_quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,)
    original_rpy = euler_from_quaternion(original_quaternion)
    roll =  original_rpy[0] 
    pitch = original_rpy[1] 
    yaw = original_rpy[2]   

    
def vins_odom_callback(odom):
    global roll, pitch, yaw
     
    original_quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,)
    original_rpy = euler_from_quaternion(original_quaternion)

# convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()
    rpy_odom.header.frame_id = "world"
    rpy_odom.pose.pose.orientation.x = (roll - original_rpy[0]) * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = (pitch - original_rpy[1]) * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = (yaw - original_rpy[2]) * 180 / math.pi
    vins_odom_rpy_pub.publish(rpy_odom)

def realsense_odom_callback(odom):
    global roll, pitch, yaw
     
    original_quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,)
    original_rpy = euler_from_quaternion(original_quaternion)

# convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()
    rpy_odom.header.frame_id = "world"
    rpy_odom.pose.pose.orientation.x = (roll - original_rpy[0]) * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = (pitch - original_rpy[1]) * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = (yaw - original_rpy[2]) * 180 / math.pi
    realsense_odom_rpy_pub.publish(rpy_odom)



def crane_str_odom_callback(odom):
    global roll, pitch, yaw
     
    original_quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,)
    original_rpy = euler_from_quaternion(original_quaternion)

# convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()
    rpy_odom.header.frame_id = "world"
    rpy_odom.pose.pose.orientation.x = (roll - original_rpy[0]) * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = (pitch - original_rpy[1]) * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = (yaw - original_rpy[2]) * 180 / math.pi
    crane_str_odom_rpy_pub.publish(rpy_odom)

if __name__ == '__main__':
    rospy.init_node('odom_in_world_frame_compare')


    # Create a publisher for the transformed odometry message
    vins_odom_rpy_pub = rospy.Publisher('/vins_estimator/odometry_world_frame_rpy', Odometry, queue_size=10)
    realsense_odom_rpy_pub = rospy.Publisher('/camera/odom/sample_rpy', Odometry, queue_size=10)
    crane_str_odom_rpy_pub = rospy.Publisher('/odom_crane_structural_info_rpy', Odometry, queue_size=10)


    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/odom_motion_cap_sys_world_frame', Odometry, gt_odom_callback)
    rospy.Subscriber('/vins_estimator/odometry_world_frame', Odometry, vins_odom_callback)
    rospy.Subscriber('/camera/odom/sample_world_frame', Odometry, realsense_odom_callback)
    rospy.Subscriber('/odom_crane_structural_info', Odometry, crane_str_odom_callback)

    rospy.spin()