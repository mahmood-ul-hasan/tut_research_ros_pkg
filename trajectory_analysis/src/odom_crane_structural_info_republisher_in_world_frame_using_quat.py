#!/usr/bin/env python3
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist
from tf.transformations import quaternion_multiply, quaternion_inverse


if __name__ == '__main__':
    rospy.init_node('combine_tf_frames_and_publish_odom_of_crane_structural')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    odom_pub = rospy.Publisher('odom_crane_structural_info', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('odom_crane_structural_info_rpy', Odometry, queue_size=10)

    initial_orientation = None
    initial_position = None

    rate = rospy.Rate(500.0)
    while not rospy.is_shutdown():
        try:
            # trans = tfBuffer.lookup_transform('world', 'rotaing_base', rospy.Time())
            trans = tfBuffer.lookup_transform('world', 'laser1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue


        original_quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w)
        original_rpy = euler_from_quaternion(original_quaternion)

        roll = original_rpy[0]
        pitch = original_rpy[1]
        yaw = original_rpy[2]    
        rotated_quaternion = quaternion_from_euler(roll, pitch, yaw)  # 180 degrees in radians

        trans.transform.rotation.x = rotated_quaternion[0]
        trans.transform.rotation.y = rotated_quaternion[1]
        trans.transform.rotation.z = rotated_quaternion[2]
        trans.transform.rotation.w = rotated_quaternion[3]



        if initial_position is None:
            initial_position = trans.transform.translation

        

        # broadcast the tf by rpy of imu
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        # t.child_frame_id = "crane_info"
        t.child_frame_id = "laser"
        t.transform.translation.x = trans.transform.translation.x - initial_position.x 
        t.transform.translation.y = trans.transform.translation.y - initial_position.y 
        t.transform.translation.z = trans.transform.translation.z - initial_position.z

        # t.transform.translation.x = trans.transform.translation.x
        # t.transform.translation.y = trans.transform.translation.y 
        # t.transform.translation.z = trans.transform.translation.z 
        t.transform.rotation.x = trans.transform.rotation.x
        t.transform.rotation.y = trans.transform.rotation.y
        t.transform.rotation.z = trans.transform.rotation.z
        t.transform.rotation.w = trans.transform.rotation.w
        br.sendTransform(t)

        tranfered_trans = t.transform.translation
        tranfered_rotate = t.transform.rotation

        # Initialize odometry message
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "rotating_base_tajectory"
        # Fill in the odometry message
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position = tranfered_trans
        odom.pose.pose.orientation = tranfered_rotate
        odom.twist.twist = Twist()

        # Publish the odometry message
        odom_pub.publish(odom)


# Initialize odometry message
        odom_rpy = Odometry()
        odom_rpy.header.frame_id = "world"
        odom_rpy.child_frame_id = "rotating_base_tajectory"
        # Fill in the odometry message
        odom_rpy.header.stamp = rospy.Time.now()
        odom_rpy.pose.pose.position = tranfered_trans

        quaternion = (
            tranfered_rotate.x,
            tranfered_rotate.y,
            tranfered_rotate.z,
            tranfered_rotate.w)
        rpy = tf.transformations.euler_from_quaternion(quaternion)

        odom_rpy.pose.pose.orientation.x = rpy[0] * 180 / math.pi
        odom_rpy.pose.pose.orientation.y = rpy[1] * 180 / math.pi
        odom_rpy.pose.pose.orientation.z = rpy[2] * 180 / math.pi
        odom_rpy.twist.twist = Twist()

        # print("rpy: ",  odom_rpy.pose.pose.orientation.x,  odom_rpy.pose.pose.orientation.y,  odom_rpy.pose.pose.orientation.z)

        # Publish the odometry message
        odom_rpy_pub.publish(odom_rpy)

        rate.sleep()
