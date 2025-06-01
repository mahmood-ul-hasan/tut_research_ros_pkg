#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_conjugate
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf2_ros
from geometry_msgs.msg import Pose, Quaternion

# Global variables to store initial values
initial_orientation = None
initial_position = None


def odom_callback(odom):
    global initial_orientation
    global initial_position

    add = .7 * math.pi / 180
    original_quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,)
    original_rpy = euler_from_quaternion(original_quaternion)

    roll = - (original_rpy[0] -  90 * math.pi / 180)+ add
    pitch = original_rpy[1] +add
    yaw = original_rpy[2]    +7  * math.pi / 180 + add
    rotated_quaternion = quaternion_from_euler(roll, pitch, yaw)  # 180 degrees in radians
    
    odom.pose.pose.orientation.x = rotated_quaternion[0]
    odom.pose.pose.orientation.y = rotated_quaternion[1]
    odom.pose.pose.orientation.z = rotated_quaternion[2]
    odom.pose.pose.orientation.w = rotated_quaternion[3]



    if initial_position is None:
        initial_position = odom.pose.pose.position

    # Create a new odometry message with the modified orientation
    new_odom_msg = Odometry()
    new_odom_msg.header.stamp = rospy.Time.now()
    new_odom_msg.header.frame_id = "world"
    new_odom_msg.pose.pose.position.x = odom.pose.pose.position.x - initial_position.x
    new_odom_msg.pose.pose.position.y = odom.pose.pose.position.y - initial_position.y
    new_odom_msg.pose.pose.position.z = odom.pose.pose.position.z - initial_position.z
    new_odom_msg.pose.pose.orientation.x = odom.pose.pose.orientation.x
    new_odom_msg.pose.pose.orientation.y = odom.pose.pose.orientation.y
    new_odom_msg.pose.pose.orientation.z = odom.pose.pose.orientation.z
    new_odom_msg.pose.pose.orientation.w = odom.pose.pose.orientation.w
    odom_pub.publish(new_odom_msg)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "vins"
    t.transform.translation = new_odom_msg.pose.pose.position
    t.transform.rotation = new_odom_msg.pose.pose.orientation
    br.sendTransform(t)


# convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()
    rpy_odom.header.frame_id = "world"
    rpy_odom.pose.pose.position = odom.pose.pose.position
    quaternion = (
        new_odom_msg.pose.pose.orientation.x,
        new_odom_msg.pose.pose.orientation.y,
        new_odom_msg.pose.pose.orientation.z,
        new_odom_msg.pose.pose.orientation.w,)
    rpy = euler_from_quaternion(quaternion)
    rpy_odom.pose.pose.orientation.x = rpy[0] * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = rpy[1] * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = rpy[2] * 180 / math.pi
    odom_rpy_pub.publish(rpy_odom)




    

if __name__ == '__main__':
    rospy.init_node('vins_odom_republisher_in_world_frame_using_quaternion')

    # Create a publisher for the transformed odometry message
    odom_pub = rospy.Publisher('/vins_estimator/odometry_world_frame', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('/vins_estimator/odometry_world_frame_rpy', Odometry, queue_size=10)

    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/vins_estimator/odometry', Odometry, odom_callback)

    # Spin the ROS node
    rospy.spin()
