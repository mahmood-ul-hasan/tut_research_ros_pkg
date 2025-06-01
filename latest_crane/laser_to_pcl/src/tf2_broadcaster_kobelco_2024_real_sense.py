#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math
import geometry_msgs.msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler



def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quat_static = quaternion_from_euler(r, p, h)

    t.header.stamp = rospy.Time.now()
    # t.header.stamp = time
    # t.header.stamp =     rospy.Time(0)
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = quat_static[0]
    t.transform.rotation.y = quat_static[1]
    t.transform.rotation.z = quat_static[2]
    t.transform.rotation.w = quat_static[3]
    # print(t)
    br.sendTransform(t)


def odom_callback(msg):
    print("callback")
    # Create a tf2_ros.TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Create a tf2_ros.TransformStamped message
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "sensor_base"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    # Broadcast the transform
    tf_broadcaster.sendTransform(t)
    print(t)


    static_transform_publisher("sensor_base", "velodyne", 0, 0, 0, -90 * math.pi / 180, 180 * math.pi / 180, 0 * math.pi / 180)
    static_transform_publisher("world", "camera_odom_frame", 0, 0, 0, 0 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180)


if __name__ == "__main__":
    rospy.init_node("odom_tf_publisher")
    print("odom_tf_publisher")

    # Subscribe to the odometry topic
    rospy.Subscriber("/vins_estimator/odometry", Odometry, odom_callback)

    # Spin the ROS node
    rospy.spin()
