#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

latest_odom_msg = None

def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quat_static = quaternion_from_euler(r, p, h)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = quat_static[0]
    t.transform.rotation.y = quat_static[1]
    t.transform.rotation.z = quat_static[2]
    t.transform.rotation.w = quat_static[3]
    br.sendTransform(t)

def odom_callback(msg):
    global latest_odom_msg
    latest_odom_msg = msg

def timer_callback(event):
    global latest_odom_msg
    if latest_odom_msg is not None:
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "sensor_base"
        t.transform.translation.x = latest_odom_msg.pose.pose.position.x
        t.transform.translation.y = latest_odom_msg.pose.pose.position.y
        t.transform.translation.z = latest_odom_msg.pose.pose.position.z
        t.transform.rotation.x = latest_odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = latest_odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = latest_odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = latest_odom_msg.pose.pose.orientation.w
        tf_broadcaster.sendTransform(t)
        print(t)


        static_transform_publisher("sensor_base", "laser", 0, 0, 0, 90 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180)

if __name__ == "__main__":
    rospy.init_node("odom_tf_publisher")

    # Subscribe to the odometry topic
    rospy.Subscriber("/vins_estimator/odometry", Odometry, odom_callback)

    # Create a timer to broadcast the transform at the desired frequency (e.g., 10 Hz)
    timer = rospy.Timer(rospy.Duration(0.01), timer_callback)

    # Spin the ROS node
    rospy.spin()
