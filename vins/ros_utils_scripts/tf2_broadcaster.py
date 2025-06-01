#!/usr/bin/env python3
# license removed for brevity

from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import *
import rospy
from nav_msgs.msg import Odometry

# Because of transformations
#import tf_conversions

import tf2_ros
import geometry_msgs.msg


from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from collections import deque
import numpy as np
from std_msgs.msg import Float32MultiArray
import numpy as np

pervious_time = 0.0


def camera_pose_callback(odom):

    global pervious_time
    current_time = rospy.Time.now().to_sec()

    if current_time >= pervious_time + 0.0:

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = "camera_odom_frame"
        t.child_frame_id = "camera_pose_frame"
        t.transform.translation = odom.pose.pose.position
        t.transform.rotation = odom.pose.pose.orientation

        br.sendTransform(t)
        # print(t.header)

        static_transform_publisher("body", "camera_pose_frame1", 0, 0, 0, 0 * math.pi / 180, -90 * math.pi / 180, -90 * math.pi / 180)
        static_transform_publisher("camera_pose_frame1", "imu_link", -0.2, 0, .13, 90 * math.pi / 180, 180 * math.pi / 180, 0 * math.pi / 180)

        static_transform_publisher("world", "camera_odom_frame", 0, 0, 0, 0, 0, -1.5708)
        static_transform_publisher("world", "odom_mot_cap_sys", 0, 0, 0, 1.9707, 0, 1.5707)          # rosrun tf2_ros static_transform_publisher 0 0 0  1.9707 0   1.5707  world odom_mot_cap_sys

        # static_transform_publisher("camera_pose_frame", "imu_link", -0.2, 0, .13, 90 * math.pi / 180, 180 * math.pi / 180, 0 * math.pi / 180)
        static_transform_publisher("camera_pose_frame", "camera_imu_optical_frame", 0.000, 0.021, 0.000, 1.571, -0.000, 1.571)

        pervious_time = current_time


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
    # print(t)
    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    pub_rpy_boom_link = rospy.Publisher("/rpy_imu_boom_link1", Float32MultiArray, queue_size=1)

    rospy.Subscriber("/camera/odom/sample", Odometry, camera_pose_callback)

    rospy.spin()

    # try:
    # main_()
    # except rospy.ROSInterruptException:
    # pass
