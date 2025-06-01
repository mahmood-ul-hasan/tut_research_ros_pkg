#!/usr/bin/env python3
# license removed for brevity

from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import QuaternionStamped
import rospy

#import tf_conversions

import tf2_ros
import geometry_msgs.msg

# imu
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from collections import deque
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np
from tf.transformations import *



# tf for laser by gazebo link
pervious_time_upper_link = 0.0
pervious_time_boom_link = 0.0

# quat to rpy
i = 0
j = 0
array = np.array([0.0, 0.0, 0.0, 0.0])
array1 = np.array([0.0, 0.0, 0.0, 0.0])
dt = 0.001


#################################################################################
def imu_upper_link_callback(msg):
    global pervious_time_upper_link
    global dt

    # Convert quat to rpy
    quat_upper = (
        msg.quaternion.x,
        msg.quaternion.y,
        msg.quaternion.z,
        msg.quaternion.w)
    euler_upper = euler_from_quaternion(quat_upper)

    global i
    i = i + 1
    array[0] = euler_upper[0] * 180 / math.pi
    array[1] = euler_upper[1] * 180 / math.pi
    array[2] = euler_upper[2] * 180 / math.pi
    array[3] = i
    #print("array \n",array)
    array_forPublish = Float32MultiArray(data=array)
    # pub_rpy_upper_link.publish(array_forPublish)
    # -------------------------------------------------------

    # broadcast the tf by rpy of imu
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    current_time = rospy.Time.now().to_sec()
    t.header.frame_id = "lower_link"
    t.child_frame_id = "upper_link"

    quat_upper_yaw = quaternion_from_euler(0, 0, euler_upper[2])
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = quat_upper_yaw[0]
    t.transform.rotation.y = quat_upper_yaw[1]
    t.transform.rotation.z = quat_upper_yaw[2]
    t.transform.rotation.w = quat_upper_yaw[3]
    if current_time > pervious_time_upper_link + dt:
        br.sendTransform(t)
    pervious_time_upper_link = current_time


####################################################################
def imu_boom_link_callback(msg):

    global pervious_time_boom_link
    global dt

    # Convert quat to rpy
    quat_boom = (
        msg.quaternion.x,
        msg.quaternion.y,
        msg.quaternion.z,
        msg.quaternion.w)
    euler_boom = euler_from_quaternion(quat_boom)
    # publish the rpy
    global j
    j = j + 1
    array1[0] = euler_boom[0] * 180 / math.pi
    array1[1] = euler_boom[1] * 180 / math.pi
    array1[2] = euler_boom[2] * 180 / math.pi
    array1[3] = j
    array_forPublish1 = Float32MultiArray(data=array1)
    # pub_rpy_boom_link.publish(array_forPublish1)

    # broadcast the tf by rpy of imu
    quat_boom_yaw = quaternion_from_euler(0, -(0* math.pi/180 + euler_boom[1]), 0,)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    current_time = rospy.Time.now().to_sec()

    t.header.frame_id = "upper_link"
    t.child_frame_id = "boom_link"
    t.transform.translation.x = 1.4
    t.transform.translation.y = 0.191
    t.transform.translation.z = 1.1
    t.transform.rotation.x = quat_boom_yaw[0]
    t.transform.rotation.y = quat_boom_yaw[1]
    t.transform.rotation.z = quat_boom_yaw[2]
    t.transform.rotation.w = quat_boom_yaw[3]



    if current_time > pervious_time_boom_link + dt:
        br.sendTransform(t)
        static_transform_publisher("world", "camera_odom_frame", 0, 0, 0, 0, 0, 0)
        # static_transform_publisher("world", "base_link", 0, 0, 0, 0, 0, 0)
        static_transform_publisher("world", "base_link", 0, 0, 0, 0, 0, 0*math.pi/180)
        static_transform_publisher("base_link", "lower_link", 0, 0, 1.425, 0, 0, 0)
        # static_transform_publisher("boom_link", "velodyne", 16.635 ,0.63, 0.82, -90*math.pi/180, 1*math.pi/180, 180*math.pi/180)
        static_transform_publisher("boom_link", "velodyne", 16 , -2.0, 0.0, -85*math.pi/180, 1*math.pi/180, 180*math.pi/180)
        # static_transform_publisher("boom_link", "velodyne", 16 , -2.0, 0.0, -97*math.pi/180, 0*math.pi/180, 160*math.pi/180)
        print(math.floor(euler_boom[0] * 180 / math.pi), math.floor(euler_boom[1] * 180 / math.pi), math.floor(euler_boom[2] * 180 / math.pi), j)


    pervious_time_boom_link = current_time


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


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_kobelco_py')
    print("hello Mahmod")
    pub_rpy_boom_link = rospy.Publisher("/rpy_imu_boom_link1", Float32MultiArray, queue_size=1)

    rospy.Subscriber("/filter/quaternion", QuaternionStamped, imu_boom_link_callback)
    rospy.Subscriber("/filter/quaternion", QuaternionStamped, imu_upper_link_callback)

    rospy.spin()
