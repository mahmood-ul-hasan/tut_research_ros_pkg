#!/usr/bin/env python3
# license removed for brevity

from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from tf.transformations import *
import rospy

import tf2_ros
import geometry_msgs.msg

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

import matplotlib.pyplot as plt
import math
from collections import deque
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np
import rosparam


# quat to rpy
i = 0
j = 0
array = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

global rotating_base
rotating_base = 0
initial_orientation = None
initial_position = None
global transfer_rotating_base


#################################################################################


def boom_rotating_base_callback(msg):

    # adding initial rotation
    original_quaternion = msg.quaternion
    rotation_quaternion = quaternion_from_euler(0, 0, 180 * math.pi / 180)  # 180 degrees in radians
    rotated_quaternion = quaternion_multiply([original_quaternion.x, original_quaternion.y, original_quaternion.z, original_quaternion.w], rotation_quaternion)
    msg.quaternion.x = rotated_quaternion[0]
    msg.quaternion.y = rotated_quaternion[1]
    msg.quaternion.z = rotated_quaternion[2]
    msg.quaternion.w = rotated_quaternion[3]

    global rotating_base
    global i

    # Convert quat to rpy
    quat_upper = (msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
    euler_upper = euler_from_quaternion(quat_upper)

    i = i + 1
    array[0] = euler_upper[0] * 180 / math.pi
    array[1] = euler_upper[1] * 180 / math.pi
    array[2] = euler_upper[2] * 180 / math.pi
    array[3] = rotating_base
    array[4] = i
    array_forPublish = Float32MultiArray(data=array)
    pub_rpy_upper_link.publish(array_forPublish)

    # broadcast the tf by rpy of imu
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "boom"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.quaternion.x
    t.transform.rotation.y = msg.quaternion.y
    t.transform.rotation.z = msg.quaternion.z
    t.transform.rotation.w = msg.quaternion.w

    br.sendTransform(t)
    # print(math.floor(euler_upper[0] * 180 / math.pi), math.floor(euler_upper[1] * 180 / math.pi), math.floor(euler_upper[2] * 180 / math.pi), math.floor(rotating_base), i)
    static_transform_publisher("boom", "rotaing_base", -1.245, 0, 0, 0, 0, 0)
    static_transform_publisher("world", "odom", 0, 0, 0, 0 * math.pi / 180, 0 * math.pi / 180, 90 * math.pi / 180)

    # print("  ",i,  euler_upper[1]*180/math.pi ,  (euler_upper[2]-upper_link_default_joint)*180/math.pi,  rotating_base)

    # ===========================================


def rotating_base_link_callback(msg):

    # laser is placed by rotation of 90 along y axis
    global transfer_rotating_base

    #q = quaternion_from_euler(-0*math.pi/180 , 90*math.pi/180, -(msg.position[0] -90)*math.pi/180)
    q = quaternion_from_euler(0, 90 * math.pi / 180, -(msg.position[0] - 90) * math.pi / 180)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "rotaing_base"
    t.child_frame_id = "laser"
    t.transform.translation.x = -1.10
    t.transform.translation.y = -0.08
    t.transform.translation.z = 0.0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    transfer_rotating_base = t
    br.sendTransform(t)

    global rotating_base
    rotating_base = msg.position[0] - 90


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
    rospy.init_node('tf2_broadcaster_by_imu_crane_structure')
    print("tf2_broadcaster_by_imu_crane_structure")

    pub_rpy_upper_link = rospy.Publisher("/rpy_imu_boom_link", Float32MultiArray, queue_size=10)

    rospy.Subscriber("/imu/quaternion", QuaternionStamped, boom_rotating_base_callback, queue_size=10)
    # Below is used rotaing_base
    rospy.Subscriber("/orion_rotating_base/joint_states", JointState, rotating_base_link_callback, queue_size=10)

    rospy.spin()
