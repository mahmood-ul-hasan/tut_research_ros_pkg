#!/usr/bin/env python3
# license removed for brevity

from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from tf.transformations import *
import rospy

# Because of transformations
#import tf_conversions

import tf2_ros
import geometry_msgs.msg


# imu
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt
import math
from collections import deque
import numpy as np
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
import numpy as np
import rosparam

laser_position = np.array([0.0, 0.0, 0.0, 0.0])
laser_pose = np.array([0.0, 0.0, 0.0, 0.0])
q1_inv = np.array([0.0, 0.0, 0.0, 0.0])


# tf for laser by gazebo link
pervious_time_upper_link = 0.0
pervious_time_boom_link = 0.0
pervious_time_hook_link = 0.0

# quat to rpy
i = 0
j = 0
array = np.array([0.0, 0.0, 0.0, 0.0])
array1 = np.array([0.0, 0.0, 0.0, 0.0])
boom_link_default_joint = -90* math.pi / 180

# noise = np.random.uniform(-0.0174533, 0.0174533 )
noise = 0
#print ("yaw noise", noise*180/math.pi)


#################################################################################
def imu_upper_link_callback(msg):
    global pervious_time_upper_link, noise

    # Convert quat to rpy
    quat_upper = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)
    euler_upper = euler_from_quaternion(quat_upper)
    # publish rpy
    global i
    i = i + 1
    array[0] = euler_upper[0] * 180 / math.pi
    array[1] = euler_upper[1] * 180 / math.pi
    array[2] = euler_upper[2] * 180 / math.pi
    array[3] = i
    #print("array \n",array)
    array_forPublish = Float32MultiArray(data=array)
    pub_rpy_upper_link.publish(array_forPublish)
    # -------------------------------------------------------

    # broadcast the tf by rpy of imu
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    current_time = rospy.Time.now().to_sec()
    t.header.frame_id = "lower_link"
    t.child_frame_id = "upper_link"

 

    # quat_upper_yaw = quaternion_from_euler(0, 0, euler_upper[2])
    quat_upper_yaw = quaternion_from_euler(0, 0, euler_upper[2] + noise* 1.2  )
    # quat_upper_yaw = quaternion_from_euler(0, 0, 0)
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = quat_upper_yaw[0]
    t.transform.rotation.y = quat_upper_yaw[1]
    t.transform.rotation.z = quat_upper_yaw[2]
    t.transform.rotation.w = quat_upper_yaw[3]
    # print(t)
    if current_time > pervious_time_upper_link + 0.00001:
        br.sendTransform(t)
    pervious_time_upper_link = current_time
    print("u upper_link ", euler_upper[2] * 180 / math.pi, "u boom_link ", euler_upper[1] * 180 / math.pi)


####################################################################
def imu_boom_link_callback(msg):

    global pervious_time_boom_link, noise, boom_link_default_joint

    # Convert quat to rpy
    quat_boom = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w)
    euler_boom = euler_from_quaternion(quat_boom)
    # publish the rpy
    global j
    j = j + 1
    array1[0] = euler_boom[0] * 180 / math.pi
    array1[1] = euler_boom[1] * 180 / math.pi
    array1[2] = euler_boom[2] * 180 / math.pi
    array1[3] = j
    array_forPublish1 = Float32MultiArray(data=array1)
    pub_rpy_boom_link.publish(array_forPublish1)
    # ------------------------------------------------------------



    # broadcast the tf by rpy of imu
    quat_boom_yaw = quaternion_from_euler(0,  boom_link_default_joint  , 0,)
    # print(quat_boom_yaw)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    current_time = rospy.Time.now().to_sec()

    t.header.frame_id = "upper_link"
    t.child_frame_id = "boom_link"
    t.transform.translation.x = 1.4
    t.transform.translation.y = 0.191
    t.transform.translation.z = 1.008
    t.transform.rotation.x = quat_boom_yaw[0]
    t.transform.rotation.y = quat_boom_yaw[1]
    t.transform.rotation.z = quat_boom_yaw[2]
    t.transform.rotation.w = quat_boom_yaw[3]

    print("u upper_link ", euler_boom[2] * 180 / math.pi, "boom_link ", (boom_link_default_joint) * 180 / math.pi)

    # print(t)
    if current_time > pervious_time_boom_link + 0.00001:
        br.sendTransform(t)
    pervious_time_boom_link = current_time

    ######################################################################


def imu_hook_link_callback(msg):
    global pervious_time_hook_link, noise

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

  

    #q = quaternion_from_euler(msg.position[0], 0, 0)
    q = quaternion_from_euler(msg.position[0] + noise*1.2 , 0, 0)

    t.header.stamp = rospy.Time.now()
    current_time = rospy.Time.now().to_sec()
    t.header.frame_id = "boom_link"
    t.child_frame_id = "front_laser_link"
    t.transform.translation.x = 20
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    if current_time > pervious_time_hook_link + 0.00001:
        br.sendTransform(t)
    pervious_time_hook_link = current_time
    print("imu_hook_link_callback", msg.position[0] * 180 / math.pi)

    # =====================================================
    static_transform_publisher("world", "base_link", 0, 0, 0)
    static_transform_publisher("base_link", "lower_link", 0, 0, 1.5)
    static_transform_publisher("boom_link", "imu_link", 0.5, 0, 0)


    static_transform_publisher("lower_link", "upper_link", 0, 0, 0)
    # static_transform_publisher("upper_link", "boom_link", 1.4, 0.2, 1)

def static_transform_publisher(header, child, x, y, z):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_by_imu')
    print("hello Mahmod")
    pub_rpy_upper_link = rospy.Publisher("/rpy_imu_upper_link", Float32MultiArray, queue_size=10)
    pub_rpy_boom_link = rospy.Publisher("/rpy_imu_boom_link", Float32MultiArray, queue_size=10)

    rospy.Subscriber("/imu_boom_link/data", Imu, imu_boom_link_callback)
    # rospy.Subscriber("/imu_boom_link/data", Imu, imu_upper_link_callback)


    rospy.Subscriber("/marvin/joint_states", JointState, imu_hook_link_callback, queue_size=10)

    rospy.spin()

    # try:
    # main_()
    # except rospy.ROSInterruptException:
    # pass
