#!/usr/bin/env python
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

# base is 1.1 meter
# IMU is  0.9 meter

# imu
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
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

# quat to rpy
i = 0
ii = 0
j = 0
array = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
array1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
boom_link_default_joint = 10 * math.pi / 180
upper_link_default_joint = 0 * math.pi / 180

global rotating_base
global t
global flag
# flag = False


def imu_callback(msg):
    global t
    global flag
    if flag == True:
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(t)
        # print("-------------------", t.header.stamp, " ", flag)
        # print(flag)

    # static_transform_publisher("camera_pose_frame", "rotation_base", 0, 0, 0, 0 * math.pi / 180, 0 * math.pi / 180, 90 * math.pi / 180)
    static_transform_publisher("body", "rotation_base", 0, 0, 0, 0 * math.pi / 180, 0 * math.pi / 180, 90 * math.pi / 180)
    static_transform_publisher("world", "camera_odom_frame", 0, 0, 0, 0 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180)


def rotating_base_link_callback(msg):

    # laser is placed by rotation of 90 along y axis
    global t
    global flag
    global ii
    flag = False
    q = quaternion_from_euler(0 * math.pi / 180, 90 * math.pi / 180, -(msg.position[0] - 90) * math.pi / 180)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    # t.header.stamp = msg.header.stamp
    t.header.frame_id = "rotation_base"
    t.child_frame_id = "laser"
    t.transform.translation.x = 0.00
    t.transform.translation.y = 0.06
    t.transform.translation.z = 0.19
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    flag = True

    print("Pos-90= ", ii, round(msg.position[0] - 90, 1), " ", flag)
    #print("Pos = ",msg.position[0])

    global rotating_base
    rotating_base = msg.position[0] - 90

    ii = ii + 1

    br.sendTransform(t)
    # br_static.sendTransform(t_static)

    # print("  ",ii,  rotating_base)

    # Dimension of crane frame
    # lower link height = 0.3
    # base link height = 0.8 m
    # boom link height = 0.3 m
    # rotating base  link height = 1.234 m
    #  t.transform.translation.x = -1.234
    #  t.transform.translation.y = -0.08

  # ----------------------------------------------------------------------
    # Testing
    # static_transform_publisher("rotation_base", "laser",  0.04, 0.19, .06,    180*math.pi/180, 90*math.pi/180, 0*math.pi/180)

    # =====================================================


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


if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_by_imu')
    print("hello Mahmod")

    pub_rpy_upper_link = rospy.Publisher("/rpy_imu_boom_link", Float32MultiArray, queue_size=5)
    # pub_rpy_boom_link = rospy.Publisher("/rpy_imu_boom_link", Float32MultiArray, queue_size=1)

    rospy.Subscriber("/camera/imu", Imu, imu_callback)

    # Below is used rotaing_base
    rospy.Subscriber("/orion_rotating_base/joint_states", JointState, rotating_base_link_callback, queue_size=5)

    rospy.spin()
    print("hello Mahmod")

    # try:
    # main_()
    # except rospy.ROSInterruptException:
    # pass
