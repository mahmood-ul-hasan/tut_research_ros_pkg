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
from nav_msgs.msg import Odometry




def imu(msg):

    # convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()

    quaternion = (
         msg.quaternion.x,
         msg.quaternion.y,
         msg.quaternion.z,
         msg.quaternion.w,)
    rpy = euler_from_quaternion(quaternion)

    rpy_odom.pose.pose.orientation.x = rpy[0] * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = rpy[1] * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = rpy[2] * 180 / math.pi
    odom_imu.publish(rpy_odom)


def vins_imu(msg):

    # convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()

    quaternion = (
         msg.quaternion.x,
         msg.quaternion.y,
         msg.quaternion.z,
         msg.quaternion.w,)
    rpy = euler_from_quaternion(quaternion)

    rpy_odom.pose.pose.orientation.x = rpy[0] * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = rpy[1] * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = rpy[2] * 180 / math.pi
    odom_vins_imu.publish(rpy_odom)


def odom_callback(odom):

    # convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()

    quaternion = (
         odom.pose.pose.orientation.x,
         odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z,
         odom.pose.pose.orientation.w,)
    rpy = euler_from_quaternion(quaternion)

    rpy_odom.pose.pose.orientation.x = rpy[0] * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = rpy[1] * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = rpy[2] * 180 / math.pi
    odom_rpy_pub.publish(rpy_odom)



if __name__ == '__main__':
    rospy.init_node('imu_quat_to_rpy')

    odom_imu = rospy.Publisher('/imu_rpy', Odometry, queue_size=10)
    odom_vins_imu = rospy.Publisher('/vins_imu_rpy', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('/camera/odom/sample_rpy1', Odometry, queue_size=10)

    rospy.Subscriber("/imu/quaternion", QuaternionStamped, imu, queue_size=10)
    # rospy.Subscriber("/vins_imu/quaternion", QuaternionStamped, vins_imu, queue_size=10)
    rospy.Subscriber("/imu_vins/quaternion", QuaternionStamped, vins_imu, queue_size=10)
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)

    rospy.spin()
