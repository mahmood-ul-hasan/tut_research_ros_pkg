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
boom_link_default_joint = 0*math.pi / 2




####################################################################
def imu_boom_link_callback(msg):

    global pervious_time_boom_link


  # Create a new IMU message
    filtered_imu = Imu()
    filtered_imu.header = msg.header
    filtered_imu.linear_acceleration = msg.linear_acceleration
    filtered_imu.angular_velocity = msg.angular_velocity
    pub_imu.publish(filtered_imu)



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
    pub_rpy_upper_link.publish(array_forPublish1)
    # ------------------------------------------------------------



if __name__ == '__main__':
    rospy.init_node('compare_upper_imu_rotation_gazebo')
    print("hello Mahmod")
    pub_rpy_upper_link = rospy.Publisher("/rpy_imu_upper_link", Float32MultiArray, queue_size=10)
    pub_imu = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)

    rospy.Subscriber("/imu_boom_upper_link/data", Imu, imu_boom_link_callback)



    rospy.spin()

