#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

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


def handle_imu_orientation(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w

    print(t)

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster_by_imu')
    rospy.Subscriber("/imu", Imu, handle_imu_orientation)
    rospy.spin()
