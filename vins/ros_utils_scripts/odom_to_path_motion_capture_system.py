#!/usr/bin/env python
# -*- coding: utf-8 -*-

''' import libraries '''
import time
import numpy as np

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseWithCovarianceStamped
import sys
import signal
import rospy
import tf2_ros
from geometry_msgs.msg import QuaternionStamped
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import math


def signal_handler(signal, frame):  # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quat_static = quaternion_from_euler(r, p, h)

    t.header.stamp = rospy.Time.now()
    print(rospy.Time.now())
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


''' class '''


class path_pub():
    def __init__(self):
        rospy.init_node('odom_to_path_mcs', anonymous=True)

        self.append_rate = 100

        self.gt_path_pub = rospy.Publisher("/csv_gt_odom_path", Path, queue_size=2)
        self.gt_poses = rospy.Subscriber('/csv_gt_odom', Odometry, self.gtcallback)

        self.my_path = Path()
        self.check = 0

        self.rate = rospy.Rate(self.append_rate)

    def gtcallback(self, msg):

        self.header = msg.header
        self.pose = msg.pose.pose
        self.check = 1


''' main '''
path_pub_class = path_pub()

if __name__ == '__main__':

    while 1:

        try:
            # static_transform_publisher("world", "odom_mot_cap_sys", 0, 0, 0, 1.9707, 0, 1.5707)          # rosrun tf2_ros static_transform_publisher 0 0 0  1.9707 0   1.5707  world odom_mot_cap_sys
            # static_transform_publisher("world", "odom_mot_cap_sys", 0, 0, 0, 90 * math.pi / 180, 0 * math.pi / 180, 90 * math.pi / 180)          # rosrun tf2_ros static_transform_publisher 0 0 0  1.9707 0   1.5707  world odom_mot_cap_sys
            if path_pub_class.check == 1:
                pose = PoseStamped()
                pose.header = path_pub_class.header
                pose.pose = path_pub_class.pose
                pose.header.stamp = rospy.Time.now()
                path_pub_class.my_path.poses.append(pose)
                path_pub_class.my_path.header = path_pub_class.header
                # path_pub_class.my_path.header.frame_id = path_pub_class.parent_frame_id
                path_pub_class.my_path.header.stamp = rospy.Time.now()
                path_pub_class.gt_path_pub.publish(path_pub_class.my_path)
                # print(path_pub_class.my_path)

            path_pub_class.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
            sys.exit(0)
