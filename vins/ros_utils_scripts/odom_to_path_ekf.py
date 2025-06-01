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


def signal_handler(signal, frame):  # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


''' class '''


class path_pub():
    def __init__(self):
        rospy.init_node('odom_to_path_ekf', anonymous=True)

        self.append_rate = rospy.get_param("~append_rate", 25)

        self.gt_path_pub = rospy.Publisher("/odometry/filtered_path_py", Path, queue_size=2)
        self.gt_poses = rospy.Subscriber('/odometry/filtered', Odometry, self.gtcallback)

        self.my_path = Path()
        self.check = 0

        self.rate = rospy.Rate(self.append_rate)

    def gtcallback(self, msg):

        self.header = msg.header
        self.pose = msg.pose.pose
        self.check = 1

        # # Original quaternion
        # original_quaternion = msg.pose.pose.quaternion
        # # Create a quaternion to represent a 180-degree rotation around the z-axis
        # rotation_quaternion = quaternion_from_euler(0, 0, 90 * math.pi / 180)  # 180 degrees in radians
        # # Multiply the original quaternion by the rotation quaternion
        # rotated_quaternion = quaternion_multiply([original_quaternion.x, original_quaternion.y, original_quaternion.z, original_quaternion.w], rotation_quaternion)
        # print(rotated_quaternion)
        # msg.quaternion.x = rotated_quaternion[0]
        # msg.quaternion.y = rotated_quaternion[1]
        # msg.quaternion.z = rotated_quaternion[2]
        # msg.quaternion.w = rotated_quaternion[3]


''' main '''
path_pub_class = path_pub()

if __name__ == '__main__':
    while 1:
        try:
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
        # except:
        #     print("exception")
        #     pass
