#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

pose_msg = PoseWithCovarianceStamped()  # Define pose_msg outside the callback function
first_value_received = False  # Flag variable to track the first value


def odom_callback(odom_msg):
    global pose_msg, first_value_received  # Use the global variables
    # Set the header and pose only for the first received value
    pose_msg.header = odom_msg.header
    pose_msg.pose.pose = odom_msg.pose.pose
    # Set covariance values
    pose_msg.pose.covariance[0] = 0.00000000000000001  # Covariance for position x
    pose_msg.pose.covariance[7] = 0.000000000000000001  # Covariance for position y
    pose_msg.pose.covariance[14] = 0.000000000000000001  # Covariance for position z
    # Set covariance for orientation (rotation)
    pose_msg.pose.covariance[21] = 0.0000000000000000001  # Covariance for rotation x
    pose_msg.pose.covariance[28] = 0.00000000000000001  # Covariance for rotation y
    pose_msg.pose.covariance[35] = 0.000000000000000001  # Covariance for rotation z
    # Set other covariance values as needed...
    first_value_received = True


if __name__ == '__main__':
    rospy.init_node('odom_to_pose_node')

    odom_topic = rospy.get_param('~odom_topic', '/camera/odom/sample')
    pose_topic = rospy.get_param('~pose_topic', '/vins_mono/pose')

    rospy.Subscriber(odom_topic, Odometry, odom_callback, queue_size=1)
    pose_publisher = rospy.Publisher(pose_topic, PoseWithCovarianceStamped, queue_size=1)

    rate = rospy.Rate(10)  # Set the desired publishing rate (10 Hz)

    while not rospy.is_shutdown():
        if first_value_received:
            pose_publisher.publish(pose_msg)
            print(pose_msg)
        rate.sleep()
