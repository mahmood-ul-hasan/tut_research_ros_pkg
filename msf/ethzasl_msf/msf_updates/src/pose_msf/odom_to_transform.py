#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped


def odom_callback(odom_msg):
    # Create a TransformStamped message
    transform_stamped = TransformStamped()
    transform_stamped.header = odom_msg.header
    transform_stamped.child_frame_id = odom_msg.child_frame_id
    transform_stamped.transform.translation = odom_msg.pose.pose.position
    transform_stamped.transform.rotation = odom_msg.pose.pose.orientation

    # Publish the TransformStamped message
    tf_pub.publish(transform_stamped)


rospy.init_node('odom_to_transform')

# Create a publisher for TransformStamped messages
tf_pub = rospy.Publisher('/vins_mono/pose', TransformStamped, queue_size=1)

# Subscribe to the Odometry message
rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback, queue_size=20)

rospy.spin()
