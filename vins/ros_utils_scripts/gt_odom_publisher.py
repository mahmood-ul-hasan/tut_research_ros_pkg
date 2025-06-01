#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def odom_callback(odom_msg):

    

    # Extract the current orientation from the received odometry message
    orientation_quat = odom_msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_quat.x,
                                                orientation_quat.y,
                                                orientation_quat.z,
                                                orientation_quat.w])
    # Add 30 degrees to the yaw angle
    yaw += 30 * (3.14159 / 180)  # Convert 30 degrees to radians

    # Convert the modified orientation back to a quaternion
    q = quaternion_from_euler(roll, pitch, yaw)

    # Create a new odometry message with the modified orientation
    new_odom_msg = Odometry()
    new_odom_msg.header = odom_msg.header
    new_odom_msg.child_frame_id = "camera_pose_frame1"
    new_odom_msg.pose.pose.orientation.x = q[0]
    new_odom_msg.pose.pose.orientation.y = q[1]
    new_odom_msg.pose.pose.orientation.z = q[2]
    new_odom_msg.pose.pose.orientation.w = q[3]

    # Publish the new odometry message
    odom_pub.publish(new_odom_msg)


if __name__ == '__main__':
    rospy.init_node('odom_publisher')

    # Subscribe to the odometry topic with frame_id "camera_odom_frame"
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)

    # Create a publisher for the modified odometry topic
    odom_pub = rospy.Publisher('modified_odom_topic', Odometry, queue_size=10)

    # Spin the node
    rospy.spin()
