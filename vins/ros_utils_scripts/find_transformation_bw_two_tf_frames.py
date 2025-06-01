#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from tf.transformations import quaternion_multiply, quaternion_conjugate, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
from tf.transformations import translation_matrix, quaternion_multiply, translation_from_matrix
import tf.transformations as tf2

import tf2_ros
import geometry_msgs.msg

translation_wc = [0, 0, 0]
translation_wb = [0, 0, 0]
rotation_wc = [0, 0, 0, 1]
rotation_wb = [0, 0, 0, 1]

if __name__ == '__main__':
    rospy.init_node('my_tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(200.0)  # Specify the listening rate
    pose_pub = rospy.Publisher('/relative_pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():

        try:

            frame1 = "world"
            frame2 = "body"
            # Lookup the transformation between "frame1" and "frame2"
            (translation_wb, rotation_wb) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
            print(translation_wb)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed translation_wb to lookup transformation")

        try:
            frame1 = "world"
            frame2 = "camera_pose_frame"
            # Lookup the transformation between "frame1" and "frame2"
            (translation_wc, rotation_wc) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
            print(translation_wc)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed translation_wc to lookup transformation")

       # Combine transformations
        translation_bc = tf2.concatenate_matrices(tf2.translation_matrix(translation_wb),
                                                  tf2.translation_matrix(translation_wc))

        # rotation_cb = tf2.quaternion_multiply(rotation_wc, rotation_wb)
        rotation_bc = tf2.quaternion_multiply(rotation_wb, rotation_wc)
        print(rotation_bc)

# -----------------------------------------------------------------------------------------------------
# Transformation between frames
        # br = tf2_ros.TransformBroadcaster()
        # t = geometry_msgs.msg.TransformStamped()
        # t.header.stamp = rospy.Time.now()

        # t.header.frame_id = "body"
        # t.child_frame_id = "camera_pose_frame"
        # t.transform.translation.x = translation_bc[0, 3]
        # t.transform.translation.y = translation_bc[1, 3]
        # t.transform.translation.z = translation_bc[2, 3]
        # t.transform.rotation.x = rotation_bc[0]
        # t.transform.rotation.y = rotation_bc[1]
        # t.transform.rotation.z = rotation_bc[2]
        # t.transform.rotation.w = rotation_bc[3]
        # # print(t)
        # br.sendTransform(t)

        rate.sleep()


# ------------------------------------------------------------------------------------------

        rotation_cb = rotation_bc
        translation_cb = translation_bc

        print("Translation from frame c to frame b:", tf2.translation_from_matrix(translation_cb))
        print("Rotation from frame c to frame b:", rotation_cb)

        # Convert quaternion to Euler angles
        euler_angles = euler_from_quaternion(rotation_cb)
        rospy.loginfo("Pose from frame c to frame b (Euler angles): %s", euler_angles)

        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()  # Set the timestamp
        pose_msg.header.frame_id = "frame_c"  # Set the frame ID
        pose_msg.pose.position.x = translation_cb[0, 3]
        pose_msg.pose.position.y = translation_cb[1, 3]
        pose_msg.pose.position.z = translation_cb[2, 3]
        pose_msg.pose.orientation.x = euler_angles[0] * 180 / math.pi
        pose_msg.pose.orientation.y = euler_angles[1] * 180 / math.pi
        pose_msg.pose.orientation.z = euler_angles[2] * 180 / math.pi

        # Publish the pose
        pose_pub.publish(pose_msg)
