#!/usr/bin/env python

import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import quaternion_multiply, quaternion_conjugate, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import tf

# Lists to store the pose error values
x_error = []
y_error = []
z_error = []
roll_error = []
pitch_error = []
yaw_error = []
euler_error = [0.0, 0.0, 0.0]  # Initialize the RPY error


rospy.init_node('pose_error_node')


def quat_to_rpy(odom):

    # Perform the conversion using tf transformations
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )
    euler = euler_from_quaternion(quaternion)

    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # Create a PoseStamped message
    pose_msg = PoseStamped()

    # Set the header timestamp
    pose_msg.header.stamp = rospy.Time.now()

    # Set the pose position (x, y, z)
    pose_msg.pose.position = odom.pose.pose.position

    # Set the pose orientation (roll, pitch, yaw)
    # quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose_msg.pose.orientation.x = roll * 180 / math.pi
    pose_msg.pose.orientation.y = pitch * 180 / math.pi
    pose_msg.pose.orientation.z = yaw * 180 / math.pi
    pose_msg.pose.orientation.w = 0

    return pose_msg


listener = tf.TransformListener()


def odom_callback(odom1, odom2):

    try:
        # Wait for the transformation to become available
        listener.waitForTransform(odom1.header.frame_id, odom2.header.frame_id, rospy.Time(), rospy.Duration(1.0))

        # print("odm1 before", odom1.header.frame_id)
        # print("odm2 ", odom2.header.frame_id)

        # Create a PoseStamped message for pose2
        pose1_stamped = PoseStamped()
        pose1_stamped.header = odom1.header
        pose1_stamped.pose = odom1.pose.pose
        # Convert the pose from frame A to frame B
        transformed_pose1 = listener.transformPose(odom2.header.frame_id, pose1_stamped)
        # print(transformed_pose1)

        # print("odm1 aftrer", transformed_pose1.header.frame_id)

        odom1 = Odometry()
        odom1.header = transformed_pose1.header
        odom1.pose.pose.position = transformed_pose1.pose.position
        odom1.pose.pose.orientation = transformed_pose1.pose.orientation

        # convert quat to rpy
        pose_msg_gt = quat_to_rpy(odom1)
        pub_pose_gt.publish(pose_msg_gt)


# =======================================================================
        # Calculate the quaternion to add
        quat_vins_mono = np.array([odom2.pose.pose.orientation.x, odom2.pose.pose.orientation.y, odom2.pose.pose.orientation.z, odom2.pose.pose.orientation.w])
        quat_gt = np.array([odom1.pose.pose.orientation.x, odom1.pose.pose.orientation.y, odom1.pose.pose.orientation.z, odom1.pose.pose.orientation.w])
        # quat_add = np.array([-0.63442269, 0.5104968, - 0.28217633, 0.50722519])
        # quat_add = np.array([0, 0, 0, 1])

        # rotate using initial values
        roll = 0 * math.pi / 180
        pitch = 0 * math.pi / 180
        yaw = 90 * math.pi / 180

        quat_add = quaternion_from_euler(roll, pitch, yaw)

        # quat_add = quaternion_multiply(quat_gt, quaternion_conjugate(quat_vins_mono))
        # print("quat_add ", quat_add)
        # # Convert quaternion B to RPY angles
        r_quat_add = euler_from_quaternion(quat_add)
        print(r_quat_add)

        # Perform quaternion multiplication to obtain orientation B
        quat_mono_after_allign = quaternion_multiply(quat_add, quat_vins_mono)
        # print("quat_after_addition: ", quat_mono_after_allign)

        # Convert quaternion B to RPY angles
        # r_after_allign = euler_from_quaternion(quat_mono_after_allign)

        # # Print the RPY angles for orientation B
        # print("Orientation B (RPY angles):")
        # print(f"Roll:  {np.degrees(r_after_addition[0])} degrees")
        # print(f"Pitch: {np.degrees(r_after_addition[1])} degrees")
        # print(f"Yaw:   {np.degrees(r_after_addition[2])} degrees")
        # print("  ")

        new_odom_msg = Odometry()
        new_odom_msg.pose.pose.orientation.x = quat_mono_after_allign[0]
        new_odom_msg.pose.pose.orientation.y = quat_mono_after_allign[1]
        new_odom_msg.pose.pose.orientation.z = quat_mono_after_allign[2]
        new_odom_msg.pose.pose.orientation.w = quat_mono_after_allign[3]

        # odom2.header.stamp = rospy.Time.now()
        # odom2.pose.pose.orientation = new_odom_msg.pose.pose.orientation

        pub_pose_vins_mono_quat.publish(odom2)
        # print(odom2)
        pose_msg_vins_mono = quat_to_rpy(odom2)
        pub_pose_vins_mono.publish(pose_msg_vins_mono)

        pose1 = odom1.pose.pose
        pose2 = odom2.pose.pose

        # Compute quaternion error
        quaternion1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        quaternion2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]
        quaternion_error = quaternion_multiply(quaternion2, quaternion_conjugate(quaternion1))

        # Convert quaternion error to Euler angles (roll, pitch, yaw)
        euler_error1 = euler_from_quaternion(quaternion_error)
        euler_error[0] = pose_msg_gt.pose.orientation.x - pose_msg_vins_mono.pose.orientation.x
        euler_error[1] = pose_msg_gt.pose.orientation.y - pose_msg_vins_mono.pose.orientation.y
        euler_error[2] = pose_msg_gt.pose.orientation.z - pose_msg_vins_mono.pose.orientation.z

        # Compute pose error
        x_error.append(pose2.position.x - pose1.position.x)
        y_error.append(pose2.position.y - pose1.position.y)
        z_error.append(pose2.position.z - pose1.position.z)
        roll_error.append(euler_error[0])
        pitch_error.append(euler_error[1])
        yaw_error.append(euler_error[2])

        # print(pose2.position.x - pose1.position.x, " ", pose2.position.y - pose1.position.y, " ", pose2.position.z - pose1.position.z)

        # Publish pose error

        pose_error = PoseStamped()

        # Set the header timestamp
        pose_error.header.stamp = rospy.Time.now()

        # pose_error.pose.position = Point(x_error[-1], y_error[-1], z_error[-1])
        # pose_error.pose.orientation = Quaternion(roll_error[-1], pitch_error[-1], yaw_error[-1], 0)
        pose_error.pose.position.x = r_quat_add[0] * 180 / math.pi
        pose_error.pose.position.y = r_quat_add[1] * 180 / math.pi
        pose_error.pose.position.z = r_quat_add[2] * 180 / math.pi
        pose_error.pose.orientation.x = new_odom_msg.pose.pose.orientation.x
        pose_error.pose.orientation.y = new_odom_msg.pose.pose.orientation.y
        pose_error.pose.orientation.z = new_odom_msg.pose.pose.orientation.z
        pose_error.pose.orientation.w = new_odom_msg.pose.pose.orientation.w
        pose_error_pub.publish(pose_error)

        # print(pose_error.position)

        # Process the converted pose
        # ...

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn('Error occurred during pose conversion')


if __name__ == '__main__':

    odom1_sub = message_filters.Subscriber('/camera/odom/sample', Odometry)
    odom2_sub = message_filters.Subscriber('/vins_estimator/odometry', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([odom1_sub, odom2_sub], queue_size=10, slop=2)
    ts.registerCallback(odom_callback)

    pose_error_pub = rospy.Publisher('/pose_error', PoseStamped, queue_size=10)
    pub_pose_gt = rospy.Publisher('/pose_gt', PoseStamped, queue_size=10)
    pub_pose_vins_mono_quat = rospy.Publisher('/pose_vins_mono_quat', Odometry, queue_size=10)
    pub_pose_vins_mono = rospy.Publisher('/pose_vins_mono', PoseStamped, queue_size=10)

    # plot_pose_error()
    # plt.show(block=True)

    rospy.spin()
