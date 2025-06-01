#!/usr/bin/env python

import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
import matplotlib.pyplot as plt

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

    # try:
    #     # Wait for the transformation to become available
    #     listener.waitForTransform(odom1.header.frame_id, odom2.header.frame_id, rospy.Time(), rospy.Duration(1.0))

    #     print("odm1 before", odom1.header.frame_id)
    #     print("odm2 ", odom2.header.frame_id)

    #     # Create a PoseStamped message for pose2
    #     pose1_stamped = PoseStamped()
    #     pose1_stamped.header = odom1.header
    #     pose1_stamped.pose = odom1.pose.pose
    #     # Convert the pose from frame A to frame B
    #     transformed_pose1 = listener.transformPose(odom2.header.frame_id, pose1_stamped)
    #     # print(transformed_pose)

    # print("odm1 aftrer", transformed_pose1.header.frame_id)

    # =======================================================================
    # rotate using initial values
    roll = 90 * math.pi / 180
    pitch = 0 * math.pi / 180
    yaw = 90 * math.pi / 180

    q_init = quaternion_from_euler(roll, pitch, yaw)

# Extract the quaternion components from transformed_pose1.pose.orientation
    quat_transfer = (
        transformed_pose1.pose.orientation.x,
        transformed_pose1.pose.orientation.y,
        transformed_pose1.pose.orientation.z,
        transformed_pose1.pose.orientation.w
    )

    # Perform quaternion multiplication
    q_rotated_by_init = quaternion_multiply(quat_transfer, q_init)

    # Create a new Pose message with the updated orientation

    new_odom_msg = Odometry()
    new_odom_msg.header = transformed_pose1.header
    new_odom_msg.child_frame_id = "camera_pose_frame1"
    new_odom_msg.pose.pose.orientation.x = q_rotated_by_init[0]
    new_odom_msg.pose.pose.orientation.y = q_rotated_by_init[1]
    new_odom_msg.pose.pose.orientation.z = q_rotated_by_init[2]
    new_odom_msg.pose.pose.orientation.w = q_rotated_by_init[3]

    odom1.header = transformed_pose1.header
    odom1.pose.pose.position = transformed_pose1.pose.position
    odom1.pose.pose.orientation = new_odom_msg.pose.pose.orientation

    # convert quat to rpy
    pose_msg_gt = quat_to_rpy(odom1)
    pub_pose_gt.publish(pose_msg_gt)
    pub_pose_gt_quat.publish(odom1)

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

    print(pose2.position.x - pose1.position.x, " ", pose2.position.y - pose1.position.y, " ", pose2.position.z - pose1.position.z)

    # Publish pose error

    pose_error = PoseStamped()

    # Set the header timestamp
    pose_error.header.stamp = rospy.Time.now()

    pose_error.pose.position = Point(x_error[-1], y_error[-1], z_error[-1])
    pose_error.pose.orientation = Quaternion(roll_error[-1], pitch_error[-1], yaw_error[-1], 0)
    pose_error_pub.publish(pose_error)

    # print(pose_error.position)

    # Process the converted pose
    # ...

    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     rospy.logwarn('Error occurred during pose conversion')


if __name__ == '__main__':

    odom1_sub = message_filters.Subscriber('/camera/odom/sample_word_frame', Odometry)
    odom2_sub = message_filters.Subscriber('/vins_estimator/odometry', Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([odom1_sub, odom2_sub], queue_size=10, slop=2)
    ts.registerCallback(odom_callback)

    pose_error_pub = rospy.Publisher('/pose_error', PoseStamped, queue_size=10)
    pub_pose_gt = rospy.Publisher('/pose_gt', PoseStamped, queue_size=10)
    pub_pose_gt_quat = rospy.Publisher('/pose_gt_quat', PoseStamped, queue_size=10)
    pub_pose_vins_mono = rospy.Publisher('/pose_vins_mono', PoseStamped, queue_size=10)

    # plot_pose_error()
    # plt.show(block=True)

    rospy.spin()
