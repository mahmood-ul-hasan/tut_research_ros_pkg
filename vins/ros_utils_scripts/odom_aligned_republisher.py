#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseStamped
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf.transformations import quaternion_multiply, quaternion_conjugate, euler_from_quaternion

r_gt = 0
p_gt = 0
y_gt = 0


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


def rotate_quaternion(odom):

    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )
    # Convert quaternion to Euler angles
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    # Swap roll and pitch components
    # roll_new, pitch_new = -pitch + 90 * 180 / math.pi, -roll

    roll_new, pitch_new = -pitch, -roll

    roll_new = roll_new + 0.48 * math.pi / 180
    pitch_new = pitch_new + 94.38 * math.pi / 180

    # Extract the original yaw component from the quaternion
    _, _, yaw_new = euler_from_quaternion(quaternion)

    yaw_new = yaw_new - 90 * math.pi / 180

    # Create quaternion for the modified roll, pitch, and yaw components
    q_rotated = quaternion_from_euler(roll_new, pitch_new, yaw_new)

    return roll_new, pitch_new, yaw_new


def rp_odom_callback(odom):
    global r_gt, p_gt, y_gt
    pose_msg_gt = quat_to_rpy(odom)
    pub_pose_gt.publish(pose_msg_gt)
    r_gt = pose_msg_gt.pose.orientation.x
    p_gt = pose_msg_gt.pose.orientation.y
    y_gt = pose_msg_gt.pose.orientation.z


def mono_odom_callback(odom):

    pose_msg_vins_mono = quat_to_rpy(odom)
    pub_pose_vins_mono.publish(pose_msg_vins_mono)

    r_mono, p_mono, y_mono = rotate_quaternion(odom)
    # r_mono, p_mono, y_mono = euler_from_quaternion(q_rotated)
    # print("error: ", r_gt - r_mono * 180 / math.pi, p_gt - p_mono * 180 / math.pi, y_gt - y_mono * 180 / math.pi)

    q_rotated = quaternion_from_euler(r_mono, p_mono, y_mono)

    odom_aligned = Odometry()
    odom_aligned.header.stamp = rospy.Time.now()
    odom_aligned.header = odom.header
    odom_aligned.pose.pose.position = odom.pose.pose.position
    odom_aligned.pose.pose.orientation.x = q_rotated[0]
    odom_aligned.pose.pose.orientation.y = q_rotated[1]
    odom_aligned.pose.pose.orientation.z = q_rotated[2]
    odom_aligned.pose.pose.orientation.w = q_rotated[3]

    pub_pose_vins_mono_aligned_quat.publish(odom_aligned)

    pose_msg_vins_mono_aligned = quat_to_rpy(odom_aligned)
    pub_pose_vins_mono_aligned.publish(pose_msg_vins_mono_aligned)

    # Normalize the quaternion
    # quaternion_normalized = odom_aligned.pose.pose.orientation

    # rotate the pose for alignment
    # rotate using initial values
    roll = 180 * math.pi / 180
    pitch = -90 * math.pi / 180
    yaw = -90 * math.pi / 180

    q_add = quaternion_from_euler(roll, pitch, yaw)

    # Perform quaternion multiplication
    # q_rotated_by_init = quaternion_multiply(q_add, q_rotated)

    # Create a new Pose message with the updated orientation

    # odom_aligned.pose.pose.orientation.x = q_rotated_by_init[0]
    # odom_aligned.pose.pose.orientation.y = q_rotated_by_init[1]
    # odom_aligned.pose.pose.orientation.z = q_rotated_by_init[2]
    # odom_aligned.pose.pose.orientation.w = q_rotated_by_init[3]

    # print(odom)


if __name__ == '__main__':
    rospy.init_node('odom_aligned_republisher')

    # Create a publisher for the transformed odometry message
    pub_pose_vins_mono_aligned = rospy.Publisher('/vins_estimator/odometry_aligned', PoseStamped, queue_size=10)
    pub_pose_vins_mono_aligned_quat = rospy.Publisher('/vins_estimator/odometry_aligned_quat', Odometry, queue_size=10)
    pose_error_pub = rospy.Publisher('/pose_error', PoseStamped, queue_size=10)
    pub_pose_gt = rospy.Publisher('/pose_gt', PoseStamped, queue_size=10)
    pub_pose_vins_mono_quat = rospy.Publisher('/pose_vins_mono_quat', Odometry, queue_size=10)
    pub_pose_vins_mono = rospy.Publisher('/pose_vins_mono', PoseStamped, queue_size=10)

    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/vins_estimator/odometry', Odometry, mono_odom_callback)
    rospy.Subscriber('/camera/odom/sample_world_frame', Odometry, rp_odom_callback)

    # Spin the ROS node
    rospy.spin()
