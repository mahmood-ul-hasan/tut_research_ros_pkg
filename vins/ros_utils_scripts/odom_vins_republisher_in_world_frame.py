#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf2_ros

initial_yaw_value = None
initial_pitch_value = None
initial_roll_value = None
initial_position = None  # Replace this with the actual initial position


def odom_callback(odom):
    global initial_yaw_value
    global initial_pitch_value
    global initial_roll_value
    global initial_position

    # Extract the current orientation from the received odometry message
    orientation_quat = odom.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orientation_quat.x,
                                                orientation_quat.y,
                                                orientation_quat.z,
                                                orientation_quat.w])

    # Check if this is the first time the callback is called
    if initial_yaw_value is None:
        initial_yaw_value = yaw
        print("Initial yaw value set to", initial_yaw_value)

    if initial_pitch_value is None:
        initial_pitch_value = pitch
        print("Initial pitch value set to", initial_pitch_value)

    if initial_roll_value is None:
        initial_roll_value = roll
        print("Initial roll value set to", initial_roll_value)

    if initial_position is None:
        initial_position = odom.pose.pose.position

    roll_world = roll - initial_roll_value
    pitch_world = pitch - initial_pitch_value
    yaw_world = yaw - initial_yaw_value

    print("relative_rotation          ", round(roll_world * 180 / math.pi, 3), round(pitch_world * 180 / math.pi, 3), round(yaw_world * 180 / math.pi, 3))

    # Convert the modified orientation back to a quaternion
    q = quaternion_from_euler(roll_world, pitch_world, yaw_world)

    # Create a new odometry message with the modified orientation
    new_odom_msg = Odometry()
    new_odom_msg.header.stamp = rospy.Time.now()
    new_odom_msg.header.frame_id = "world"
    new_odom_msg.pose.pose.position.x = odom.pose.pose.position.x - initial_position.x
    new_odom_msg.pose.pose.position.y = odom.pose.pose.position.y - initial_position.y
    new_odom_msg.pose.pose.position.z = odom.pose.pose.position.z - initial_position.z
    new_odom_msg.pose.pose.orientation.x = q[0]
    new_odom_msg.pose.pose.orientation.y = q[1]
    new_odom_msg.pose.pose.orientation.z = q[2]
    new_odom_msg.pose.pose.orientation.w = q[3]

    # odom.header.stamp = rospy.Time.now()
    # odom.header.frame_id = "world"
    # odom.pose.pose = transformed_pose.pose
    # # Publish the transformed odometry message
    odom_pub.publish(new_odom_msg)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()

    t.header.frame_id = "world"
    t.child_frame_id = "vins_odom_world_frame"
    t.transform.translation = new_odom_msg.pose.pose.position
    t.transform.rotation = new_odom_msg.pose.pose.orientation
    br.sendTransform(t)

    # convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()
    rpy_odom.header.frame_id = "world"
    rpy_odom.pose.pose.position = new_odom_msg.pose.pose.position
    quaternion = (
        new_odom_msg.pose.pose.orientation.x,
        new_odom_msg.pose.pose.orientation.y,
        new_odom_msg.pose.pose.orientation.z,
        new_odom_msg.pose.pose.orientation.w,)
    rpy = euler_from_quaternion(quaternion)
    rpy_odom.pose.pose.orientation.x = rpy[0] * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = rpy[1] * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = rpy[2] * 180 / math.pi
    odom_rpy_pub.publish(rpy_odom)


if __name__ == '__main__':
    rospy.init_node('vins_odom_republisher_in_world_frame')

    # Create a publisher for the transformed odometry message
    odom_pub = rospy.Publisher('/vins_estimator/odometry_world_frame', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('/vins_estimator/odometry_world_frame_rpy', Odometry, queue_size=10)

    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/vins_estimator/odometry', Odometry, odom_callback)

    # Spin the ROS node
    rospy.spin()
