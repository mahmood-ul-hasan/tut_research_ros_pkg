#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg
from tf.transformations import quaternion_multiply, quaternion_inverse, quaternion_conjugate
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf2_ros
from geometry_msgs.msg import Pose, Quaternion

# Global variables to store initial values
initial_orientation = None
initial_position = None


def odom_callback(odom):
    global initial_orientation
    global initial_position

# convert the odom into rpy and publish rpy
    rpy_odom = Odometry()
    rpy_odom.header.stamp = rospy.Time.now()
    rpy_odom.header.frame_id = "world"
    rpy_odom.pose.pose.position = odom.pose.pose.position
    quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w,)
    rpy = euler_from_quaternion(quaternion)
    rpy_odom.pose.pose.orientation.x = rpy[0] * 180 / math.pi
    rpy_odom.pose.pose.orientation.y = rpy[1] * 180 / math.pi
    rpy_odom.pose.pose.orientation.z = rpy[2] * 180 / math.pi
    odom_rpy_pub.publish(rpy_odom)



    # adding initial rotation
    original_quaternion = odom.pose.pose.orientation
    rotation_quaternion = quaternion_from_euler(0 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180)
    rotated_quaternion = quaternion_multiply([original_quaternion.x, original_quaternion.y, original_quaternion.z, original_quaternion.w], rotation_quaternion)
    pitch_orig, roll_orig, yaw_orig = euler_from_quaternion([rotated_quaternion[0], rotated_quaternion[1], rotated_quaternion[2], rotated_quaternion[3]])

    r = roll_orig   # r = pitch_orig
    p = pitch_orig  # p = -roll_orig
    y = yaw_orig    # y = yaw_orig

    rotated_quaternion = quaternion_from_euler(r, p, y)

    odom.pose.pose.orientation.x = rotated_quaternion[0]
    odom.pose.pose.orientation.y = rotated_quaternion[1]
    odom.pose.pose.orientation.z = rotated_quaternion[2]
    odom.pose.pose.orientation.w = rotated_quaternion[3]

    # Check if this is the first time the callback is called
    if initial_orientation is None:
        initial_orientation = (
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)
        print("Initial orientation set to", initial_orientation)

    if initial_position is None:
        initial_position = odom.pose.pose.position

       # Calculate the relative quaternion using quaternion multiplication
    current_orientation = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    )

    # Calculate the relative quaternion using quaternion multiplication

    relative_rotation = quaternion_multiply(quaternion_inverse(initial_orientation), current_orientation)

    inv_roll, inv_pitch, inv_yaw = euler_from_quaternion(relative_rotation)
    print("relative_rotation quat     ", round(inv_roll * 180 / math.pi, 3), round(inv_pitch * 180 / math.pi, 3), round(inv_yaw * 180 / math.pi, 3))

    # Create a new odometry message with the modified orientation
    new_odom_msg = Odometry()
    new_odom_msg.header.stamp = rospy.Time.now()
    new_odom_msg.header.frame_id = "world"
    new_odom_msg.pose.pose.position.x = odom.pose.pose.position.x - initial_position.x
    new_odom_msg.pose.pose.position.y = odom.pose.pose.position.y - initial_position.y
    new_odom_msg.pose.pose.position.z = odom.pose.pose.position.z - initial_position.z

    new_odom_msg.pose.pose.orientation = Quaternion(x=relative_rotation[0], y=relative_rotation[1], z=relative_rotation[2], w=relative_rotation[3])

    odom_pub.publish(new_odom_msg)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()

    t.header.frame_id = "world"
    t.child_frame_id = "vins"
    t.transform.translation = new_odom_msg.pose.pose.position
    t.transform.rotation = new_odom_msg.pose.pose.orientation
    br.sendTransform(t)

    # # convert the odom into rpy and publish rpy
    # rpy_odom = Odometry()
    # rpy_odom.header.stamp = rospy.Time.now()
    # rpy_odom.header.frame_id = "world"
    # rpy_odom.pose.pose.position = new_odom_msg.pose.pose.position
    # quaternion = (
    #     new_odom_msg.pose.pose.orientation.x,
    #     new_odom_msg.pose.pose.orientation.y,
    #     new_odom_msg.pose.pose.orientation.z,
    #     new_odom_msg.pose.pose.orientation.w,)
    # rpy = euler_from_quaternion(quaternion)
    # rpy_odom.pose.pose.orientation.x = rpy[0] * 180 / math.pi
    # rpy_odom.pose.pose.orientation.y = rpy[1] * 180 / math.pi
    # rpy_odom.pose.pose.orientation.z = rpy[2] * 180 / math.pi
    # odom_rpy_pub.publish(rpy_odom)
    # print("relative_rotation2     ", round(rpy[0] * 180 / math.pi, 3), round(rpy[1] * 180 / math.pi, 3), round(rpy[2] * 180 / math.pi, 3))


if __name__ == '__main__':
    rospy.init_node('vins_odom_republisher_in_world_frame_using_quaternion')

    # Create a publisher for the transformed odometry message
    odom_pub = rospy.Publisher('/vins_estimator/odometry_world_frame', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('/vins_estimator/odometry_world_frame_rpy', Odometry, queue_size=10)

    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/vins_estimator/odometry', Odometry, odom_callback)

    # Spin the ROS node
    rospy.spin()
