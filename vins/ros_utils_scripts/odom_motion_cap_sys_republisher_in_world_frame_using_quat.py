#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_inverse
from geometry_msgs.msg import Pose, Quaternion

initial_orientation = None
initial_position = None


def static_transform_publisher(header, child, x, y, z, r, p, h):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    quat_static = quaternion_from_euler(r, p, h)

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = quat_static[0]
    t.transform.rotation.y = quat_static[1]
    t.transform.rotation.z = quat_static[2]
    t.transform.rotation.w = quat_static[3]
    br.sendTransform(t)


def odom_callback(odom, listener):
    global initial_orientation
    global initial_position

    try:
        # Convert the odometry message from the base frame to the world frame
        listener.waitForTransform('world', 'odom_mot_cap_sys', rospy.Time(), rospy.Duration(5.0))

        # Create a PoseStamped message for pose2
        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = odom.pose.pose
        # Transform the pose from the base frame to the world frame
        transformed_pose = listener.transformPose('world', pose_stamped)

        # adding initial rotation
        original_quaternion = transformed_pose.pose.orientation
        rotation_quaternion = quaternion_from_euler(-90 * math.pi / 180, 0, 0)  # 180 degrees in radians
        rotated_quaternion = quaternion_multiply([original_quaternion.x, original_quaternion.y, original_quaternion.z, original_quaternion.w], rotation_quaternion)
        transformed_pose.pose.orientation.x = rotated_quaternion[0]
        transformed_pose.pose.orientation.y = rotated_quaternion[1]
        transformed_pose.pose.orientation.z = rotated_quaternion[2]
        transformed_pose.pose.orientation.w = rotated_quaternion[3]

        # Check if this is the first time the callback is called
        if initial_orientation is None:
            initial_orientation = (
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w)

        if initial_position is None:
            initial_position = transformed_pose.pose.position

        # Calculate the relative quaternion using quaternion multiplication
        current_orientation = (
            transformed_pose.pose.orientation.x,
            transformed_pose.pose.orientation.y,
            transformed_pose.pose.orientation.z,
            transformed_pose.pose.orientation.w
        )

        # Calculate the relative quaternion using quaternion multiplication
        relative_rotation = quaternion_multiply(
            quaternion_inverse(initial_orientation), current_orientation)
        inv_roll, inv_pitch, inv_yaw = euler_from_quaternion(relative_rotation)
        print("relative_rotation      ", round(inv_roll * 180 / math.pi, 3), round(inv_pitch * 180 / math.pi, 3), round(inv_yaw * 180 / math.pi, 3))

        # Create a new odometry message with the modified orientation
        new_odom_msg = Odometry()
        new_odom_msg.header.stamp = rospy.Time.now()
        new_odom_msg.header.frame_id = "world"
        new_odom_msg.pose.pose.position.x = transformed_pose.pose.position.x - initial_position.x
        new_odom_msg.pose.pose.position.y = transformed_pose.pose.position.y - initial_position.y
        new_odom_msg.pose.pose.position.z = transformed_pose.pose.position.z - initial_position.z
        new_odom_msg.pose.pose.orientation = Quaternion(x=relative_rotation[0], y=relative_rotation[1], z=relative_rotation[2], w=relative_rotation[3])

        odom_pub.publish(new_odom_msg)

        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = "world"
        t.child_frame_id = "motion_cap_sys"
        # t.child_frame_id = "camera_pose_frame_quat"
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

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Failed to transform odometry from camera_odom_frame frame to world frame: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('motion_cap_sys_republisher_in_world_frame_using_quaternion')

    # Create a TransformListener object
    listener = tf.TransformListener()

    # Create a publisher for the transformed odometry message
    odom_pub = rospy.Publisher('/odom_motion_cap_sys_world_frame', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('/odom_motion_cap_sys_world_frame_rpy', Odometry, queue_size=10)

    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/odom_motion_cap_sys', Odometry, odom_callback, listener)

    rate = rospy.Rate(1000)  # 10 Hz (adjust to your desired frequency)
    # rospy.spin()

    while not rospy.is_shutdown():
        # Add your custom logic here
        # This loop will keep running until the node is shut down
        # You can perform additional tasks within this loop
        static_transform_publisher("world", "odom_mot_cap_sys", 0, 0, 0, 93 * math.pi / 180, 0 * math.pi / 180, 110 * math.pi / 180)
        # Sleep to control the loop rate
        rate.sleep()
