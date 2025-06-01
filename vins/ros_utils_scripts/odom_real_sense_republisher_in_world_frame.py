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
    # print(t)
    br.sendTransform(t)


def odom_callback(odom, listener):
    global initial_yaw_value
    global initial_pitch_value
    global initial_roll_value
    global initial_position

    try:
        # Convert the odometry message from the base frame to the world frame
        listener.waitForTransform('world', 'camera_odom_frame', rospy.Time(), rospy.Duration(5.0))

        # Create a PoseStamped message for pose2
        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = odom.pose.pose
        # Transform the pose from the base frame to the world frame
        transformed_pose = listener.transformPose('world', pose_stamped)

        # Extract the current orientation from the received odometry message
        orientation_quat = transformed_pose.pose.orientation
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
            initial_position = transformed_pose.pose.position

        # roll_world = - (pitch - initial_pitch_value)
        # pitch_world = - (roll - initial_roll_value)
        # yaw_world = yaw - initial_yaw_value

        roll_world = (pitch - initial_pitch_value)
        pitch_world = (roll - initial_roll_value)
        yaw_world = yaw - initial_yaw_value

        # Convert the modified orientation back to a quaternion
        q = quaternion_from_euler(roll_world, pitch_world, yaw_world)

        # Create a new odometry message with the modified orientation
        new_odom_msg = Odometry()
        new_odom_msg.header.stamp = rospy.Time.now()
        new_odom_msg.header.frame_id = "world"
        new_odom_msg.pose.pose.position.x = transformed_pose.pose.position.x - initial_position.x
        new_odom_msg.pose.pose.position.y = transformed_pose.pose.position.y - initial_position.y
        new_odom_msg.pose.pose.position.z = transformed_pose.pose.position.z - initial_position.z
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
        t.child_frame_id = "camera_pose_frame"
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

        # print(odom)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Failed to transform odometry from camera_odom_frame frame to world frame: %s", str(e))


if __name__ == '__main__':
    rospy.init_node('gt_odom_republisher_in_world_frame')

    # Create a TransformListener object
    listener = tf.TransformListener()

    # Create a publisher for the transformed odometry message
    odom_pub = rospy.Publisher('/camera/odom/sample_world_frame', Odometry, queue_size=10)
    odom_rpy_pub = rospy.Publisher('/camera/odom/sample_rpy', Odometry, queue_size=10)

    # Subscribe to the base frame odometry topic
    rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback, listener)

    # Spin the ROS node
    # rospy.spin()
    # Create a ROS rate object to control the loop frequency
    rate = rospy.Rate(500)  # 10 Hz (adjust to your desired frequency)

    while not rospy.is_shutdown():
        # Add your custom logic here
        # This loop will keep running until the node is shut down
        # You can perform additional tasks within this loop

        static_transform_publisher("world", "camera_odom_frame", 0, 0, 0, 180 * math.pi / 180, 180 * math.pi / 180, 90 * math.pi / 180)
        static_transform_publisher("camera_pose_frame", "camera_imu_optical_frame", 0, 0.021, 0, 90 * math.pi / 180, 0 * math.pi / 180, 90 * math.pi / 180)

        # Sleep to control the loop rate
        rate.sleep()
