#!/usr/bin/env python


# The code you provided computes and publishes the transformation from the estimated_path frame to the ground_truth_path frame. 
# The transformation is computed from estimated_pose to ground_truth_pose, 
# which means it represents the transformation from the estimated_path frame to the ground_truth_path frame.


import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import math
    

if __name__ == '__main__':
    
    rospy.init_node('transformation_publisher')
    # Create a TF broadcaster
    br = tf.TransformBroadcaster()

    # Assuming you have two nav_msgs/Path topics for estimated and ground truth
    estimated_pose_topic = "/odom_crane_structural_info_path"
    ground_truth_pose_topic = "/camera/odom/sample_path"

    rate = rospy.Rate(10.0)  # Set your desired publishing rate
    pose_transform_pub = rospy.Publisher('/pose_error', PoseStamped, queue_size=10)

    
    while not rospy.is_shutdown():
        try:
            # Get the latest estimated and ground truth paths
            estimated_path = rospy.wait_for_message(estimated_pose_topic, Path)
            ground_truth_path = rospy.wait_for_message(ground_truth_pose_topic, Path)

            # Extract the latest poses from the paths (e.g., the last pose)
            estimated_pose = estimated_path.poses[-1]
            ground_truth_pose = ground_truth_path.poses[-1]

            # Compute the transformation from estimated_pose to ground_truth_pose
            transformation_eg = tf.transformations.concatenate_matrices(
                tf.transformations.inverse_matrix(
                    tf.transformations.quaternion_matrix(
                        [estimated_pose.pose.orientation.x,
                         estimated_pose.pose.orientation.y,
                         estimated_pose.pose.orientation.z,
                         estimated_pose.pose.orientation.w]
                    )
                ),
                tf.transformations.translation_matrix(
                    [estimated_pose.pose.position.x,
                     estimated_pose.pose.position.y,
                     estimated_pose.pose.position.z]
                ),
                tf.transformations.translation_matrix(
                    [ground_truth_pose.pose.position.x,
                     ground_truth_pose.pose.position.y,
                     ground_truth_pose.pose.position.z]
                ),
                tf.transformations.quaternion_matrix(
                    [ground_truth_pose.pose.orientation.x,
                     ground_truth_pose.pose.orientation.y,
                     ground_truth_pose.pose.orientation.z,
                     ground_truth_pose.pose.orientation.w]
                )
            )

            # Compute the transformation from ground_truth_pose to estimated_pose (inverse)
            transformation_ge = tf.transformations.inverse_matrix(transformation_eg)

            # Extract translation (x, y, z) from both transformations
            translation_eg = tf.transformations.translation_from_matrix(transformation_eg)
            translation_ge = tf.transformations.translation_from_matrix(transformation_ge)

            # Extract rotation (roll, pitch, yaw) in radians from both transformations
            rotation_eg = tf.transformations.euler_from_matrix(transformation_eg, 'sxyz')
            rotation_ge = tf.transformations.euler_from_matrix(transformation_ge, 'sxyz')

            # Print the transformations
            print("Transformation (estimated to ground truth \n - X: {:.3f}, Y: {:.3f}, Z: {:.3f}, Roll: {:.3f}, Pitch: {:.3f}, Yaw: {:.3f}".format(
                translation_eg[0], translation_eg[1], translation_eg[2],
                math.degrees(rotation_eg[0]), math.degrees(rotation_eg[1]), math.degrees(rotation_eg[2])))
            

            print("Transformation (ground truth to estimated) \n - X: {:.3f}, Y: {:.3f}, Z: {:.3f}, Roll: {:.3f}, Pitch: {:.3f}, Yaw: {:.3f}".format(
                translation_ge[0], translation_ge[1], translation_ge[2],
                math.degrees(rotation_ge[0]), math.degrees(rotation_ge[1]), math.degrees(rotation_ge[2])))
            

            pose_transform = PoseStamped()
            # Set the header timestamp
            pose_transform.header.stamp = rospy.Time.now()
            # pose_error.pose.position = Point(x_error[-1], y_error[-1], z_error[-1])
            # pose_error.pose.orientation = Quaternion(roll_error[-1], pitch_error[-1], yaw_error[-1], 0)
            pose_transform.pose.position.x = translation_eg[0]
            pose_transform.pose.position.y = translation_eg[1]
            pose_transform.pose.position.z = translation_eg[2]
            pose_transform.pose.orientation.x = math.degrees(rotation_eg[0])
            pose_transform.pose.orientation.y = math.degrees(rotation_eg[1])
            pose_transform.pose.orientation.z = math.degrees(rotation_eg[2])
            pose_transform.pose.orientation.w = 0
            pose_transform_pub.publish(pose_transform)



            # # Publish the transformation from estimated_pose to ground_truth_pose
            # br.sendTransform(
            #     (translation_eg[0], translation_eg[1], translation_eg[2]),
            #     tf.transformations.quaternion_from_matrix(transformation_eg),
            #     rospy.Time.now(),
            #     "estimated_to_ground_truth_frame",
            #     "ground_truth_frame"
            # )

            # # Publish the transformation from ground_truth_pose to estimated_pose
            # br.sendTransform(
            #     (translation_ge[0], translation_ge[1], translation_ge[2]),
            #     tf.transformations.quaternion_from_matrix(transformation_ge),
            #     rospy.Time.now(),
            #     "ground_truth_to_estimated_frame",
            #     "estimated_frame"
            # )

        except (rospy.ROSInterruptException, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

