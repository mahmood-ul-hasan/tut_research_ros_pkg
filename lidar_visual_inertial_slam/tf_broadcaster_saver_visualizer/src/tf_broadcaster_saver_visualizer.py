#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
import math
import numpy as np
import os  # Importing os to check directory existence
import rospkg  # Importing rospkg to get the package path


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) to a quaternion."""
    quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion


def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) to a 3x3 rotation matrix."""
    rotation_matrix = tf_conversions.transformations.euler_matrix(roll, pitch, yaw)[:3, :3]
    return rotation_matrix


def create_4x4_transformation(translation, rotation_matrix):
    """Create a 4x4 transformation matrix from translation and rotation matrix."""
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation
    return transformation_matrix


def invert_transformation(transformation_matrix):
    """Invert a 4x4 transformation matrix."""
    rotation_inv = transformation_matrix[:3, :3].T  # Inverse of rotation matrix is its transpose
    translation_inv = -np.dot(rotation_inv, transformation_matrix[:3, 3])
    transformation_inv = np.eye(4)
    transformation_inv[:3, :3] = rotation_inv
    transformation_inv[:3, 3] = translation_inv
    return transformation_inv


def broadcast_transform(broadcaster, parent_frame, child_frame, translation, euler_angles):
    """Broadcast a transformation between two frames using Euler angles for rotation."""
    transform = geometry_msgs.msg.TransformStamped()

    # Set the time and frame IDs
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame

    # Set the translation
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]

    # Convert Euler angles to quaternion and set the rotation
    roll, pitch, yaw = euler_angles
    quaternion = euler_to_quaternion(roll, pitch, yaw)
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]

    # Broadcast the transformation
    broadcaster.sendTransform(transform)


def save_transforms_to_txt(transforms):
    """Save all transformations to a single text file."""
    # Get the package path
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('tf_broadcaster_saver_visualizer')
    
    # Specify the file path
    file_path = os.path.join(package_path, 'transform.txt')

    # Write the transformations to the text file
    with open(file_path, 'w') as f:
        for key, value in transforms.items():
            f.write(f"{key}:\n")
            f.write(f"  Translation: {value['translation']}\n")
            f.write(f"  Rotation Matrix:\n")
            for row in value['rotation_matrix']:
                f.write(f"    {row}\n")
            f.write(f"  Euler Angles (Radians): {value['euler_angles']}\n")
            f.write(f"  Quaternion: {value['quaternion']}\n")
            f.write(f"  4x4 Transformation Matrix:\n")
            for row in value['4x4_transformation_matrix']:
                f.write(f"    {row}\n")
            f.write("\n")


def collect_transforms(translation, euler_angles, parent_frame, child_frame):
    """Collect the transformation data for text storage."""
    roll, pitch, yaw = euler_angles
    rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)
    transformation_matrix = create_4x4_transformation(translation, rotation_matrix)
    
    # Convert Euler angles to quaternion
    quaternion = euler_to_quaternion(roll, pitch, yaw)

    # Prepare the transformation data
    transform_data = {
        f"{parent_frame} to {child_frame}": {
            "translation": translation,
            "rotation_matrix": rotation_matrix.tolist(),
            "euler_angles": [roll, pitch, yaw],  # Store Euler angles in radians
            "quaternion": quaternion.tolist(),  # Store quaternion
            "4x4_transformation_matrix": transformation_matrix.tolist()
        },
        f"{child_frame} to {parent_frame}": {
            "translation": invert_transformation(transformation_matrix)[:3, 3].tolist(),
            "rotation_matrix": invert_transformation(transformation_matrix)[:3, :3].tolist(),
            "euler_angles": tf_conversions.transformations.euler_from_matrix(invert_transformation(transformation_matrix)[:3, :3]),  # Extract Euler angles in radians
            "quaternion": euler_to_quaternion(*tf_conversions.transformations.euler_from_matrix(invert_transformation(transformation_matrix))),  # Store quaternion
            "4x4_transformation_matrix": invert_transformation(transformation_matrix).tolist()
        }
    }
    return transform_data


def combine_transformations(imu_to_lidar, lidar_to_camera):
    """Combine two 4x4 transformation matrices to get the transformation from IMU to Camera."""
    return np.dot(imu_to_lidar, lidar_to_camera)


def main():
    # Initialize the ROS node
    rospy.init_node('tf_broadcaster_and_saver_bidirectional', anonymous=True)

    # Create a TransformBroadcaster object
    broadcaster = tf2_ros.TransformBroadcaster()

    # Define the rate at which to broadcast transformations (10 Hz)
    rate = rospy.Rate(10)

    # Define the translations and Euler angles (roll, pitch, yaw) for the transformations
    imu_to_lidar_translation = (0, 0.0, 0.08)
    imu_to_lidar_euler = (math.radians(0), 0.0, 0.0)

    lidar_to_camera_translation = (0.0, 0.0, 0.13)
    lidar_to_camera_euler = (0.0, 0.0, math.radians(-90))

    # Initialize a dictionary to collect all transformation data
    all_transforms = {}

    while not rospy.is_shutdown():
        # Broadcast and collect IMU to LiDAR transform
        broadcast_transform(broadcaster, 'imu', 'lidar', imu_to_lidar_translation, imu_to_lidar_euler)
        all_transforms.update(collect_transforms(imu_to_lidar_translation, imu_to_lidar_euler, 'imu', 'lidar'))

        # Broadcast and collect LiDAR to Camera transform
        broadcast_transform(broadcaster, 'lidar', 'camera', lidar_to_camera_translation, lidar_to_camera_euler)
        all_transforms.update(collect_transforms(lidar_to_camera_translation, lidar_to_camera_euler, 'lidar', 'camera'))

        # Combine the IMU -> LiDAR and LiDAR -> Camera transformations
        imu_to_lidar_matrix = create_4x4_transformation(imu_to_lidar_translation, euler_to_rotation_matrix(*imu_to_lidar_euler))
        lidar_to_camera_matrix = create_4x4_transformation(lidar_to_camera_translation, euler_to_rotation_matrix(*lidar_to_camera_euler))

        imu_to_camera_matrix = combine_transformations(imu_to_lidar_matrix, lidar_to_camera_matrix)

        # Extract translation and rotation (as Euler angles) from the IMU -> Camera transformation
        imu_to_camera_translation = imu_to_camera_matrix[:3, 3]
        imu_to_camera_rotation_matrix = imu_to_camera_matrix[:3, :3]
        imu_to_camera_euler = tf_conversions.transformations.euler_from_matrix(imu_to_camera_rotation_matrix)

        # Broadcast and collect IMU to Camera transform
        broadcast_transform(broadcaster, 'imu', 'camera', imu_to_camera_translation, imu_to_camera_euler)
        all_transforms.update(collect_transforms(imu_to_camera_translation, imu_to_camera_euler, 'imu', 'camera'))

        # Save all transformations to the text file
        save_transforms_to_txt(all_transforms)

        # Sleep to maintain the desired rate
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
