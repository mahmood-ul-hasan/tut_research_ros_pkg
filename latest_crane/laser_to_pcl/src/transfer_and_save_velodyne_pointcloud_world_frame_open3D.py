#!/usr/bin/env python3

import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry


class VelodyneTransformer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('velodyne_transformer', anonymous=True)
        
        # Subscriber and Publisher
        self.pointcloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.pointcloud_callback)
        self.transformed_pub = rospy.Publisher('/velodyne_points_world', PointCloud2, queue_size=10)
        self.odom_pub = rospy.Publisher("/odom_world_to_velodyne", Odometry, queue_size=10)
        self.output_dir = rospy.get_param('~output_dir', '/media/aisl2/aisl_data/catkin_ws/src/PointCloudProcessing/pointcloud_registration/kobelco_exp_2024/world_frame_5_pitch_angle_40_and_80_yaw_cycle_slow')
        self.save_rate = rospy.get_param('~save_rate', 0.50)        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.file_index = 0
        self.last_saved_time = rospy.Time.now()
        self.target_frame = rospy.get_param('~target_frame', 'world')
    
    def pointcloud_callback(self, pointcloud_msg):
        try:
            # Lookup the transform from LiDAR frame to World frame
            transform = self.tf_buffer.lookup_transform(
                "world",  # Target frame
                pointcloud_msg.header.frame_id,  # Source frame (e.g., "velodyne")
                rospy.Time(0),  # Get the latest transform available
                rospy.Duration(1.0)  # Timeout duration
            )
            
                # Extract position (translation)
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Extract orientation (rotation in quaternion)
            qw = transform.transform.rotation.w
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
        
           # Create Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = transform.header.stamp
            odom_msg.header.frame_id = self.target_frame  # e.g., "world"
            odom_msg.child_frame_id = pointcloud_msg.header.frame_id   # e.g., "velodyne"
            # Set position (translation)
            odom_msg.pose.pose.position.x = tx
            odom_msg.pose.pose.position.y = ty
            odom_msg.pose.pose.position.z = tz
            odom_msg.pose.pose.orientation.w = qw
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            # Publish the message
            self.odom_pub.publish(odom_msg)     


            # Transform the PointCloud2 message
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(pointcloud_msg, transform)
            # Publish the transformed PointCloud2 message
            self.transformed_pub.publish(transformed_cloud)
        
            current_time = rospy.Time.now()
            if (current_time - self.last_saved_time).to_sec() >= self.save_rate:

                odom_filename = f"{self.output_dir}/pose.json"
                with open(odom_filename, "a") as file:
                    file.write(f"{tx} {ty} {tz} {qw} {qx} {qy} {qz}\n")
          
        # Convert transformed_cloud (PointCloud2) to a NumPy array
                # points = np.array([
                #     [p[0], p[1], p[2]] for p in pc2.read_points(pointcloud_msg, skip_nans=True)
                # ])

                points = np.array([
                    [p[0], p[1], p[2]] for p in pc2.read_points(transformed_cloud, skip_nans=True)
                ])
                
                # Create Open3D point cloud object
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)

                # Save the transformed point cloud as a PCD file
                filename = f"{self.output_dir}/pcd/{self.file_index}.pcd"
                o3d.io.write_point_cloud(filename, pcd)
                rospy.loginfo(f"PointCloud saved to {filename}")
                

                  # Increment file index for next cloud
                self.file_index += 1
                self.last_saved_time = current_time
           


        
        except tf2_ros.LookupException as e:
            rospy.logwarn(f"Transform lookup failed: {e}")
        except tf2_ros.TransformException as e:
            rospy.logwarn(f"Transform error: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        transformer = VelodyneTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down Velodyne Transformer node.")






# import rospy
# import tf
# import tf2_ros
# import open3d as o3d
# import numpy as np
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs import point_cloud2

# class PointCloudTransformer:
#     def __init__(self):
#         rospy.init_node('pointcloud_transformer', anonymous=True)

#         # Parameters
#         self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/velodyne_points')
#         self.target_frame = rospy.get_param('~target_frame', 'world')
#         self.output_dir = rospy.get_param('~output_dir', '/media/aisl2/aisl_data/catkin_ws/src/pointcloud_registration/kobelco_exp_2024')
#         self.save_rate = rospy.get_param('~save_rate', 10.0)


#         self.file_index = 1
#         self.last_saved_time = rospy.Time.now()

#         # TF Listener
#         self.tf_listener = tf.TransformListener()

#         # Point cloud subscriber
#         self.pc_subscriber = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)

#         # Publisher for transformed point clouds
#         self.transformed_pc_publisher = rospy.Publisher('velodyne_points_world', PointCloud2, queue_size=10)

#     def transform_to_matrix(self, transform, rotation):
#         """Convert translation and quaternion to a 4x4 transformation matrix."""
#         # Convert quaternion to a rotation matrix
#         rotation_matrix = tf.transformations.quaternion_matrix(rotation)

#         # Add translation to the matrix
#         rotation_matrix[0:3, 3] = transform
#         return rotation_matrix

#     def pointcloud_callback(self, msg):
#         current_time = rospy.Time.now()

#         # Only process the point cloud if the rate interval has passed
#         if (current_time - self.last_saved_time).to_sec() >= self.save_rate:
#             try:
#                 # Lookup the transform from the target frame to the point cloud frame
#                 self.tf_listener.waitForTransform(self.target_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
#                 (trans, rot) = self.tf_listener.lookupTransform(self.target_frame, msg.header.frame_id, rospy.Time(0))

#                 # Convert transform to a matrix
#                 transform_matrix = self.transform_to_matrix(trans, rot)

#                 # Convert PointCloud2 to numpy array using point_cloud2
#                 pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
#                 points = np.array(list(pc_data))

#                 # Transform points
#                 ones = np.ones((points.shape[0], 1))
#                 homogeneous_points = np.hstack((points, ones))  # Convert to homogeneous coordinates
#                 transformed_points = (transform_matrix @ homogeneous_points.T).T[:, :3]  # Back to 3D

#                 # Create Open3D point cloud object
#                 pcd = o3d.geometry.PointCloud()
#                 pcd.points = o3d.utility.Vector3dVector(transformed_points)

#                 # Save the transformed point cloud as a PCD file
#                 filename = f"{self.output_dir}/cloud_{self.file_index}.pcd"
#                 o3d.io.write_point_cloud(filename, pcd)
#                 rospy.loginfo(f"PointCloud saved to {filename}")

#                 # Publish the transformed point cloud
#                 transformed_pc = point_cloud2.create_cloud_xyz32(msg.header, transformed_points.tolist())
#                 self.transformed_pc_publisher.publish(transformed_pc)

#                 # Increment file index for next cloud
#                 self.file_index += 1
#                 self.last_saved_time = current_time

#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#                 rospy.logwarn(f"Error in transforming point cloud: {e}")

# if __name__ == '__main__':
#     try:
#         PointCloudTransformer()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
