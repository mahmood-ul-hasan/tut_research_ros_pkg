#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import PointCloud2
import pcl
from pcl_conversions import toPCL
import os

class PointCloudTransformer:
    def __init__(self):
        rospy.init_node('pointcloud_transformer', anonymous=True)
        
        # Parameters
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/velodyne_points')
        self.target_frame = rospy.get_param('~target_frame', 'world')
        self.output_dir = rospy.get_param('~output_dir', '//media/aisl2/aisl_data/catkin_ws/src/pointcloud_registration/kobelco_exp_2024')
        self.save_rate = rospy.get_param('~save_rate', 2.0)  # Save one point cloud per second
        
        self.file_index = 1
        self.last_saved_time = rospy.Time.now()
        
        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Point cloud subscriber
        self.pc_subscriber = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_callback)
    
    def pointcloud_callback(self, msg):
        current_time = rospy.Time.now()
        if (current_time - self.last_saved_time).to_sec() < self.save_rate:
            # Skip if saving too frequently
            return

        try:
            # Lookup transform
            transform = self.tf_buffer.lookup_transform(self.target_frame, 
                                                        msg.header.frame_id, 
                                                        msg.header.stamp, 
                                                        rospy.Duration(1.0))
            
            # Transform the point cloud
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(msg, transform)
            
            # Save the transformed point cloud
            self.save_pointcloud(transformed_cloud)
            
            # Update last saved time
            self.last_saved_time = current_time
        
        except tf2_ros.TransformException as e:
            rospy.logwarn(f"Transform error: {e}")
    
    def save_pointcloud(self, cloud_msg):
        # Convert ROS PointCloud2 to PCL PointCloud
        pcl_cloud = toPCL(cloud_msg)
        
        # Generate unique filename
        filename = os.path.join(self.output_dir, f"cloud_{self.file_index}.pcd")
        pcl.save(pcl_cloud, filename)
        rospy.loginfo(f"PointCloud saved to {filename}")
        
        # Increment file index for the next cloud
        self.file_index += 1

if __name__ == '__main__':
    try:
        PointCloudTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

