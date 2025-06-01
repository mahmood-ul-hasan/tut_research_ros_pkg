#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points, create_cloud
from std_msgs.msg import Header

# Convert ROS PointCloud2 to Open3D point cloud
def ros_to_o3d(ros_cloud):
    points = np.array([[p[0], p[1], p[2]] for p in read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True)])
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(points)
    return o3d_cloud

# Convert Open3D point cloud to ROS PointCloud2
from sensor_msgs.msg import PointField
import struct

# Convert Open3D point cloud to ROS PointCloud2
def o3d_to_ros(o3d_cloud, frame_id="velodyne"):
    points = np.asarray(o3d_cloud.points, dtype=np.float32)

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Define PointField structure
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # Pack points into binary data
    point_cloud_data = []
    for p in points:
        point_cloud_data.append(struct.pack('fff', *p))
    point_cloud_data = b''.join(point_cloud_data)

    # Create and return the PointCloud2 message
    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=12,  # Each point consists of 3 floats (4 bytes each)
        row_step=12 * len(points),
        data=point_cloud_data,
        is_dense=True
    )


# Main CropBox Filter class
class CropBoxFilter:
    def __init__(self):
        rospy.init_node('crop_box_filter', anonymous=True)

        # CropBox parameters
        self.min_x = rospy.get_param('~min_x', -1e6)
        self.max_x = rospy.get_param('~max_x', 1e6)
        self.min_y = rospy.get_param('~min_y', -1e6)
        self.max_y = rospy.get_param('~max_y', 1e6)
        self.min_z = rospy.get_param('~min_z', -1e6)
        self.max_z = rospy.get_param('~max_z', 1e6)

        # Subscribing and publishing topics
        self.sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.cloud_callback)
        self.pub = rospy.Publisher('/velodyne_points_filter', PointCloud2, queue_size=10)

    def cloud_callback(self, ros_cloud):
        # Convert ROS PointCloud2 to Open3D point cloud
        o3d_cloud = ros_to_o3d(ros_cloud)

        # Define the crop box limits
        bounding_box = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=np.array([self.min_x, self.min_y, self.min_z]),
            max_bound=np.array([self.max_x, self.max_y, self.max_z])
        )

        # Apply the crop box filter
        cropped_cloud = o3d_cloud.crop(bounding_box)

        # Convert filtered Open3D point cloud back to ROS PointCloud2
        filtered_ros_cloud = o3d_to_ros(cropped_cloud)

        # Publish the filtered cloud
        self.pub.publish(filtered_ros_cloud)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = CropBoxFilter()
        node.run()
    except rospy.ROSInterruptException:
        pass
