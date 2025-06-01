import numpy as np
import open3d as o3d
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import rospy

def open3d_to_ros(open3d_cloud, frame_id="world"):
    points = np.asarray(open3d_cloud.points)
    colors = np.asarray(open3d_cloud.colors)

    # Combine points and colors
    if colors.size == 0:
        cloud_data = points
    else:
        colors = (colors * 255).astype(np.uint8)
        cloud_data = np.hstack([points, colors])

    # Define fields for ROS PointCloud2
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    if colors.size > 0:
        fields.extend([
            PointField('r', 12, PointField.UINT8, 1),
            PointField('g', 13, PointField.UINT8, 1),
            PointField('b', 14, PointField.UINT8, 1),
        ])

    # Create ROS PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    ros_cloud = pc2.create_cloud(header, fields, cloud_data)
    return ros_cloud


# Convert ROS PointCloud2 to Open3D PointCloud
def ros_to_open3d(ros_cloud):
    # Extract points from ROS PointCloud2
    field_names = [field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names=field_names))

    # Convert to Open3D point cloud
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data) > 0:
        points = np.array([(x, y, z) for x, y, z, _ in cloud_data])
        open3d_cloud.points = o3d.utility.Vector3dVector(points)

        # If RGB data is available
        if 'rgb' in field_names:
            colors = np.array([_rgb_to_float(c) for _, _, _, c in cloud_data]) / 255.0
            open3d_cloud.colors = o3d.utility.Vector3dVector(colors)

    return open3d_cloud

