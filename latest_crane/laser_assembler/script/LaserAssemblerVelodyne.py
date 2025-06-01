#!/usr/bin/env python3

import rospy 
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

from pcl_helper import ros_to_open3d, open3d_to_ros
import open3d as o3d


if __name__ == '__main__':

    rospy.init_node("LaserAssemblerVelodyne")

    # rospy.wait_for_service("laser_assembler_2/assemble_scans2")
    # assemble_scans = rospy.ServiceProxy('laser_assembler_2/assemble_scans2', AssembleScans2)

    rospy.wait_for_service("/assemble_scans2")
    assemble_scans = rospy.ServiceProxy('/assemble_scans2', AssembleScans2)
    # assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
    pub = rospy.Publisher ("/laser_pointcloud_assembler", PointCloud2, queue_size=1)

    r = rospy.Rate (0.2)

    while (True):
        try:
            resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            print("Got cloud with points",  len(resp.cloud.data))

             # Convert to Open3D point cloud
            open3d_cloud = ros_to_open3d(resp.cloud)

            # Apply voxel grid filter
            voxel_size = 0.5  # Adjust voxel size as needed
            downsampled_cloud = open3d_cloud.voxel_down_sample(voxel_size)

            # Convert back to ROS PointCloud2
            filtered_ros_cloud = open3d_to_ros(downsampled_cloud)

            # Publish filtered point cloud
            pub.publish(filtered_ros_cloud)
            print("Published filtered cloud with points:", len(downsampled_cloud.points))




            # pub.publish (resp.cloud)

        except rospy.ServiceException as e:
            print("Service call failed: ", e)

        r.sleep()