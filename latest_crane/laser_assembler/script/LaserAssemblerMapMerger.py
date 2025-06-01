#!/usr/bin/env python3

import rospy 
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2


rospy.init_node("assemble_scans_to_cloud")
rospy.wait_for_service("assemble_scans2")
assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
pub = rospy.Publisher ("/laser_pointcloud_assembler", PointCloud2, queue_size=1)
pub_current_map = rospy.Publisher ("/current_map", PointCloud2, queue_size=1)
pub_pervious_map = rospy.Publisher ("/pervious_map", PointCloud2, queue_size=1)

r = rospy.Rate (3)
pervious_time = rospy.Time(0,0)
pervious_map = None  # Initialize map_cycle initially
current_map = None  # Initialize map_cycle initially


pcds = []
voxel_size = 1
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5

def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")
    icp_coarse = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph


def save_pointcloud_in_pcd(msg, file_name):
    
    cloud = pc2.read_points(msg, field_names=("x", "y", "z"))
    cloud_list = []
    for p in cloud:
        cloud_list.append([p[0], p[1], p[2]])

    # Convert the PCL PointCloud to a numpy array
    cloud_np = np.array(cloud_list, dtype=np.float32)

    # Create an Open3D PointCloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(cloud_np)

    # Save the PointCloud as a PCD file
    o3d.io.write_point_cloud(f"{file_name}.pcd", pcd)





while (True):
    try:
        current_time = rospy.get_rostime()
        resp = assemble_scans(pervious_time, current_time)
        print("Got cloud with points",  len(resp.cloud.data))
        pub.publish (resp.cloud)

    
        if current_time >= pervious_time + rospy.Duration.from_sec(60*2):  # 60.1 = One minute and one tenth of a second
             current_map =  pervious_map
             pervious_map = resp.cloud
             pervious_time = current_time


        
        if current_map is not None:
           pub_current_map.publish(current_map)
           print("current_map")
           save_pointcloud_in_pcd(current_map, "current_map")


        if pervious_map is not None:
         pub_pervious_map.publish(pervious_map)
         print("pervious_map")
         save_pointcloud_in_pcd(pervious_map, "pervious_map")


        # if current_map and pervious_map is not None:
                         # Convert ROS PointCloud2 to NumPy array
            # pc_data = pc2.read_points(current_map, field_names=("x", "y", "z"), skip_nans=True)
            # pc_np = np.array(list(pc_data))
            # pervious_point_cloud = o3d.geometry.PointCloud()
            # pervious_point_cloud.points = o3d.utility.Vector3dVector(pc_np)
            # pervious_point_cloud_down = pervious_point_cloud.voxel_down_sample(voxel_size=voxel_size)
            # pcds.append(pervious_point_cloud_down)
            
            # current_point_cloud = o3d.geometry.PointCloud()
            # current_point_cloud.points = o3d.utility.Vector3dVector(current_map.data[:, :3])  # Assuming the data includes XYZ coordinates
            # current_point_cloud_down = current_point_cloud.voxel_down_sample(voxel_size=voxel_size)
            # pcds.append(current_point_cloud_down)

            # o3d.visualization.draw_geometries(pcds,
            #                       zoom=0.3412,
            #                       front=[0.4257, -0.2125, -0.8795],
            #                       lookat=[2.6172, 2.0475, 1.532],
            #                       up=[-0.0694, -0.9768, 0.2024])


            #  pub_current_map.publish(current_map)
            #  pub_pervious_map.publish(pervious_map)



    except rospy.ServiceException as e:
        print("Service call failed: ", e)

    r.sleep()