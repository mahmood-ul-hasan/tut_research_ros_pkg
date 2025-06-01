import open3d as o3d
import numpy as np


def load_point_clouds(voxel_size=0.0):
    pcds = []

    # Load the provided point cloud files
    pcd1 = o3d.io.read_point_cloud("/media/aisl2/aisl_data/code_ws/pairwise_incremental_registration_using_pcl/build_5_varying_boom_Angle_during_cycle/capture00001.pcd")
    pcd1_down = pcd1.voxel_down_sample(voxel_size=voxel_size)
    pcds.append(pcd1_down)

    pcd2 = o3d.io.read_point_cloud("/media/aisl2/aisl_data/code_ws/pairwise_incremental_registration_using_pcl/build_5_varying_boom_Angle_during_cycle/capture00002.pcd")
    pcd2_down = pcd2.voxel_down_sample(voxel_size=voxel_size)
    pcds.append(pcd2_down)

    pcd3 = o3d.io.read_point_cloud("/media/aisl2/aisl_data/code_ws/pairwise_incremental_registration_using_pcl/build_5_varying_boom_Angle_during_cycle/capture00003.pcd")
    pcd3_down = pcd3.voxel_down_sample(voxel_size=voxel_size)
    pcds.append(pcd3_down)

    return pcds


def compute_normal_and_assign_colors(pcds):
    for pcd in pcds:
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        o3d.visualization.draw_geometries([pcd])


def pairwise_registration(source, target, max_correspondence_distance_coarse,
                          max_correspondence_distance_fine):
    print("Apply point-to-plane ICP")
    source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

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
                pcds[source_id], pcds[target_id],
                max_correspondence_distance_coarse,
                max_correspondence_distance_fine)
            print(f"Build PoseGraph between node {source_id} and {target_id}")
            if target_id == source_id + 1:  # Odometry case
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
            else:  # Loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph


if __name__ == "__main__":
    voxel_size = 2
    pcds_down = load_point_clouds(voxel_size)

    print("Compute normal vectors and assign colors ...")
    compute_normal_and_assign_colors(pcds_down)

    print("Full registration ...")
    max_correspondence_distance_coarse = voxel_size * 30
    max_correspondence_distance_fine = voxel_size * 5
    pose_graph = full_registration(pcds_down,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=max_correspondence_distance_fine,
        edge_prune_threshold=2.5,
        reference_node=0)
    o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)

    print("Transform points and display")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)

    o3d.visualization.draw_geometries(pcds_down)
