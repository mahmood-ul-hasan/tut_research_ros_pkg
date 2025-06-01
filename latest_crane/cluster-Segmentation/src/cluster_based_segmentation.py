import rospy
import pcl

from sensor_msgs.msg import PointCloud2
from pcl_msgs.msg import PointIndices
from pcl_conversions import fromROSMsg, toROSMsg

cluster_pub = rospy.Publisher("cluster_topic", PointCloud2, queue_size=10)


def euclidean_clustering(msg):
    # Convert ROS point cloud message to PCL point cloud
    cloud = fromROSMsg(msg)

    # Apply a voxel grid filter
    voxel_grid = cloud.make_voxel_grid_filter()
    voxel_grid.set_leaf_size(0.1, 0.1, 0.1)
    cloud_filtered = voxel_grid.filter()

    # Apply a statistical outlier removal filter
    sor = cloud_filtered.make_statistical_outlier_filter()
    sor.set_mean_k(50)
    sor.set_std_dev_mul_thresh(1.0)
    cloud_filtered = sor.filter()

    # Apply Euclidean clustering
    tree = cloud_filtered.make_kdtree()
    ec = cloud_filtered.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.2)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(10000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Publish the clusters as separate point cloud messages
    for i, indices in enumerate(cluster_indices):
        indices_msg = PointIndices()
        indices_msg.header = msg.header
        indices_msg.indices = indices
        cluster_msg = toROSMsg(cloud_filtered, msg.header)
        cluster_msg.header.frame_id = "base_link"
        cluster_msg.width = len(indices)
        cluster_msg.height = 1
        cluster_msg.is_dense = True
        cluster_msg.fields = cloud.fields
        cluster_pub.publish(cluster_msg)


rospy.init_node('lidar_subscriber')
rospy.Subscriber('/cloud_pcd', PointCloud2, euclidean_clustering)
rospy.spin()
