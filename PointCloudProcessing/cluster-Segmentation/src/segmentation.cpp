#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void cloudCallback(const PointCloud::ConstPtr& cloud_in)
{
// Voxel grid downsampling
std::cout << " Voxel grid downsampling" << std::endl;
pcl::VoxelGrid<pcl::PointXYZ> vg;
vg.setInputCloud(cloud_in);
vg.setLeafSize(0.5f, 0.5f, 0.5f);
PointCloud::Ptr cloud_downsampled(new PointCloud);
vg.filter(*cloud_downsampled);

// Statistical outlier removal
std::cout << " Statistical outlier removal" << std::endl;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
sor.setInputCloud(cloud_downsampled);
sor.setMeanK(50);
sor.setStddevMulThresh(1.0);
PointCloud::Ptr cloud_filtered(new PointCloud);
sor.filter(*cloud_filtered);

//=============================================================
// Plane segmentation for ground
std::cout << " Ground plane segmentation" << std::endl;
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
seg.setInputCloud(cloud_filtered);
seg.setModelType(pcl::SACMODEL_PLANE);
seg.setMethodType(pcl::SAC_RANSAC);
seg.setDistanceThreshold(0.8);
seg.segment(*inliers, *coefficients);

// Extract ground plane points
std::cout << " Extract ground plane points " << std::endl;
pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
extract_indices.setInputCloud(cloud_filtered);
extract_indices.setIndices(inliers);
extract_indices.setNegative(false);
PointCloud::Ptr cloud_ground(new PointCloud);
extract_indices.filter(*cloud_ground);

// Extract non-ground points
std::cout << " Extract non-ground points " << std::endl;
extract_indices.setNegative(true);
PointCloud::Ptr cloud_non_ground(new PointCloud);
extract_indices.filter(*cloud_non_ground);

// Extract walls
PointCloud::Ptr cloud_walls(new PointCloud);
PointCloud::Ptr cloud_remaining(new PointCloud);
*cloud_remaining = *cloud_non_ground;
int wall_count = 0;
// Set a maximum number of iterations to prevent infinite loop
const int MAX_ITERATIONS = 5;

while (wall_count < MAX_ITERATIONS && cloud_remaining->size() > 0)
{
    std::cout << " Wall plane segmentation " << wall_count << std::endl;
    inliers->indices.clear();
    coefficients.reset(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud_remaining);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(0.1);
    seg.setDistanceThreshold(.5);

seg.setOptimizeCoefficients(false); // disable least-squares optimization
seg.setMaxIterations(5); // set maximum RANSAC iterations
seg.setProbability(0.99); // set RANSAC success probability
seg.setRadiusLimits(0.0, 8.0);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
        break;

        if (cloud_remaining->size() < 5000) 
        break;

    // Extract wall points
    std::cout << " Extract wall points " << wall_count << std::endl;
    extract_indices.setInputCloud(cloud_remaining);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(false);
    PointCloud::Ptr cloud_wall(new PointCloud);
    extract_indices.filter(*cloud_wall);
    *cloud_walls += *cloud_wall;

    // Extract non-wall points
    std::cout << " Extract non-wall points " << wall_count << std::endl;
    extract_indices.setNegative(true);
    PointCloud::Ptr cloud_non_walls(new PointCloud);
    extract_indices.filter(*cloud_non_walls);
    *cloud_remaining = *cloud_non_walls;
    wall_count++;

}


      // Publish non ground plane points
    ros::NodeHandle nh;
    ros::Publisher pub_cloud_filtered = nh.advertise<PointCloud>("cloud_objects", 1);
    for (int i = 0; i < 5; i++) {
        pub_cloud_filtered.publish(cloud_remaining);
        ros::spinOnce();
        ros::Duration(0.10).sleep(); // Wait for 1 second
    }

    std::cout << " Ended Wall plane segmentation " << wall_count << std::endl;
    std::cout << " ----------------------------- " << wall_count << std::endl;


//==================================================================================

    // Euclidean cluster segmentation
    std::cout << " Euclidean cluster segmentation " << std::endl;
    // std::cout << "cloud_remaining points " << cloud_remaining->size() << std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_remaining);
    // std::cout << "Number of points in search tree: " << tree->getInputCloud()->size() << std::endl;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.8);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_remaining);
    ec.extract(cluster_indices);
    std::cout << " cluster_indices: " << cluster_indices.size() << std::endl;


// Publish segmented objects
std::cout << " Publish segmented objects " << std::endl;
int cluster_num = 0;
for (auto indices : cluster_indices)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract_clusters;
    extract_clusters.setInputCloud(cloud_remaining);
    extract_clusters.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    PointCloud::Ptr cloud_cluster(new PointCloud);
    extract_clusters.filter(*cloud_cluster);
    std::cout << "cloud_cluster points " << cloud_cluster->size() << std::endl;

    // Publish cluster points as a new topic
    std::stringstream ss;
    ss << "output_cluster_" << cluster_num;
    std::string topic_name = ss.str();
    ros::Publisher pub_cluster = nh.advertise<PointCloud>(topic_name, 1);
    
    
  // Publish cluster points
         for (int i = 0; i < 5; i++) {
        pub_cluster.publish(cloud_cluster);
        ros::spinOnce();
        ros::Duration(0.050).sleep(); 
    }
    
    std::cout << " cluster_num " << cluster_num << " cloud_cluster points " << cloud_cluster->size() << std::endl;
    cluster_num++;
}


    // // Publish segmented objects
    // std::cout << " Publish segmented objects " << std::endl;
    // ros::Publisher pub_clusters = nh.advertise<PointCloud>("output_clusters", 1);
    // PointCloud::Ptr cloud_clusters(new PointCloud);
    // std::cout << "Number of loop iterations: " << cluster_indices.size() << std::endl;
    // for (auto indices : cluster_indices)
    // {
    //     pcl::ExtractIndices<pcl::PointXYZ> extract_clusters;
    //     extract_clusters.setInputCloud(cloud_remaining);
    //     extract_clusters.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    //     PointCloud::Ptr cloud_cluster(new PointCloud);
    //     extract_clusters.filter(*cloud_cluster);
    //     *cloud_clusters += *cloud_cluster;

    //   // Publish cluster points
    //      for (int i = 0; i < 5; i++) {
    //     pub_clusters.publish(cloud_cluster);
    //     ros::spinOnce();
    //     ros::Duration(0.10).sleep(); // Wait for 1 second
    // }
    //   std::cout << "cloud_cluster points " << cloud_cluster->size() << std::endl;
    //   std::cout << "cloud_clusters points " << cloud_clusters->size() << std::endl;
    // // std::cout << *cloud_clusters << std::endl;
    // }

    std::cout << " ====================================================================== " << wall_count << std::endl;

   
    

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_segmentation");
    std::cout << "ros_init." << std::endl;


    ros::NodeHandle nh;

    ros::Subscriber sub_cloud = nh.subscribe<PointCloud>("/cloud_pcd", 1, cloudCallback);

    ros::spin();

    return 0;
}
