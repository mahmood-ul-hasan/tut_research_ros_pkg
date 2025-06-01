#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

namespace hdl_graph_slam {

InformationMatrixCalculator::InformationMatrixCalculator(){ //ros::NodeHandle& nh) {
  use_const_inf_matrix = false; //nh.param<double>("use_const_inf_matrix", false);
  const_stddev_xyz = 400; //nh.param<double>("const_stddev_x", 0.5);
  const_stddev_rpy = 400; //nh.param<double>("const_stddev_q", 0.1);


  const_stddev_x = 40; 
  const_stddev_y = 40; 
  const_stddev_z = 600; 
  const_stddev_q1 = 600;
  const_stddev_q2 = 40;
  const_stddev_q3 = 40;

  var_gain_a = .20; //nh.param<double>("var_gain_a", 20.0);
  min_stddev_x = 0.1; //nh.param<double>("min_stddev_x", 0.1);
  max_stddev_x = .050; //nh.param<double>("max_stddev_x", 5.0);
  min_stddev_q = 0.05; //nh.param<double>("min_stddev_q", 0.05);
  max_stddev_q = 0.2; //nh.param<double>("max_stddev_q", 0.2);
  fitness_score_thresh = 0.01; //nh.param<double>("fitness_score_thresh", 0.5);
}

InformationMatrixCalculator::~InformationMatrixCalculator() {}

Eigen::MatrixXd InformationMatrixCalculator::calc_const_information_matrix() const {
  if(use_const_inf_matrix) {
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    // inf.topLeftCorner(3, 3).array() /= const_stddev_xyz;
    // inf.bottomRightCorner(3, 3).array() /= const_stddev_rpy;

    inf(0,0) /= const_stddev_x;
    inf(1,1) /= const_stddev_y;
    inf(2,2) /= const_stddev_z;
    inf(3,3) /= const_stddev_q1;
    inf(4,4) /= const_stddev_q2;
    inf(5,5) /= const_stddev_q3;
    return inf;
  }
}


Eigen::MatrixXd InformationMatrixCalculator::calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose) const {
  if(use_const_inf_matrix) {
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= const_stddev_xy;
    inf.bottomRightCorner(3, 3).array() /= const_stddev_q1;
    return inf;
  }

  double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);
  std::cout << " fitness_score: " << fitness_score;
 
 fitness_score_vec.push_back(fitness_score);
  double min_var_x = std::pow(min_stddev_x, 2);
  double max_var_x = std::pow(max_stddev_x, 2);
  double min_var_q = std::pow(min_stddev_q, 2);
  double max_var_q = std::pow(max_stddev_q, 2);

  float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
  float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

  Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
  inf.topLeftCorner(3, 3).array() /= w_x;
  inf.bottomRightCorner(3, 3).array() /= w_q;
  return inf;
}

double InformationMatrixCalculator::calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range) {
  pcl::search::KdTree<PointT>::Ptr tree_(new pcl::search::KdTree<PointT>());
  tree_->setInputCloud(cloud1);

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  pcl::PointCloud<PointT> input_transformed;
  pcl::transformPointCloud (*cloud2, input_transformed, relpose.cast<float>());

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;

  std::cout       << " (squared distance:" << std::endl;

  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

  // std::cout << "    "  <<   (*cloud1)[ nn_indices[0] ].x 
  //               << " " << (*cloud1)[ nn_indices[0] ].y 
  //               << " " << (*cloud1)[ nn_indices[0] ].z 
  //               << " (squared distance: " << nn_dists[0] << ")" << std::endl;

  //         std::cout       << " input_transformed.points[i] " << input_transformed.points[i] << "  " << cloud1->points[i] << std::endl;

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] <= max_range)
    {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }


  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());
}

}

