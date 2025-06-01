#ifndef INFORMATION_MATRIX_CALCULATOR_HPP
#define INFORMATION_MATRIX_CALCULATOR_HPP

//#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <math.h>
#include <thread>

namespace hdl_graph_slam {
  extern std::vector<double> fitness_score_vec;
  extern std::vector<double> weight_wx;
  extern std::vector<double> weight_wq;


class InformationMatrixCalculator {

public:
  using PointT = pcl::PointXYZI;
  // std::vector<double> fitness_score_vec;



  InformationMatrixCalculator();//{}
  //InformationMatrixCalculator(ros::NodeHandle& nh);
  ~InformationMatrixCalculator();

  /*template<typename ParamServer>
  void load(ParamServer& params) {
    use_const_inf_matrix = params.template param<bool>("use_const_inf_matrix", false);
    const_stddev_x = params.template param<double>("const_stddev_x", 0.5);
    const_stddev_q = params.template param<double>("const_stddev_q", 0.1);

    var_gain_a = params.template param<double>("var_gain_a", 20.0);
    min_stddev_x = params.template param<double>("min_stddev_x", 0.1);
    max_stddev_x = params.template param<double>("max_stddev_x", 5.0);
    min_stddev_q = params.template param<double>("min_stddev_q", 0.05);
    max_stddev_q = params.template param<double>("max_stddev_q", 0.2);
    fitness_score_thresh = params.template param<double>("fitness_score_thresh", 2.5);
  }*/

  static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose, double max_range = std::numeric_limits<double>::max());

  Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1, const pcl::PointCloud<PointT>::ConstPtr& cloud2, const Eigen::Isometry3d& relpose);
  Eigen::MatrixXd calc_const_information_matrix() const;
private:
  double weight(double a, double max_x, double min_y, double max_y, double x) const {
    double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
    return min_y + (max_y - min_y) * y;
  }

private:
  bool use_const_inf_matrix;
  double const_stddev_xy;
  double const_stddev_xyz;
  double const_stddev_rpy;
  double const_stddev_x;
  double const_stddev_y;
  double const_stddev_z;
  double const_stddev_q1;
  double const_stddev_q2;
  double const_stddev_q3;

  double var_gain_a;
  double min_stddev_x;
  double max_stddev_x;
  double min_stddev_q;
  double max_stddev_q;
  double fitness_score_thresh;
  double max_range;
};

}

#endif // INFORMATION_MATRIX_CALCULATOR_HPP
