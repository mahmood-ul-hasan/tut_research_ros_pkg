#include "Model2Scan.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "model2scan_node");
  ros::NodeHandle nh("~");

  Model2Scan m2c(&nh);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
