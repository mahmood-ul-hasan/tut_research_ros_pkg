#include "pedestrian_respawner/RespawnPedestrian.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "pedestrian_respawner_node");
  ros::NodeHandle nh("~");

  RespawnPedestrian rp(&nh);

  if (ros::ok())
  {
    double spinCycle;
    // double defaultFrequency = RotatingBase::DEFAULT_FREQUENCY;
    nh.param("spin_cycle", spinCycle, 0.1);
    // ros::param::param<double>("~frequency", frequency, defaultFrequency);
    ros::Timer spin_timer = nh.createTimer(ros::Duration(spinCycle), &RespawnPedestrian::respawnFromTf, &rp);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
  }

  return 0;
}
