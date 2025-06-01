#include "load_respawner/RespawnLoad.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "load_respawner_node");
  ros::NodeHandle nh("~");

  RespawnLoad rl(&nh);
  rl.respawnFromTf();

  // rl.respawnFromTf();
  // while (ros::ok()) {
  //   ros::spinOnce();
  // }
  // if (ros::ok())
  // {
  //   double spinCycle;
  //   // double defaultFrequency = RotatingBase::DEFAULT_FREQUENCY;
  //   nh.param("spin_cycle", spinCycle, 0.1);
  //   // ros::param::param<double>("~frequency", frequency, defaultFrequency);
  //   ros::Timer spin_timer = nh.createTimer(ros::Duration(spinCycle), &RespawnLoad::respawnFromTf, &rl);

  //   ros::MultiThreadedSpinner spinner(2);
  //   spinner.spin();
  // }

  return 0;
}
