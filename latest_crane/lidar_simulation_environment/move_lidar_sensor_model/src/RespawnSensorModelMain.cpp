#include "sensor_model_respawner/RespawnSensorModel.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "sensor_model_respawner_node");
  ros::NodeHandle nh("~");

  RespawnSensorModel rsm(&nh);

  // rsm.respawnFromTf();
  // while (ros::ok()) {
  //   ros::spinOnce();
  // }
  if (ros::ok())
  {
    double spinCycle;
    // double defaultFrequency = RotatingBase::DEFAULT_FREQUENCY;
    nh.param("spin_cycle", spinCycle, 0.1);
    // ros::param::param<double>("~frequency", frequency, defaultFrequency);
    ros::Timer spin_timer = nh.createTimer(ros::Duration(spinCycle), &RespawnSensorModel::respawnFromTf, &rsm);

    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
  }

  return 0;
}
