#include "sensor_model_tf_broadcaster/SensorModelTfBroadcaster.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "sensor_model_tf_broadcaster_node");
  ros::NodeHandle nh("~");

  SensorModelTfBroadcaster smtb(&nh);
  // msm.broadcasSensorModelTf();
  // while (ros::ok()) {
  //   ros::spinOnce();
  // }

  // if (ros::ok())
  // {
  //   ros::MultiThreadedSpinner spinner(2);
  //   spinner.spin();
  // }
  ros::Rate loop_rate(1000);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
