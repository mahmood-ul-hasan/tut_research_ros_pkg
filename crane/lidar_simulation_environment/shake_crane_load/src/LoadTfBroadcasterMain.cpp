#include "load_tf_broadcaster/LoadTfBroadcaster.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "load_tf_broadcaster_node");
  ros::NodeHandle nh("~");

  LoadTfBroadcaster ltb(&nh);
  ltb.broadcastLoadTf();
  // msm.broadcasSensorModelTf();
  // while (ros::ok()) {
  //   ros::spinOnce();
  // }

  // if (ros::ok())
  // {
  //   ros::MultiThreadedSpinner spinner(2);
  //   spinner.spin();
  // }
  // while (ros::ok()) {
  //   ros::spinOnce();
  // }
  
  return 0;
}
