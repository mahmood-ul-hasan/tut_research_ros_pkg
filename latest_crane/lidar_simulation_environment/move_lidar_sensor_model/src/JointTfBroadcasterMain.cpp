#include "k_crane_joint_tf_broadcaster/JointTfBroadcaster.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "k_crane_joint_tf_broadcaster_node");
  ros::NodeHandle nh("~");

  JointTfBroadcaster jtb(&nh);
  ros::Rate loop_rate(1000);
  while (ros::ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
