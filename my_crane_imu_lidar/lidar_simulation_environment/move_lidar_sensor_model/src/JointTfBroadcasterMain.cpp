#include "k_crane_joint_tf_broadcaster/JointTfBroadcaster.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "k_crane_joint_tf_broadcaster_node");
  ros::NodeHandle nh("~");

  JointTfBroadcaster jtb(&nh);
  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
