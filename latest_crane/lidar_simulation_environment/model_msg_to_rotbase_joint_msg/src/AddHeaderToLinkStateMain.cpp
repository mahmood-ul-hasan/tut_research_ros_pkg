#include "AddHeaderToLinkState.h"

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "add_header_to_link_state_node");
  ros::NodeHandle nh("~");

  AddHeaderToLinkState ahtls(&nh);

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
