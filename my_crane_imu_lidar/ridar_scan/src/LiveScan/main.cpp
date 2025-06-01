#include "LiveScan/LiveScan.h"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "live_scan");
    ROS_INFO("Starting LiveScan-node with node name %s", ros::this_node::getName().c_str());
    ros::NodeHandle nodeHandle;
    ros::NodeHandle localNodeHandle("~");

    // Create a LiveScan instance
    LiveScan livescan(&nodeHandle, &localNodeHandle);
  
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
  
    return 0;
}
