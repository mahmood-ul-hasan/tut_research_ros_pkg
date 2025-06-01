#include "CollisionWarning/CollisionWarning.h"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "collision_warning");
    // ROS_INFO("Starting LiveScan-node with node name %s", ros::this_node::getName().c_str());
    ros::NodeHandle nodeHandle("~");

    bool isSimulation = false;
    nodeHandle.param("is_simulation", isSimulation, false);

    // Create a LiveScan instance
    CollisionWarning collisionwarning(&nodeHandle, isSimulation);
  
    if (isSimulation)
    {
        ros::Rate loop_rate(1000);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else
    {
        collisionwarning.executeWarningSystem();
    }
  
    return 0;
}
