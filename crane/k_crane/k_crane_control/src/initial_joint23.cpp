#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_msgs/SetModelConfiguration.h>

int main(int argc, char *argv[])
{

    //ros node initialize
    ros::init(argc, argv, "start_pose_publisher");

    // create ros node handler
    ros::NodeHandle node_handler;

    ////Set joint 0 ////
    ros::ServiceClient client = node_handler.serviceClient<gazebo_msgs::SetModelConfiguration>("gazebo/set_model_configuration");
    //create requast
    gazebo_msgs::SetModelConfiguration service_msgs;
    service_msgs.request.model_name = "k_crane";
    service_msgs.request.urdf_param_name = "robot_description";
    service_msgs.request.joint_names.push_back("joint23");
    service_msgs.request.joint_positions.push_back(0);

    //send request
    if (client.call(service_msgs))
    {
        ROS_INFO_STREAM(service_msgs.response);
    }
    else
    {
        ROS_WARN_STREAM("Failed to call service gazebo initial pose");
        return 1;
    }
    return 0;
}
