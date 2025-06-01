#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <std_srvs/Empty.h>

int main(int argc, char *argv[])
{

    //ros node initialize
    ros::init(argc, argv, "start_pose_publisher");

    // create ros node handler
    ros::NodeHandle node_handler;

    ////Set joint 0 ////
    ros::ServiceClient client_angle = node_handler.serviceClient<gazebo_msgs::SetModelConfiguration>("gazebo/set_model_configuration");
    ros::ServiceClient client_pause = node_handler.serviceClient<std_srvs::Empty>("gazebo/pause_physics");
    ros::ServiceClient client_unpause = node_handler.serviceClient<std_srvs::Empty>("gazebo/unpause_physics");

	// pause 
	std_srvs::Empty empty_srvs;
    if (client_pause.call(empty_srvs)) {
        ROS_INFO_STREAM(empty_srvs.response);
    }
    else
    {
        ROS_WARN_STREAM("Failed to call service pause");
        return 1;
    }

    //create requast
    gazebo_msgs::SetModelConfiguration service_msgs;
    service_msgs.request.model_name = "k_crane";
    service_msgs.request.urdf_param_name = "robot_description";
    service_msgs.request.joint_names.push_back("joint_boom1");
    service_msgs.request.joint_positions.push_back(M_PI);

    //send request
    if (client_angle.call(service_msgs)) {
        ROS_INFO_STREAM(service_msgs.response);
    }
    else
    {
        ROS_WARN_STREAM("Failed to call service gazebo initial pose");
        return 1;
    }

	// unpause 
    //if (client_unpause.call(empty_srvs)) {
    //    ROS_INFO_STREAM(empty_srvs.response);
    //}
    //else
    //{
    //    ROS_WARN_STREAM("Failed to call service pause");
    //    return 1;
    //}
    return 0;
}
