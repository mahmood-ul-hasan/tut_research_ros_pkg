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

    // make publisher
    ros::Publisher start_pose_publisher = node_handler.advertise<trajectory_msgs::JointTrajectory>("/k_crane/k_crane_joint_controller/command", 1);

    // make loop timer
    ros::Rate timer(5);

    //// Move the target pose /////
    // make goal
    //const double pitch_joint_goal = 1.20428;
    //const double yaw_joint_goal   = 0;
    const double pitch_joint_goal = M_PI / 3;
    const double yaw_joint_goal = 0;
    const int iteration_max = 1000;

    for (int itr = 0; itr < iteration_max; itr++)
    {

        // make message for publisherin
        trajectory_msgs::JointTrajectory joint_trajectory_msgs;
        joint_trajectory_msgs.joint_names = {"pitch_joint", "yaw_joint"};

        trajectory_msgs::JointTrajectoryPoint joint_trajectory_points;
        joint_trajectory_points.positions.push_back((pitch_joint_goal / iteration_max) * itr);
        joint_trajectory_points.positions.push_back((yaw_joint_goal / iteration_max) * itr);

        joint_trajectory_points.time_from_start = ros::Duration(0.1);

        joint_trajectory_msgs.points.push_back(joint_trajectory_points);
        start_pose_publisher.publish(joint_trajectory_msgs);

        ROS_INFO_STREAM("Publish once");
        timer.sleep();
    }

    ROS_INFO_STREAM("Moving to target pose is finished");
    ROS_INFO_STREAM("Waiting for swinging");
	ros::Duration(5*60).sleep();
    ROS_INFO_STREAM("Aligning to target joint is start");

    ////Set joint 0 ////
    ros::ServiceClient client = node_handler.serviceClient<gazebo_msgs::SetModelConfiguration>("gazebo/set_model_configuration");
    //create requast
    gazebo_msgs::SetModelConfiguration service_msgs;
    service_msgs.request.model_name = "k_crane";
    service_msgs.request.urdf_param_name = "robot_description";

	//joint4payload
    service_msgs.request.joint_names.push_back("joint4payload");
    service_msgs.request.joint_positions.push_back(0.0);
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
	ros::Duration(60*3).sleep();

	////joint12
	//service_msgs.request.joint_names.at(0) = "joint12";
	//service_msgs.request.joint_positions.at(0) = 2.6153;
    ////send request
    //if (client.call(service_msgs))
    //{
    //    ROS_INFO_STREAM(service_msgs.response);
    //}
    //else
    //{
    //    ROS_WARN_STREAM("Failed to call service gazebo initial pose");
    //    return 1;
    //}
	//ros::Duration(60*3).sleep();

	//joint23
    service_msgs.request.joint_names.at(0) = "joint23";
    service_msgs.request.joint_positions.at(0) = 0.0;
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
