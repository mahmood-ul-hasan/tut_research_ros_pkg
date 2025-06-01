#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

int main(int argc, char* argv[])
{
	// ros node initialize
	ros::init(argc, argv, "pause_gazebo_node");

	// create ros node handler
	ros::NodeHandle node_handler;

	// create service
	std_srvs::Empty pause_srv;

	// create service client
	ros::ServiceClient client = node_handler.serviceClient<std_srvs::Empty>("gazebo/pause_physics");

	if (client.call(pause_srv)) {
		ROS_INFO_STREAM(pause_srv.response);
	}
	else {
		ROS_WARN_STREAM(pause_srv.response);
		return -1;
	}
	
	return 0;
}


