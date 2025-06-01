#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>

class CommandListener {
	public:
		void call_back(const sensor_msgs::Joy joy_msgs);
		trajectory_msgs::JointTrajectory get();
		void print();
		double pitch_angle = 0;
		double yaw_angle = 0;
		const double scale_factor = 0.001;
};

void CommandListener::call_back(const sensor_msgs::Joy joy_msgs){
	this->pitch_angle += joy_msgs.axes[0] * this->scale_factor;
	this->yaw_angle   += joy_msgs.axes[1] * this->scale_factor;
	this->print();
}

trajectory_msgs::JointTrajectory CommandListener::get(){
	// make a header
	trajectory_msgs::JointTrajectory joint_trajectory_msgs;
	joint_trajectory_msgs.joint_names = {"pitch_joint", "yaw_joint"};

	// joint trajectory point
	trajectory_msgs::JointTrajectoryPoint joint_trajectory_points;
	joint_trajectory_points.positions.push_back(this->pitch_angle);
	joint_trajectory_points.positions.push_back(this->yaw_angle);
	joint_trajectory_points.time_from_start = ros::Duration(0.001);

	// prepare for publishing
	joint_trajectory_msgs.points.push_back(joint_trajectory_points);

	// return
	return joint_trajectory_msgs;
}

void CommandListener::print(){
	ROS_INFO_STREAM("pitch angle : " << this->pitch_angle << "\t" << "yaw angle : " << this->yaw_angle);
}


int main(int argc, char* argv[])
{
	// ros initialization
	ros::init(argc, argv, "ds4totwist_node");

	// make node handle
	ros::NodeHandle node_handler;

	// make subscriber
	CommandListener command_listener;
	ros::Subscriber subscriber = node_handler.subscribe("/joy", 1, &CommandListener::call_back, &command_listener);

	// make publisher
	ros::Publisher publisher = node_handler.advertise<trajectory_msgs::JointTrajectory>("/c_crane/c_crane_joint_controller/command", 1);
	ros::Rate timer(100);

	while (ros::ok()) {
		publisher.publish(command_listener.get());
		ros::spinOnce();
		timer.sleep();
	}
	return 0;
}
