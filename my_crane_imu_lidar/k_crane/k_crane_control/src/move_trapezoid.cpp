#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

int main(int argc, char* argv[])
{
	// ros node initialize
	ros::init(argc, argv, "move_trapezoid_node");

	// create ros node handler
	ros::NodeHandle node_handler;

	// make publisher
	ros::Publisher trajectory_publisher = node_handler.advertise<trajectory_msgs::JointTrajectory>("/c_crane/c_crane_joint_controller/command", 1);

	// make loop timer
	// 10 Hz
	ros::Rate timer(10);

	// make goal
	const double joint_goal = M_PI;

	// make trajectory
	const int iteration_max = 999;
	// 1. acceleration stage
	const double acceleration_stage_start = M_PI;
	const double acceleration_stage_goal = M_PI/4;
	// 2. steady state
	const double steady_stage_start = M_PI/4;
	const double steady_stage_goal  = -M_PI/4;
	// 3. decceleration stage
	const double decceleration_stage_start = -M_PI/4;
	const double decceleration_stage_goal  = -M_PI;


	for (int itr = 0; itr < iteration_max; itr++) {
		// make message for publish
		trajectory_msgs::JointTrajectory joint_trajectory_msgs;
		joint_trajectory_msgs.joint_names = {"pitch_joint", "yaw_joint"};
		trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;

		// 1. acceleration stage
		if (itr < iteration_max/3) {
			const double target_yaw_angle = ((M_PI/4)/333)*itr - (M_PI/2);
			joint_trajectory_point.positions.push_back(1.0472);
			joint_trajectory_point.positions.push_back(target_yaw_angle);
			ROS_INFO_STREAM("Acc. stage : " << target_yaw_angle);
		}
		// 2. steady stage
		else if (iteration_max/3 <= itr && itr < iteration_max*2/3){
			const double target_yaw_angle = ((M_PI/2)/333)*itr - (3*M_PI)/4;
			joint_trajectory_point.positions.push_back(1.0472);
			joint_trajectory_point.positions.push_back(target_yaw_angle);
			ROS_INFO_STREAM("Std. stage : " << target_yaw_angle);
		}
		// 3. decceleration stage
		else {
			const double target_yaw_angle = ((M_PI/4)/333)*itr - (M_PI/4);
			joint_trajectory_point.positions.push_back(1.0472);
			joint_trajectory_point.positions.push_back(target_yaw_angle);
			ROS_INFO_STREAM("Dec. stage : " << target_yaw_angle);
		}

		joint_trajectory_point.time_from_start = ros::Duration(0.1);
		joint_trajectory_msgs.points.push_back(joint_trajectory_point);

		trajectory_publisher.publish(joint_trajectory_msgs);
		timer.sleep();
	}


	return 0;
}
