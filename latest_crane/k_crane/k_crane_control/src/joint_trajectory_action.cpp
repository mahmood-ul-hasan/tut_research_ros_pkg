#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include <fanda/Csv.hpp>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajectoryClient;

class Crane {
	public:
		Crane();
		~Crane();
		void start_trajectory(control_msgs::FollowJointTrajectoryGoal goal);
		control_msgs::FollowJointTrajectoryGoal crane_extension_trajectory(sensor_msgs::JointState joint_state);
		actionlib::SimpleClientGoalState get_state();

	private:
		TrajectoryClient* trajectory_client;
};

/**
 * @brief Initialize the action client and wait for action server to bring up
 */
Crane::Crane() {
	// Tell the action client that we want to spin with a thread by default
	this->trajectory_client = new TrajectoryClient("/k_crane/k_crane_joint_controller/follow_joint_trajectory", true);
	
	// Wait until the action server bring up
	while (!this->trajectory_client->waitForServer(ros::Duration(1.0))) {
		ROS_INFO_STREAM("Waiting for the Follow Joint Trajectory Server");
	}
	ROS_INFO_STREAM("Connected to Follow Joint Trajectory Server");
}

/**
 * @brief Clean up the action client
 */
Crane::~Crane() {
	delete this->trajectory_client;
}

/**
 * @brief Sending the message to start with given trajectory
 *
 * @param goal 
 */
void Crane::start_trajectory(control_msgs::FollowJointTrajectoryGoal goal){
	goal.trajectory.header.stamp = ros::Time::now();
	this->trajectory_client->sendGoal(goal);
}

control_msgs::FollowJointTrajectoryGoal Crane::crane_extension_trajectory(sensor_msgs::JointState joint_state){
	// Read CSV file
	CSV::CsvFile angle_data("/home/aisl/catkin_ws/src/latest_crane/k_crane/k_crane_control/src/a.csv");
	ROS_INFO_STREAM("Read data size : " << angle_data.collumn_size() << "\t" << angle_data.row_size());

	// The foal variable
	control_msgs::FollowJointTrajectoryGoal goal;

	// First, Setting joint name 
	// goal.trajectory.joint_names.push_back("pitch_joint");
	goal.trajectory.joint_names.push_back("yaw_joint");
	ROS_INFO_STREAM("Goal trajectory gets joint names");

	// Second, Setting angle data
	goal.trajectory.points.resize(angle_data.collumn_size());
	ROS_INFO_STREAM("goal trajectory size : " << goal.trajectory.points.size());
	ROS_INFO_STREAM("Start to store Goal trajectory message with given data");
	for (int i = 0; i < angle_data.collumn_size(); i++) {
		// Angle
		goal.trajectory.points[i].positions.resize(2);
		goal.trajectory.points[i].positions[0] = 1 * M_PI/180;						// pitch joint
		// goal.trajectory.points[i].positions[1] = angle_data(i,0).get_as_double();	// yaw joint
		goal.trajectory.points[i].positions[1] = 1* M_PI/180;		// yaw joint
		// Velocity
		goal.trajectory.points[i].velocities.resize(2);
		goal.trajectory.points[i].velocities[0] = 0;	// pitch joint
		goal.trajectory.points[i].velocities[1] = 0;	// yaw joint
		// Time
		goal.trajectory.points[i].time_from_start = ros::Duration(i*0.01);;
	}
	ROS_INFO_STREAM("Goal trajectory message storing was finished");

	// Third, Setting timestamp
	goal.trajectory.header.stamp = ros::Time::now();
	ROS_INFO_STREAM("Goal trajectory gets time stamp");

	return goal;
}

actionlib::SimpleClientGoalState Crane::get_state(){
	return this->trajectory_client->getState();
}

class JointStateListener {
	public:
		void call_back(const sensor_msgs::JointState joint_state_msgs);
		sensor_msgs::JointState joint_state;
};

void JointStateListener::call_back(const sensor_msgs::JointState joint_state_msgs){
	this->joint_state = joint_state_msgs;
	ROS_INFO_STREAM("Joint state : " << joint_state_msgs.position[0] << " : " << joint_state_msgs.position[1]);
}

int main(int argc, char* argv[])
{
	// Initialize this ros node
	ros::init(argc, argv, "crane_action_client");
	ros::NodeHandle node_handle;

	// Initialize Joint state listener
	JointStateListener joint_state_listener;
	ros::Subscriber initial_joint_state_listener = node_handle.subscribe("/joint_state", 1, &JointStateListener::call_back, &joint_state_listener);
	for (int i = 0; i < 3; i++) {
		ros::spinOnce();
		ros::Duration(0.3).sleep();
	}

	// Initialize Action client
	Crane crane;

	std::cout << "I am here" << std::endl;



	// Start the trajectory
	ROS_INFO_STREAM("Sending the trajectory message");
	auto trajectory_msgs = crane.crane_extension_trajectory(joint_state_listener.joint_state);
	std::cout << "crane trajectory" << std::endl;
	for (auto&& e : trajectory_msgs.trajectory.points){
		std::cout << e.positions[0] << "\t" << e.positions[1] << std::endl;
	}
	crane.start_trajectory(trajectory_msgs);

	// Wait for trajectory finishing
	while (!crane.get_state().isDone() && ros::ok()) {
		auto a = crane.get_state();
		ros::Duration(0.1).sleep();
		ROS_INFO_STREAM("I'm waiting for finishing trajectory\t" << a.toString());
	}
	
	return 0;
}
