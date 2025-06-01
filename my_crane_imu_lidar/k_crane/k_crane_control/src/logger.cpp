#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuLstener {
	public:
		ImuLstener();
		void call_back(const sensor_msgs::Imu msgs);
		std::string unstruct_imu_msgs(const sensor_msgs::Imu msgs);
		std::ofstream log_file;
		sensor_msgs::Imu data;
};

ImuLstener::ImuLstener(){
	this->log_file.open("/home/harumo/catkin_ws/src/k_crane/data/IMU_experiment/imu_log_sim.csv");
	if (this->log_file.is_open()) {
		ROS_INFO_STREAM("log file could open successfully");
	}
	else {
		ROS_ERROR_STREAM("can't open log file. exit");
		std::exit(1);
	}
}

std::string ImuLstener::unstruct_imu_msgs(const sensor_msgs::Imu msgs){
	std::stringstream ss;
	ss << msgs.angular_velocity.x << "," << msgs.angular_velocity.y << "," << msgs.angular_velocity.z << "," << msgs.linear_acceleration.x << "," << msgs.linear_acceleration.y << "," << msgs.linear_acceleration.z;
	return ss.str();
}

void ImuLstener::call_back(const sensor_msgs::Imu msgs){
	ROS_INFO_STREAM("rec once");
	this->log_file << this->unstruct_imu_msgs(msgs) << std::endl;
}
int main(int argc, char* argv[])
{
	ros::init(argc, argv, "imu_logger_node");
	ros::NodeHandle node;
	ImuLstener imu_listener;
	const auto imu_subscriber = node.subscribe("imu_hook/data", 1, &ImuLstener::call_back, &imu_listener);

	ROS_INFO_STREAM("waitiing for imu msgs");
	ros::topic::waitForMessage<sensor_msgs::Imu>("imu_hook/data");
	ROS_INFO_STREAM("logging start");

	ros::spin();

	imu_listener.log_file.close();

	
	return 0;
}
