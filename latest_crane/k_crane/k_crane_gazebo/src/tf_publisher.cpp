//gazeboを起動しただけではtfをpublishしてくれないので，
//(本当はpublishしてくれるが，できてないので「urdf_sim_turorial 13-diffdrive.launch」参照
//自前でやるコードgazebo_msgs/ModelStatesをtfに変換しpublishするプログラム．

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

gazebo_msgs::ModelStates hook_msgs;	//フックの
gazebo_msgs::ModelStates upper_msgs;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "gazebo_tf_bridge_node");
	ros::NodeHandle node_handler;

	
	return 0;
}
