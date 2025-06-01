#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>

class CraneController
{
private:
	bool isSimplePattern_;
	size_t globalCount_;
	ros::NodeHandle* nodeHandlePtr_;
	ros::Publisher start_pose_publisher_;
    ros::Subscriber resultSub_;
	bool isOkValue_;
    ros::Time previousTime_;
    ros::Time nowTime_;
    bool isFirst_;
	double moveSpeed_; // [deg/sec]
	double nowPitch_;
	double nowYaw_;

	bool moveBoom(const bool isPitch, const bool isClockWise, const double targetAngle, const double subscribeCycle);
	void callback(const std_msgs::Bool::ConstPtr& resultMsg);
	double deg2rad(double degrees);

public:
	CraneController(ros::NodeHandle* nodeHandlePtr);
	~CraneController();
	bool ok();
};

CraneController::CraneController(ros::NodeHandle* nodeHandlePtr)
  : isSimplePattern_(false),
    globalCount_(0),
    nodeHandlePtr_(nodeHandlePtr),
	isOkValue_(true),
    isFirst_(true),
    moveSpeed_(1.0),
    nowPitch_(0.0),
    nowYaw_(0.0)
{
	// make publisher
	start_pose_publisher_ = nodeHandlePtr_->advertise<trajectory_msgs::JointTrajectory>("/k_crane/k_crane_joint_controller/command", 1);

	// make sbscriber
    resultSub_ = nodeHandlePtr->subscribe("/collision_warning/warning_result", 1, &CraneController::callback, this);
}

CraneController::~CraneController()
{
}


// bool CraneController::moveBoom(const double pitch_joint_goal, const double yaw_joint_goal,double subscribeCycle
// 		const double pitch_joint_bias, const double yaw_joint_bias, const int count_bias)
// {
// 	// make loop timer
// 	ros::Rate timer(5);
	
// 	// // make goal
// 	// //const double pitch_joint_goal = 1.20428;
// 	// //const double yaw_joint_goal   = 0;
// 	// const double pitch_joint_goal = M_PI/3;
// 	// const double yaw_joint_goal   = 0;
// 	// const int iteration_max = 1000;

// 	// for (int itr = 0; itr < iteration_max; itr++) {
//         double moveAnglInCycle = deg2rad(moveSpeed_) * subscribeCycle;

// 		// make message for publisherin
// 		trajectory_msgs::JointTrajectory joint_trajectory_msgs;
// 		joint_trajectory_msgs.joint_names = {"pitch_joint", "yaw_joint"};

// 		trajectory_msgs::JointTrajectoryPoint joint_trajectory_points;
// 		// joint_trajectory_points.positions.push_back((pitch_joint_goal / iteration_max) * globalCount_ + pitch_joint_bias);
// 		// joint_trajectory_points.positions.push_back((yaw_joint_goal / iteration_max) * globalCount_ + yaw_joint_bias);
// 		joint_trajectory_points.positions.push_back(previosPitch_ + moveAnglInCycle + pitch_joint_bias);
// 		joint_trajectory_points.positions.push_back(previosYaw_ + moveAnglInCycle + yaw_joint_bias);

// 		joint_trajectory_points.time_from_start = ros::Duration(0.1);

// 		joint_trajectory_msgs.points.push_back(joint_trajectory_points);
// 		start_pose_publisher_.publish(joint_trajectory_msgs);

// 		ROS_INFO_STREAM("Publish once");
// 		timer.sleep();
// 	// }
// }

bool CraneController::moveBoom(const bool isPitch, const bool isClockWise, const double targetAngle, const double subscribeCycle)
{
	bool returnBool = false;
	// make loop timer
	// ros::Rate timer(5);
	
	// // make goal
	// //const double pitch_joint_goal = 1.20428;
	// //const double yaw_joint_goal   = 0;
	// const double pitch_joint_goal = M_PI/3;
	// const double yaw_joint_goal   = 0;
	// const int iteration_max = 1000;

	// for (int itr = 0; itr < iteration_max; itr++) {
	double moveAnglInCycle = deg2rad(moveSpeed_) * subscribeCycle;

	// make message for publisherin
	trajectory_msgs::JointTrajectory joint_trajectory_msgs;
	joint_trajectory_msgs.joint_names = {"pitch_joint", "yaw_joint"};

	trajectory_msgs::JointTrajectoryPoint joint_trajectory_points;
	// joint_trajectory_points.positions.push_back((pitch_joint_goal / iteration_max) * globalCount_ + pitch_joint_bias);
	// joint_trajectory_points.positions.push_back((yaw_joint_goal / iteration_max) * globalCount_ + yaw_joint_bias);
	if (isPitch)
	{
		double nextPitchAngle;
		if ((isClockWise && (nowPitch_ - moveAnglInCycle <= targetAngle))
		|| (!isClockWise && (nowPitch_ + moveAnglInCycle >= targetAngle)))
		{
			nextPitchAngle = targetAngle;
			returnBool = true;
		}
		else if (isClockWise)
		{
			nextPitchAngle = nowPitch_ - moveAnglInCycle;
		}
		else
		{
			nextPitchAngle = nowPitch_ + moveAnglInCycle;
		}
		nowPitch_ = nextPitchAngle;
		joint_trajectory_points.positions.push_back(nextPitchAngle);
		joint_trajectory_points.positions.push_back(nowYaw_);
	}
	else
	{
		joint_trajectory_points.positions.push_back(nowPitch_);
		double nextYawAngle;
		if ((isClockWise && (nowYaw_ - moveAnglInCycle <= targetAngle))
		|| (!isClockWise && (nowYaw_ + moveAnglInCycle >= targetAngle)))
		{
			nextYawAngle = targetAngle;
			returnBool = true;
		}
		else if (isClockWise)
		{
			nextYawAngle = nowYaw_ - moveAnglInCycle;
		}
		else
		{
			nextYawAngle = nowYaw_ + moveAnglInCycle;
		}
		nowYaw_ = nextYawAngle;
		joint_trajectory_points.positions.push_back(nextYawAngle);
	}
	
	joint_trajectory_points.time_from_start = ros::Duration(0.1);

	joint_trajectory_msgs.points.push_back(joint_trajectory_points);
	start_pose_publisher_.publish(joint_trajectory_msgs);

	// ROS_INFO_STREAM("Publish once. nowPitch_ = " << nowPitch_ << ", nowYaw_" << nowYaw_);
	// timer.sleep();
	// }

	return returnBool;
}

void CraneController::callback(const std_msgs::Bool::ConstPtr& resultMsg)
{
    if (isFirst_)
    {
        previousTime_ = ros::Time::now();
        isFirst_ = false;
        return;
    }
    nowTime_ = ros::Time::now();
    ros::Duration callbackDuration = nowTime_ - previousTime_;
    previousTime_ = nowTime_;
    double subscribeCycle = callbackDuration.sec + (static_cast<double>(callbackDuration.nsec) * 0.000000001);
    
	if (resultMsg->data)
	{
		isOkValue_ = false;
	}
	else if (isOkValue_)
	{
		if (isSimplePattern_)
		{
			if (globalCount_ <= 0)
			{
				if (moveBoom(false, true, (-0.28) * 2, subscribeCycle)) // (def_yaw) * 2
				{
					globalCount_++;
				}
			}
			else
			{
				isOkValue_ = false;
			}
		}
		else
		{		
			if (globalCount_ <= 0)
			{
				if (moveBoom(true, false, 0.12, subscribeCycle))
				{
					globalCount_++;
				}
			}
			else if (globalCount_ <= 1)
			{
				if (moveBoom(false, true, (-0.28) * 2, subscribeCycle)) // (def_yaw) * 2
				{
					globalCount_++;
				}
			}
			else if (globalCount_ <= 2)
			{
				if (moveBoom(true, true, 0.0, subscribeCycle))
				{
					globalCount_++;
				}
			}
			else
			{
				isOkValue_ = false;
			}
		}


		// // double pitch_joint_goal = M_PI/3;
		// // double yaw_joint_goal   = 0;
		// // int iteration_max = 1000;
		// double pitch_joint_bias;
		// double yaw_joint_bias;
		// double pitch_joint_goal;
		// double yaw_joint_goal;
		// int iteration_max;

		// int ite1 = 30;
		// int ite2 = 45;
		// int ite3 = 30;

		// if (globalCount_ < ite1)
		// {
		// 	pitch_joint_bias = 0.0;
		// 	yaw_joint_bias   = 0.0;
		// 	pitch_joint_goal = 0.12;
		// 	yaw_joint_goal   = 0.0;
		// 	iteration_max = ite1;
		// 	moveBoom(pitch_joint_goal - pitch_joint_bias, yaw_joint_goal - yaw_joint_bias, iteration_max,
		// 		pitch_joint_bias, yaw_joint_bias, 0);
		// 	globalCount_++;
		// }
		// else if (globalCount_ < ite1 + ite2)
		// {
		// 	pitch_joint_bias = pitch_joint_goal;
		// 	yaw_joint_bias   = yaw_joint_goal;
		// 	pitch_joint_goal = 0.12;
		// 	yaw_joint_goal   = (-0.28) * 2; // (def_yaw) * 2
		// 	iteration_max = ite2;
		// 	moveBoom(pitch_joint_goal - pitch_joint_bias, yaw_joint_goal - yaw_joint_bias, iteration_max,
		// 		pitch_joint_bias, yaw_joint_bias, ite1);
		// 	globalCount_++;
		// }
		// else if (globalCount_ < ite1 + ite2 + ite3)
		// {
		// 	pitch_joint_bias = pitch_joint_goal;
		// 	yaw_joint_bias   = yaw_joint_goal;
		// 	pitch_joint_goal = 0.0;
		// 	yaw_joint_goal   = (-0.28) * 2; // (def_yaw) * 2
		// 	iteration_max = ite3;
		// 	moveBoom(pitch_joint_goal - pitch_joint_bias, yaw_joint_goal - yaw_joint_bias, iteration_max,
		// 		pitch_joint_bias, yaw_joint_bias, ite1 + ite2);
		// 	globalCount_++;
		// }
	}
}

bool CraneController::ok()
{
	// if (isOkValue_)
	// {
	// 	ROS_INFO("isOkValue_ is true");
	// } else {
	// 	ROS_INFO("isOkValue_ is false");
	// }
	
	return isOkValue_;
}

double CraneController::deg2rad(double degrees)
{
    return degrees * 4.0 * std::atan(1.0) / 180.0;
}

int main(int argc, char* argv[])
{
	
	//ros node initialize
	ros::init(argc, argv, "crane_controller_node");

	// create ros node handler
	ros::NodeHandle node_handler;

	CraneController cc(&node_handler);

	while (ros::ok() && cc.ok()) {
		ros::spinOnce();
	}

	return 0;
}
