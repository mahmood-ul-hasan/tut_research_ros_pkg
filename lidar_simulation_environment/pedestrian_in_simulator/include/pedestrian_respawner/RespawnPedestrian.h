#pragma once

// C++ specific includes
#include <string>
#include <mutex>
#include <vector>

// ROS specific includes
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

// Gazebo specific includes
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

class RespawnPedestrian
{
public:
    RespawnPedestrian(ros::NodeHandle* nodeHandlePtr);
    virtual ~RespawnPedestrian();
    void respawnFromTf(const ros::TimerEvent&);
    void callbackFromResultMessage(const std_msgs::Bool::ConstPtr& resultMsg);

private:
    // void quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const;
    geometry_msgs::Quaternion eulerAngles2Quaternion(double roll, double pitch, double yaw) const;

    ros::NodeHandle* nodeHandlePtr_;
    ros::ServiceClient respawnClient_;
    ros::Subscriber resultSub_;
    // ros::Subscriber angleSub_;
    // ros::Publisher anglePub_;

    double respawnCycle_;
    // std::string sensorModelName_;
    std::string referenceFrame_;
    // std::string sensorModelFrameId_;
    // bool isKCrane_;
    // bool isLidarSensorSystemOnBoom_;
    // bool isCollisionWarning_;
    std::string pedestrianModelName_;
    std::mutex mutex_;
    // sensor_msgs::JointState jointState_;
    bool wasWarningOfCollision_;
    bool isOk_;


    // std::vector<double> initialPosXYZ_;
    std::vector<double> initialPosX_;
    std::vector<double> initialPosY_;
    // double initialPosZ_;
    std::vector<double> targetPosX_;
    std::vector<double> targetPosY_;
    // double targetPosZ_;
    std::vector<double> moveSpeed_;

    std::vector<double> nowPosX_;
    std::vector<double> nowPosY_;
    // double nowPosZ_;
    std::vector<double> moveAngle_;

    std::vector<bool> wasReached_;

    const size_t NUM_OF_RETRY = 10;
    const size_t NUM_OF_PERSON = 5;
};
