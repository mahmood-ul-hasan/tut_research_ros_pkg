#pragma once

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/JointState.h>

// #include "model_msg_to_rotbase_joint_msg/PoseWithHeader.h"

class Model2Scan
{
public:
    Model2Scan(ros::NodeHandle* nodeHandlePtr);
    virtual ~Model2Scan();

private:
    void callback(const gazebo_msgs::ModelStates::ConstPtr& modelMsg);
    bool pickOutSensorDataFromModelMsg(const gazebo_msgs::ModelStates& modelMsg, const ros::Time stampTime, sensor_msgs::JointState& rotatingBaseMsg/*, model_msg_to_rotbase_joint_msg::PoseWithHeader& poseWithHeader*/) const;
    void quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const;
    ros::Publisher  jointPublisher_;
    // ros::Publisher  posePublisher_;
    ros::Subscriber  modelSub_;
    ros::NodeHandle* nodeHandlePtr_;
    std::string sensorModelName_;
};
