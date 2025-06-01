#pragma once

#include <string>
#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo_msgs/LinkStates.h>
// #include <sensor_msgs/JointState.h>

class AddHeaderToLinkState
{
public:
    AddHeaderToLinkState(ros::NodeHandle* nodeHandlePtr);
    virtual ~AddHeaderToLinkState();

private:
    void callback(const gazebo_msgs::LinkStates::ConstPtr& linkMsg);
    // bool pickOutSensorDataFromModelMsg(const gazebo_msgs::ModelStates& modelMsg, const ros::Time stampTime, sensor_msgs::JointState& rotatingBaseMsg/*, model_msg_to_rotbase_joint_msg::PoseWithHeader& poseWithHeader*/) const;
    // void quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const;
    ros::Publisher  withHederPublisher_;
    // ros::Publisher  posePublisher_;
    ros::Subscriber  linkSub_;
    ros::NodeHandle* nodeHandlePtr_;
    // std::string sensorModelName_;
};
