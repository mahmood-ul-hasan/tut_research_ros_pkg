#pragma once

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/LinkStates.h>
// #include <mutex>
#include <sensor_msgs/JointState.h>

class SensorModelTfBroadcaster
{
public:
    SensorModelTfBroadcaster(ros::NodeHandle* nodeHandlePtr);
    virtual ~SensorModelTfBroadcaster();
    // void broadcasSensorModelTf() const;
    void callbackFromLinkMessage(const gazebo_msgs::LinkStates::ConstPtr& linkMsg);
    void quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const;
    // void callbackFromAngleMessage(const sensor_msgs::JointState::ConstPtr& jointMsg);

private:
    ros::NodeHandle* nodeHandlePtr_;
    std::string sensorModelFrameId_;
    ros::Subscriber  linkSub_;
    // ros::Subscriber  angleSub_;
    // bool isCollisionWarning_;
    bool isKCrane_;
    bool isLidarSensorSystemOnBoom_;
    bool isSensorShake_;
    // double jointAngle_;
    // std::mutex mutex_;
    ros::Publisher anglePub_;
    ros::Time previousTime_;
    ros::Time nowTime_;
    bool isFirst_;
    double accumulationTime_;
    // sensor_msgs::JointState jointState_;
};
