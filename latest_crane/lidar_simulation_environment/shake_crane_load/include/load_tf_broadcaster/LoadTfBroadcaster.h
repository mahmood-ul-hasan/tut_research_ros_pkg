#pragma once

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/LinkStates.h>
// #include <mutex>
#include <sensor_msgs/JointState.h>

class LoadTfBroadcaster
{
public:
    LoadTfBroadcaster(ros::NodeHandle* nodeHandlePtr);
    virtual ~LoadTfBroadcaster();
    // void broadcastLoadTf() const;
    // void broadcasSensorModelTf() const;
    void callbackFromJointMessage(const sensor_msgs::JointState::ConstPtr& jointMsg);
    // void quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const;
    // void callbackFromAngleMessage(const sensor_msgs::JointState::ConstPtr& jointMsg);

private:
    ros::NodeHandle* nodeHandlePtr_;
    std::string loadFrameId_;
    ros::Subscriber  jointSub_;
    // ros::Subscriber  angleSub_;
    // bool isCollisionWarning_;
    // bool isKCrane_;
    // bool isLidarSensorSystemOnBoom_;
    // double jointAngle_;
    // std::mutex mutex_;
    // ros::Publisher anglePub_;
    // sensor_msgs::JointState jointState_;
    // double timeInterval_;
    double boomLength_;
    double boomLinkDefaultJoint_;
    ros::Time previousTime_;
    ros::Time nowTime_;
    bool isFirst_;
    double minTimeInterval_;

    double angleMaxRoll_;
    double angleMaxPitch_;
    double angleMaxYaw_;
    double shakeSpeedRoll_;
    double shakeSpeedPitch_;
    double shakeSpeedYaw_;
};
