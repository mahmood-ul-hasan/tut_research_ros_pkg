#pragma once

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>

class JointTfBroadcaster
{
public:
    JointTfBroadcaster(ros::NodeHandle* nodeHandlePtr);
    virtual ~JointTfBroadcaster();
    void callback(const sensor_msgs::JointState::ConstPtr& jointMsg);
    // void broadcasSensorModelTf() const;
    // void quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const;

private:
    ros::NodeHandle* nodeHandlePtr_;
    // std::string sensorModelFrameId_;
    ros::Subscriber  jointSub_;
    // bool isLidarSensorSystemOnBoom_;
    double upperLinkDefaultX_;
    double upperLinkDefaultY_;
    double upperLinkDefaultZ_;
    double upperLinkDefaultJoint_;
    double boomLinkDefaultX_;
    double boomLinkDefaultY_;
    double boomLinkDefaultZ_;
    double boomLinkDefaultJoint_;
    ros::Time previousTime_;
    ros::Time nowTime_;
    bool isFirst_;
    double minTimeInterval_;
};
