#include "load_tf_broadcaster/LoadTfBroadcaster.h"
// #include "model_msg_to_rotbase_joint_msg/PoseWithHeader.h"
// #include <tf/transform_broadcaster.h>
#include <vector>

// #define _USE_MATH_DEFINES
// // use M_PI
#include <cmath>
// use tf2_ros::TransformBroadcaster
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
// use tf2::Quaternion
#include <tf2/LinearMath/Quaternion.h>

LoadTfBroadcaster::LoadTfBroadcaster(ros::NodeHandle* nodeHandlePtr)
 : nodeHandlePtr_(nodeHandlePtr),
   isFirst_(true)
//    jointAngle_(0.0)
{
    std::string defaultLoadFrameId = "rope_load_frame";
    nodeHandlePtr_->param("load_frame_id", loadFrameId_, defaultLoadFrameId);
    // nodeHandlePtr_->param("is_k_crane", isKCrane_, true);
    // nodeHandlePtr_->param("is_lidar_sensor_system_on_boom", isLidarSensorSystemOnBoom_, true);
    // nodeHandlePtr_->param("is_collision_warning_", isCollisionWarning_, true);
    // nodeHandlePtr_->param("time_interval", timeInterval_, 0.1);
    nodeHandlePtr_->param("boom_length", boomLength_, 31.5);
    nodeHandlePtr->param("boom_link_default_joint", boomLinkDefaultJoint_, -M_PI/4); // def_pitch
    nodeHandlePtr->param("angle_max_roll", angleMaxRoll_, M_PI / 36.0);
    nodeHandlePtr->param("angle_max_pitch", angleMaxPitch_, M_PI / 36.0);
    nodeHandlePtr->param("angle_max_yaw", angleMaxYaw_, M_PI / 24.0);
    nodeHandlePtr->param("shake_speed_roll", shakeSpeedRoll_, M_PI / 22.0);
    nodeHandlePtr->param("shake_speed_pitch", shakeSpeedPitch_, M_PI / 26.0);
    nodeHandlePtr->param("shake_speed_yaw", shakeSpeedYaw_, M_PI / 30.0);
    nodeHandlePtr->param("min_time_interval", minTimeInterval_, 0.1);
    jointSub_ = nodeHandlePtr_->subscribe("/k_crane/joint_states", 1, &LoadTfBroadcaster::callbackFromJointMessage, this);
    // angleSub_ = nodeHandlePtr_->subscribe((isCollisionWarning_ ? "/collision_warning/jointMsg" : "/ridar_scan/jointMsg"), 1, &LoadTfBroadcaster::callbackFromAngleMessage, this);
    // anglePub_ = nodeHandlePtr_->advertise<sensor_msgs::JointState>("joint_angle", 1);

    // jointState_.header.stamp = ros::Time::now();
    // jointState_.name.resize(1);
    // jointState_.position.resize(1);
    // jointState_.velocity.resize(1);
    // jointState_.name[0] = "rotating_base [rad, rad/sec]";
    // jointState_.position[0] = 0.0;
    // jointState_.velocity[0] = 0.0;
}

LoadTfBroadcaster::~LoadTfBroadcaster() {}

void LoadTfBroadcaster::callbackFromJointMessage(const sensor_msgs::JointState::ConstPtr& jointMsg)
{
    if (isFirst_)
    {
        previousTime_ = ros::Time::now();
        isFirst_ = false;
        return;
    }
    // nowTime_ = ros::Time::now();
    // ros::Duration callbackDuration = nowTime_ - previousTime_;
    // previousTime_ = nowTime_;
    // double timeInterval = callbackDuration.sec + (static_cast<double>(callbackDuration.nsec) * 0.000000001);
    nowTime_ = ros::Time::now();
    ros::Duration callbackDuration = nowTime_ - previousTime_;
    double timeInterval = callbackDuration.sec + (static_cast<double>(callbackDuration.nsec) * 0.000000001);
    if (timeInterval < minTimeInterval_)
    {
        return;
    }
    previousTime_ = nowTime_;

    std::vector<std::string> nameStrs = jointMsg->name;
    auto itr = std::find(nameStrs.begin(), nameStrs.end(), "pitch_joint");
    size_t index = std::distance(nameStrs.begin(), itr);
    if (nameStrs.size() == index)
    {
        ROS_ERROR("Error at callback() in JointTfBroadcaster");
        return;
    }
    // static tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = (loopIdx == 0 ? "upper_link_processed" : "lower_link");
    // transformStamped.child_frame_id = (loopIdx == 0 ? "boom_link_processed" : "upper_link_processed");

    // transformStamped.transform.translation.x = (loopIdx == 0 ? boomLinkDefaultX_ : upperLinkDefaultX_);
    // transformStamped.transform.translation.y = (loopIdx == 0 ? boomLinkDefaultY_ : upperLinkDefaultY_);
    // transformStamped.transform.translation.z = (loopIdx == 0 ? boomLinkDefaultZ_ : upperLinkDefaultZ_);
    // tf2::Quaternion q;
    // q.setRPY(0.0,
    //             (loopIdx == 0 ? boomLinkDefaultJoint_ - jointMsg->position[index] : 0.0),
    //             (loopIdx == 0 ? 0.0 : upperLinkDefaultJoint_ - jointMsg->position[index]));
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    // br.sendTransform(transformStamped);
    // // ROS_INFO("broadcasted");

    // static tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.stamp = ros::Time::now();
    // // transformStamped.header.frame_id = "boom_link";
    // transformStamped.header.frame_id = "boom_link_processed";
    // transformStamped.child_frame_id = sensorModelFrameId_;

    // transformStamped.transform.translation.x = 15.0;
    // transformStamped.transform.translation.y = 0.0;
    // transformStamped.transform.translation.z = -1.1;
    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, 0.0);
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();

    // br.sendTransform(transformStamped);

    double xPos = 0.0;
    double yPos = 0.0;
    double zPos = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    // const double timeInterval = 0.1;
    std::vector<double> angleMaxes(3);
    std::vector<double> shakeSpeeds(3);
    static std::vector<double> angleValues(3, 0.0);
    static std::vector<bool> shakeDirections(3, true);
    angleMaxes[0] = angleMaxRoll_;
    angleMaxes[1] = angleMaxPitch_;
    angleMaxes[2] = angleMaxYaw_;
    shakeSpeeds[0] = shakeSpeedRoll_;
    shakeSpeeds[1] = shakeSpeedPitch_;
    shakeSpeeds[2] = shakeSpeedYaw_;
    // angleValues[0] = 0.0;
    // angleValues[1] = 0.0;
    // angleValues[2] = 0.0;
    // shakeDirections[0] = true;
    // shakeDirections[1] = true;
    // shakeDirections[2] = true;
    // const double rollMax = M_PI / 12;
    // const double pitchMax =  M_PI / 12;
    // const double yawMax =  M_PI / 12;
    // const double rollSpeed = M_PI / 12;
    // const double pitchSpeed = M_PI / 12;
    // const double yawSpeed = M_PI / 12;

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // size_t count = 0;

    /////////////////// start define pose
    // Please calculate and assign the values.
    xPos = boomLength_;
    yPos = 0.0;
    zPos = -0.5;

    for (size_t i = 0; i < 3; i++)
    {
        if (angleMaxes[0] != 0.0 && shakeSpeeds[0] != 0.0)
        {
            if (shakeDirections[i])
            {
                angleValues[i] += shakeSpeeds[i] * timeInterval;
                if (angleValues[i] >= angleMaxes[i])
                {
                    shakeDirections[i] = false;
                }
            } else
            {
                angleValues[i] -= shakeSpeeds[i] * timeInterval;
                if (angleValues[i] <= -angleMaxes[i])
                {
                    shakeDirections[i] = true;
                }
            }
        }
    }
    roll = angleValues[0];
    pitch = angleValues[1] - boomLinkDefaultJoint_ + jointMsg->position[index];
    yaw = angleValues[2];
    // roll = 0.0;
    // pitch = 0.0;
    // yaw = 0.0;

    // // The sample is below: (please un-commentout)
    // const double centerX = 0.0;
    // const double centerY = 0.0;
    // const double centerZ = 10.0;
    // const double shakeRadius = 3.0;
    // const double shakeAngleDegree = 4.0;
    // const double shakeCycleSec = 2.0;
    // // shake axis is Y

    // double nowTime = timeInterval_ * count;
    // double sectionTime = nowTime;
    // while (sectionTime - shakeCycleSec >= 0)
    // {
    //     sectionTime -= shakeCycleSec;
    // }
    // double shakeCycleSecQuarter = shakeCycleSec / 4;
    // double shakeCycleSecHalf = shakeCycleSec / 2;

    // double angleTmpDeg;
    // double angleTmpRad;
    // if (sectionTime < shakeCycleSecHalf)
    // {
    //     if (sectionTime < shakeCycleSecQuarter)
    //     {
    //         angleTmpDeg = ((sectionTime / shakeCycleSecQuarter) * shakeAngleDegree);
    //     } else
    //     {
    //         angleTmpDeg = (((shakeCycleSecHalf - sectionTime) / shakeCycleSecQuarter) * shakeAngleDegree);
    //     }
    //     angleTmpRad = ((angleTmpDeg /360) * 2*M_PI);
    //     yPos = centerY - (shakeRadius * std::sin(angleTmpRad));
    //     roll = -angleTmpRad;
    // } else
    // {
    //     if (sectionTime < shakeCycleSecQuarter * 3)
    //     {
    //         angleTmpDeg = (((sectionTime - shakeCycleSecHalf) / shakeCycleSecQuarter) * shakeAngleDegree);
    //     }else
    //     {
    //         angleTmpDeg = (((shakeCycleSec - sectionTime) / shakeCycleSecQuarter) * shakeAngleDegree);
    //     }
    //     angleTmpRad = ((angleTmpDeg /360) * 2*M_PI);
    //     yPos = centerY + (shakeRadius * std::sin(angleTmpRad));
    //     roll = angleTmpRad;
    // }
    // zPos = centerZ - (shakeRadius * std::cos(angleTmpRad));

    // xPos = centerX;
    // pitch = 0.0;
    // yaw = 0.0;
    /////////////////// end define pose

    // transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.stamp = jointMsg->header.stamp;
    transformStamped.header.frame_id = "boom_link_processed";
    transformStamped.child_frame_id = loadFrameId_;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    transformStamped.transform.translation.x = xPos;
    transformStamped.transform.translation.y = yPos;
    transformStamped.transform.translation.z = zPos;

    br.sendTransform(transformStamped);
    ROS_INFO("broadcasted");

    // count++;
    // ros::Duration(timeInterval_).sleep();
}

// void LoadTfBroadcaster::callbackFromLinkMessage(const gazebo_msgs::LinkStates::ConstPtr& linkMsg)
// {
    // static tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // mutex_.lock();
    // sensor_msgs::JointState jointStateLocal = jointState_;
    // mutex_.unlock();
    // ROS_INFO_STREAM("jointStateLocal.position[0] = " << jointStateLocal.position[0]);

    // if (isKCrane_ && isLidarSensorSystemOnBoom_)
    // {
    //     transformStamped.header.stamp = ros::Time::now();
    //     // transformStamped.header.frame_id = "boom_link";
    //     transformStamped.header.frame_id = "boom_link_processed";
    //     transformStamped.child_frame_id = sensorModelFrameId_;

    //     transformStamped.transform.translation.x = 15.0;
    //     transformStamped.transform.translation.y = 0.0;
    //     transformStamped.transform.translation.z = -1.1;
    //     tf2::Quaternion q;
    //     q.setRPY(0.0, 0.0, 0.0);
    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();
    // }
    // else if (isKCrane_ && !isLidarSensorSystemOnBoom_) // if lidar sensor system on payload
    // {
    //     std::vector<std::string> nameStrs = linkMsg->name;
    //     auto itr = std::find(nameStrs.begin(), nameStrs.end(), (isLidarSensorSystemOnBoom_ ? "k_crane::rope1_link" : "k_crane::imu_hook_link"));
    //     // ROS_INFO_STREAM("*itr = " << *itr);
    //     // ROS_INFO_STREAM("nameStrs.size() = " << nameStrs.size());
    //     size_t index = std::distance(nameStrs.begin(), itr);
    //     // ROS_INFO_STREAM("index = " << index);
    //     if (nameStrs.size() == index)
    //     {
    //         ROS_ERROR("Error at callback() in LoadTfBroadcaster");
    //         return;
    //     } else {
    //         const double R_IMU = 6.5;
    //         const double R_LOAD = 11.6;
    //         double sinPitch = 0;
    //         double sinRoll = 0;
    //         double cosTheta = 0;
    //         double roll, pitch, yaw;
    //         quaternion2EulerAngles(roll, pitch, yaw, linkMsg->pose[index].orientation);
    //         sinPitch = std::sin(pitch);
    //         sinRoll = std::sin(roll);
    //         double pIMUx = R_IMU * sinPitch;
    //         double pIMUy = R_IMU * sinRoll;
    //         cosTheta = std::sqrt(std::pow(R_IMU, 2.0) - (std::pow(pIMUx, 2.0) + std::pow(pIMUy, 2.0))) / R_IMU;
            
    //         transformStamped.header.stamp = ros::Time::now();
    //         transformStamped.header.frame_id = "world";
    //         transformStamped.child_frame_id = sensorModelFrameId_;

    //         // if (isLidarSensorSystemOnBoom_)
    //         // {
    //         //     transformStamped.transform.translation.x = linkMsg->pose[index].position.x;
    //         //     transformStamped.transform.translation.y = linkMsg->pose[index].position.y;
    //         //     transformStamped.transform.translation.z = linkMsg->pose[index].position.z;
    //         //     tf2::Quaternion q;
    //         //     q.setRPY(0.0, 0.0, 0.0);
    //         //     transformStamped.transform.rotation.x = q.x();
    //         //     transformStamped.transform.rotation.y = q.y();
    //         //     transformStamped.transform.rotation.z = q.z();
    //         //     transformStamped.transform.rotation.w = q.w();
    //         // } else {
    //         transformStamped.transform.translation.x = linkMsg->pose[index].position.x + ((R_LOAD - R_IMU) *sinPitch);
    //         transformStamped.transform.translation.y = linkMsg->pose[index].position.y + ((R_LOAD - R_IMU) *sinRoll);
    //         transformStamped.transform.translation.z = linkMsg->pose[index].position.z - ((R_LOAD - R_IMU) * cosTheta);
    //         transformStamped.transform.rotation =linkMsg->pose[index].orientation;
    //         // tf2::Quaternion q;
    //         // q.setRPY(roll, pitch, yaw);
    //         // transformStamped.transform.rotation.x = q.x();
    //         // transformStamped.transform.rotation.y = q.y();
    //         // transformStamped.transform.rotation.z = q.z();
    //         // transformStamped.transform.rotation.w = q.w();
    //         // }
    //     }
    // } else if (!isKCrane_)
    // {
    //     transformStamped.header.stamp = ros::Time::now();
    //     transformStamped.header.frame_id = "world";
    //     transformStamped.child_frame_id = sensorModelFrameId_;
    //     tf2::Quaternion q;
    //     q.setRPY(0.0, 0.0, 0.0);
    //     transformStamped.transform.rotation.x = q.x();
    //     transformStamped.transform.rotation.y = q.y();
    //     transformStamped.transform.rotation.z = q.z();
    //     transformStamped.transform.rotation.w = q.w();

    //     transformStamped.transform.translation.x =  0.0;
    //     transformStamped.transform.translation.y =  0.0;
    //     transformStamped.transform.translation.z = 10.0;
    // }

    // br.sendTransform(transformStamped);
    // ROS_INFO("broadcasted");
    // anglePub_.publish(jointStateLocal);

    // poseWithHeader.header.stamp = stampTime;
    // poseWithHeader.pose = linkMsg.pose[index];
    // return true;

    // ros::Time stampTime = ros::Time::now();
    // sensor_msgs::JointState rotatingBaseMsg;
    // model_msg_to_rotbase_joint_msg::PoseWithHeader poseWithHeader;
    // if (!pickOutSensorDataFromlinkMsg(*linkMsg, stampTime, rotatingBaseMsg/*, poseWithHeader*/))
    // {
    //     ROS_ERROR("Error at callback() in LoadTfBroadcaster");
    // }

    // jointBroadcaster_.publish(rotatingBaseMsg);
    // poseBroadcaster_.publish(poseWithHeader);
// }

// void LoadTfBroadcaster::quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const
// {
//     tf::Quaternion quat;
//     quaternionMsgToTF(geometry_quat, quat);
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
// }

// void LoadTfBroadcaster::callbackFromAngleMessage(const sensor_msgs::JointState::ConstPtr& jointMsg)
// {
//     mutex_.lock();
//     jointState_ = *jointMsg;
//     // jointAngle_ = jointMsg->position[0];
//     mutex_.unlock();
// }

// void LoadTfBroadcaster::broadcasSensorModelTf() const
// {
//     double xPos = 0.0;
//     double yPos = 0.0;
//     double zPos = 0.0;
//     double roll = 0.0;
//     double pitch = 0.0;
//     double yaw = 0.0;
//     const double timeInterval = 0.1;

//     static tf2_ros::TransformBroadcaster br;
//     geometry_msgs::TransformStamped transformStamped;

//     size_t count = 0;

//     while (ros::ok())
//     {
//         /////////////////// start define pose
//         // Please calculate and assign the values.
//         xPos = 0.0;
//         yPos = 0.0;
//         zPos = 10.0;
//         roll = 0.0;
//         pitch = 0.0;
//         yaw = 0.0;

//         // // The sample is below: (please un-commentout)
//         // const double centerX = 0.0;
//         // const double centerY = 0.0;
//         // const double centerZ = 10.0;
//         // const double shakeRadius = 3.0;
//         // const double shakeAngleDegree = 4.0;
//         // const double shakeCycleSec = 2.0;
//         // // shake axis is Y

//         // double nowTime = timeInterval * count;
//         // double sectionTime = nowTime;
//         // while (sectionTime - shakeCycleSec >= 0)
//         // {
//         //     sectionTime -= shakeCycleSec;
//         // }
//         // double shakeCycleSecQuarter = shakeCycleSec / 4;
//         // double shakeCycleSecHalf = shakeCycleSec / 2;

//         // double angleTmpDeg;
//         // double angleTmpRad;
//         // if (sectionTime < shakeCycleSecHalf)
//         // {
//         //     if (sectionTime < shakeCycleSecQuarter)
//         //     {
//         //         angleTmpDeg = ((sectionTime / shakeCycleSecQuarter) * shakeAngleDegree);
//         //     } else
//         //     {
//         //         angleTmpDeg = (((shakeCycleSecHalf - sectionTime) / shakeCycleSecQuarter) * shakeAngleDegree);
//         //     }
//         //     angleTmpRad = ((angleTmpDeg /360) * 2*M_PI);
//         //     yPos = centerY - (shakeRadius * std::sin(angleTmpRad));
//         //     roll = -angleTmpRad;
//         // } else
//         // {
//         //     if (sectionTime < shakeCycleSecQuarter * 3)
//         //     {
//         //         angleTmpDeg = (((sectionTime - shakeCycleSecHalf) / shakeCycleSecQuarter) * shakeAngleDegree);
//         //     }else
//         //     {
//         //         angleTmpDeg = (((shakeCycleSec - sectionTime) / shakeCycleSecQuarter) * shakeAngleDegree);
//         //     }
//         //     angleTmpRad = ((angleTmpDeg /360) * 2*M_PI);
//         //     yPos = centerY + (shakeRadius * std::sin(angleTmpRad));
//         //     roll = angleTmpRad;
//         // }
//         // zPos = centerZ - (shakeRadius * std::cos(angleTmpRad));
        
//         // xPos = centerX;
//         // pitch = 0.0;
//         // yaw = 0.0;
//         /////////////////// end define pose

//         transformStamped.header.stamp = ros::Time::now();
//         transformStamped.header.frame_id = "world";
//         transformStamped.child_frame_id = sensorModelFrameId_;
//         tf2::Quaternion q;
//         q.setRPY(roll, pitch, yaw);
//         transformStamped.transform.rotation.x = q.x();
//         transformStamped.transform.rotation.y = q.y();
//         transformStamped.transform.rotation.z = q.z();
//         transformStamped.transform.rotation.w = q.w();

//         transformStamped.transform.translation.x = xPos;
//         transformStamped.transform.translation.y = yPos;
//         transformStamped.transform.translation.z = zPos;

//         br.sendTransform(transformStamped);

//         count++;
//         ros::Duration(timeInterval).sleep();
//     }
// }
