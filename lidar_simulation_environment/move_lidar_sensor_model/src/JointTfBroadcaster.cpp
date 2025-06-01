#include "k_crane_joint_tf_broadcaster/JointTfBroadcaster.h"
// #include "model_msg_to_rotbase_joint_msg/PoseWithHeader.h"
// #include <tf/transform_broadcaster.h>

// #define _USE_MATH_DEFINES
// // use M_PI
// #include <cmath>
// use tf2_ros::TransformBroadcaster
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
// use tf2::Quaternion
#include <tf2/LinearMath/Quaternion.h>

JointTfBroadcaster::JointTfBroadcaster(ros::NodeHandle* nodeHandlePtr)
 : nodeHandlePtr_(nodeHandlePtr),
   isFirst_(true)
{
    // std::string defaultSensorModelFrameId = "lidar_sensor_model";
    // nodeHandlePtr->param("sensor_model_frame_id", sensorModelFrameId_, defaultSensorModelFrameId);
    nodeHandlePtr->param("upper_link_default_x", upperLinkDefaultX_, 0.0);
    nodeHandlePtr->param("upper_link_default_y", upperLinkDefaultY_, 0.0);
    nodeHandlePtr->param("upper_link_default_z", upperLinkDefaultZ_, 0.0);
    nodeHandlePtr->param("upper_link_default_joint", upperLinkDefaultJoint_, 0.0); // def_yaw
    nodeHandlePtr->param("boom_link_default_x", boomLinkDefaultX_, 1.4);
    nodeHandlePtr->param("boom_link_default_y", boomLinkDefaultY_, 0.191);
    nodeHandlePtr->param("boom_link_default_z", boomLinkDefaultZ_, 1.008);
    nodeHandlePtr->param("boom_link_default_joint", boomLinkDefaultJoint_, -M_PI/4); // def_pitch
    nodeHandlePtr->param("min_time_interval", minTimeInterval_, 0.1);
    // bool defaultLidarSensorSystemOnBoom = true;
    // nodeHandlePtr->param("is_lidar_sensor_system_on_boom", isLidarSensorSystemOnBoom_, defaultLidarSensorSystemOnBoom);
    jointSub_ = nodeHandlePtr->subscribe("/k_crane/joint_states", 1, &JointTfBroadcaster::callback, this);
}

JointTfBroadcaster::~JointTfBroadcaster() {}

void JointTfBroadcaster::callback(const sensor_msgs::JointState::ConstPtr& jointMsg)
{
    if (isFirst_)
    {
        previousTime_ = ros::Time::now();
        isFirst_ = false;
        return;
    }
    nowTime_ = ros::Time::now();
    ros::Duration callbackDuration = nowTime_ - previousTime_;
    double timeInterval = callbackDuration.sec + (static_cast<double>(callbackDuration.nsec) * 0.000000001);
    if (timeInterval < minTimeInterval_)
    {
        return;
    }
    previousTime_ = nowTime_;

    std::vector<std::string> nameStrs = jointMsg->name;
    for (size_t loopIdx = 0; loopIdx < 2; loopIdx++)
    {
        auto itr = std::find(nameStrs.begin(), nameStrs.end(), (loopIdx == 0 ? "pitch_joint" : "yaw_joint"));
        size_t index = std::distance(nameStrs.begin(), itr);
        if (nameStrs.size() == index)
        {
            ROS_ERROR("Error at callback() in JointTfBroadcaster");
            // return false;
        } else {
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;

            // transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.stamp = jointMsg->header.stamp;
            transformStamped.header.frame_id = (loopIdx == 0 ? "upper_link_processed" : "lower_link");
            transformStamped.child_frame_id = (loopIdx == 0 ? "boom_link_processed" : "upper_link_processed");

            transformStamped.transform.translation.x = (loopIdx == 0 ? boomLinkDefaultX_ : upperLinkDefaultX_);
            transformStamped.transform.translation.y = (loopIdx == 0 ? boomLinkDefaultY_ : upperLinkDefaultY_);
            transformStamped.transform.translation.z = (loopIdx == 0 ? boomLinkDefaultZ_ : upperLinkDefaultZ_);
            tf2::Quaternion q;
            q.setRPY(0.0,
                        (loopIdx == 0 ? boomLinkDefaultJoint_ - jointMsg->position[index] : 0.0),
                        (loopIdx == 0 ? 0.0 : upperLinkDefaultJoint_ - jointMsg->position[index]));
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            br.sendTransform(transformStamped);
            // ROS_INFO("broadcasted");
        }
    }
}

// void JointTfBroadcaster::quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const
// {
//     tf::Quaternion quat;
//     quaternionMsgToTF(geometry_quat, quat);
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
// }

// void JointTfBroadcaster::broadcasSensorModelTf() const
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
