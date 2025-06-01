#include "pedestrian_respawner/RespawnPedestrian.h"
#include <cmath>

RespawnPedestrian::RespawnPedestrian(ros::NodeHandle* nodeHandlePtr)
  : nodeHandlePtr_(nodeHandlePtr),
    wasWarningOfCollision_(false),
    isOk_(true)
{
    respawnClient_ = nodeHandlePtr_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    // std::string defaultModelName = "dtw_robot";
    std::string defaultReferenceFrame = "world";
    // std::string defaultSensorModelFrameId = "lidar_sensor_model";
    std::string defaultPedestrianModelName = "target";
    nodeHandlePtr_->param("respawn_cycle", respawnCycle_, 0.1);
    // nodeHandlePtr_->param("sensor_model_name", sensorModelName_, defaultModelName);
    nodeHandlePtr_->param("reference_frame", referenceFrame_, defaultReferenceFrame);
    // nodeHandlePtr_->param("sensor_model_name", sensorModelFrameId_, defaultSensorModelFrameId);
    // nodeHandlePtr_->param("is_k_crane", isKCrane_, true);
    // nodeHandlePtr_->param("is_lidar_sensor_system_on_boom", isLidarSensorSystemOnBoom_, true);
    // nodeHandlePtr_->param("is_collision_warning", isCollisionWarning_, true);
    nodeHandlePtr_->param("pedestrian_model_name", pedestrianModelName_, defaultPedestrianModelName);
    nodeHandlePtr_->param("initial_pos_x", initialPosX_, 30.0);
    nodeHandlePtr_->param("initial_pos_y", initialPosY_, 10.0);
    // nodeHandlePtr_->param("initial_pos_z", initialPosZ_, 0.0);
    nodeHandlePtr_->param("target_pos_x", targetPosX_, 20.0);
    nodeHandlePtr_->param("target_pos_y", targetPosY_, 5.0);
    // nodeHandlePtr_->param("target_pos_z", targetPosZ_, 0.0);
    nodeHandlePtr_->param("move_speed", moveSpeed_, 1.0);
    // angleSub_ = nodeHandlePtr->subscribe((isCollisionWarning_ ? "/collision_warning/jointMsg" : "/ridar_scan/jointMsg"), 1, &RespawnPedestrian::callbackFromAngleMessage, this);
    // anglePub_ = nodeHandlePtr_->advertise<sensor_msgs::JointState>("joint_angle", 1);
    resultSub_ = nodeHandlePtr->subscribe("/collision_warning/warning_result", 1, &RespawnPedestrian::callbackFromResultMessage, this);

    nowPosX_ = initialPosX_;
    nowPosY_ = initialPosY_;
    moveAngle_ = std::atan2(targetPosY_ - initialPosY_, targetPosX_ - initialPosX_);

    // jointState_.header.stamp = ros::Time::now();
    // jointState_.name.resize(1);
    // jointState_.position.resize(1);
    // jointState_.velocity.resize(1);
    // jointState_.name[0] = "rotating_base [rad, rad/sec]";
    // jointState_.position[0] = 0.0;
    // jointState_.velocity[0] = 0.0;
}

RespawnPedestrian::~RespawnPedestrian()
{}

void RespawnPedestrian::respawnFromTf(const ros::TimerEvent&)
{
    // mutex_.lock();
    // sensor_msgs::JointState jointStateLocal = jointState_;
    // mutex_.unlock();

    // geometry_msgs::Pose targetPose;
    // static tf2_ros::Buffer tfBuffer;
    // static tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped nowTransformStamped;
    // try{
    //     nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, sensorModelFrameId_, ros::Time(0));
    //     // nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, sensorModelFrameId_, jointStateLocal.header.stamp);
    //         // ↑referenceFrame_, sensorModelFrameId_ を逆にするとなぜかnowTransformStamped.transform.translation.zがおかしく（マイナスに）なる
    //     // nowTransformStamped = tfBuffer.lookupTransform(sensorModelFrameId_, referenceFrame_, ros::Time(0));
    // }
    // catch (tf2::TransformException &ex) {
    //     ROS_WARN("%s",ex.what());
    //     ros::Duration(respawnCycle_).sleep();
    //     return;
    // }
    // targetPose.position.x = nowTransformStamped.transform.translation.x;
    // targetPose.position.y = nowTransformStamped.transform.translation.y;
    // targetPose.position.z = nowTransformStamped.transform.translation.z;
    // double roll, pitch, yaw;
    // quaternion2EulerAngles(roll, pitch, yaw, nowTransformStamped.transform.rotation);
    // if (isKCrane_ && isLidarSensorSystemOnBoom_)
    // {
    //     // geometry_msgs::Quaternion targetQuat = eulerAngles2Quaternion(0.0, 0.0, yaw);
    //     targetPose.orientation = eulerAngles2Quaternion(0.0, 0.0, yaw + jointStateLocal.position[0]);
    // }
    // else
    // {
    //     // targetPose.orientation = nowTransformStamped.transform.rotation;
    //     targetPose.orientation = eulerAngles2Quaternion(roll, pitch, yaw + jointStateLocal.position[0]);
    // }

    mutex_.lock();
    bool wasWarnedLocal = wasWarningOfCollision_;
    mutex_.unlock();
    if (!isOk_)
    {
        return;
    } else if (wasWarnedLocal)
    {
        ROS_WARN("stoped becase collision warning node warned of collision.");
        isOk_ = false;
        return;
    } else if ((initialPosX_ <= targetPosX_ && nowPosX_ >= targetPosX_) || (initialPosX_ > targetPosX_ && nowPosX_ <= targetPosX_)
            || (initialPosY_ <= targetPosY_ && nowPosY_ >= targetPosY_) || (initialPosY_ > targetPosY_ && nowPosY_ <= targetPosY_))
    {
        ROS_INFO("stoped becase pedestrian reached target pose.");
        isOk_ = false;
        return;
    } else
    { // respawnCycle_ [s] * moveSpeed_ [m/s]
        double moveLength = respawnCycle_ * moveSpeed_;
        nowPosX_ += moveLength * std::cos(moveAngle_);
        nowPosY_ += moveLength * std::sin(moveAngle_);

        geometry_msgs::Pose targetPose;
        targetPose.position.x = nowPosX_;
        targetPose.position.y = nowPosY_;
        targetPose.position.z = 0.0;
        targetPose.orientation = eulerAngles2Quaternion(0.0, 0.0, moveAngle_);

        gazebo_msgs::SetModelState setModelState;
        gazebo_msgs::ModelState    modelState;

        // Set ModelState INFO
        modelState.model_name      = pedestrianModelName_;
        modelState.pose            = targetPose;
        modelState.reference_frame = referenceFrame_;
        
        // Servcie Call
        setModelState.request.model_state = modelState;
        size_t jndex = 0;
        while ( !respawnClient_.call(setModelState))
        {
            jndex++;
            if (jndex >= NUM_OF_RETRY)
            {
                ROS_ERROR("failed to connect");
                return;
            }
        }
        ROS_INFO("respawned");
    }

    // sensor_msgs::JointState jointStateLocal;
    // jointStateLocal.header.stamp = ros::Time::now();
    // jointStateLocal.name.resize(1);
    // jointStateLocal.position.resize(1);
    // jointStateLocal.velocity.resize(1);
    // jointStateLocal.name[0] = jointStateLocal.name[0];
    // jointStateLocal.position[0] = jointStateLocal.position[0];
    // jointStateLocal.velocity[0] = jointStateLocal.velocity[0];
    // anglePub_.publish(jointStateLocal);
    // anglePub_.publish(jointStateLocal);
    
    // ros::Duration(respawnCycle_).sleep();
}

void RespawnPedestrian::callbackFromResultMessage(const std_msgs::Bool::ConstPtr& resultMsg)
{
    mutex_.lock();
    wasWarningOfCollision_ = resultMsg->data;
    // jointState_ = *jointMsg;
    // // jointAngle_ = jointMsg->position[0];
    mutex_.unlock();
}

// void RespawnPedestrian::quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const
// {
//     tf::Quaternion quat;
//     quaternionMsgToTF(geometry_quat, quat);
//     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
// }

geometry_msgs::Quaternion RespawnPedestrian::eulerAngles2Quaternion(double roll, double pitch, double yaw) const
{
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}
