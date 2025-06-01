#include "pedestrian_respawner/RespawnPedestrian.h"
#include <cmath>

RespawnPedestrian::RespawnPedestrian(ros::NodeHandle* nodeHandlePtr)
  : nodeHandlePtr_(nodeHandlePtr),
    wasWarningOfCollision_(false),
    isOk_(true),
    initialPosX_(NUM_OF_PERSON, 0.0),
    initialPosY_(NUM_OF_PERSON, 0.0),
    targetPosX_(NUM_OF_PERSON, 0.0),
    targetPosY_(NUM_OF_PERSON, 0.0),
    moveSpeed_(NUM_OF_PERSON, 0.0),
    nowPosX_(NUM_OF_PERSON, 0.0),
    nowPosY_(NUM_OF_PERSON, 0.0),
    moveAngle_(NUM_OF_PERSON, 0.0),
    wasReached_(NUM_OF_PERSON, false)
{
    respawnClient_ = nodeHandlePtr_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    // std::string defaultModelName = "dtw_robot";
    std::string defaultReferenceFrame = "world";
    // std::string defaultSensorModelFrameId = "lidar_sensor_model";
    std::string defaultPedestrianModelName = "target_person";
    nodeHandlePtr_->param("respawn_cycle", respawnCycle_, 0.1);
    // nodeHandlePtr_->param("sensor_model_name", sensorModelName_, defaultModelName);
    nodeHandlePtr_->param("reference_frame", referenceFrame_, defaultReferenceFrame);
    // nodeHandlePtr_->param("sensor_model_name", sensorModelFrameId_, defaultSensorModelFrameId);
    // nodeHandlePtr_->param("is_k_crane", isKCrane_, true);
    // nodeHandlePtr_->param("is_lidar_sensor_system_on_boom", isLidarSensorSystemOnBoom_, true);
    // nodeHandlePtr_->param("is_collision_warning", isCollisionWarning_, true);

    nodeHandlePtr_->param("pedestrian_model_name", pedestrianModelName_, defaultPedestrianModelName);
    nodeHandlePtr_->param("initial_pos_x_0", initialPosX_[0], 30.0);
    nodeHandlePtr_->param("initial_pos_y_0", initialPosY_[0], 10.0);
    // nodeHandlePtr_->param("initial_pos_z", initialPosZ_, 0.0);
    nodeHandlePtr_->param("target_pos_x_0", targetPosX_[0], 20.0);
    nodeHandlePtr_->param("target_pos_y_0", targetPosY_[0], 5.0);
    // nodeHandlePtr_->param("target_pos_z", targetPosZ_, 0.0);
    nodeHandlePtr_->param("move_speed_0", moveSpeed_[0], 1.0);
    nodeHandlePtr_->param("initial_pos_x_1", initialPosX_[1], 31.0);
    nodeHandlePtr_->param("initial_pos_y_1", initialPosY_[1], 11.0);
    nodeHandlePtr_->param("target_pos_x_1", targetPosX_[1], 21.0);
    nodeHandlePtr_->param("target_pos_y_1", targetPosY_[1], 6.0);
    nodeHandlePtr_->param("move_speed_1", moveSpeed_[1], 1.0);
    nodeHandlePtr_->param("initial_pos_x_2", initialPosX_[2], 32.0);
    nodeHandlePtr_->param("initial_pos_y_2", initialPosY_[2], 12.0);
    nodeHandlePtr_->param("target_pos_x_2", targetPosX_[2], 22.0);
    nodeHandlePtr_->param("target_pos_y_2", targetPosY_[2], 7.0);
    nodeHandlePtr_->param("move_speed_2", moveSpeed_[2], 1.0);
    nodeHandlePtr_->param("initial_pos_x_3", initialPosX_[3], 33.0);
    nodeHandlePtr_->param("initial_pos_y_3", initialPosY_[3], 13.0);
    nodeHandlePtr_->param("target_pos_x_3", targetPosX_[3], 23.0);
    nodeHandlePtr_->param("target_pos_y_3", targetPosY_[3], 8.0);
    nodeHandlePtr_->param("move_speed_3", moveSpeed_[3], 1.0);
    nodeHandlePtr_->param("initial_pos_x_4", initialPosX_[4], 34.0);
    nodeHandlePtr_->param("initial_pos_y_4", initialPosY_[4], 14.0);
    nodeHandlePtr_->param("target_pos_x_4", targetPosX_[4], 24.0);
    nodeHandlePtr_->param("target_pos_y_4", targetPosY_[4], 9.0);
    nodeHandlePtr_->param("move_speed_4", moveSpeed_[4], 1.0);

    // angleSub_ = nodeHandlePtr->subscribe((isCollisionWarning_ ? "/collision_warning/jointMsg" : "/ridar_scan/jointMsg"), 1, &RespawnPedestrian::callbackFromAngleMessage, this);
    // anglePub_ = nodeHandlePtr_->advertise<sensor_msgs::JointState>("joint_angle", 1);
    resultSub_ = nodeHandlePtr->subscribe("/collision_warning/warning_result", 1, &RespawnPedestrian::callbackFromResultMessage, this);

    for (size_t index = 0; index < NUM_OF_PERSON; index++)
    {
        nowPosX_[index] = initialPosX_[index];
        nowPosY_[index] = initialPosY_[index];
        moveAngle_[index] = std::atan2(targetPosY_[index] - initialPosY_[index], targetPosX_[index] - initialPosX_[index]);
    }

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
    if (!isOk_)
    {
        return;
    }

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

    bool wasReachedLocal = true;
    for (size_t index = 0; index < NUM_OF_PERSON; index++)
    {
        if (!wasReached_[index])
        {
            wasReachedLocal = false;
        }
    }
    if (wasReachedLocal)
    {
        ROS_INFO("stoped becase pedestrian reached target pose.");
        isOk_ = false;
        return;
    }

    for (size_t index = 0; index < NUM_OF_PERSON; index++)
    {
        if (wasWarnedLocal)
        {
            ROS_WARN("stoped becase collision warning node warned of collision.");
            isOk_ = false;
            return;
        } else if (!wasReached_[index]
                && ((initialPosX_[index] <= targetPosX_[index] && nowPosX_[index] >= targetPosX_[index])
                ||  (initialPosX_[index] >  targetPosX_[index] && nowPosX_[index] <= targetPosX_[index])
                ||  (initialPosY_[index] <= targetPosY_[index] && nowPosY_[index] >= targetPosY_[index])
                ||  (initialPosY_[index] >  targetPosY_[index] && nowPosY_[index] <= targetPosY_[index])
                   )
                  )
        {
            wasReached_[index] = true;
        } else if (!wasReached_[index])
        { // respawnCycle_ [s] * moveSpeed_[index] [m/s]
            double moveLength = respawnCycle_ * moveSpeed_[index];
            nowPosX_[index] += moveLength * std::cos(moveAngle_[index]);
            nowPosY_[index] += moveLength * std::sin(moveAngle_[index]);

            geometry_msgs::Pose targetPose;
            targetPose.position.x = nowPosX_[index];
            targetPose.position.y = nowPosY_[index];
            targetPose.position.z = 0.0;
            targetPose.orientation = eulerAngles2Quaternion(0.0, 0.0, moveAngle_[index]);

            gazebo_msgs::SetModelState setModelState;
            gazebo_msgs::ModelState    modelState;

            // Set ModelState INFO
            modelState.model_name      = pedestrianModelName_ + std::to_string(index);
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
            // ROS_INFO("respawned");
        }
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
