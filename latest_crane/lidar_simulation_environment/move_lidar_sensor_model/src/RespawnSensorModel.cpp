#include "sensor_model_respawner/RespawnSensorModel.h"

RespawnSensorModel::RespawnSensorModel(ros::NodeHandle* nodeHandlePtr)
  : nodeHandlePtr_(nodeHandlePtr)
{
    respawnClient_ = nodeHandlePtr_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    std::string defaultModelName = "dtw_robot";
    std::string defaultReferenceFrame = "world";
    std::string defaultSensorModelFrameId = "front_laser_link";
    // nodeHandlePtr_->param("respawn_cycle", respawnCycle_, 0.1);
    nodeHandlePtr_->param("respawn_cycle", respawnCycle_, 1.0/8.0);//8.0 Hz
    nodeHandlePtr_->param("sensor_model_name", sensorModelName_, defaultModelName);
    nodeHandlePtr_->param("reference_frame", referenceFrame_, defaultReferenceFrame);
    nodeHandlePtr_->param("sensor_model_frame_id", sensorModelFrameId_, defaultSensorModelFrameId);
    nodeHandlePtr_->param("is_k_crane", isKCrane_, true);
    nodeHandlePtr_->param("is_lidar_sensor_system_on_boom", isLidarSensorSystemOnBoom_, true);
    nodeHandlePtr_->param("is_collision_warning", isCollisionWarning_, true);
    angleSub_ = nodeHandlePtr->subscribe((isCollisionWarning_ ? "/collision_warning/jointMsg" : "/ridar_scan/jointMsg"), 1, &RespawnSensorModel::callbackFromAngleMessage, this);
    anglePub_ = nodeHandlePtr_->advertise<sensor_msgs::JointState>("joint_angle", 1);

    jointState_.header.stamp = ros::Time::now();
    jointState_.name.resize(1);
    jointState_.position.resize(1);
    jointState_.velocity.resize(1);
    jointState_.name[0] = "rotating_base [rad, rad/sec]";
    jointState_.position[0] = 0.0 + std::asin(1.0);
    jointState_.velocity[0] = 0.0;
}

RespawnSensorModel::~RespawnSensorModel()
{}

void RespawnSensorModel::respawnFromTf(const ros::TimerEvent&)
{
    mutex_.lock();
    sensor_msgs::JointState jointStateLocal = jointState_;
    mutex_.unlock();

    geometry_msgs::Pose targetPose;
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped nowTransformStamped;
    try{
        nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, sensorModelFrameId_, ros::Time(0));
        // nowTransformStamped = tfBuffer.lookupTransform(referenceFrame_, sensorModelFrameId_, jointStateLocal.header.stamp);
            // ↑referenceFrame_, sensorModelFrameId_ を逆にするとなぜかnowTransformStamped.transform.translation.zがおかしく（マイナスに）なる
        // nowTransformStamped = tfBuffer.lookupTransform(sensorModelFrameId_, referenceFrame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(respawnCycle_).sleep();
        return;
    }
    targetPose.position.x = nowTransformStamped.transform.translation.x;
    targetPose.position.y = nowTransformStamped.transform.translation.y;
    targetPose.position.z = nowTransformStamped.transform.translation.z;
    double roll, pitch, yaw;
    quaternion2EulerAngles(roll, pitch, yaw, nowTransformStamped.transform.rotation);
    if (isKCrane_ && isLidarSensorSystemOnBoom_)
    {
        // geometry_msgs::Quaternion targetQuat = eulerAngles2Quaternion(0.0, 0.0, yaw);
        targetPose.orientation = eulerAngles2Quaternion(0.0, 0.0, yaw + jointStateLocal.position[0]);
    }
    else
    {
        // targetPose.orientation = nowTransformStamped.transform.rotation;
        targetPose.orientation = eulerAngles2Quaternion(roll, pitch, yaw + jointStateLocal.position[0]);
    }
    ROS_INFO_STREAM("targetPose.orientation" << targetPose.orientation);

    gazebo_msgs::SetModelState setModelState;
    gazebo_msgs::ModelState    modelState;

    // Set ModelState INFO
    modelState.model_name      = sensorModelName_;
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

    // sensor_msgs::JointState jointStateLocal;
    // jointStateLocal.header.stamp = ros::Time::now();
    // jointStateLocal.name.resize(1);
    // jointStateLocal.position.resize(1);
    // jointStateLocal.velocity.resize(1);
    // jointStateLocal.name[0] = jointStateLocal.name[0];
    // jointStateLocal.position[0] = jointStateLocal.position[0];
    // jointStateLocal.velocity[0] = jointStateLocal.velocity[0];
    // anglePub_.publish(jointStateLocal);
    jointStateLocal.header.stamp = ros::Time::now();
    anglePub_.publish(jointStateLocal);
    
    ros::Duration(respawnCycle_).sleep();
}

void RespawnSensorModel::callbackFromAngleMessage(const sensor_msgs::JointState::ConstPtr& jointMsg)
{
    mutex_.lock();
    jointState_ = *jointMsg;
    // jointAngle_ = jointMsg->position[0];
    mutex_.unlock();
}

void RespawnSensorModel::quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

geometry_msgs::Quaternion RespawnSensorModel::eulerAngles2Quaternion(double roll, double pitch, double yaw) const
{
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion geometry_quat;
    quaternionTFToMsg(quat, geometry_quat);
    return geometry_quat;
}
