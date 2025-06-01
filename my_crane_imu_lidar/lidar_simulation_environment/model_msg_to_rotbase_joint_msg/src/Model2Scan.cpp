#include "Model2Scan.h"
// #include "model_msg_to_rotbase_joint_msg/PoseWithHeader.h"
#include <tf/transform_broadcaster.h>

Model2Scan::Model2Scan(ros::NodeHandle* nodeHandlePtr)
 : nodeHandlePtr_(nodeHandlePtr)
{
    jointPublisher_ = nodeHandlePtr_->advertise<sensor_msgs::JointState>("model_joint_states", 1);
    // posePublisher_ = nodeHandlePtr_->advertise<model_msg_to_rotbase_joint_msg::PoseWithHeader>("model_pose", 1);
    modelSub_ = nodeHandlePtr->subscribe("/gazebo/model_states", 1, &Model2Scan::callback, this);
    std::string defaultModelName = "dtw_robot";
    nodeHandlePtr->param("sensor_model_name", sensorModelName_, defaultModelName);
}

Model2Scan::~Model2Scan() {}

void Model2Scan::callback(const gazebo_msgs::ModelStates::ConstPtr& modelMsg)
{
    ros::Time stampTime = ros::Time::now();
    sensor_msgs::JointState rotatingBaseMsg;
    // model_msg_to_rotbase_joint_msg::PoseWithHeader poseWithHeader;
    if (!pickOutSensorDataFromModelMsg(*modelMsg, stampTime, rotatingBaseMsg/*, poseWithHeader*/))
    {
        ROS_ERROR("Error at callback()");
    }

    jointPublisher_.publish(rotatingBaseMsg);
    // posePublisher_.publish(poseWithHeader);
}

bool Model2Scan::pickOutSensorDataFromModelMsg(const gazebo_msgs::ModelStates& modelMsg, const ros::Time stampTime, sensor_msgs::JointState& rotatingBaseMsg/*, model_msg_to_rotbase_joint_msg::PoseWithHeader& poseWithHeader*/) const
{
    std::vector<std::string> nameStrs = modelMsg.name;
    // ROS_INFO("sensorModelName_ = %s",sensorModelName_.c_str());
    // ROS_INFO_STREAM("sensorModelName_ = " << sensorModelName_.c_str());
    // for (size_t jndex = 0; jndex < nameStrs.size(); jndex++)
    // {
    //     ROS_INFO_STREAM("nameStrs[" << jndex << "] = " << nameStrs[jndex]);
    // }
    
    auto itr = std::find(nameStrs.begin(), nameStrs.end(), sensorModelName_);
    // ROS_INFO_STREAM("*itr = " << *itr);
    // ROS_INFO_STREAM("nameStrs.size() = " << nameStrs.size());
    size_t index = std::distance(nameStrs.begin(), itr);
    // ROS_INFO_STREAM("index = " << index);
    if (nameStrs.size() == index)
    {
        return false;
    } else {
        double roll, pitch, yaw;
        quaternion2EulerAngles(roll, pitch, yaw, modelMsg.pose[index].orientation);
        rotatingBaseMsg.header.stamp = stampTime;
        rotatingBaseMsg.name.resize(1);
        rotatingBaseMsg.position.resize(1);
        rotatingBaseMsg.velocity.resize(1);
        rotatingBaseMsg.name[0] = "rotating_base [rad]";
        rotatingBaseMsg.position[0] = yaw;
        rotatingBaseMsg.velocity[0] = 0;

        // poseWithHeader.header.stamp = stampTime;
        // poseWithHeader.pose = modelMsg.pose[index];
        return true;
    }
}

void Model2Scan::quaternion2EulerAngles(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion& geometry_quat) const
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}
