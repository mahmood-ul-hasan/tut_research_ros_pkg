#define _USE_MATH_DEFINES
#include <cmath>
#include <unistd.h>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "orion_rotating_base/rotating_base.h"
#include "orion_rotating_base/sendCommand.h"
#include "orion_rotating_base/returnOrigin.h"

namespace rotating_base
{

class Node
{
public:
  Node(ros::NodeHandle& nodeHandle);
  virtual ~Node();

  void connect();
  bool isOK() const
  {
    return rotatingBase_ != NULL;
  }
  void disconnect();

  // Service Execution
  void spinCallback(const ros::TimerEvent&);

  // Callback Methods
  bool returnOriginCommand(orion_rotating_base::returnOrigin::Request& req, orion_rotating_base::returnOrigin::Response& res);
  bool actionCommand(orion_rotating_base::sendCommand::Request& req, orion_rotating_base::sendCommand::Response& res);

protected:
  RotatingBase* rotatingBase_;
  ros::NodeHandle nodeHandle_;
  ros::Publisher  publisher_;
  ros::ServiceServer returnOriginSrv_;
  ros::ServiceServer actionSrv_;
  double currentPos_;
  int currentSpeed_;
  std::string joint_name_;
  std::mutex mutex_;
};

Node::Node(ros::NodeHandle& nodeHandle)
    : rotatingBase_(NULL), nodeHandle_(nodeHandle)
{
  // publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("state", 1);
  publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_states", 1);
  returnOriginSrv_ = nodeHandle_.advertiseService("return_origin", &Node::returnOriginCommand, this);
  actionSrv_ = nodeHandle_.advertiseService("cmd", &Node::actionCommand, this);
  ros::param::param<std::string>("~joint_name", joint_name_, "rotating_base [rad, rad/sec]");
}

Node::~Node()
{
  disconnect();
}

void Node::connect()
{
  if (isOK())
  {
    disconnect();
  }

  rotatingBase_ = new RotatingBase();

  std::string host;
  int port;
  std::string defaultHost = rotatingBase_->DEFAULT_HOST;
  int defaultPort = RotatingBase::DEFAULT_PORT;
  ros::param::param<std::string>("~host", host, defaultHost);
  ros::param::param<int>("~port", port, defaultPort);

  while (!rotatingBase_->connect(host, port))
  {
    ROS_WARN("Unable to connect, retrying.");
    ros::Duration(1).sleep();
  }
  
  ROS_INFO("Connected to rotating_base.");

  mutex_.lock();
  currentPos_ = rotatingBase_->getPosition();
  currentSpeed_ = rotatingBase_->getSpeed();
  mutex_.unlock();
  if (currentPos_ > 360.0)
  {
    ROS_INFO_STREAM("currentPos_: " << currentPos_);
    ROS_ERROR("getPosition() was failed!");
  } else {
    ROS_INFO_STREAM("currentPos_: " << currentPos_);
  }
  if (currentSpeed_ == -1)
  {
    ROS_ERROR("getSpeed() was failed!");
  } else {
    ROS_INFO_STREAM("currentSpeed_: " << currentSpeed_);
  }
}

void Node::disconnect()
{
  if (rotatingBase_ != NULL)
  {
    rotatingBase_->disconnect();
    delete rotatingBase_;
    rotatingBase_ = NULL;
  }
}

void Node::spinCallback(const ros::TimerEvent&)
{
  if (!isOK())
  {
    return;
  }

  mutex_.lock();
  currentPos_ = rotatingBase_->getPosition();
  currentSpeed_ = rotatingBase_->getSpeed();
  mutex_.unlock();
  if (currentPos_ > 360.0)
  {
    ROS_ERROR("getPosition() was failed!");
  } else {
    ROS_DEBUG_STREAM("currentPos_: " << currentPos_);
  }
  if (currentSpeed_ == -1)
  {
    ROS_ERROR("getSpeed() was failed!");
  } else {
    ROS_DEBUG_STREAM("currentSpeed_: " << currentSpeed_);
  }

  // Publish Position
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(1);
  joint_state.position.resize(1);
  joint_state.velocity.resize(1);
  joint_state.name[0] = joint_name_;
  joint_state.position[0] = currentPos_;
  joint_state.velocity[0] = static_cast<double>(currentSpeed_);
  
  publisher_.publish(joint_state);
}

bool Node::returnOriginCommand(orion_rotating_base::returnOrigin::Request& req, orion_rotating_base::returnOrigin::Response& res)
{
  int nowSpeed;
  if (isOK())
  {
    nowSpeed = rotatingBase_->getSpeed();
  } else {
    ROS_ERROR("laser was disconnected!");
    res.success = false;
    return false;
  }
  
  if (nowSpeed == -1)
  {
    ROS_ERROR("getSpeed() was failed!");
    res.success = false;
    return false;
  } else {
    ROS_DEBUG_STREAM("nowSpeed: " << nowSpeed);
  }

  if (!isOK() || !rotatingBase_->setSpeed(RotatingBase::MAX_SPEED))
  {
    ROS_ERROR("setSpeed() was failed!");
    res.success = false;
    return false;
  }
  if (!isOK() || !rotatingBase_->returnOrigin())
  {
    ROS_ERROR("returnOrigin() was failed!");
    res.success = false;
    return false;
  }
  if (!isOK() || !rotatingBase_->setSpeed(nowSpeed))
  {
    ROS_ERROR("setSpeed() was failed!");
    res.success = false;
    return false;
  }

  mutex_.lock();
  currentPos_ = rotatingBase_->getPosition();
  currentSpeed_ = rotatingBase_->getSpeed();
  mutex_.unlock();
  if (currentPos_ > 360.0)
  {
    ROS_ERROR("getPosition() was failed!");
  } else {
    ROS_INFO_STREAM("currentPos_: " << currentPos_);
  }
  if (currentSpeed_ == -1)
  {
    ROS_ERROR("getSpeed() was failed!");
  } else {
    ROS_INFO_STREAM("currentSpeed_: " << currentSpeed_);
  }

  res.success = true;
  return true;
}

bool Node::actionCommand(orion_rotating_base::sendCommand::Request& req, orion_rotating_base::sendCommand::Response& res)
{
  if (!isOK() || !rotatingBase_->setSpeed(req.speed))
  {
    ROS_ERROR("setSpeed() was failed!");
    res.success = false;
    return false;
  }
  if (rotatingBase_->getSpeed() != req.speed)
  {
    ROS_ERROR("getSpeed() was failed!");
    res.success = false;
    return false;
  }

  if (!rotatingBase_->setTargetPos(std::round(req.end_pos * 10) / 10))
  {
    ROS_ERROR("setTargetPos() was failed!");
    res.success = false;
    return false;
  }
  if (rotatingBase_->getTargetPos() != std::round(req.end_pos * 10) / 10)
  {
    ROS_ERROR("getTargetPos() was failed!");
    res.success = false;
    return false;
  }

  if (!rotatingBase_->goTargetPos())
  {
    ROS_ERROR("goTargetPos() was failed!");
    res.success = false;
    return false;
  }
  
  mutex_.lock();
  currentPos_ = rotatingBase_->getPosition();
  currentSpeed_ = rotatingBase_->getSpeed();
  mutex_.unlock();
  if (currentPos_ > 360.0)
  {
    ROS_ERROR("getPosition() was failed!");
  } else {
    ROS_INFO_STREAM("currentPos_: " << currentPos_);
  }
  if (currentSpeed_ == -1)
  {
    ROS_ERROR("getSpeed() was failed!");
  } else {
    ROS_INFO_STREAM("currentSpeed_: " << currentSpeed_);
  }

  res.success = true;
  return true;
}

} // namespace rotating_base

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotating_base_node");
  ros::NodeHandle nodeHandle("~");

  // display ROS_DEBUG log
  // if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  rotating_base::Node node(nodeHandle);
  node.connect();

  if (ros::ok())
  {
    double frequency;
    double defaultFrequency = RotatingBase::DEFAULT_FREQUENCY;
    ros::param::param<double>("~frequency", frequency, defaultFrequency);
    ros::Timer spin_timer = nodeHandle.createTimer(ros::Duration(1 / frequency), &rotating_base::Node::spinCallback, &node);
    
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
  }

  node.disconnect();

  return 0;
}
