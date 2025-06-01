#define _USE_MATH_DEFINES
#include <cmath>
#include <unistd.h>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "orion_rotating_base/rotating_base.h"
#include "orion_rotating_base/sendCommand.h"
//#include "orion_rotating_base/returnOrigin.h"
#include <sys/time.h>
#include <ctime>
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
  //bool returnOriginCommand(orion_rotating_base::returnOrigin::Request& req, orion_rotating_base::returnOrigin::Response& res);
  bool actionCommand(orion_rotating_base::sendCommand::Request& req, orion_rotating_base::sendCommand::Response& res);

protected:
  RotatingBase* rotatingBase_;
  ros::NodeHandle nodeHandle_;
  ros::Publisher  publisher_;
  //ros::ServiceServer returnOriginSrv_;
  ros::ServiceServer actionSrv_;
  double currentPos_;
  int currentSpeed_;
  int setSpeed_;
  double goTargetPos_;
  double setTargetPos_;

  std::string joint_name_;
  std::mutex mutex_;
};

Node::Node(ros::NodeHandle& nodeHandle)
    : rotatingBase_(NULL), nodeHandle_(nodeHandle)
{
  // publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("state", 1);
  publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_states", 1);
  //returnOriginSrv_ = nodeHandle_.advertiseService("return_origin", &Node::returnOriginCommand, this);
  actionSrv_ = nodeHandle_.advertiseService("cmd", &Node::actionCommand, this);
  ros::param::param<std::string>("~joint_name", joint_name_, "rotating_base [rad, rad/sec]");
}

Node::~Node()
{
  disconnect();
}

void Node::connect()
{
   std::cout << "connect(): " << std::endl;

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
  std::cout << "currentPos___connect: " << currentPos_ << std::endl;
  mutex_.unlock();

  mutex_.lock();
  currentSpeed_ = rotatingBase_->getSpeed();
  std::cout << "currentSpeed___connect: " << currentSpeed_ << std::endl;
  mutex_.unlock();

  if (currentPos_ > 360.0)
  {
    ROS_INFO_STREAM("currentPos___connect: " << currentPos_);
    ROS_ERROR("getPosition() was failed!");
  } else {
    ROS_INFO_STREAM("currentPos___connect: " << currentPos_);
  }
  if (currentSpeed_ == -1)
  {
    ROS_ERROR("getSpeed() was failed!");
  } else {
    ROS_INFO_STREAM("currentSpeed___connect: " << currentSpeed_);
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



time_t pervious_time = time(nullptr);
time_t current_time;

void Node::spinCallback(const ros::TimerEvent&)
{
  //std::cout << "spinCallback(): " << std::endl;

  if (!isOK())
  {
    return;
  }
  
  // mutex_.lock();
  // currentPos_ = rotatingBase_->getPosition();
  // currentSpeed_ = rotatingBase_->getSpeed();
  // mutex_.unlock();

 

 current_time = time(nullptr);
  //if (current_time >  pervious_time +  (0.0000))
  {
  mutex_.lock();
  currentPos_ = rotatingBase_->getPosition();
 // std::cout << "currentPos___spinCallback: " << currentPos_ << std::endl;
  mutex_.unlock();

//   mutex_.lock();
//   currentSpeed_ = rotatingBase_->getSpeed();
//  // std::cout << "currentSpeed___spinCallback: " << currentSpeed_ << std::endl;
//   mutex_.unlock();


  if (currentPos_ > 360.0)
  {
    std::cout << "currentPos___spinCallback: " << currentPos_ << std::endl;
    std::cout << "getPosition() was failed! at spinCallback: " << currentPos_ << std::endl;
    // ROS_ERROR("getPosition() was failed! at spinCallback");
  } else {
    ROS_DEBUG_STREAM("currentPos___spinCallback: " << currentPos_);
  }
  
  if (currentSpeed_ == -1)
  {
    std::cout << "currentSpeed___spinCallback: " << currentSpeed_ << std::endl;
    std::cout << "getSpeed() was failed! at spinCallback: " << currentSpeed_ << std::endl;
    // ROS_ERROR("getSpeed() was failed! at spinCallback");
  } else {
    ROS_DEBUG_STREAM("currentSpeed___spinCallback: " << currentSpeed_);
  }

  }
  pervious_time = current_time ;


  
  
  //if ((currentPos_ >= 90) && (currentPos_ <= 270) && (currentSpeed_ >= 0) && (currentSpeed_ <=3))
  if ((currentPos_ >= 90) && (currentPos_ <= 270))
  {
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


   /*
  current_time = time(nullptr);
  if (current_time >  pervious_time +  (0.1)){
  std::cout << "currentPos___Publish: " << currentPos_ << std::endl;
  std::cout << "currentSpeed___Publish: " << currentSpeed_ << std::endl;
  }
  pervious_time = current_time ;
  */
  
}


bool Node::actionCommand(orion_rotating_base::sendCommand::Request& req, orion_rotating_base::sendCommand::Response& res)
{

  //mutex_.lock();
  //currentPos_ = rotatingBase_->getPosition();
  //currentSpeed_ = rotatingBase_->getSpeed();
  //mutex_.unlock();


   mutex_.lock();
  currentPos_ = rotatingBase_->getPosition();
  std::cout << "currentPos___actionCommand: " << currentPos_ << std::endl;
  mutex_.unlock();

  mutex_.lock();
  currentSpeed_ = rotatingBase_->getSpeed();
  std::cout << "currentSpeed___actionCommand: " << currentSpeed_ << std::endl;
  mutex_.unlock();

/*
 mutex_.lock();
  setSpeed_ = rotatingBase_->setSpeed(req.speed);
  std::cout << "setSpeed_actionCommand: " << std::endl;
  mutex_.unlock();

 mutex_.lock();
  setTargetPos_ = rotatingBase_->setTargetPos(std::round(req.end_pos * 10) / 10);
  std::cout << "setTargetPos__actionCommand: " << std::endl;
  mutex_.unlock();

  mutex_.lock();
  goTargetPos_ = rotatingBase_->goTargetPos();
  std::cout << "goTargetPos___actionCommand: " << std::endl;
  mutex_.unlock();
*/


  if (!isOK() || !rotatingBase_->setSpeed(req.speed))
  {
    ROS_WARN("setSpeed() was failed! at actionCommand() 280");
    res.success = false;
    return false;
  }
  if (rotatingBase_->getSpeed() != req.speed)
 // if (currentSpeed_ != req.speed)
  {
    ROS_WARN("getSpeed() was failed! at actionCommand() 286");
    res.success = false;
    return false;
  }

  if (!rotatingBase_->setTargetPos(std::round(req.end_pos * 10) / 10))
  //if (!setTargetPos_)
  {
    ROS_WARN("setTargetPos() was failed! at actionCommand() 293");
    res.success = false;
    return false;
  }
  if (rotatingBase_->getTargetPos() != std::round(req.end_pos * 10) / 10)
  {
    ROS_WARN("getTargetPos() was failed! at actionCommand() 299");
    res.success = false;
    return false;
  }

  if (!rotatingBase_->goTargetPos())
  //if (!goTargetPos_)
  {
    ROS_WARN("goTargetPos() was failed! at actionCommand() 306");
    res.success = false;
    return false;
  }
  
  
  
  

  if (currentPos_ > 360.0)
  {
    ROS_WARN("getPosition() was failed! at actionCommand() 330");
  } else {
    ROS_INFO_STREAM("currentPos_: at actionCommand() 332: " << currentPos_);
  }
  if (currentSpeed_ == -1)
  {
    ROS_WARN("getSpeed() was failed! at actionCommand() 336");
  } else {
    ROS_INFO_STREAM("currentSpeed_: at actionCommand() 338: " << currentSpeed_);
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
