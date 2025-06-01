/*
 * flir_ptu_driver ROS package
 * Copyright (C) 2014 Mike Purvis (mpurvis@clearpathrobotics.com)
 *
 * PTU ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 * Author: Toby Collett (University of Auckland)
 * Date: 2003-02-10
 *
 * Player - One Hell of a Robot Server
 * Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                     gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <flir_ptu_driver/driver.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
#include "flir_ptu_driver/initCommand.h"
#include "flir_ptu_driver/sendCommand.h"

#include <string>
#include <cmath>
#include <thread>
#include <mutex>


namespace flir_ptu_driver
{

class Node
{
public:
  explicit Node(ros::NodeHandle& node_handle);
  Node();
  ~Node();

  // Service Control
  void connect();
  bool ok()
  {
    return m_pantilt != NULL;
  }
  void disconnect();

  // Service Execution
  void spinCallback(const ros::TimerEvent&);

  // Callback Methods
  void cmdCallback(const sensor_msgs::JointState::ConstPtr& msg);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  
  bool initCommand(flir_ptu_driver::initCommand::Request& req, flir_ptu_driver::initCommand::Response& res);

  bool actionCommand(flir_ptu_driver::sendCommand::Request& req, flir_ptu_driver::sendCommand::Response& res);
  
  void waitUntilPosition(const double pan, const double tilt);
  
protected:
  diagnostic_updater::Updater* m_updater;
  PTU* m_pantilt;
  ros::NodeHandle m_node;
  ros::Publisher  m_joint_pub;
  ros::Subscriber m_joint_sub;
  ros::ServiceServer m_init_srv;
  ros::ServiceServer m_action_srv;

  serial::Serial m_ser;
  std::string m_joint_name_prefix;
  
  int current_pan_pos;
  int current_tilt_pos;
  double pan_resolution;
  double tilt_resolution;
  std::mutex mutex;
};

Node::Node()
{

}

Node::Node(ros::NodeHandle& node_handle)
  : m_pantilt(NULL), m_node(node_handle)
{
  m_updater = new diagnostic_updater::Updater();
  m_updater->setHardwareID("none");
  m_updater->add("PTU Status", this, &Node::produce_diagnostics);

  ros::param::param<std::string>("~joint_name_prefix", m_joint_name_prefix, "ptu_");
}

Node::~Node()
{
  disconnect();
  delete m_updater;
}

/** Opens the connection to the PTU and sets appropriate parameters.
    Also manages subscriptions/publishers */
void Node::connect()
{
  // If we are reconnecting, first make sure to disconnect
  if (ok())
  {
    disconnect();
  }

  // Query for serial configuration
  std::string port;
  int32_t baud;
  ros::param::param<std::string>("~port", port, PTU_DEFAULT_PORT);
  ros::param::param<int32_t>("~baud", baud, PTU_DEFAULT_BAUD);

  // Connect to the PTU
  ROS_INFO_STREAM("Attempting to connect to FLIR PTU on " << port);

  try
  {
    m_ser.setPort(port);
    m_ser.setBaudrate(baud);
    serial::Timeout to = serial::Timeout(200, 200, 0, 200, 0);
    m_ser.setTimeout(to);
    m_ser.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to open port " << port);
    return;
  }

  ROS_INFO_STREAM("FLIR PTU serial port opened, now initializing.");

  m_pantilt = new PTU(&m_ser);

  if (!m_pantilt->initialize())
  {
    ROS_ERROR_STREAM("Could not initialize FLIR PTU on " << port);
    disconnect();
    return;
  }

  ROS_INFO("FLIR PTU initialized.");

  m_node.setParam("min_tilt", m_pantilt->getMin(PTU_TILT));
  m_node.setParam("max_tilt", m_pantilt->getMax(PTU_TILT));
  m_node.setParam("min_tilt_speed", m_pantilt->getMinSpeed(PTU_TILT));
  m_node.setParam("max_tilt_speed", m_pantilt->getMaxSpeed(PTU_TILT));
  m_node.setParam("tilt_step", m_pantilt->getResolution(PTU_TILT));

  m_node.setParam("min_pan", m_pantilt->getMin(PTU_PAN));
  m_node.setParam("max_pan", m_pantilt->getMax(PTU_PAN));
  m_node.setParam("min_pan_speed", m_pantilt->getMinSpeed(PTU_PAN));
  m_node.setParam("max_pan_speed", m_pantilt->getMaxSpeed(PTU_PAN));
  m_node.setParam("pan_step", m_pantilt->getResolution(PTU_PAN));

  mutex.lock();
  current_pan_pos = m_pantilt->getIntPosition(PTU_PAN);
  current_tilt_pos = m_pantilt->getIntPosition(PTU_TILT);
  pan_resolution = m_pantilt->getResolution(PTU_PAN);
  tilt_resolution = m_pantilt->getResolution(PTU_TILT);
  mutex.unlock();
  
  // Publishers : Only publish the most recent reading
  m_joint_pub = m_node.advertise
                <sensor_msgs::JointState>("state", 1);

  // Subscribers : Only subscribe to the most recent instructions
  m_joint_sub = m_node.subscribe
                <sensor_msgs::JointState>("cmd", 1, &Node::cmdCallback, this);
		
  m_init_srv = m_node.advertiseService("init", &Node::initCommand, this);

  m_action_srv = m_node.advertiseService("cmd", &Node::actionCommand, this);
}

/** Disconnect */
void Node::disconnect()
{
  if (m_pantilt != NULL)
  {
    delete m_pantilt;   // Closes the connection
    m_pantilt = NULL;   // Marks the service as disconnected
  }
}

/** Callback for getting new Goal JointState */
void Node::cmdCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_DEBUG("PTU command callback.");
  if (!ok()) return;

  if (msg->position.size() != 2 || msg->velocity.size() != 2)
  {
    ROS_ERROR("JointState command to PTU has wrong number of elements.");
    return;
  }

  double pan = msg->position[0];
  double tilt = msg->position[1];
  double panspeed = msg->velocity[0];
  double tiltspeed = msg->velocity[1];
  m_pantilt->setPosition(PTU_PAN, pan);
  m_pantilt->setPosition(PTU_TILT, tilt);
  m_pantilt->setSpeed(PTU_PAN, panspeed);
  m_pantilt->setSpeed(PTU_TILT, tiltspeed);
}

void Node::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All normal.");
  stat.add("PTU Mode", m_pantilt->getSpeedMode() == PTU_POSITION ? "Position" : "Velocity");
}

/**
 * Publishes a joint_state message with position and speed.
 * Also sends out updated TF info.
 */
void Node::spinCallback(const ros::TimerEvent&)
{
  if (!ok()) return;

  mutex.lock();
  
  // Read Position 
  current_pan_pos  = m_pantilt->getIntPosition(PTU_PAN);
  current_tilt_pos = m_pantilt->getIntPosition(PTU_TILT);
  
  mutex.unlock();
  
  double pan_rad = (double)current_pan_pos * pan_resolution;
  double tilt_rad = (double)current_tilt_pos * tilt_resolution;

  // Publish Position
  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.name[0] = m_joint_name_prefix + "pan";
  joint_state.position[0] = pan_rad;
  joint_state.name[1] = m_joint_name_prefix + "tilt";
  joint_state.position[1] = tilt_rad;
  
  m_joint_pub.publish(joint_state);
}

bool Node::initCommand(initCommand::Request& req, initCommand::Response& res)
{ 
  if (!ok())  return false;
  
  if (!(m_pantilt->setSpeed(PTU_PAN, req.speed)))
  {
    ROS_ERROR("Error setting pan speed in initCommand\n");
    return false;
  }
    
  if (!(m_pantilt->setPosition(PTU_PAN, req.pan_pos)) || !(m_pantilt->setPosition(PTU_TILT, req.tilt_pos)))
  {
    ROS_ERROR("Error setting pan-tilt pos in initCommand\n");
    return false;
  }
  
  waitUntilPosition(req.pan_pos, req.tilt_pos);
  
  return true;
}

bool Node::actionCommand(flir_ptu_driver::sendCommand::Request& req, flir_ptu_driver::sendCommand::Response& res)
{ 
  ROS_DEBUG("PTU command callback.");
  if (!ok()) {
    return false;
  }

  if (!(m_pantilt->setSpeed(PTU_PAN, req.speed)))
  {
    ROS_ERROR("Error setting pan speed in initCommand\n");
    return false;
  }

  double pan_pos = req.end_pos;
  
  if (!(m_pantilt->setPosition(PTU_PAN, pan_pos)))
  {
    ROS_ERROR("Error setting pan position\n");
    return false;
  }
  
  int expect_pan_pos = static_cast<int>(pan_pos / pan_resolution);
  int expect_scan_start_pos = static_cast<int>(req.start_scan_pos / pan_resolution);
  int expect_scan_end_pos = static_cast<int>(req.end_scan_pos / pan_resolution);
    
  // wayの値が0ならPTUのpositionの値が小さい方へ移動すること表す
  // wayの値が非0ならPTUのpositionの値が大きい方へ移動することを表す
  int way = std::signbit(pan_pos);
   
  while (ok())
  {
    mutex.lock();
    
    // if reache the scan range, record a start time
    if (current_pan_pos >= expect_scan_start_pos && way != 0)
    {
      res.start_stamp = ros::Time::now();
      res.start_scan_pos = current_pan_pos * pan_resolution;
      
    }
    if (current_pan_pos <= expect_scan_start_pos && way == 0)
    {
      res.start_stamp = ros::Time::now();
      res.start_scan_pos = current_pan_pos * pan_resolution;
    }
    
    // if exit the scan range, record a end time
    if (current_pan_pos > expect_scan_end_pos && way != 0)
    {
      res.end_stamp = ros::Time::now();
    }
    if (current_pan_pos < expect_scan_end_pos && way == 0)
    {
      res.end_stamp = ros::Time::now();
    }
    
    // check ptu reached the target position
    if (current_pan_pos == expect_pan_pos)
    {
      mutex.unlock();
      break;
    }
    
    mutex.unlock();
  }
  
  return true;  
}


void Node::waitUntilPosition(const double pan, const double tilt)
{
  int expect_pan  = static_cast<int>(pan / pan_resolution);
  int expect_tilt = static_cast<int>(tilt / tilt_resolution);

  while (ok())
  { 
    mutex.lock();
    
    if (current_pan_pos == expect_pan)
    {
      if (current_tilt_pos == expect_tilt)
      {
	mutex.unlock();
	break;
      }
    }
    
    mutex.unlock();
    usleep(1000);
  }
}
  
}  // namespace flir_ptu_driver

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ptu");
  ros::NodeHandle n;

  while (ros::ok())
  {
    // Connect to PTU
    flir_ptu_driver::Node ptu_node(n);
    ptu_node.connect();
    
    // Set up polling callback
    double hz;
    ros::param::param<double>("~hz", hz, PTU_DEFAULT_HZ);
    ros::Timer spin_timer = n.createTimer(ros::Duration(1 / hz), &flir_ptu_driver::Node::spinCallback, &ptu_node);
    
    // Spin until there's a problem or we're in shutdown
    ros::MultiThreadedSpinner spinner(2); // Use 2 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
    
    if (!ptu_node.ok())
    {
      ROS_ERROR("FLIR PTU disconnected, attempting reconnection.");
      ros::Duration(1.0).sleep();
    }
    
  }
  
  return 0;
}
