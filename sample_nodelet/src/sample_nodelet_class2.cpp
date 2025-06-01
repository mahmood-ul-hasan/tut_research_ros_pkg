/*
 * sample_nodelet_class2.cpp
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#include "sample_nodelet_class2.h"
#include <iostream>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "std_msgs/String.h"



namespace sample_nodelet_ns
{
SampleNodeletClass2::SampleNodeletClass2()
{
  ROS_INFO("SampleNodeletClass2 Constructor");
}

SampleNodeletClass2::~SampleNodeletClass2()
{
  ROS_INFO("SampleNodeletClass2 Destructor");
}


void SampleNodeletClass2::onInit()
{
    n = getNodeHandle();   

    NODELET_INFO("SampleNodeletClass2 - %s", __FUNCTION__);
    num_sub = n.subscribe("/num", 50, &SampleNodeletClass2::num_Callback,this);
    
}

void SampleNodeletClass2::num_Callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    ros::Duration(0.5).sleep();
} 

} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(sample_nodelet_ns::SampleNodeletClass2, nodelet::Nodelet)
