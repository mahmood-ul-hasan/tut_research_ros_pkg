/*
 * sample_nodelet_class.cpp
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#include<iostream>
#include "sample_nodelet_class.h"
#include <pluginlib/class_list_macros.h>
#include "std_msgs/String.h"

namespace sample_nodelet_ns
{
SampleNodeletClass::SampleNodeletClass()
{
  ROS_INFO("SampleNodeletClass Constructor");
}

SampleNodeletClass::~SampleNodeletClass()
{
  ROS_INFO("SampleNodeletClass Destructor");
}

void SampleNodeletClass::onInit()
{
    NODELET_INFO("SampleNodeletClass - %s", __FUNCTION__);      
    nh = getNodeHandle();
    num_pub = nh.advertise<std_msgs::String>("/num", 50,true);
    std_msgs::String str;
    str.data = "hello world from onInit";
    num_pub.publish(str);
    d_sub = nh.subscribe("/num",50,&SampleNodeletClass::sub_callback,this);
}

void SampleNodeletClass::sub_callback(const std_msgs::String::ConstPtr& msg1)
{
    std_msgs::StringPtr str(new std_msgs::String);
    str->data = "hello world from sub_callback";
    num_pub.publish(str);
}
} // namespace sample_nodelet_ns

PLUGINLIB_EXPORT_CLASS(sample_nodelet_ns::SampleNodeletClass, nodelet::Nodelet)
