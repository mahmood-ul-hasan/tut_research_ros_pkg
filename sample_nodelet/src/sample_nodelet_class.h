/*
 * sample_nodelet_class.h
 *
 *  Created on: 2016/09/18
 *      Author: cryborg21
 */
#ifndef SAMPLE_NODELET_CLASS_SRC_SAMPLE_NODELET_CLASS_H_
#define SAMPLE_NODELET_CLASS_SRC_SAMPLE_NODELET_CLASS_H_
#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace sample_nodelet_ns
{
class SampleNodeletClass : public nodelet::Nodelet
{
public:
    SampleNodeletClass();
    ~SampleNodeletClass();

    virtual void onInit();
   

private:
void sub_callback(const std_msgs::String::ConstPtr& msg1);
ros::NodeHandle nh;
ros::Publisher num_pub;
ros::Subscriber d_sub;
};
} // namespace sample_nodelet_ns

#endif /* SAMPLE_NODELET_CLASS_SRC_SAMPLE_NODELET_CLASS_H_ */
