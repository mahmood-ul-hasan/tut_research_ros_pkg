// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak .. 2009, Willow Garage, Inc.

/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
 * Copyright (c) 2009, Willow Garage, Inc.
 *
 *        All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *        modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *       AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *       IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *       DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *       FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *       DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *       SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *       OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *       OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <string>

#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/raw_subscriber.h>

namespace point_cloud_transport
{

std::string RawSubscriber::getTransportName() const
{
  return "raw";
}

std::string RawSubscriber::getTopicToSubscribe(const std::string& base_topic) const
{
  return base_topic;
}

void RawSubscriber::callback(const sensor_msgs::PointCloud2ConstPtr& message, const SubscriberPlugin::Callback& user_cb)
{
  user_cb(message);
}

SubscriberPlugin::DecodeResult RawSubscriber::decodeTyped(const sensor_msgs::PointCloud2ConstPtr& compressed,
                                                          const NoConfigConfig&) const
{
  return compressed;
}

SubscriberPlugin::DecodeResult RawSubscriber::decodeTyped(const sensor_msgs::PointCloud2& compressed,
                                                          const NoConfigConfig& config) const
{
  sensor_msgs::PointCloud2Ptr compressedPtr(new sensor_msgs::PointCloud2);
  *compressedPtr = compressed;
  return this->decodeTyped(compressedPtr, config);
}

bool RawSubscriber::matchesTopic(const std::string& topic, const std::string& datatype) const
{
  return datatype == ros::message_traits::DataType<sensor_msgs::PointCloud2>::value();
}

}
