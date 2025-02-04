/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 1/3/21.
//

#pragma once

#include <utility>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

class TfHandler: public rclcpp::Node
{
public:
  TfHandler(std::string name): Node(name+"_robot_state"){
    tf_buffer_ =
  std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  geometry_msgs::msg::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const rclcpp::Time& time)
  {
    return tf_buffer_->lookupTransform(target_frame, source_frame, time);
  }
  geometry_msgs::msg::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame)
  {
    return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  }
  bool setTransform(const geometry_msgs::msg::TransformStamped& transform, const std::string& authority,
                    bool is_static = false) const
  {
    return tf_buffer_->setTransform(transform, authority, is_static);
  }
  bool setTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms, const std::string& authority,
                    bool is_static = false) const
  {
    for (const auto& transform : transforms)
      tf_buffer_->setTransform(transform, authority, is_static);
    return true;
  }

private:
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

class TfRtBroadcaster:public rclcpp::Node
{
public:
  TfRtBroadcaster(std::string name):Node(name+"tf_broadcaster"){
    pub_=create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
    realtime_pub_=std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(pub_);
  }
  virtual void sendTransform(const geometry_msgs::msg::TransformStamped& transform){
    std::vector<geometry_msgs::msg::TransformStamped> v1;
    v1.push_back(transform);
    sendTransform(v1);
  }
  virtual void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms){
    tf2_msgs::msg::TFMessage message;
    for (const auto& transform : transforms)
    {
      message.transforms.push_back(transform);
    }
    if (realtime_pub_->trylock())
    {
      realtime_pub_->msg_ = message;
      realtime_pub_->unlockAndPublish();
    }
  }

protected:
  std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> realtime_pub_{};
  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> pub_;
};
