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
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

class TfHandler
{
public:
  TfHandler(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr) : node_ptr_(node_ptr)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_ptr->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  geometry_msgs::msg::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                       const rclcpp::Time& time)
  {
    auto time_point = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
        std::chrono::nanoseconds(time.nanoseconds()));
    if (time_point > tf2::TimePointZero)
    {
      RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
                           "Time is not greater than zero. Using default time.");
      return lookupTransform(target_frame, source_frame);
    }
    else
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
  void clear()
  {
    tf_buffer_->clear();
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

class TfRtBroadcaster
{
public:
  TfRtBroadcaster(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr)
  {
    pub_ = node_ptr->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS());
    realtime_pub_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(pub_);
  }
  virtual void sendTransform(const geometry_msgs::msg::TransformStamped& transform)
  {
    std::vector<geometry_msgs::msg::TransformStamped> v1;
    v1.push_back(transform);
    sendTransform(v1);
  }
  virtual void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
  {
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

class StaticTfRtBroadcaster : public TfRtBroadcaster
{
public:
  StaticTfRtBroadcaster(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr) : TfRtBroadcaster(node_ptr)
  {
    pub_ = node_ptr->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::SystemDefaultsQoS());
    realtime_pub_ = std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(pub_);
  }
  void sendTransform(const geometry_msgs::msg::TransformStamped& transform) override
  {
    std::vector<geometry_msgs::msg::TransformStamped> v1;
    v1.push_back(transform);
    sendTransform(v1);
  }
  void sendTransform(const std::vector<geometry_msgs::msg::TransformStamped>& transforms) override
  {
    for (const auto& transform : transforms)
    {
      bool match_found = false;
      for (auto& it_msg : net_message_.transforms)
      {
        if (transform.child_frame_id == it_msg.child_frame_id)
        {
          it_msg = transform;
          match_found = true;
          break;
        }
      }
      if (!match_found)
        net_message_.transforms.push_back(transform);
    }
    if (realtime_pub_->trylock())
    {
      realtime_pub_->msg_ = net_message_;
      realtime_pub_->unlockAndPublish();
    }
  }

private:
  tf2_msgs::msg::TFMessage net_message_{};
};
