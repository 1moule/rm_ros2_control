//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>

template <typename ParameterT>
auto getParam(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node, const std::string& name,
              const ParameterT& default_value)
{
  if (!node->has_parameter(name))
  {
    return node->declare_parameter<ParameterT>(name, default_value);
  }
  else
  {
    return node->get_parameter(name).get_value<ParameterT>();
  }
}
