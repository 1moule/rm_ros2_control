//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

template <typename ParameterT>
bool getParam(rclcpp::Node::SharedPtr& node, const std::string& name, ParameterT& value, const ParameterT& default_value)
{
  if (!node->has_parameter(name))
  {
    value = node->declare_parameter<ParameterT>(name, default_value);
    return false;
  }
  value = node->get_parameter(name).get_value<ParameterT>();
  return true;
}
