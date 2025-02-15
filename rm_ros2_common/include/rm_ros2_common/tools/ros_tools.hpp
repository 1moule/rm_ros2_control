//
// Created by guanlin on 25-2-13.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

template <typename ParameterT>
auto getParam(rclcpp::Node::SharedPtr& node, const std::string& name, const ParameterT& default_value)
{
  node->declare_parameter<ParameterT>(name, default_value);
  return node->get_parameter(name).get_value<ParameterT>();
}

template <typename ParameterT>
bool getParam(const rclcpp::Node::SharedPtr& node, const std::string& name, const ParameterT& default_value,
              ParameterT& value)
{
  rclcpp::Parameter param;
  if (node->get_parameter(name, param))
  {
    value = param.get_value<ParameterT>();
    return true;
  }
  else
  {
    value = default_value;
    return false;
  }
}