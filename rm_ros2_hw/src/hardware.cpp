//
// Created by guanlin on 25-2-19.
//

#include "rm_ros2_hw/hardware.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <iostream>

namespace rm_ros2_hw
{
CallbackReturn RmSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  for (const auto& joint : info_.joints)
  {
    for (const auto& interface : joint.state_interfaces)
      joint_interfaces[interface.name].push_back(joint.name);
    std::cout << joint.name << std::endl;
  }

  joint_position_.resize(info_.joints.size(), 0.);
  joint_velocities_.resize(info_.joints.size(), 0.);
  joint_efforts_.resize(info_.joints.size(), 0.);
  joint_effort_command_.resize(info_.joints.size(), 0.);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto& joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto& joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  ind = 0;
  for (const auto& joint_name : joint_interfaces["effort"])
  {
    state_interfaces.emplace_back(joint_name, "effort", &joint_efforts_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto& joint_name : joint_interfaces["effort"])
  {
    command_interfaces.emplace_back(joint_name, "effort", &joint_effort_command_[ind++]);
  }

  return command_interfaces;
}

hardware_interface::return_type RmSystemHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  std::cout << "Hardware::read()" << std::endl;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmSystemHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::cout << "Hardware::write()" << std::endl;
  return hardware_interface::return_type::OK;
}
}  // namespace rm_ros2_hw

PLUGINLIB_EXPORT_CLASS(rm_ros2_hw::RmSystemHardware, hardware_interface::SystemInterface)
