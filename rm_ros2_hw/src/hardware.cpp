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

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("RmSystemHardware"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  parse_act_coeff(type2act_coeffs_);
  for (const auto& joint : info_.joints)
  {
    if (joint.parameters.find("bus") != joint.parameters.end() &&
        joint.parameters.find("id") != joint.parameters.end() &&
        joint.parameters.find("type") != joint.parameters.end())
    {
      int id;
      std::stringstream(joint.parameters.at("id")) >> id;
      std::string bus = joint.parameters.at("bus");
      std::string type = joint.parameters.at("type");
      bus_id2act_data_[bus].insert(std::make_pair(id, ActData{ joint.name,
                                                               type,
                                                               get_clock()->now(),
                                                               false,
                                                               false,
                                                               false,
                                                               false,
                                                               false,
                                                               0,
                                                               0,
                                                               0,
                                                               0,
                                                               0,
                                                               0.,
                                                               0,
                                                               0.,
                                                               0.,
                                                               0.,
                                                               0.,
                                                               0.,
                                                               0.,
                                                               0.,
                                                               std::make_unique<LowPassFilter>(100.0) }));
    }
    else
      RCLCPP_ERROR(get_logger(), "Joint %s need to designate: bus id type", joint.name.c_str());
    for (const auto& interface : joint.state_interfaces)
      joint_interfaces[interface.name].push_back(joint.name);
  }
  for (const auto& param : info_.hardware_parameters)
  {
    if (param.first.find("bus") != std::string::npos)
    {
      can_buses_.push_back(std::make_unique<CanBus>(param.second,
                                                    CanDataPtr{ &type2act_coeffs_, &bus_id2act_data_[param.second],
                                                                &bus_id2imu_data_[param.second] },
                                                    99, logger_, clock_));
    }
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

hardware_interface::return_type RmSystemHardware::read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  for (auto& bus : can_buses_)
    bus->read(time);
  for (auto& id2act_datas : bus_id2act_data_)
    for (auto& act_data : id2act_datas.second)
    {
      try
      {  // Duration will be out of dual 32-bit range while motor failure
        act_data.second.halted = (time - act_data.second.stamp).seconds() > 0.1 || act_data.second.temp > 99;
      }
      catch (std::runtime_error& ex)
      {
      }
      if (act_data.second.halted)
      {
        act_data.second.seq = 0;
        act_data.second.vel = 0;
        act_data.second.effort = 0;
        act_data.second.calibrated = false;  // set the actuator no calibrated
      }
    }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmSystemHardware::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  // std::cout << "Hardware::write()" << std::endl;
  return hardware_interface::return_type::OK;
}

void RmSystemHardware::parse_act_coeff(std::unordered_map<std::string, ActCoeff>& type2act_coeffs)
{
  type2act_coeffs.insert(std::make_pair("rm_3508", ActCoeff{ 0.0007669903, 0.1047197551, 1.90702994e-5, 0., 0.,
                                                             52437.561519, 16384, 0., 0., 0., 0., 0. }));
  type2act_coeffs.insert(std::make_pair("rm_6020", ActCoeff{ 0.0007670840, 0.1047197551, 5.880969e-5, 0., 0., 25000,
                                                             30000, 0., 0., 0., 0., 0. }));
  type2act_coeffs.insert(std::make_pair("rm_2006", ActCoeff{ 2.13078897e-5, 0.0029088820, 0.00018, 0., 0., 5555.5555555,
                                                             10000, 0., 0., 0., 0., 0. }));
  type2act_coeffs.insert(std::make_pair("cheetah", ActCoeff{ 3.81475547e-4, 0.0317446031, 0.008791208, 2621.4, 31.5,
                                                             113.75, 0., -12.5, -65.0, -18.0, 8.19, 819 }));
}
}  // namespace rm_ros2_hw

PLUGINLIB_EXPORT_CLASS(rm_ros2_hw::RmSystemHardware, hardware_interface::SystemInterface)
