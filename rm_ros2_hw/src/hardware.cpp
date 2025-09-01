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
                                                               rclcpp::Clock().now(),
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

  node_ = std::make_shared<rclcpp::Node>("actuator_state_pub");
  actuator_state_pub_ =
      node_->create_publisher<rm_ros2_msgs::msg::ActuatorState>("actuator_state", rclcpp::SystemDefaultsQoS());
  actuator_state_pub_rt_ =
      std::make_shared<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::ActuatorState>>(actuator_state_pub_);

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

  // Transfer actuator state data to joint state variables
  int pos_index = 0;
  for (const auto& joint_name : joint_interfaces["position"])
  {
    for (const auto& bus_data : bus_id2act_data_)
    {
      for (const auto& act_data : bus_data.second)
      {
        if (act_data.second.name == joint_name)
        {
          joint_position_[pos_index] = act_data.second.pos;
          break;
        }
      }
    }
    pos_index++;
  }

  int vel_index = 0;
  for (const auto& joint_name : joint_interfaces["velocity"])
  {
    for (const auto& bus_data : bus_id2act_data_)
    {
      for (const auto& act_data : bus_data.second)
      {
        if (act_data.second.name == joint_name)
        {
          joint_velocities_[vel_index] = act_data.second.vel;
          break;
        }
      }
    }
    vel_index++;
  }

  int effort_index = 0;
  for (const auto& joint_name : joint_interfaces["effort"])
  {
    for (const auto& bus_data : bus_id2act_data_)
    {
      for (const auto& act_data : bus_data.second)
      {
        if (act_data.second.name == joint_name)
        {
          joint_efforts_[effort_index] = act_data.second.effort;
          break;
        }
      }
    }
    effort_index++;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmSystemHardware::write(const rclcpp::Time& time, const rclcpp::Duration&)
{
  // Transfer joint effort commands to actuator data
  int effort_cmd_index = 0;
  for (const auto& joint_name : joint_interfaces["effort"])
  {
    // Find the corresponding actuator data for this joint
    for (auto& bus_data : bus_id2act_data_)
    {
      for (auto& act_data : bus_data.second)
      {
        if (act_data.second.name == joint_name)
        {
          // Set the command effort and executed effort
          act_data.second.cmd_effort = joint_effort_command_[effort_cmd_index];
          act_data.second.exe_effort = joint_effort_command_[effort_cmd_index];
          // For Cheetah motors, also set position and velocity commands to current values for now
          // as they use position and velocity in the write logic
          if (act_data.second.type.find("cheetah") != std::string::npos)
          {
            act_data.second.cmd_pos = act_data.second.pos;
            act_data.second.cmd_vel = act_data.second.vel;
          }
          break;
        }
      }
    }
    effort_cmd_index++;
  }

  // Send commands to all CAN buses
  for (auto& bus : can_buses_)
    bus->write();

  publishActuatorState(time);
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

void RmSystemHardware::publishActuatorState(const rclcpp::Time& /*time*/)
{
  if (actuator_state_pub_rt_->trylock())
  {
    rm_ros2_msgs::msg::ActuatorState actuator_state;
    for (const auto& id2act_datas : bus_id2act_data_)
      for (const auto& act_data : id2act_datas.second)
      {
        actuator_state.stamp.push_back(act_data.second.stamp);
        actuator_state.name.push_back(act_data.second.name);
        actuator_state.type.push_back(act_data.second.type);
        actuator_state.bus.push_back(id2act_datas.first);
        actuator_state.id.push_back(act_data.first);
        actuator_state.halted.push_back(act_data.second.halted);
        actuator_state.need_calibration.push_back(act_data.second.need_calibration);
        actuator_state.calibrated.push_back(act_data.second.calibrated);
        actuator_state.calibration_reading.push_back(act_data.second.calibration_reading);
        actuator_state.position_raw.push_back(act_data.second.q_raw);
        actuator_state.velocity_raw.push_back(act_data.second.qd_raw);
        actuator_state.temperature.push_back(act_data.second.temp);
        actuator_state.circle.push_back(act_data.second.q_circle);
        actuator_state.last_position_raw.push_back(act_data.second.q_last);
        actuator_state.frequency.push_back(act_data.second.frequency);
        actuator_state.position.push_back(act_data.second.pos);
        actuator_state.velocity.push_back(act_data.second.vel);
        actuator_state.effort.push_back(act_data.second.effort);
        actuator_state.commanded_effort.push_back(act_data.second.cmd_effort);
        actuator_state.executed_effort.push_back(act_data.second.exe_effort);
        actuator_state.offset.push_back(act_data.second.offset);
      }
    actuator_state_pub_rt_->msg_ = actuator_state;
    actuator_state_pub_rt_->unlockAndPublish();
  }
}
}  // namespace rm_ros2_hw

PLUGINLIB_EXPORT_CLASS(rm_ros2_hw::RmSystemHardware, hardware_interface::SystemInterface)
