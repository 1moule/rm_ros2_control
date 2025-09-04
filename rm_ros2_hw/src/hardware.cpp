//
// Created by guanlin on 25-2-19.
//

#include "rm_ros2_hw/hardware.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <iostream>
#include <fmt/core.h>

namespace rm_ros2_hw
{
CallbackReturn RmSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  parse_act_coeff(type2act_coeffs_);
  // for (const auto& joint : info_.joints)
  // {
  //   if (joint.parameters.find("bus") != joint.parameters.end() &&
  //       joint.parameters.find("id") != joint.parameters.end() &&
  //       joint.parameters.find("type") != joint.parameters.end())
  //   {
  //     int id;
  //     std::stringstream(joint.parameters.at("id")) >> id;
  //     std::string bus = joint.parameters.at("bus");
  //     std::string type = joint.parameters.at("type");
  //     bus_id2act_data_[bus].insert(std::make_pair(id, ActData{ joint.name,
  //                                                              type,
  //                                                              rclcpp::Clock().now(),
  //                                                              false,
  //                                                              false,
  //                                                              false,
  //                                                              false,
  //                                                              false,
  //                                                              0,
  //                                                              0,
  //                                                              0,
  //                                                              0,
  //                                                              0,
  //                                                              0.,
  //                                                              0,
  //                                                              0.,
  //                                                              0.,
  //                                                              0.,
  //                                                              0.,
  //                                                              0.,
  //                                                              0.,
  //                                                              0.,
  //                                                              std::make_unique<LowPassFilter>(100.0) }));
  //   }
  //   else
  //     RCLCPP_ERROR(rclcpp::get_logger("RmSystemHardware"), "Joint %s need to designate: bus id type",
  //                  joint.name.c_str());
  // }
  // for (const auto& param : info_.hardware_parameters)
  // {
  //   if (param.first.find("bus") != std::string::npos)
  //   {
  //     can_buses_.push_back(std::make_unique<CanBus>(
  //         param.second,
  //         CanDataPtr{ &type2act_coeffs_, &bus_id2act_data_[param.second], &bus_id2imu_data_[param.second] }, 99));
  //   }
  // }

  node_ = std::make_shared<rclcpp::Node>("actuator_state_pub");
  actuator_state_pub_ =
      node_->create_publisher<rm_ros2_msgs::msg::ActuatorState>("actuator_state", rclcpp::SystemDefaultsQoS());
  actuator_state_pub_rt_ =
      std::make_shared<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::ActuatorState>>(actuator_state_pub_);

  // Transmission
  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();
  for (const auto& transmission_info : info_.transmissions)
  {
    std::shared_ptr<transmission_interface::Transmission> transmission;
    try
    {
      transmission = transmission_loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException& exc)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RmSystemHardware"), "Error while loading %s: %s", transmission_info.name.c_str(),
                   exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<transmission_interface::JointHandle> joint_handles;
    for (const auto& joint_info : transmission_info.joints)
    {
      const auto joint_interface = joint_interfaces_.insert(joint_interfaces_.end(), InterfaceData(joint_info.name));
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_POSITION,
                                 &joint_interface->transmissionPassthrough_[0]);
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_VELOCITY,
                                 &joint_interface->transmissionPassthrough_[1]);
      joint_handles.emplace_back(joint_info.name, hardware_interface::HW_IF_EFFORT,
                                 &joint_interface->transmissionPassthrough_[2]);
    }

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto& actuator_info : transmission_info.actuators)
    {
      const auto actuator_interface =
          actuator_interfaces_.insert(actuator_interfaces_.end(), InterfaceData(actuator_info.name));
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_POSITION,
                                    &actuator_interface->transmissionPassthrough_[0]);
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_VELOCITY,
                                    &actuator_interface->transmissionPassthrough_[1]);
      actuator_handles.emplace_back(actuator_info.name, hardware_interface::HW_IF_EFFORT,
                                    &actuator_interface->transmissionPassthrough_[2]);
    }

    try
    {
      transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException& exc)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RmSystemHardware"), "Error while configuring %s: %s",
                   transmission_info.name.c_str(), exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    transmissions_.push_back(transmission);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto& joint : info_.joints)
  {
    /// @pre all joint interfaces exist, checked in on_init()
    auto joint_interface = std::find_if(joint_interfaces_.begin(), joint_interfaces_.end(),
                                        [&](const InterfaceData& interface) { return interface.name_ == joint.name; });

    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint_interface->state_[0]);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint_interface->state_[1]);
    state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint_interface->state_[2]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto& joint : info_.joints)
  {
    auto joint_interface = std::find_if(joint_interfaces_.begin(), joint_interfaces_.end(),
                                        [&](const InterfaceData& interface) { return interface.name_ == joint.name; });

    command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint_interface->command_[2]);
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

  // actuator: state -> transmission
  std::for_each(actuator_interfaces_.begin(), actuator_interfaces_.end(), [](auto& actuator_interface) {
    actuator_interface.transmissionPassthrough_ = actuator_interface.state_;
  });

  // transmission: actuator -> joint
  std::for_each(transmissions_.begin(), transmissions_.end(),
                [](auto& transmission) { transmission->actuator_to_joint(); });

  // joint: transmission -> state
  std::for_each(joint_interfaces_.begin(), joint_interfaces_.end(),
                [](auto& joint_interface) { joint_interface.state_ = joint_interface.transmissionPassthrough_; });

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmSystemHardware::write(const rclcpp::Time& time, const rclcpp::Duration&)
{
  // std::cout << "Hardware::write()" << std::endl;
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
