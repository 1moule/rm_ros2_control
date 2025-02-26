//
// Created by guanlin on 25-2-19.
//

#pragma once

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rm_ros2_hw/hardware_interface/can_bus.hpp"
#include "rm_ros2_msgs/msg/actuator_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace rm_ros2_hw
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RmSystemHardware final : public hardware_interface::SystemInterface
{
public:
  RmSystemHardware() = default;
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;
  rclcpp::Logger get_logger() const
  {
    return *logger_;
  }
  rclcpp::Clock::SharedPtr get_clock() const
  {
    return clock_;
  }

protected:
  static void parse_act_coeff(std::unordered_map<std::string, ActCoeff>& type2act_coeffs);
  void publishActuatorState(const rclcpp::Time& time);
  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_effort_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = { { "position", {} },
                                                                                 { "velocity", {} },
                                                                                 { "effort", {} } };
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::vector<std::unique_ptr<CanBus>> can_buses_{};

  // Actuator
  std::unordered_map<std::string, ActCoeff> type2act_coeffs_{};
  std::unordered_map<std::string, std::unordered_map<int, ActData>> bus_id2act_data_{};

  // Imu
  std::unordered_map<std::string, std::unordered_map<int, ImuData>> bus_id2imu_data_{};
  bool is_actuator_specified_ = false;

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<rm_ros2_msgs::msg::ActuatorState>> actuator_state_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::ActuatorState>> actuator_state_pub_rt_;
};
};  // namespace rm_ros2_hw
