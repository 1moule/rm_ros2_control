//
// Created by guanlin on 25-2-19.
//

#pragma once

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "rm_ros2_hw/hardware_interface/can_bus.hpp"
#include "rm_ros2_msgs/msg/actuator_state.hpp"

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

protected:
  static void parse_act_coeff(std::unordered_map<std::string, ActCoeff>& type2act_coeffs);
  void publishActuatorState(const rclcpp::Time& time);

  struct InterfaceData
  {
    explicit InterfaceData(const std::string& name) : name_(name)
    {
    }
    std::string name_;
    std::array<double, 3> state_{ 0, 0, 0 };
    std::array<double, 3> command_{ 0, 0, 0 };
    std::array<double, 3> transmissionPassthrough_{ 0, 0, 0 };
  };

  std::vector<InterfaceData> joint_interfaces_;
  std::vector<InterfaceData> actuator_interfaces_;
  std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;
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
