//
// Created by guanlin on 25-2-14.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

namespace rm_ros2_common
{
class ControllerManager final : public rclcpp::Node
{
public:
  ControllerManager() : Node("controller_manager")
  {
    // Get controllers
    this->declare_parameter<std::vector<std::string>>("controllers.main_controllers", std::vector<std::string>());
    this->get_parameter("controllers.main_controllers", main_controllers_);

    // Load and configure controllers
    loadAndConfigureControllers(main_controllers_);
  }
  void activateMainControllers()
  {
    activateControllers(main_controllers_);
  }
  void deactivateMainControllers()
  {
    deactivateControllers(main_controllers_);
  }

private:
  void loadAndConfigureControllers(const std::vector<std::string>& controllers)
  {
    // Load controllers
    for (const auto& controller : controllers)
    {
      auto load_client =
          this->create_client<controller_manager_msgs::srv::LoadController>("controller_manager/load_controller");
      auto load_request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
      load_request->name = controller;
      if (load_client->wait_for_service(std::chrono::seconds(10)))
      {
        auto load_result = load_client->async_send_request(load_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), load_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "Loaded controller: %s", load_request->name.c_str());
        }
      }
    }
    for (const auto& controller : controllers)
    {
      // Configure the controller
      auto configure_client = this->create_client<controller_manager_msgs::srv::ConfigureController>(
          "controller_manager/configure_controller");
      auto configure_request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
      configure_request->name = controller;
      if (configure_client->wait_for_service(std::chrono::seconds(10)))
      {
        auto configure_result = configure_client->async_send_request(configure_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), configure_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "Configured controller: %s", configure_request->name.c_str());
        }
      }
    }
  }
  void activateControllers(const std::vector<std::string>& controllers)
  {
    // Switch the controller to active mode
    auto switch_client =
        this->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->activate_controllers = controllers;
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    if (switch_client->wait_for_service(std::chrono::seconds(10)))
    {
      auto switch_result = switch_client->async_send_request(switch_request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), switch_result) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Activated controllers:");
        for (const auto& controller : controllers)
          RCLCPP_INFO(this->get_logger(), controller.c_str());
      }
    }
  }
  void deactivateControllers(const std::vector<std::string>& controllers)
  {
    // Switch the controller to active mode
    auto switch_client =
        this->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->deactivate_controllers = controllers;
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    if (switch_client->wait_for_service(std::chrono::seconds(10)))
    {
      auto switch_result = switch_client->async_send_request(switch_request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), switch_result) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "Deactivated controllers:");
        for (const auto& controller : controllers)
          RCLCPP_INFO(this->get_logger(), controller.c_str());
      }
    }
  }

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_{};
  std::vector<std::string> main_controllers_;
};
}  // namespace rm_ros2_common