//
// Created by guanlin on 25-2-14.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

namespace rm_ros2_common
{
class ControllerManager
{
public:
  explicit ControllerManager(rclcpp::Node::SharedPtr node) : node_(std::move(node))
  {
    // Create an executor
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Initialize the ControllerManager
    controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
        executor, "controller_manager", node_->get_namespace(), rclcpp::NodeOptions());

    node_->declare_parameter<std::vector<std::string>>("controllers", std::vector<std::string>());
    node_->get_parameter("controllers", controllers_);

    // Load and configure controllers
    loadAndConfigureControllers(controllers_);
    activateControllers(controllers_);
    deactivateControllers(controllers_);
  }

private:
  void loadAndConfigureControllers(const std::vector<std::string>& controllers) const
  {
    // Load controllers
    for (const auto& controller : controllers)
    {
      auto load_client =
          node_->create_client<controller_manager_msgs::srv::LoadController>("controller_manager/load_controller");
      auto load_request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
      load_request->name = controller;
      if (load_client->wait_for_service(std::chrono::seconds(10)))
      {
        auto load_result = load_client->async_send_request(load_request);
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), load_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node_->get_logger(), "Loaded controller: %s", load_request->name.c_str());
        }
      }
    }
    for (const auto& controller : controllers)
    {
      // Configure the controller
      auto configure_client = node_->create_client<controller_manager_msgs::srv::ConfigureController>(
          "controller_manager/configure_controller");
      auto configure_request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
      configure_request->name = controller;
      if (configure_client->wait_for_service(std::chrono::seconds(10)))
      {
        auto configure_result = configure_client->async_send_request(configure_request);
        if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), configure_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_INFO(node_->get_logger(), "Configured controller: %s", configure_request->name.c_str());
        }
      }
    }
  }
  void activateControllers(const std::vector<std::string>& controllers) const
  {
    // Switch the controller to active mode
    auto switch_client =
        node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->activate_controllers = controllers;
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    if (switch_client->wait_for_service(std::chrono::seconds(10)))
    {
      auto switch_result = switch_client->async_send_request(switch_request);
      if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), switch_result) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(node_->get_logger(), "Activated controllers:");
        for (const auto& controller : controllers)
          RCLCPP_INFO(node_->get_logger(), controller.c_str());
      }
    }
  }
  void deactivateControllers(const std::vector<std::string>& controllers) const
  {
    // Switch the controller to active mode
    auto switch_client =
        node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller");
    auto switch_request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_request->deactivate_controllers = controllers;
    switch_request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    if (switch_client->wait_for_service(std::chrono::seconds(10)))
    {
      auto switch_result = switch_client->async_send_request(switch_request);
      if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(), switch_result) ==
          rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(node_->get_logger(), "Deactivated controllers:");
        for (const auto& controller : controllers)
          RCLCPP_INFO(node_->get_logger(), controller.c_str());
      }
    }
  }

  std::shared_ptr<controller_manager::ControllerManager> controller_manager_{};
  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> controllers_;
};
}  // namespace rm_ros2_common