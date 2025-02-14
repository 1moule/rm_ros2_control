//
// Created by guanlin on 25-2-14.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rm_ros2_msgs/msg/chassis_cmd.hpp>
#include <rm_ros2_common/filters/linear_interpolation.hpp>
#include <rm_ros2_common/tools/ros_tools.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rm_ros2_common
{
template <class MsgType>
class CommandSenderBase
{
public:
  virtual ~CommandSenderBase() = default;
  explicit CommandSenderBase(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    node_->declare_parameter<std::string>("topic", "error");
    if (!node_->get_parameter("topic", topic_))
      RCLCPP_ERROR(node_->get_logger(), "Topic name no defined (namespace: %s)", node_->get_name());
    node_->declare_parameter<double>("qos", 10.0);
    node_->get_parameter("qos", qos_);
    pub_ = node_->create_publisher<MsgType>(topic_, qos_);
  }
  void setMode(int mode)
  {
    if (!std::is_same_v<MsgType, geometry_msgs::msg::Twist> && !std::is_same_v<MsgType, std_msgs::msg::Float64>)
      msg_.mode = mode;
  }
  virtual void sendCommand(const rclcpp::Time& time)
  {
    pub_->publish(msg_);
  }
  virtual void setZero() = 0;
  MsgType* getMsg()
  {
    return &msg_;
  }

protected:
  MsgType msg_;
  double qos_;
  std::string topic_;
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
};

class Vel2DCommandSender final : public CommandSenderBase<geometry_msgs::msg::Twist>
{
public:
  explicit Vel2DCommandSender(const rclcpp::Node::SharedPtr& node)
    : CommandSenderBase(node), max_linear_x_(node), max_linear_y_(node), max_angular_z_(node)
  {
    std::vector<double> x, y;
    x = getParam<std::vector<double>>(node_, "max_linear_x.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, "max_linear_x.y", { 0., 0., 0. });
    max_linear_x_.init(x, y);
    x = getParam<std::vector<double>>(node_, "max_linear_y.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, "max_linear_y.y", { 0., 0., 0. });
    max_linear_y_.init(x, y);
    x = getParam<std::vector<double>>(node_, "max_angular_z.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, "max_angular_z.y", { 0., 0., 0. });
    max_angular_z_.init(x, y);

    const std::string topic = getParam(node_, "power_limit_topic", std::string("/cmd_chassis"));
    chassis_power_limit_subscriber_ = node_->create_subscription<rm_ros2_msgs::msg::ChassisCmd>(
        topic, rclcpp::SystemDefaultsQoS(),
        std::bind(&Vel2DCommandSender::chassisCmdCallback, this, std::placeholders::_1));
  }
  void setLinearXVel(double scale)
  {
    msg_.linear.x = scale * max_linear_x_.output(power_limit_);
  };
  void setLinearYVel(double scale)
  {
    msg_.linear.y = scale * max_linear_y_.output(power_limit_);
  };
  void setAngularZVel(double scale)
  {
    msg_.angular.z = scale * max_angular_z_.output(power_limit_);
  };
  void setZero() override
  {
    msg_.linear.x = 0.;
    msg_.linear.y = 0.;
    msg_.angular.z = 0.;
  }

protected:
  void chassisCmdCallback(const rm_ros2_msgs::msg::ChassisCmd::SharedPtr msg)
  {
    power_limit_ = msg->power_limit;
  }
  double power_limit_ = 0.;
  LinearInterp max_linear_x_, max_linear_y_, max_angular_z_;
  rclcpp::Subscription<rm_ros2_msgs::msg::ChassisCmd>::SharedPtr chassis_power_limit_subscriber_;
};
}  // namespace rm_ros2_common
