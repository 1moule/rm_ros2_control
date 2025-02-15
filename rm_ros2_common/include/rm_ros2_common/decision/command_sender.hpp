//
// Created by guanlin on 25-2-14.
//

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rm_ros2_msgs/msg/chassis_cmd.hpp>
#include <rm_ros2_common/filters/linear_interpolation.hpp>
#include <rm_ros2_common/tools/ros_tools.hpp>
#include <utility>

namespace rm_ros2_common
{
template <class MsgType>
class CommandSenderBase
{
public:
  virtual ~CommandSenderBase() = default;
  CommandSenderBase(rclcpp::Node::SharedPtr node, std::string param_prefix)
    : param_prefix_(std::move(param_prefix)), node_(std::move(node))
  {
    node_->declare_parameter<std::string>(param_prefix_ + ".topic", "error");
    if (!node_->get_parameter(param_prefix_ + ".topic", topic_))
      RCLCPP_ERROR(node_->get_logger(), "Topic name no defined (namespace: %s)", node_->get_name());
    node_->declare_parameter<double>(param_prefix_ + ".qos", 10.0);
    node_->get_parameter(param_prefix_ + ".qos", qos_);
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
  double qos_{};
  MsgType msg_;
  std::string topic_, param_prefix_;
  rclcpp::Node::SharedPtr node_;
  typename rclcpp::Publisher<MsgType>::SharedPtr pub_;
};

template <class MsgType>
class TimeStampCommandSenderBase : public CommandSenderBase<MsgType>
{
public:
  TimeStampCommandSenderBase(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
    : CommandSenderBase<MsgType>(node, param_prefix)
  {
  }
  void sendCommand(const rclcpp::Time& time) override
  {
    CommandSenderBase<MsgType>::msg_.stamp = time;
    CommandSenderBase<MsgType>::sendCommand(time);
  }
};

class Vel2DCommandSender final : public CommandSenderBase<geometry_msgs::msg::Twist>
{
public:
  explicit Vel2DCommandSender(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
    : CommandSenderBase(node, param_prefix), max_linear_x_(node), max_linear_y_(node), max_angular_z_(node)
  {
    std::vector<double> x, y;
    x = getParam<std::vector<double>>(node_, param_prefix_ + ".max_linear_x.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, param_prefix_ + ".max_linear_x.y", { 0., 0., 0. });
    max_linear_x_.init(x, y);
    x = getParam<std::vector<double>>(node_, param_prefix_ + ".max_linear_y.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, ".max_linear_y.y", { 0., 0., 0. });
    max_linear_y_.init(x, y);
    x = getParam<std::vector<double>>(node_, param_prefix_ + ".max_angular_z.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, param_prefix_ + ".max_angular_z.y", { 0., 0., 0. });
    max_angular_z_.init(x, y);

    const std::string topic = getParam(node_, param_prefix_ + ".power_limit_topic", std::string("/cmd_chassis"));
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

class ChassisCommandSender final : public TimeStampCommandSenderBase<rm_ros2_msgs::msg::ChassisCmd>
{
public:
  ChassisCommandSender(const rclcpp::Node::SharedPtr& node, const std::string& param_prefix)
    : TimeStampCommandSenderBase(node, param_prefix), accel_x_(node), accel_y_(node), accel_z_(node)
  {
    std::vector<double> x, y;
    x = getParam<std::vector<double>>(node_, param_prefix_ + ".accel_x.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, param_prefix_ + ".accel_x.y", { 0., 0., 0. });
    accel_x_.init(x, y);
    x = getParam<std::vector<double>>(node_, param_prefix_ + ".accel_y.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, ".accel_y.y", { 0., 0., 0. });
    accel_y_.init(x, y);
    x = getParam<std::vector<double>>(node_, param_prefix_ + ".accel_z.x", { 0., 0., 0. });
    y = getParam<std::vector<double>>(node_, param_prefix_ + ".accel_z.y", { 0., 0., 0. });
    accel_z_.init(x, y);
  }

  // void updateSafetyPower(int safety_power)
  // {
  // power_limit_->updateSafetyPower(safety_power);
  // }
  // void updateGameRobotStatus(const rm_msgs::GameRobotStatus data) override
  // {
  // power_limit_->setGameRobotData(data);
  // }
  // void updatePowerHeatData(const rm_msgs::PowerHeatData data) override
  // {
  // power_limit_->setChassisPowerBuffer(data);
  // }
  // void updateCapacityData(const rm_msgs::PowerManagementSampleAndStatusData data) override
  // {
  // power_limit_->setCapacityData(data);
  // }
  // void updateRefereeStatus(bool status)
  // {
  // power_limit_->setRefereeStatus(status);
  // }
  void setFollowVelDes(double follow_vel_des)
  {
    msg_.follow_vel_des = follow_vel_des;
  }
  void sendChassisCommand(const rclcpp::Time& time, bool /*is_gyro*/)
  {
    // power_limit_->setLimitPower(msg_, is_gyro);
    msg_.accel.linear.x = accel_x_.output(msg_.power_limit);
    msg_.accel.linear.y = accel_y_.output(msg_.power_limit);
    msg_.accel.angular.z = accel_z_.output(msg_.power_limit);
    TimeStampCommandSenderBase::sendCommand(time);
  }
  void setZero() override {};
  // PowerLimit* power_limit_;

private:
  LinearInterp accel_x_, accel_y_, accel_z_;
};
}  // namespace rm_ros2_common
