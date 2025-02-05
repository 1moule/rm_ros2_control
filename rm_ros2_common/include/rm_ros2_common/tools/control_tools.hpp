//
// Created by guanlin on 25-2-5.
//

#ifndef CONTROL_TOOLS_HPP
#define CONTROL_TOOLS_HPP

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <control_msgs/msg/pid_state.hpp>

namespace control_tools
{
class Pid
{
public:
  Pid( std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr, std::string prefix = std::string("")):
    node_ptr_(node_ptr),param_prefix_(prefix),topic_prefix_(prefix){
    if (!param_prefix_.empty() && param_prefix_.back() != '.') {
      param_prefix_.append(".");
    }
    std::replace(topic_prefix_.begin(), topic_prefix_.end(), '.', '/');
    if (!topic_prefix_.empty() && topic_prefix_.back() != '/') {
      topic_prefix_.append("/");
    }

    p_ = i_ = d_ = i_min_ = i_max_ = std::numeric_limits<double>::quiet_NaN();
    p_ =  node_ptr_->get_parameter(param_prefix_ + "p").get_value<double>();
    i_ = node_ptr_->get_parameter(param_prefix_ + "i").get_value<double>();
    d_ = node_ptr_->get_parameter(param_prefix_ + "d").get_value<double>();
    i_max_ = node_ptr_->get_parameter(param_prefix_ + "i_clamp_max").get_value<double>();
    i_min_ = node_ptr_->get_parameter(param_prefix_ + "i_clamp_min").get_value<double>();
    antiwindup_ = node_ptr_->get_parameter(param_prefix_ + "antiwindup").get_value<bool>();
    set_parameter_event_callback();

    state_pub_ = rclcpp::create_publisher<control_msgs::msg::PidState>(
      node_ptr->get_node_topics_interface(), topic_prefix_ + "pid_state", rclcpp::SensorDataQoS());
    rt_state_pub_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::msg::PidState>(state_pub_));
  }
  double computeCommand(double error, rclcpp::Duration period){
    if (period.seconds() <= 0.0 || !std::isfinite(error)) {
      return 0.0;
    }
    d_error_ = (error - p_error_last_) / period.seconds();
    p_error_last_ = error;
    return computeCommand(error, d_error_, period);
  }
  virtual double computeCommand(double error, double error_dot, rclcpp::Duration period){
    double p_term, d_term, i_term;
    p_error_ = error;  // this is error = target - state
    d_error_ = error_dot;
    if ( period.seconds() <= 0.0 || !std::isfinite(error) || !std::isfinite(error_dot))
      return 0.0;
    p_term = p_* p_error_;
    i_error_ += period.seconds() * p_error_;
    if (antiwindup_ && i_ != 0) {
      std::pair<double, double> bounds = std::minmax<double>(i_min_ / i_, i_max_ / i_);
      i_error_ = std::clamp(i_error_, bounds.first, bounds.second);
    }
    i_term = i_ * i_error_;
    if (!antiwindup_)
      i_term = std::clamp(i_term, i_min_, i_max_);
    d_term = d_ * d_error_;
    cmd_ = p_term + i_term + d_term;
    publish_pid_state(error,p_term,i_term,d_term,period);
    return cmd_;
  }
  double getCurrentCommand(){
    return cmd_;
  }

private:
  void set_parameter_event_callback()
  {
    auto on_parameter_event_callback = [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      bool changed = false;
      double i_min,i_max;
      for (auto & parameter : parameters) {
        const std::string param_name = parameter.get_name();
        try {
          if (param_name == param_prefix_ + "p") {
            p_ = parameter.get_value<double>();
            changed = true;
          } else if (param_name == param_prefix_ + "i") {
            i_ = parameter.get_value<double>();
            changed = true;
          } else if (param_name == param_prefix_ + "d") {
            d_ = parameter.get_value<double>();
            changed = true;
          } else if (param_name == param_prefix_ + "i_clamp_max") {
            i_max = parameter.get_value<double>();
            changed = true;
          } else if (param_name == param_prefix_ + "i_clamp_min") {
            i_min = parameter.get_value<double>();
            changed = true;
          } else if (param_name == param_prefix_ + "antiwindup") {
            antiwindup_ = parameter.get_value<bool>();
            changed = true;
          }
        } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
          RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Please use the right type: " << e.what());
        }
      }
      if (changed) {
        /// @note don't call set_gains() from inside a callback
        if (i_min > i_max) {
          RCLCPP_ERROR(node_ptr_->get_logger(), "received i_min > i_max, skip new gains");
        } else {
            i_min_ = i_min;
            i_max_ = i_max;
        }
      }
      return result;
    };
    parameter_callback_ = node_ptr_->get_node_parameters_interface()->add_on_set_parameters_callback(on_parameter_event_callback);
  }
  void publish_pid_state(double error,double p_term,double i_term,double d_term, rclcpp::Duration period)
  {
    if (rt_state_pub_) {
      if (rt_state_pub_->trylock()) {
        rt_state_pub_->msg_.header.stamp = rclcpp::Clock().now();
        rt_state_pub_->msg_.timestep = period;
        rt_state_pub_->msg_.error = error;
        rt_state_pub_->msg_.error_dot = d_error_;
        rt_state_pub_->msg_.p_error = p_error_;
        rt_state_pub_->msg_.i_error = i_error_;
        rt_state_pub_->msg_.d_error = d_error_;
        rt_state_pub_->msg_.p_term = p_term;
        rt_state_pub_->msg_.i_term = i_term;
        rt_state_pub_->msg_.d_term = d_term;
        rt_state_pub_->msg_.i_max = i_max_;
        rt_state_pub_->msg_.i_min = i_min_;
        rt_state_pub_->msg_.output = cmd_;
        rt_state_pub_->unlockAndPublish();
      }
    }
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::string param_prefix_,topic_prefix_;
  std::shared_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::PidState>> rt_state_pub_;
  std::shared_ptr<rclcpp::Publisher<control_msgs::msg::PidState>> state_pub_;
  double p_{}, i_{}, d_{}, i_min_{}, i_max_{},cmd_{};
  double p_error_{}, i_error_{}, d_error_{}, p_error_last_{};
  bool antiwindup_ = false;
};
}
#endif //CONTROL_TOOLS_HPP
