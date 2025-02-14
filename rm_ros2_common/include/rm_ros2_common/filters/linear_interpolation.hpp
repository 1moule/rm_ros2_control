//
// Created by guanlin on 25-2-14.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace rm_ros2_common
{
class LinearInterp
{
public:
  explicit LinearInterp(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
  }
  void init(const std::vector<double>& x, const std::vector<double>& y)
  {
    for (int i = 0; i < x.size(); i++)
    {
      if (!input_vector_.empty())
        if (static_cast<double>(x[i]) < input_vector_.back())
        {
          RCLCPP_ERROR(node_->get_logger(), "Please sort the point's abscissa from smallest to largest. %lf < %lf",
                       static_cast<double>(x[i]), input_vector_.back());
          return;
        }
      input_vector_.push_back(x[i]);
      output_vector_.push_back(y[i]);
    }
  }
  double output(const double input) const
  {
    if (input >= input_vector_.back())
      return output_vector_.back();
    if (input <= input_vector_.front())
      return output_vector_.front();
    for (size_t i = 0; i < input_vector_.size(); i++)
    {
      if (input >= input_vector_[i] && input <= input_vector_[i + 1])
        return output_vector_[i] +
               ((output_vector_[i + 1] - output_vector_[i]) / (input_vector_[i + 1] - input_vector_[i])) *
                   (input - input_vector_[i]);
    }
    RCLCPP_ERROR(node_->get_logger(), "The point's abscissa aren't sorted from smallest to largest.");
    return 0;
  }

private:
  std::vector<double> input_vector_;
  std::vector<double> output_vector_;
  rclcpp::Node::SharedPtr node_;
};
}  // namespace rm_ros2_common