//
// Created by guanlin on 25-2-10.
//

#pragma once

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bullet_solver
{
class TargetKinematicsBase
{
public:
  virtual ~TargetKinematicsBase() = default;
  virtual geometry_msgs::msg::Point position(double time) = 0;
  virtual geometry_msgs::msg::Vector3 velocity(double time) = 0;
};

class ArmorKinematics : public TargetKinematicsBase
{
public:
  ArmorKinematics(geometry_msgs::msg::Point pos, geometry_msgs::msg::Vector3 vel, double yaw, double v_yaw, double r)
    : pos_(pos), vel_(vel), yaw_(yaw), v_yaw_(v_yaw), r_(r)
  {
  }

protected:
  geometry_msgs::msg::Point pos_;
  geometry_msgs::msg::Vector3 vel_;
  double yaw_;
  double v_yaw_;
  double r_;
};

class TrackedArmorKinematic final : public ArmorKinematics
{
public:
  TrackedArmorKinematic(geometry_msgs::msg::Point pos, geometry_msgs::msg::Vector3 vel, double yaw, double v_yaw,
                        double r)
    : ArmorKinematics(pos, vel, yaw, v_yaw, r)
  {
  }
  geometry_msgs::msg::Point position(double time) override
  {
    geometry_msgs::msg::Point target_pos;
    target_pos.x = pos_.x + vel_.x * time - r_ * cos(yaw_ + v_yaw_ * time);
    target_pos.y = pos_.y + vel_.y * time - r_ * sin(yaw_ + v_yaw_ * time);
    target_pos.z = pos_.z + vel_.z * time;
    return target_pos;
  }
  geometry_msgs::msg::Vector3 velocity(double time) override
  {
    geometry_msgs::msg::Vector3 target_vel;
    target_vel.x = vel_.x + v_yaw_ * r_ * sin(yaw_ + v_yaw_ * time);
    target_vel.y = vel_.y - v_yaw_ * r_ * cos(yaw_ + v_yaw_ * time);
    return target_vel;
  }
};

class UntrackedArmorKinematic final : public ArmorKinematics
{
public:
  UntrackedArmorKinematic(geometry_msgs::msg::Point pos, geometry_msgs::msg::Vector3 vel, double yaw, double v_yaw,
                          double r)
    : ArmorKinematics(pos, vel, yaw, v_yaw, r)
  {
  }
  geometry_msgs::msg::Point position(double time) override
  {
    geometry_msgs::msg::Point target_pos;
    double target_center_pos[2];
    target_center_pos[0] = pos_.x + vel_.x * time;
    target_center_pos[1] = pos_.y + vel_.y * time;
    target_pos.x = target_center_pos[0] - r_ * cos(atan2(target_center_pos[1], target_center_pos[0]));
    target_pos.y = target_center_pos[1] - r_ * sin(atan2(target_center_pos[1], target_center_pos[0]));
    target_pos.z = pos_.z + vel_.z * time;
    return target_pos;
  }
  geometry_msgs::msg::Vector3 velocity(double /*time*/) override
  {
    geometry_msgs::msg::Vector3 target_vel;
    return target_vel;
  }
};

class WindmillKinematics : public TargetKinematicsBase
{
public:
  WindmillKinematics(double theta, double theta_dot, double radius, geometry_msgs::msg::TransformStamped windmill2odom)
  {
    theta_ = theta;
    theta_dot_ = theta_dot;
    radius_ = radius;
    windmill2odom_ = windmill2odom;
  }
  geometry_msgs::msg::Point position(double time) override
  {
    geometry_msgs::msg::Point target_pos;
    target_pos.x = 0.;
    target_pos.y = -radius_ * sin(theta_ + theta_dot_ * time);
    target_pos.z = radius_ * cos(theta_ + theta_dot_ * time);
    tf2::doTransform(target_pos, target_pos, windmill2odom_);
    return target_pos;
  }
  geometry_msgs::msg::Vector3 velocity(double time) override
  {
    geometry_msgs::msg::Vector3 target_vel;
    target_vel.x = 0.;
    target_vel.y = -theta_dot_ * radius_ * cos(theta_ + theta_dot_ * time);
    target_vel.z = -theta_dot_ * radius_ * sin(theta_ + theta_dot_ * time);
    tf2::doTransform(target_vel, target_vel, windmill2odom_);
    return target_vel;
  }

private:
  double theta_;
  double theta_dot_;
  double radius_;
  geometry_msgs::msg::TransformStamped windmill2odom_;
};
}  // namespace bullet_solver
