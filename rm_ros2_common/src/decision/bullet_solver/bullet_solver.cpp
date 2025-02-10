//
// Created by guanlin on 25-2-10.
//

#include <rm_ros2_common/decision/bullet_solver/bullet_solver.hpp>

namespace bullet_solver
{
BulletSolver::BulletSolver(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr) : node_ptr_(std::move(node_ptr))
{
  config_.g = node_ptr_->get_parameter("bullet_solver.g").get_value<double>();
  config_.resistance_coff_qd_16 = node_ptr_->get_parameter("bullet_solver.resistance_coff_qd_16").get_value<double>();
  config_.resistance_coff_qd_25 = node_ptr_->get_parameter("bullet_solver.resistance_coff_qd_25").get_value<double>();

  target_selector_ = std::make_shared<TargetSelector>();
}

void BulletSolver::selectTarget(geometry_msgs::msg::Point pos, geometry_msgs::msg::Vector3 vel, double bullet_speed,
                                double yaw, double v_yaw, double r1, double r2, double dz, int id)
{
  bullet_speed_ = bullet_speed;
  resistance_coff_ = getResistanceCoefficient(bullet_speed_) != 0 ? getResistanceCoefficient(bullet_speed_) : 0.001;
  target_selector_->reset(pos, vel, bullet_speed, yaw, v_yaw, r1, r2, dz, resistance_coff_, id);
  if (target_selector_->getTarget() == WINDMILL)
    target_kinematics_ = std::make_shared<TrackedArmorKinematic>(pos, vel, yaw, v_yaw, r1);
  else
  {
    double r{};
    switch (target_selector_->getArmor())
    {
      case FRONT:
        r = r1;
        break;
      case BACK:
        r = r1;
        yaw += M_PI;
        break;
      case LEFT:
        r = r2;
        yaw -= M_PI / 2;
        pos.z += dz;
        break;
      case RIGHT:
        r = r2;
        yaw += M_PI / 2;
        pos.z += dz;
        break;
      default:
        r = r1;
        break;
    }
    if (std::abs(v_yaw) < config_.max_track_target_vel_)
      target_kinematics_ = std::make_shared<TrackedArmorKinematic>(pos, vel, yaw, v_yaw, r);
    else
      target_kinematics_ = std::make_shared<UntrackedArmorKinematic>(pos, vel, yaw, v_yaw, r);
  }
}

bool BulletSolver::solve()
{
  int count{};
  double error = 999;

  target_pos_ = target_kinematics_->position(0.);

  double temp_z = target_pos_.z;
  while (error >= 0.001)
  {
    output_yaw_ = std::atan2(target_pos_.y, target_pos_.x);
    double target_rho = std::sqrt(std::pow(target_pos_.x, 2) + std::pow(target_pos_.y, 2));
    output_pitch_ = std::atan2(temp_z, target_rho);
    fly_time_ =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_)))) / resistance_coff_;
    double real_z = (bullet_speed_ * std::sin(output_pitch_) + (config_.g / resistance_coff_)) *
                        (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                    config_.g * fly_time_ / resistance_coff_;

    target_pos_ = target_kinematics_->position(fly_time_);

    double target_yaw = std::atan2(target_pos_.y, target_pos_.x);
    double error_theta = target_yaw - output_yaw_;
    double error_z = target_pos_.z - real_z;
    temp_z += error_z;
    error = std::sqrt(std::pow(error_theta * target_rho, 2) + std::pow(error_z, 2));
    count++;

    if (count >= 20 || std::isnan(error))
      return false;
  }
  return true;
}
}  // namespace bullet_solver
