//
// Created by guanlin on 25-2-10.
//

#include <rm_ros2_common/decision/bullet_solver/bullet_solver.hpp>

namespace bullet_solver
{
BulletSolver::BulletSolver(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr) : node_ptr_(std::move(node_ptr))
{
  config_.g = node_ptr_->get_parameter("bullet_solver.g").get_value<double>();
  config_.delay = node_ptr_->get_parameter("bullet_solver.delay").get_value<double>();
  config_.max_track_target_vel_ = node_ptr_->get_parameter("bullet_solver.max_track_target_vel").get_value<double>();
  config_.resistance_coff_qd_16 = node_ptr_->get_parameter("bullet_solver.resistance_coff_qd_16").get_value<double>();
  config_.resistance_coff_qd_25 = node_ptr_->get_parameter("bullet_solver.resistance_coff_qd_25").get_value<double>();

  state_pub_ = node_ptr_->create_publisher<rm_ros2_msgs::msg::BulletSolverState>("state", rclcpp::SystemDefaultsQoS());
  rt_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::BulletSolverState>>(state_pub_);

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
    const int armor = target_selector_->getArmor();
    double r = (armor == FRONT || armor == BACK) ? r1 : r2;
    yaw += (M_PI / 2) * (armor - 1);
    if (armor == LEFT || armor == RIGHT)
      pos.z += dz;
    if (std::abs(v_yaw) < config_.max_track_target_vel_)
    {
      track_target_ = true;
      target_kinematics_ = std::make_shared<TrackedArmorKinematic>(pos, vel, yaw, v_yaw, r);
    }
    else
    {
      track_target_ = false;
      target_kinematics_ = std::make_shared<UntrackedArmorKinematic>(pos, vel, yaw, v_yaw, r);
    }
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

void BulletSolver::getYawVelDes(double& vel_des) const
{
  const geometry_msgs::msg::Vector3 target_vel = target_kinematics_->velocity(fly_time_);
  const double yaw_vel_des =
      (target_pos_.x * target_vel.y - target_pos_.y * target_vel.x) / (pow(target_pos_.x, 2) + pow(target_pos_.y, 2));
  vel_des = yaw_vel_des;
}

void BulletSolver::getPitchVelDes(double& vel_des) const
{
  constexpr double dt = 0.01;
  const geometry_msgs::msg::Point pos = target_kinematics_->position(fly_time_ + dt);
  const double target_rho = std::sqrt(std::pow(pos.x, 2) + std::pow(pos.y, 2));
  double temp_z = target_rho * tan(output_pitch_);
  double output_pitch_next = output_pitch_;
  double error_z = 999;
  while (std::abs(error_z) >= 1e-9)
  {
    output_pitch_next = std::atan2(temp_z, target_rho);
    const double fly_time =
        (-std::log(1 - target_rho * resistance_coff_ / (bullet_speed_ * std::cos(output_pitch_next)))) /
        resistance_coff_;
    const double real_z = (bullet_speed_ * std::sin(output_pitch_next) + (config_.g / resistance_coff_)) *
                              (1 - std::exp(-resistance_coff_ * fly_time)) / resistance_coff_ -
                          config_.g * fly_time / resistance_coff_;
    error_z = pos.z - real_z;
    temp_z += error_z;
  }
  const double pitch_vel_des = (output_pitch_next - output_pitch_) / dt;
  vel_des = pitch_vel_des;
}

double BulletSolver::getResistanceCoefficient(const double bullet_speed) const
{
  double resistance_coff;
  if (bullet_speed <= 16)
    resistance_coff = config_.resistance_coff_qd_16;
  else
    resistance_coff = config_.resistance_coff_qd_25;
  return resistance_coff;
}

double BulletSolver::getGimbalError(double yaw_real, double pitch_real) const
{
  double error;
  if ((target_selector_->getTarget() == ARMOR && track_target_) || target_selector_->getTarget() == WINDMILL)
  {
    double bullet_rho =
        bullet_speed_ * std::cos(pitch_real) * (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_;
    double bullet_x = bullet_rho * std::cos(yaw_real);
    double bullet_y = bullet_rho * std::sin(yaw_real);
    double bullet_z = (bullet_speed_ * std::sin(pitch_real) + (config_.g / resistance_coff_)) *
                          (1 - std::exp(-resistance_coff_ * fly_time_)) / resistance_coff_ -
                      config_.g * fly_time_ / resistance_coff_;
    error = std::sqrt(std::pow(target_pos_.x - bullet_x, 2) + std::pow(target_pos_.y - bullet_y, 2) +
                      std::pow(target_pos_.z - bullet_z, 2));
  }
  else
  {
    double delay = config_.delay;
    geometry_msgs::msg::Point target_pos_after_fly_time_and_delay{};
    target_pos_after_fly_time_and_delay = target_kinematics_->position(fly_time_ + delay);
    error = std::sqrt(std::pow(target_pos_.x - target_pos_after_fly_time_and_delay.x, 2) +
                      std::pow(target_pos_.y - target_pos_after_fly_time_and_delay.y, 2) +
                      std::pow(target_pos_.z - target_pos_after_fly_time_and_delay.z, 2));
  }
  return error;
}

void BulletSolver::publishState() const
{
  if (rt_state_pub_)
  {
    if (rt_state_pub_->trylock())
    {
      rt_state_pub_->unlockAndPublish();
    }
  }
}
}  // namespace bullet_solver
