//
// Created by guanlin on 25-2-7.
//

#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rm_ros2_msgs/msg/bullet_solver_state.hpp>
#include <rm_ros2_common/decision/bullet_solver/target_kinematics.hpp>
#include <rm_ros2_common/decision/bullet_solver/target_selector.hpp>

namespace bullet_solver
{
struct Config
{
  double resistance_coff_qd_16, resistance_coff_qd_25, g, delay;
  double max_track_target_vel_;
};
class BulletSolver
{
public:
  explicit BulletSolver(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr);
  void selectTarget(geometry_msgs::msg::Point pos, geometry_msgs::msg::Vector3 vel, double bullet_speed, double yaw,
                    double v_yaw, double r1, double r2, double dz, int id);
  bool solve();
  void getYawVelDes(double& vel_des) const;
  void getPitchVelDes(double& vel_des) const;
  [[nodiscard]] double getYaw() const
  {
    return output_yaw_;
  }
  [[nodiscard]] double getPitch() const
  {
    return -output_pitch_;
  }
  [[nodiscard]] double getResistanceCoefficient(double bullet_speed) const;
  double BulletSolver::getGimbalError(double yaw_real, double pitch_real) const;
  void publishState() const;
  ~BulletSolver() = default;

private:
  std::shared_ptr<TargetKinematicsBase> target_kinematics_;
  std::shared_ptr<TargetSelector> target_selector_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_ros2_msgs::msg::BulletSolverState>> rt_state_pub_;
  std::shared_ptr<rclcpp::Publisher<rm_ros2_msgs::msg::BulletSolverState>> state_pub_;
  geometry_msgs::msg::Point target_pos_{};
  Config config_{};
  bool track_target_{};
  double output_yaw_{}, output_pitch_{};
  double bullet_speed_{}, resistance_coff_{}, fly_time_{};
};
}  // namespace bullet_solver
