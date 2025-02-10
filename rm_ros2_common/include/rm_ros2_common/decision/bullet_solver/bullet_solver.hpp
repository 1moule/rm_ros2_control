//
// Created by guanlin on 25-2-7.
//

#ifndef BULLET_SOLVER_HPP
#define BULLET_SOLVER_HPP

#include <realtime_tools/realtime_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <rm_ros2_common/decision/bullet_solver/target_kinematics.hpp>
#include <rm_ros2_common/decision/bullet_solver/target_selecter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
// #include <visualization_msgs/Marker.h>
// #include <rm_msgs/TrackData.h>

namespace bullet_solver
{
struct Config
{
  double resistance_coff_qd_16, resistance_coff_qd_25, g;
  double max_track_target_vel_;
};
class BulletSolver
{
public:
  explicit BulletSolver(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr);
  void selectTarget(geometry_msgs::msg::Point pos, geometry_msgs::msg::Vector3 vel, double bullet_speed, double yaw,
                    double v_yaw, double r1, double r2, double dz, int id);
  bool solve();
  //  double getGimbalError(geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw, double r1,
  //                        double r2, double dz, int armors_num, double yaw_real, double pitch_real, double bullet_speed);
  double getResistanceCoefficient(double bullet_speed);
  double getYaw() const
  {
    return output_yaw_;
  }
  double getPitch() const
  {
    return -output_pitch_;
  }
  //  void getSelectedArmorPosAndVel(geometry_msgs::Point& armor_pos, geometry_msgs::Vector3& armor_vel,
  //                                 geometry_msgs::Point pos, geometry_msgs::Vector3 vel, double yaw, double v_yaw,
  //                                 double r1, double r2, double dz, int armors_num);
  //  void judgeShootBeforehand(const ros::Time& time, double v_yaw);
  //  void bulletModelPub(const geometry_msgs::TransformStamped& odom2pitch, const ros::Time& time);
  //  void identifiedTargetChangeCB(const std_msgs::BoolConstPtr& msg);
  //  void reconfigCB(rm_gimbal_controllers::BulletSolverConfig& config, uint32_t);
  ~BulletSolver() = default;

private:
  std::shared_ptr<TargetKinematicsBase> target_kinematics_;
  std::shared_ptr<TargetSelector> target_selector_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_ptr_;

  //  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_desire_pub_;
  //  std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::Marker>> path_real_pub_;
  //  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::ShootBeforehandCmd>> shoot_beforehand_cmd_pub_;
  //  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> fly_time_pub_;
  //  ros::Subscriber identified_target_change_sub_;
  //  rclcpp::Time switch_armor_time_{};
  //  realtime_tools::RealtimeBuffer<Config> config_rt_buffer_;
  //  dynamic_reconfigure::Server<rm_gimbal_controllers::BulletSolverConfig>* d_srv_{};
  double output_yaw_{}, output_pitch_{};
  double bullet_speed_{}, resistance_coff_{}, fly_time_{};
  bool track_target_{};
  Config config_{};
  geometry_msgs::msg::Point target_pos_{};

  //  visualization_msgs::Marker marker_desire_;
  //  visualization_msgs::Marker marker_real_;
};
}  // namespace bullet_solver
#endif  // BULLET_SOLVER_HPP
