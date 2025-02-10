//
// Created by guanlin on 25-2-10.
//

#pragma once

namespace bullet_solver
{
enum Target
{
  ARMOR,
  WINDMILL
};
enum Armor
{
  FRONT,
  LEFT,
  BACK,
  RIGHT,
  CENTER
};
class TargetSelector
{
public:
  TargetSelector() = default;
  void reset(geometry_msgs::msg::Point pos, geometry_msgs::msg::Vector3 vel, double bullet_speed, double yaw,
             double v_yaw, double r1, double r2, double dz, int id)
  {
    pos_ = pos;
    vel_ = vel;
    bullet_speed_ = bullet_speed;
    yaw_ = yaw;
    v_yaw_ = v_yaw;
    r1_ = r1;
    r2_ = r2;
    dz_ = dz;
    id_ = id;
  }
  [[nodiscard]] int getTarget() const
  {
    if (id_ == 12)
      return WINDMILL;
    return ARMOR;
  }
  int getArmor()
  {
    // some select logic
    return FRONT;
  }

private:
  geometry_msgs::msg::Point pos_;
  geometry_msgs::msg::Vector3 vel_;
  double bullet_speed_{}, yaw_{}, v_yaw_{}, r1_{}, r2_{}, dz_{};
  int id_{};
};
}  // namespace bullet_solver
