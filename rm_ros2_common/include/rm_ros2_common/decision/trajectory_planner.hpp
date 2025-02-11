//
// Created by guanlin on 25-2-9.
//

#pragma once

#include <cmath>
#include <string>

template <typename T>
class NonlinearTrackingDifferentiator
{
public:
  NonlinearTrackingDifferentiator(T r, T h) : r_(r), h_(h)
  {
  }
  void clear(T x1, T x2)
  {
    x1_ = x1;
    x2_ = x2;
  }
  void update(T v, T v_dot)
  {
    if ((v > 3. && x1_ < -3.) || (v < -3 && x1_ > 3))
    {
      x1_ = v;
      x2_ = 0;
    }
    else
    {
      T y = x1_ - v + h_ * x2_;
      T a0 = sqrt(h_ * h_ * r_ * r_ + 8 * r_ * fabs(y));
      T a = x2_ + 0.5 * (a0 - h_ * r_) * (y > 0 ? 1 : -1);
      T u = fabs(a) > h_ * r_ ? -r_ * (a > 0 ? 1 : -1) : -r_ * a / (h_ * r_);
      x1_ = x1_ + h_ * (x2_ + v_dot);
      x2_ = x2_ + h_ * u;
    }
  }
  void update(T v)
  {
    update(v, 0.);
  }

  T getX1() const
  {
    return x1_;
  }
  T getX2() const
  {
    return x2_;
  }

private:
  T r_{}, h_{};
  T x1_{}, x2_{};
};
