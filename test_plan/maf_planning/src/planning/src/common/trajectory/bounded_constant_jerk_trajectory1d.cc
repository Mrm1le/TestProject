#include "common/trajectory/bounded_constant_jerk_trajectory1d.h"

#include <algorithm>
#include <limits>

namespace msquare {

inline double clip_func(double value, double min_value, double max_value) {
  return std::max(std::min(value, max_value), min_value);
};

BoundedConstantJerkTrajectory1d::BoundedConstantJerkTrajectory1d(
    const double p0, const double v0, const double a0, const double j,
    const double delta)
    : p0_(p0), v0_(v0), a0_(a0), p1_(p0), v1_(v0), a1_(a0), delta_(delta),
      param_(0.0), jerk_(j) {
  v_max_ = std::numeric_limits<double>::max();
  v_min_ = std::numeric_limits<double>::lowest();

  a_max_ = std::numeric_limits<double>::max();
  a_min_ = std::numeric_limits<double>::lowest();
}

void BoundedConstantJerkTrajectory1d::set_bound(double v_min, double v_max,
                                                double a_min, double a_max) {
  v_min_ = v_min;
  v_max_ = v_max;
  a_min_ = a_min;
  a_max_ = a_max;

  v0_ = clip_func(v0_, v_min_, v_max_);
  a0_ = clip_func(a0_, a_min_, a_max_);
  v1_ = clip_func(v1_, v_min_, v_max_);
  a1_ = clip_func(a1_, a_min_, a_max_);
}

double BoundedConstantJerkTrajectory1d::evaluate(const int order,
                                                 double param) {
  if (param < param_) {
    reset();
  }

  while (param_ + delta_ / 2.0 < param) {
    step();
  }

  switch (order) {
  case 0: {
    return p1_;
  }
  case 1: {
    return v1_;
  }
  case 2: {
    return a1_;
  }
  case 3: {
    return jerk_;
  }
  default:
    return 0.0;
  }
}

void BoundedConstantJerkTrajectory1d::step() {
  param_ += delta_;

  p1_ = p1_ + v1_ * delta_;
  v1_ = clip_func(v1_ + a1_ * delta_, v_min_, v_max_);
  a1_ = clip_func(a1_ + jerk_ * delta_, a_min_, a_max_);
}

void BoundedConstantJerkTrajectory1d::reset() {
  p1_ = p0_;
  v1_ = v0_;
  a1_ = a0_;
  param_ = 0.0;
}

double BoundedConstantJerkTrajectory1d::start_position() const { return p0_; }

double BoundedConstantJerkTrajectory1d::start_velocity() const { return v0_; }

double BoundedConstantJerkTrajectory1d::start_acceleration() const {
  return a0_;
}

double BoundedConstantJerkTrajectory1d::position() const { return p1_; }

double BoundedConstantJerkTrajectory1d::velocity() const { return v1_; }

double BoundedConstantJerkTrajectory1d::acceleration() const { return a1_; }

double BoundedConstantJerkTrajectory1d::jerk() const { return jerk_; }

} // namespace msquare
