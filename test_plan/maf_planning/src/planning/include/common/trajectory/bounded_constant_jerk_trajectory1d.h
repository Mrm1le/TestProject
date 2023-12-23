#pragma once

#include <stdint.h>

namespace msquare {

class BoundedConstantJerkTrajectory1d {
public:
  BoundedConstantJerkTrajectory1d(const double p0, const double v0,
                                  const double a0, const double jerk,
                                  const double delta);

  virtual ~BoundedConstantJerkTrajectory1d() = default;

  void set_bound(double v_min, double v_max, double a_min, double a_max);

  double evaluate(const int order, const double param);

  void step();

  void reset();

  double start_position() const;

  double start_velocity() const;

  double start_acceleration() const;

  double position() const;

  double velocity() const;

  double acceleration() const;

  double jerk() const;

private:
  double p0_;
  double v0_;
  double a0_;

  double p1_;
  double v1_;
  double a1_;

  double delta_;
  double param_;

  double jerk_;

  double v_min_;
  double v_max_;
  double a_min_;
  double a_max_;
};

} // namespace msquare
