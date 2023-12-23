#pragma once

#include <stdint.h>

namespace msquare {

class SmoothBrakeTrajectory {
public:
  struct SVTPoint {
    double s;
    double v;
    double a;
  };

public:
  SmoothBrakeTrajectory(const double s0, const double a, const double jerk);

  virtual ~SmoothBrakeTrajectory() = default;

  SVTPoint evaluateByDistance(const double &s);

  static SVTPoint evaluateByConstAccel(const double &s, const double &final_s,
                                       const double &v,
                                       const double &brake_distance,
                                       const double &min_brake_accel);

private:
  void convert(SVTPoint &res);

  SVTPoint evaluateByConstJerk(const double &s);

private:
  double s0_;
  double a_;
  double jerk_;
};

} // namespace msquare
