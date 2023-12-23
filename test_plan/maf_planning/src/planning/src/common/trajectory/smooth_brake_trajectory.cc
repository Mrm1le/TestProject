#include "common/trajectory/smooth_brake_trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace msquare {

SmoothBrakeTrajectory::SmoothBrakeTrajectory(const double s0, const double a,
                                             const double jerk)
    : s0_(s0), a_(a), jerk_(jerk) {}

void SmoothBrakeTrajectory::convert(SVTPoint &res) { res.s += s0_; }

SmoothBrakeTrajectory::SVTPoint
SmoothBrakeTrajectory::evaluateByConstJerk(const double &s) {
  double a = std::pow(6 * s * jerk_ * jerk_, 1.0 / 3.0);
  double v = 0.5 * a * a / jerk_;
  return {s, v, a};
}

SmoothBrakeTrajectory::SVTPoint
SmoothBrakeTrajectory::evaluateByDistance(const double &s) {
  const double s1 = 1.0 / 6.0 * a_ * a_ * a_ / jerk_ / jerk_;
  const double s_offseted = s - s0_;

  if (s_offseted <= s1) {
    auto res = evaluateByConstJerk(s_offseted);
    convert(res);
    return res;
  } else {
    auto point1 = evaluateByConstJerk(s1);
    double v2 =
        std::sqrt(point1.v * point1.v + 2 * point1.a * (s_offseted - s1));
    return {s, v2, point1.a};
  }
}

SmoothBrakeTrajectory::SVTPoint SmoothBrakeTrajectory::evaluateByConstAccel(
    const double &s, const double &final_s, const double &v,
    const double &brake_distance, const double &min_brake_accel) {
  const double s_offseted = final_s - s;
  const double accel = std::fmax(min_brake_accel, v * v / 2.0 / brake_distance);
  return {s, std::sqrt(2 * accel * s_offseted), accel};
}

} // namespace msquare
