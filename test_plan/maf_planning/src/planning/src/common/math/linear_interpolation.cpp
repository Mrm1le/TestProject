#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "planning/common/common.h"

#include "mph_assert.h"
#include <cmath>
#include <iostream>

namespace msquare {
namespace planning_math {

double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    MSD_LOG(INFO, "input time difference is too small");
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

SLPoint InterpolateUsingLinearApproximation(const SLPoint &p0,
                                            const SLPoint &p1, const double w) {
  mph_assert(w > 0.0);

  SLPoint p;
  p.s = (1 - w) * p0.s + w * p1.s;
  p.l = (1 - w) * p0.l + w * p1.l;
  return p;
}

PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s;
  double s1 = p1.s;

  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x + weight * p1.x;
  double y = (1 - weight) * p0.y + weight * p1.y;
  double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
  double ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;
  path_point.x = x;
  path_point.y = y;
  path_point.theta = theta;
  path_point.kappa = kappa;
  path_point.dkappa = dkappa;
  path_point.ddkappa = ddkappa;
  path_point.s = s;
  return path_point;
}

TrajectoryPoint InterpolateUsingLinearApproximation(const TrajectoryPoint &tp0,
                                                    const TrajectoryPoint &tp1,
                                                    const double t) {
  const PathPoint pp0 = tp0.path_point;
  const PathPoint pp1 = tp1.path_point;
  double t0 = tp0.relative_time;
  double t1 = tp1.relative_time;

  TrajectoryPoint tp;
  tp.v = lerp(tp0.v, t0, tp1.v, t1, t);
  tp.a = lerp(tp0.a, t0, tp1.a, t1, t);
  tp.sigma_x = lerp(tp0.sigma_x, t0, tp1.sigma_x, t1, t);
  tp.sigma_y = lerp(tp0.sigma_y, t0, tp1.sigma_y, t1, t);
  tp.prediction_prob =
      lerp(tp0.prediction_prob, t0, tp1.prediction_prob, t1, t);
  tp.velocity_direction =
      // std::abs(t -t0) < std::abs(t -t1) ? tp0.velocity_direction :
      // tp1.velocity_direction;
      slerp(tp0.velocity_direction, t0, tp1.velocity_direction, t1, t);
  tp.relative_time = t;
  // tp.steer = slerp(tp0.steer, t0, tp1.steer, t1, t);

  PathPoint *path_point = &tp.path_point;
  path_point->x = lerp(pp0.x, t0, pp1.x, t1, t);
  path_point->y = lerp(pp0.y, t0, pp1.y, t1, t);
  path_point->theta = slerp(pp0.theta, t0, pp1.theta, t1, t);
  path_point->kappa = lerp(pp0.kappa, t0, pp1.kappa, t1, t);
  path_point->dkappa = lerp(pp0.dkappa, t0, pp1.dkappa, t1, t);
  path_point->ddkappa = lerp(pp0.ddkappa, t0, pp1.ddkappa, t1, t);
  path_point->s = lerp(pp0.s, t0, pp1.s, t1, t);
  // tp.relative_ego_x = lerp(tp0.relative_ego_x, t0, tp1.relative_ego_x, t1,
  // t); tp.relative_ego_y = lerp(tp0.relative_ego_y, t0, tp1.relative_ego_y,
  // t1, t);
  tp.relative_ego_yaw =
      lerp(tp0.relative_ego_yaw, t0, tp1.relative_ego_yaw, t1, t);
  // tp.relative_ego_speed = lerp(tp0.relative_ego_speed, t0,
  // tp1.relative_ego_speed, t1, t); tp.relative_ego_std_dev_x =
  // lerp(tp0.relative_ego_std_dev_x, t0, tp1.relative_ego_std_dev_x, t1, t);
  // tp.relative_ego_std_dev_y = lerp(tp0.relative_ego_std_dev_y, t0,
  // tp1.relative_ego_std_dev_y, t1, t); tp.relative_ego_std_dev_yaw =
  // lerp(tp0.relative_ego_std_dev_yaw, t0, tp1.relative_ego_std_dev_yaw, t1,
  // t); tp.relative_ego_std_dev_speed = lerp(tp0.relative_ego_std_dev_speed,
  // t0, tp1.relative_ego_std_dev_speed, t1, t);
  return tp;
}

} // namespace planning_math
} // namespace msquare