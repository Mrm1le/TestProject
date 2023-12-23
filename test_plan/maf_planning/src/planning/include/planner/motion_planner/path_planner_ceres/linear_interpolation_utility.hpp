#pragma once
#include "common/math/math_utils.h"
#include "common/planning_common_type.h"
#include "common/utils/geometry.h"
#include "path_planner_types.hpp"

namespace msquare {
namespace planning_math {

template <typename T>
using InterpolationData = std::vector<std::pair<double, T>>;

template <typename T>
inline typename std::vector<T>::const_iterator
QueryLowerBound(const std::vector<T> &input_vector, const double path_s) {
  auto func = [](const T &tp, const double path_s) {
    return tp.first < path_s;
  };
  return std::lower_bound(input_vector.begin(), input_vector.end(), path_s,
                          func);
}

inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

class LinearInterpation {
public:
  LinearInterpation() = delete;

  template <typename T>
  static inline T interpolate(const InterpolationData<T> &input_data_pair_list,
                              const double s) {
    T tmp;
    if (input_data_pair_list.size() == 0)
      return tmp; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
    auto it_lower = QueryLowerBound(input_data_pair_list, s);
    if (it_lower == input_data_pair_list.begin()) {
      tmp = input_data_pair_list.front().second;
    } else if (it_lower == input_data_pair_list.end()) {
      tmp = input_data_pair_list.back().second;
    } else {
      tmp = InterpolateUsingLinearApproximation(*(it_lower - 1), *it_lower, s);
    }
    return tmp; // parasoft-suppress AUTOSAR-A8_5_0-a-2 "f-drop"
  }

  template <typename T>
  static inline T interpolate(const T &p0, const T &p1, const double s) {
    return InterpolateUsingLinearApproximation(p0, p1, s);
  }

  template <typename T>
  static inline T interpolation(T x, const std::vector<T> &xp,
                                const std::vector<T> &fp) {
    const size_t N = xp.size();
    size_t hi;
    for (hi = 0; hi < N && x > xp[hi]; hi++)
      ;

    if (hi == 0)
      return fp[0];

    if (hi == N && x > xp[N - 1])
      return fp[N - 1];

    const size_t low = hi - 1;
    const T xp_diff = xp[hi] - xp[low];
    if (xp_diff < static_cast<T>(1e-5f) && xp_diff > static_cast<T>(-1e-5f))
      return fp[low];

    return (x - xp[low]) * (fp[hi] - fp[low]) / xp_diff + fp[low];
  }

private:
  static inline Point2D InterpolateUsingLinearApproximation(const Point2D &p0,
                                                            const Point2D &p1,
                                                            const double s) {
    double s0 = p0.x;
    double s1 = p1.x;

    Point2D point;
    double weight = (s - s0) / (s1 - s0);
    point.x = (1 - weight) * p0.x + weight * p1.x;
    point.y = (1 - weight) * p0.y + weight * p1.y;
    return point;
  }

  static inline path_planner::Point2d
  InterpolateUsingLinearApproximation(const path_planner::Point2d &p0,
                                      const path_planner::Point2d &p1,
                                      const double s) {
    double s0 = p0.x;
    double s1 = p1.x;

    path_planner::Point2d point;
    double weight = (s - s0) / (s1 - s0);
    point.x = (1 - weight) * p0.x + weight * p1.x;
    point.y = (1 - weight) * p0.y + weight * p1.y;
    return point;
  }

  static inline path_planner::Point2d InterpolateUsingLinearApproximation(
      const std::pair<double, path_planner::Point2d> &p0,
      const std::pair<double, path_planner::Point2d> &p1, const double s) {
    return InterpolateUsingLinearApproximation(p0.second, p1.second, s);
  }

  static inline double
  InterpolateUsingLinearApproximation(const std::pair<double, double> &pair0,
                                      const std::pair<double, double> &pair1,
                                      const double s) {
    double s0 = pair0.first;
    double s1 = pair1.first;

    double weight = (s - s0) / (s1 - s0);
    return (1 - weight) * pair0.second + weight * pair1.second;
  }

  static inline path_planner::PathPoint InterpolateUsingLinearApproximation(
      const std::pair<double, path_planner::PathPoint> &pair0,
      const std::pair<double, path_planner::PathPoint> &pair1, const double s) {

    double s0 = pair0.first;
    double s1 = pair1.first;

    double weight = (s - s0) / (s1 - s0);

    const auto &p0 = pair0.second;
    const auto &p1 = pair1.second;

    path_planner::PathPoint path_point;
    path_point.heading_yaw =
        (1 - weight) * p0.heading_yaw + weight * p1.heading_yaw;
    path_point.curvature = (1 - weight) * p0.curvature + weight * p1.curvature;
    path_point.path_follow_strength = (1 - weight) * p0.path_follow_strength +
                                      weight * p1.path_follow_strength;

    path_point.position_enu.x =
        (1 - weight) * p0.position_enu.x + weight * p1.position_enu.x;
    path_point.position_enu.y =
        (1 - weight) * p0.position_enu.y + weight * p1.position_enu.y;

    return path_point;
  }

  static inline path_planner::SpeedPlan InterpolateUsingLinearApproximation(
      const std::pair<double, path_planner::SpeedPlan> &pair0,
      const std::pair<double, path_planner::SpeedPlan> &pair1, const double s) {

    double s0 = pair0.first;
    double s1 = pair1.first;

    double weight = (s - s0) / (s1 - s0);

    const auto &p0 = pair0.second;
    const auto &p1 = pair1.second;

    path_planner::SpeedPlan speed_point;
    speed_point.v = (1 - weight) * p0.v + weight * p1.v;
    speed_point.a = (1 - weight) * p0.a + weight * p1.a;
    speed_point.j = (1 - weight) * p0.j + weight * p1.j;
    return speed_point;
  }

  static inline path_planner::RefPointInfo InterpolateUsingLinearApproximation(
      const std::pair<double, path_planner::RefPointInfo> &p0,
      const std::pair<double, path_planner::RefPointInfo> &p1, const double s) {
    double s0 = p0.first;
    double s1 = p1.first;

    path_planner::RefPointInfo ref_point_info;
    double weight = (s - s0) / (s1 - s0);

    ref_point_info.time =
        (1 - weight) * p0.second.time + weight * p1.second.time;
    ref_point_info.x = (1 - weight) * p0.second.x + weight * p1.second.x;
    ref_point_info.y = (1 - weight) * p0.second.y + weight * p1.second.y;
    ref_point_info.cos_theta_ref = (1 - weight) * p0.second.cos_theta_ref +
                                   weight * p1.second.cos_theta_ref;
    ref_point_info.sin_theta_ref = (1 - weight) * p0.second.sin_theta_ref +
                                   weight * p1.second.sin_theta_ref;
    ref_point_info.theta_ref =
        std::atan2(ref_point_info.sin_theta_ref, ref_point_info.cos_theta_ref);
    ref_point_info.curvature =
        (1 - weight) * p0.second.curvature + weight * p1.second.curvature;
    ref_point_info.current_lane_width =
        (1 - weight) * p0.second.current_lane_width +
        weight * p1.second.current_lane_width;

    ref_point_info.left_lane_width = (1 - weight) * p0.second.left_lane_width +
                                     weight * p1.second.left_lane_width;
    ref_point_info.right_lane_width =
        (1 - weight) * p0.second.right_lane_width +
        weight * p1.second.right_lane_width;
    ref_point_info.left_lane_border =
        (1 - weight) * p0.second.left_lane_border +
        weight * p1.second.left_lane_border;
    ref_point_info.right_lane_border =
        (1 - weight) * p0.second.right_lane_border +
        weight * p1.second.right_lane_border;
    ref_point_info.left_road_border =
        (1 - weight) * p0.second.left_road_border +
        weight * p1.second.left_road_border;
    ref_point_info.right_road_border =
        (1 - weight) * p0.second.right_road_border +
        weight * p1.second.right_road_border;
    ref_point_info.left_road_border_type = p0.second.left_road_border_type;
    ref_point_info.right_road_border_type = p0.second.right_road_border_type;
    ref_point_info.left_lane_border_type = p0.second.left_lane_border_type;
    ref_point_info.right_lane_border_type = p0.second.right_lane_border_type;

    return ref_point_info;
  }

  static inline RefPointFrenet InterpolateUsingLinearApproximation(
      const std::pair<double, RefPointFrenet> &p0,
      const std::pair<double, RefPointFrenet> &p1, const double s) {
    double s0 = p0.first;
    double s1 = p1.first;

    RefPointFrenet ref_point_info;
    double weight = (s - s0) / (s1 - s0);
    ref_point_info.s = (1 - weight) * p0.second.s + weight * p1.second.s;
    ref_point_info.lane_width =
        (1 - weight) * p0.second.lane_width + weight * p1.second.lane_width;
    ref_point_info.left_lane_border =
        (1 - weight) * p0.second.left_lane_border +
        weight * p1.second.left_lane_border;
    ref_point_info.right_lane_border =
        (1 - weight) * p0.second.right_lane_border +
        weight * p1.second.right_lane_border;
    ref_point_info.left_road_border =
        std::min(p0.second.left_road_border, p1.second.left_road_border);
    ref_point_info.right_road_border =
        std::min(p0.second.right_road_border, p1.second.right_road_border);

    // As a safety precaution, use conservative road border type
    ref_point_info.left_road_border_type = std::max(
        p0.second.left_road_border_type, p1.second.left_road_border_type);
    ref_point_info.right_road_border_type = std::max(
        p0.second.right_road_border_type, p1.second.right_road_border_type);

    return ref_point_info;
  }

  static inline path_planner::PathSampleInfo
  InterpolateUsingLinearApproximation(const path_planner::PathSampleInfo &p0,
                                      const path_planner::PathSampleInfo &p1,
                                      const double s) {
    double s0 = p0.sample_s;
    double s1 = p1.sample_s;

    path_planner::PathSampleInfo tmp_path_sample;
    tmp_path_sample.sample_s = s;
    tmp_path_sample.refline_info = interpolate<path_planner::RefPointInfo>(
        InterpolationData<path_planner::RefPointInfo>{{s0, p0.refline_info},
                                                      {s1, p1.refline_info}},
        s);
    tmp_path_sample.last_cart_traj = interpolate<path_planner::PathPoint>(
        InterpolationData<path_planner::PathPoint>{{s0, p0.last_cart_traj},
                                                   {s1, p1.last_cart_traj}},
        s);
    tmp_path_sample.speed_plan = interpolate<path_planner::SpeedPlan>(
        InterpolationData<path_planner::SpeedPlan>{{s0, p0.speed_plan},
                                                   {s1, p1.speed_plan}},
        s);
    return tmp_path_sample;
  }

  static inline path_planner::PathSampleInfo
  InterpolateUsingLinearApproximation(
      const std::pair<double, path_planner::PathSampleInfo> &p0,
      const std::pair<double, path_planner::PathSampleInfo> &p1,
      const double s) {

    return InterpolateUsingLinearApproximation(p0.second, p1.second, s);
  }

  static inline path_planner::PathPlannerPoint
  InterpolateUsingLinearApproximation(
      const std::pair<double, path_planner::PathPlannerPoint> &pair0,
      const std::pair<double, path_planner::PathPlannerPoint> &pair1,
      const double s) {
    double s0 = pair0.first;
    double s1 = pair1.first;

    double weight = (s - s0) / (s1 - s0);

    path_planner::PathPlannerPoint tmp;
    tmp.s = (1 - weight) * pair0.second.s + weight * pair1.second.s;
    tmp.v = (1 - weight) * pair0.second.v + weight * pair1.second.v;
    tmp.t = (1 - weight) * pair0.second.t + weight * pair1.second.t;
    tmp.a = (1 - weight) * pair0.second.a + weight * pair1.second.a;
    tmp.da_ds = (1 - weight) * pair0.second.da_ds + weight * pair1.second.da_ds;
    tmp.x = (1 - weight) * pair0.second.x + weight * pair1.second.x;
    tmp.dx_ds = (1 - weight) * pair0.second.dx_ds + weight * pair1.second.dx_ds;
    tmp.d2x_ds2 =
        (1 - weight) * pair0.second.d2x_ds2 + weight * pair1.second.d2x_ds2;

    tmp.y = (1 - weight) * pair0.second.y + weight * pair1.second.y;
    tmp.dy_ds = (1 - weight) * pair0.second.dy_ds + weight * pair1.second.dy_ds;
    tmp.d2y_ds2 =
        (1 - weight) * pair0.second.d2y_ds2 + weight * pair1.second.d2y_ds2;
    tmp.l = (1 - weight) * pair0.second.l + weight * pair1.second.l;
    tmp.dl = (1 - weight) * pair0.second.dl + weight * pair1.second.dl;
    tmp.ddl = (1 - weight) * pair0.second.ddl + weight * pair1.second.ddl;
    tmp.jerk = (1 - weight) * pair0.second.jerk + weight * pair1.second.jerk;
    tmp.curvature =
        (1 - weight) * pair0.second.curvature + weight * pair1.second.curvature;

    tmp.sample_info = InterpolateUsingLinearApproximation(
        pair0.second.sample_info, pair1.second.sample_info, s);
    return tmp;
  }

  static inline path_planner::ObsPrediction InterpolateUsingLinearApproximation(
      const std::pair<double, path_planner::ObsPrediction> &pair0,
      const std::pair<double, path_planner::ObsPrediction> &pair1,
      const double t) {
    double t0 = pair0.first;
    double t1 = pair1.first;

    path_planner::ObsPrediction obj_at_t = {};
    double weight = (t - t0) / (t1 - t0);
    obj_at_t.rel_s =
        (1 - weight) * pair0.second.rel_s + weight * pair1.second.rel_s;
    obj_at_t.s = (1 - weight) * pair0.second.s + weight * pair1.second.s;
    obj_at_t.v = (1 - weight) * pair0.second.v + weight * pair1.second.v;
    obj_at_t.v_frenet =
        (1 - weight) * pair0.second.v_frenet + weight * pair1.second.v_frenet;
    obj_at_t.a = (1 - weight) * pair0.second.a + weight * pair1.second.a;
    obj_at_t.heading =
        (1 - weight) * pair0.second.heading + weight * pair1.second.heading;

    return obj_at_t;
  }

  static inline speed_planner::ModelPoint InterpolateUsingLinearApproximation(
      const std::pair<double, speed_planner::ModelPoint> &pair0,
      const std::pair<double, speed_planner::ModelPoint> &pair1,
      const double s) {
    double s0 = pair0.first;
    double s1 = pair1.first;
    double weight = (s - s0) / (s1 - s0);

    speed_planner::ModelPoint point;
    point.t = (1 - weight) * pair0.second.t + weight * pair1.second.t;
    point.v = (1 - weight) * pair0.second.v + weight * pair1.second.v;
    point.a = (1 - weight) * pair0.second.a + weight * pair1.second.a;
    point.s = (1 - weight) * pair0.second.s + weight * pair1.second.s;

    point.heading_angle = interpolate_angle(pair0.second.heading_angle,
                                            pair1.second.heading_angle, weight);
    point.x = (1 - weight) * pair0.second.x + weight * pair1.second.x;
    point.y = (1 - weight) * pair0.second.y + weight * pair1.second.y;

    return point;
  }
};

} // namespace planning_math
} // namespace msquare
