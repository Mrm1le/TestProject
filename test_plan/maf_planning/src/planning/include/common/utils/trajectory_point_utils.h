#pragma once
#include "common/math/math_utils.h"
#include "common/parking_planner_types.h"
#include <vector>

namespace msquare {
void apply_trajpoint_s(std::vector<TrajectoryPoint> &traj_points);

void modify_trajpoints_v(std::vector<TrajectoryPoint> &traj_points,
                         double v_forward, double v_backward);

void deprecate_velocity_invalid_trajpoints(
    std::vector<TrajectoryPoint> &traj_points, double min_valid_velocity);

bool is_zigzag_too_much(const std::vector<TrajectoryPoint> &traj_points,
                        int max_zigzag_allowd);

void double_dense(std::vector<TrajectoryPoint> &traj_points, int start,
                  int end);

void smooth_v(std::vector<TrajectoryPoint> &traj_points, double step_size,
              double max_acc, int start, int end, bool is_invert = false);

// std::vector<TrajectoryPoint> generate_deccelerate_segment(TrajectoryPoint&
// start, TrajectoryPoint& end, double acc);
void apply_trajpoints_curvature(std::vector<TrajectoryPoint> &traj_points_);

void extract_last_segment(const std::vector<TrajectoryPoint> &traj,
                          std::vector<TrajectoryPoint> &last_back_seg);
void lengthen_or_shorten(std::vector<TrajectoryPoint> &traj,
                         const planning_math::Vec2d stopper_pt1,
                         const planning_math::Vec2d stopper_pt2,
                         const double wheel_stop_buffer = 0.3);
double calc_dist_to_stopper(const Pose2D &curr_ego_pose,
                            const std::vector<TrajectoryPoint> &trajectory,
                            const double wheel_stop_buffer);

PathPoint operator+(PathPoint a, PathPoint b);

template <typename T> PathPoint operator*(T times, PathPoint &a) {
  PathPoint result;
  result.x = a.x * times;
  result.y = a.y * times;
  result.z = a.z * times;
  double wrapped_a_theta = planning_math::WrapAngle(a.theta);
  result.theta = planning_math::NormalizeAngle(wrapped_a_theta * times);
  result.s = a.s * times;
  result.l = a.l * times;
  result.x_derivative = a.x_derivative * times;
  result.y_derivative = a.y_derivative * times;
  result.kappa = a.kappa * times;
  result.dkappa = a.dkappa * times;
  result.ddkappa = a.ddkappa * times;
  result.rho = a.rho * times;
  return result;
}

TrajectoryPoint operator+(TrajectoryPoint a, TrajectoryPoint b);

template <typename T> TrajectoryPoint operator*(T times, TrajectoryPoint &a) {
  TrajectoryPoint result;
  result.path_point = times * a.path_point;
  result.v = a.v * times;
  result.a = a.a * times;
  result.steer = a.steer * times;
  result.relative_time = a.relative_time * times;
  result.velocity_direction = a.velocity_direction * times;
  return result; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
}

double td_fhan(double x1, double x2, double r, double h);
void td_filter(double v, double &x, double &d_x);

void link_target(std::vector<TrajectoryPoint> &traj,
                 const TrajectoryPoint &target_state, double acc, double vel,
                 double ds);

} // namespace msquare