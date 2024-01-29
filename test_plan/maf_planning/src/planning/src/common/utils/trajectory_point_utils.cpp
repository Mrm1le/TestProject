#include "common/utils/trajectory_point_utils.h"
#include "common/math/math_utils.h"
#include <iostream>

namespace msquare {

void apply_trajpoint_s(std::vector<TrajectoryPoint> &traj_point) {
  if (traj_point.size() == 0)
    return;
  else {
    traj_point[0].path_point.s = 0.0;
    for (std::size_t i = 1; i < traj_point.size(); i++) {
      double ds = pow(
          pow(traj_point[i].path_point.x - traj_point[i - 1].path_point.x,
              2.0) +
              pow(traj_point[i].path_point.y - traj_point[i - 1].path_point.y,
                  2.0),
          0.5);
      traj_point[i].path_point.s = traj_point[i - 1].path_point.s + ds;
    }
  }
}

void modify_trajpoints_v(std::vector<TrajectoryPoint> &traj_points,
                         double v_forward, double v_backward) {
  for (std::vector<TrajectoryPoint>::iterator iter = traj_points.begin();
       iter != traj_points.end(); ++iter) {
    if (iter->v > 0)
      iter->v = v_forward;
    else if (iter->v < 0)
      iter->v = -v_backward;
    else {
      // throw std::logic_error("traj points shold not contain zero velocity!");
    }
  }
}

void deprecate_velocity_invalid_trajpoints(
    std::vector<TrajectoryPoint> &traj_points, double min_valid_velocity) {
  for (std::vector<TrajectoryPoint>::iterator iter = traj_points.begin();
       iter != traj_points.end(); ++iter) {
    if (std::abs(iter->v) < min_valid_velocity && std::fabs(iter->v) > 1e-5) {
      iter->v = iter->v > 0 ? min_valid_velocity : -min_valid_velocity;
    }
  }
}

// bool is_zigzag_too_much(const std::vector<TrajectoryPoint> &traj_points,
//                         int max_zigzag_allowd) {
//   if (max_zigzag_allowd < 0) // allow infinit zig zags
//   {
//     return false;
//   }

//   ZigzagPath zigzag_path(traj_points);
//   if (zigzag_path.get_num_stages() <= max_zigzag_allowd) {
//     return false;
//   }

//   return true;
//   return false;
// }

void double_dense(std::vector<TrajectoryPoint> &traj_points, int start,
                  int end) {
  using namespace planning_math;
  if (start == end)
    return;
  if (start < 0 || end < 0 || start > end || end > traj_points.size()) {
    // throw std::invalid_argument("[double_dense] of traj points");
  }
  std::vector<TrajectoryPoint>::iterator iter = traj_points.begin() + start;
  int total_num = end - start - 1;
  for (int i = 0; i < total_num; ++i) {
    TrajectoryPoint mid_point;
    mid_point = 0.5 * (*iter) + 0.5 * (*(iter + 1));
    // note that eular angle won't work
    Vec2d a(cos(iter->path_point.theta), sin(iter->path_point.theta));
    Vec2d b(cos((iter + 1)->path_point.theta),
            sin((iter + 1)->path_point.theta));
    mid_point.path_point.theta = (a + b).Angle();
    iter++;
    iter = traj_points.insert(iter, mid_point);
    iter++;
    if (iter >= traj_points.end() - 1)
      break;
  }
}

void smooth_v(std::vector<TrajectoryPoint> &traj_points, double step_size,
              double max_acc, int start, int end, bool is_invert) {
  if (start == end)
    return;
  if (start < 0 || end < 0 || start > end || end > traj_points.size()) {
    // throw std::invalid_argument("[smooth_v] of traj points");
  }
  double d_v_max = step_size * max_acc;
  if (is_invert) {
    double v = traj_points.at(end - 1).v;
    double d_v = 0;
    for (int i = end - 1; i >= start; --i) {
      d_v = traj_points.at(i).v - v;
      v += std::max(-d_v_max, std::min(d_v, d_v_max));
      traj_points.at(i).a =
          std::max(-max_acc, std::min(traj_points.at(i).a, max_acc));
      traj_points.at(i).v = v;
    }
  } else {
    double v = traj_points.at(start).v;
    double d_v = 0;
    for (int i = start; i < end; ++i) {
      d_v = traj_points.at(i).v - v;
      v += std::max(-d_v_max, std::min(d_v, d_v_max));
      traj_points.at(i).a =
          std::max(-max_acc, std::min(traj_points.at(i).a, max_acc));
      traj_points.at(i).v = v;
    }
  }
}

// std::vector<TrajectoryPoint> generate_deccelerate_segment(TrajectoryPoint&
// start, TrajectoryPoint& target_state, double acc, double dt)
// {
//   Pose2D target_pose;
//   target_pose.x = target_state.path_point.x;
//   target_pose.y = target_state.path_point.y;
//   target_pose.theta =  target_state.path_point.theta;

//   double vel = start.v;

//   std::vector<TrajectoryPoint> path_segment_decc;

//   auto decc_init_pose = target_pose;
//   decc_init_pose.x = target_pose.x - s_path_segment_decc * mid_cos;
//   decc_init_pose.y = target_pose.y - s_path_segment_decc * mid_sin;
//   decc_init_pose.theta = target_pose.theta;

//   int point_num = std::ceil(vel / acc / dt);
//   for(int i = 0; i < point_num; i++){
//       double t = i * dt;
//       double s = vel * t - acc * t * t / 2.0;
//       TrajectoryPoint pp(
//         decc_init_pose.x + s * mid_cos, decc_init_pose.y + s * mid_sin, 0.0,
//         decc_init_pose.theta, 0.0, (vel - acc * t) * mid_direction, -acc *
//         mid_direction, 0.0, 0.0
//       );
//       path_segment_decc.push_back(pp);
//       // std::cout << "openspaceadd: " << pp.path_point.x << ", " <<
//       pp.path_point.y << ", " <<  pp.path_point.theta << std::endl;
//   }
//   TrajectoryPoint pp(
//     target_pose.x, target_pose.y, 0.0, target_pose.theta, 0.0, 0.0,
//     -acc * mid_direction, 0.0, 0.0
//   );
//   path_segment_decc.push_back(pp);
// }
void apply_trajpoints_curvature(std::vector<TrajectoryPoint> &traj_points_) {
  if (traj_points_.size() <= 2) {
    return;
  }
  for (size_t i = 1; i < traj_points_.size(); i++) {
    double theta1 = traj_points_[i - 1].path_point.theta;
    double theta2 = traj_points_[i].path_point.theta;
    double deltas_x =
        traj_points_[i].path_point.x - traj_points_[i - 1].path_point.x;
    double deltas_y =
        traj_points_[i].path_point.y - traj_points_[i - 1].path_point.y;
    traj_points_[i - 1].path_point.rho =
        std::hypot(deltas_x, deltas_y) /
        (2 * sin(planning_math::NormalizeAngle(theta2 - theta1) / 2.0));
    traj_points_[i - 1].path_point.kappa =
        1.0 / traj_points_[i - 1].path_point.rho;
  }
  traj_points_[traj_points_.size() - 1].path_point.rho =
      traj_points_[traj_points_.size() - 2].path_point.rho;
  traj_points_[traj_points_.size() - 1].path_point.kappa =
      traj_points_[traj_points_.size() - 2].path_point.kappa;
}

PathPoint operator+(PathPoint a, PathPoint b) {
  PathPoint result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  result.theta = a.theta + b.theta;
  result.s = a.s + b.s;
  result.l = a.l + b.l;
  result.x_derivative = a.x_derivative + b.x_derivative;
  result.x_derivative = a.y_derivative + b.y_derivative;
  result.kappa = a.kappa + b.kappa;
  result.dkappa = a.dkappa + b.dkappa;
  result.ddkappa = a.ddkappa + b.ddkappa;
  result.rho = a.rho + b.rho;
  return result;
}

TrajectoryPoint operator+(TrajectoryPoint a, TrajectoryPoint b) {
  TrajectoryPoint result;
  result.path_point = a.path_point + b.path_point;
  result.v = a.v + b.v;
  result.a = a.a + b.a;
  result.steer = a.steer + b.steer;
  result.relative_time = a.relative_time + b.relative_time;
  result.velocity_direction = a.velocity_direction + b.velocity_direction;
  return result; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
}

void link_target(std::vector<TrajectoryPoint> &traj,
                 const TrajectoryPoint &target_state, double acc, double vel,
                 double ds) {
  if (traj.empty()) {
    return;
  }
  Pose2D target_pose;
  target_pose.x = target_state.path_point.x;
  target_pose.y = target_state.path_point.y;
  target_pose.theta = target_state.path_point.theta;

  auto mid_pose = traj.back().path_point;

  auto diff = mid_pose;
  diff.x = target_pose.x - mid_pose.x;
  diff.y = target_pose.y - mid_pose.y;
  diff.theta = 0;

  double diff_len = std::max(std::hypot(diff.x, diff.y), 1e-6);
  double diff_cos = diff.x / diff_len;
  double diff_sin = diff.y / diff_len;
  double diff_theta = atan2(diff_sin, diff_cos);
  double a = planning_math::NormalizeAngle(diff_theta - mid_pose.theta);
  int mid_direction = 0;
  if (std::abs(a) > (M_PI / 2))
    mid_direction = -1;
  else
    mid_direction = 1;
  double theta = mid_direction > 0
                     ? diff_theta
                     : planning_math::NormalizeAngle(diff_theta + M_PI);

  double s_path_segment_decc = vel * vel / (2 * acc);

  std::vector<TrajectoryPoint> path_segment_decc;

  auto decc_init_pose = target_pose;
  decc_init_pose.x = target_pose.x - s_path_segment_decc * diff_cos;
  decc_init_pose.y = target_pose.y - s_path_segment_decc * diff_sin;
  decc_init_pose.theta = target_pose.theta;

  int point_num = std::ceil(s_path_segment_decc / ds);
  for (int i = 0; i < point_num; i++) {
    double s = i * ds;
    double tmp_vel = std::sqrt(vel * vel - 2 * acc * s);
    TrajectoryPoint pp(decc_init_pose.x + s * diff_cos,
                       decc_init_pose.y + s * diff_sin, 0.0, theta, 0.0,
                       tmp_vel * mid_direction, -acc * mid_direction, 0.0, 0.0);
    path_segment_decc.push_back(pp);
  }

  std::vector<TrajectoryPoint> path_segment_const_v;
  Pose2D total_diff_pose;
  total_diff_pose.x = target_pose.x - mid_pose.x;
  total_diff_pose.y = target_pose.y - mid_pose.y;
  total_diff_pose.theta = 0;
  double s_total = std::hypot(total_diff_pose.x, total_diff_pose.y);
  double s_path_segment_const_v = s_total - s_path_segment_decc;
  if (s_path_segment_const_v < 0.0) {
    path_segment_decc.clear();
  } else {
    int point_num = ceil(s_path_segment_const_v / ds);
    for (int i = 1; i < point_num; i++) {
      double s = i * ds;
      TrajectoryPoint pp(mid_pose.x + s * diff_cos, mid_pose.y + s * diff_sin,
                         0.0, theta, 0.0, vel * mid_direction, 0.0, 0.0, 0.0);
      path_segment_const_v.push_back(pp);
    }
  }

  traj.insert(traj.end(), path_segment_const_v.begin(),
              path_segment_const_v.end());
  traj.insert(traj.end(), path_segment_decc.begin(), path_segment_decc.end());
  TrajectoryPoint &tmp = traj.back();
  if (std::hypot(target_state.path_point.x - tmp.path_point.x,
                 target_state.path_point.y - tmp.path_point.y) < ds) {
    traj.pop_back();
  }
  TrajectoryPoint pp(target_state);
  pp.v = vel * mid_direction;
  pp.a = -acc * mid_direction;
  traj.push_back(pp);
}

void extract_last_segment(const std::vector<TrajectoryPoint> &traj,
                          std::vector<TrajectoryPoint> &last_back_seg) {
  if (traj.size() < 2) {
    return;
  }
  if (traj.back().v > 0) {
    return;
  }
  int i = traj.size() - 2;
  for (; i > -1; --i) {
    if (traj.at(i).v > 0) {
      break;
    }
  }
  last_back_seg =
      std::vector<TrajectoryPoint>(traj.begin() + i + 1, traj.end());
}

void lengthen_or_shorten(std::vector<TrajectoryPoint> &traj,
                         const planning_math::Vec2d stopper_pt1,
                         const planning_math::Vec2d stopper_pt2,
                         const double wheel_stop_buffer) {
  if (traj.size() < 2) {
    return;
  }

  planning_math::Vec2d back_unit_vec(cos(traj.back().path_point.theta),
                                     sin(traj.back().path_point.theta));
  planning_math::Vec2d back_to_pt_vec1(
      stopper_pt1.x() - traj.back().path_point.x,
      stopper_pt1.y() - traj.back().path_point.y);
  double length_diff1 =
      -back_to_pt_vec1.InnerProd(back_unit_vec) - wheel_stop_buffer;
  planning_math::Vec2d back_to_pt_vec2(
      stopper_pt2.x() - traj.back().path_point.x,
      stopper_pt2.y() - traj.back().path_point.y);
  double length_diff2 =
      -back_to_pt_vec2.InnerProd(back_unit_vec) - wheel_stop_buffer;
  double length_diff = std::min(length_diff1, length_diff2);
  if (std::abs(length_diff) < 0.02) {
    return;
  }

  double accumulated_s = 0.0;
  if (length_diff < 0) {
    while (traj.size() > 1) {
      accumulated_s += std::hypot(
          traj.back().path_point.x - traj[traj.size() - 2].path_point.x,
          traj.back().path_point.y - traj[traj.size() - 2].path_point.y);
      traj.pop_back();
      if (accumulated_s >= std::abs(length_diff)) {
        break;
      }
    }
    TrajectoryPoint last_point;
    last_point.path_point.x =
        traj.back().path_point.x - (accumulated_s - std::abs(length_diff)) *
                                       cos(traj.back().path_point.theta);
    last_point.path_point.y =
        traj.back().path_point.y - (accumulated_s - std::abs(length_diff)) *
                                       sin(traj.back().path_point.theta);
    last_point.path_point.theta = traj.back().path_point.theta;
    last_point.v = -0.5;
    traj.push_back(last_point);
  } else {
    while (accumulated_s < length_diff) {
      accumulated_s += 0.1;
      TrajectoryPoint new_point;
      new_point.path_point.x =
          traj.back().path_point.x - 0.1 * cos(traj.back().path_point.theta);
      new_point.path_point.y =
          traj.back().path_point.y - 0.1 * sin(traj.back().path_point.theta);
      new_point.path_point.theta = traj.back().path_point.theta;
      new_point.v = -0.5;
      traj.push_back(new_point);
    }
    traj.pop_back();
    accumulated_s -= 0.1;
    TrajectoryPoint last_point;
    last_point.path_point.x =
        traj.back().path_point.x -
        (length_diff - accumulated_s) * cos(traj.back().path_point.theta);
    last_point.path_point.y =
        traj.back().path_point.y -
        (length_diff - accumulated_s) * sin(traj.back().path_point.theta);
    last_point.path_point.theta = traj.back().path_point.theta;
    last_point.v = -0.5;
    traj.push_back(last_point);
  }
}

double calc_dist_to_stopper(const Pose2D &curr_ego_pose,
                            const std::vector<TrajectoryPoint> &trajectory,
                            const double wheel_stop_buffer) {
  double dist_to_stopper = 2.0;

  if (trajectory.empty()) {
    return dist_to_stopper;
  } else {
    std::vector<double> dist_egopose_to_traj;
    for (size_t j = 0; j < trajectory.size(); ++j) {
      dist_egopose_to_traj.emplace_back(
          std::hypot(curr_ego_pose.x - trajectory.at(j).path_point.x,
                     curr_ego_pose.y - trajectory.at(j).path_point.y));
    }

    std::vector<double>::iterator closest_point = std::min_element(
        dist_egopose_to_traj.begin(), dist_egopose_to_traj.end());
    int index_closest_point =
        std::distance(dist_egopose_to_traj.begin(), closest_point);
    Pose2D point1 = Pose2D(trajectory.at(index_closest_point).path_point.x,
                           trajectory.at(index_closest_point).path_point.y,
                           trajectory.at(index_closest_point).path_point.theta);
    Pose2D point2;

    if ((closest_point + 1) == dist_egopose_to_traj.end()) {
      return dist_to_stopper;
    } else {
      if (closest_point == dist_egopose_to_traj.begin() ||
          *(closest_point + 1) < *(closest_point - 1)) {
        point2 =
            Pose2D(trajectory.at(index_closest_point + 1).path_point.x,
                   trajectory.at(index_closest_point + 1).path_point.y,
                   trajectory.at(index_closest_point + 1).path_point.theta);
        auto projection_point =
            planning_math::calc_projection_point(point1, point2, curr_ego_pose);
        double dist_proj_to_end = std::hypot(projection_point.x - point2.x,
                                             projection_point.y - point2.y);
        for (int i = index_closest_point + 1;
             i < dist_egopose_to_traj.size() - 1; i++) {
          dist_proj_to_end += std::hypot(trajectory.at(i).path_point.x -
                                             trajectory.at(i + 1).path_point.x,
                                         trajectory.at(i).path_point.y -
                                             trajectory.at(i + 1).path_point.y);
        }
        dist_to_stopper = dist_proj_to_end + wheel_stop_buffer;
      } else {
        point2 =
            Pose2D(trajectory.at(index_closest_point - 1).path_point.x,
                   trajectory.at(index_closest_point - 1).path_point.y,
                   trajectory.at(index_closest_point - 1).path_point.theta);
        auto projection_point =
            planning_math::calc_projection_point(point1, point2, curr_ego_pose);
        double dist_proj_to_end = std::hypot(projection_point.x - point2.x,
                                             projection_point.y - point2.y);
        for (int i = index_closest_point; i < dist_egopose_to_traj.size() - 1;
             i++) {
          dist_proj_to_end += std::hypot(trajectory.at(i).path_point.x -
                                             trajectory.at(i + 1).path_point.x,
                                         trajectory.at(i).path_point.y -
                                             trajectory.at(i + 1).path_point.y);
        }
        dist_to_stopper = dist_proj_to_end + wheel_stop_buffer;
      }
    }
  }
  return dist_to_stopper;
}

} // namespace msquare