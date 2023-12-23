#include "common/math/curve_join.h"
#include "common/math/math_utils.h"

namespace msquare {

namespace {
  constexpr double kMathEpsilonTemp = 1e-6;
}

bool CurveJoin(const Pose2D &start, const Pose2D &target, const double r_min,
               const double step_size, std::vector<Pose2D> &path) {
  Pose2D start_local = planning_math::tf2d(target, start);
  Pose2D target_local{0.0, 0.0, 0.0};
  double dx = start_local.x - target_local.x;
  double dy = start_local.y - target_local.y;
  // this is a pseudo dtheta
  double dtheta =
      planning_math::NormalizeAngle(start_local.theta - target_local.theta);
  std::vector<Pose2D> key_points;
  key_points.emplace_back(start_local);
  bool pre_requirement = dx > 0.0;
  bool success{false};
  const double kMathEpsilon = 1e-3;
  if (pre_requirement) {
    // dtheta is small enough(<kMathEpsilon), dtheta ~= 0.04 = 2.95degree
    if (std::abs(1 - std::cos(dtheta)) < kMathEpsilon) {
      // dy is small enough
      if (std::abs(dy) < kMathEpsilon) {
        double r = r_min;
        double sign = dtheta > kMathEpsilonTemp ? 1.0 : -1.0;
        Pose2D via_point{start_local.x - sign * r * sin(dtheta),
                         start_local.y - sign * r * (1 - std::cos(dtheta)),
                         target_local.theta};
        key_points.emplace_back(via_point);
        key_points.emplace_back(target_local);
        success = true;
      }
    } else {
      double r = dy / (1 - std::cos(dtheta));
      if ((std::abs(r) > r_min) &&
          (start_local.x - r * std::sin(dtheta) > target_local.x + 0.5) &&
          (r * std::sin(dtheta) > 0.0)) {
        Pose2D via_point{start_local.x - r * sin(dtheta), target_local.y,
                         target_local.theta};
        key_points.emplace_back(via_point);
        key_points.emplace_back(target_local);
        success = true;
      }
    }
    if (!success) {
      std::vector<double> sgn_list{1.0, -1.0};
      for (auto sgn : sgn_list) {
        for (double ratio = 5.0; ratio > 0.99; ratio -= 0.5) {
          double x_o1 =
              start_local.x + sgn * ratio * r_min * std::sin(start_local.theta);
          double y_o1 =
              start_local.y - sgn * ratio * r_min * std::cos(start_local.theta);
          double y_o2 = target_local.y + sgn * ratio * r_min;
          double delta_y = y_o1 - y_o2;
          if (std::abs(delta_y) < 2 * ratio * r_min) {
            double theta1 = (std::acos(std::abs(delta_y) / (2 * ratio * r_min))) * sgn;
            double x2 = x_o1 - sgn * ratio * r_min * std::sin(theta1) -
                        sgn * ratio * r_min * std::sin(theta1);
            if ((x2 > target_local.x + 1.0) &&
                (x_o1 - sgn * ratio * r_min * std::sin(theta1) > x2) &&
                (x_o1 - sgn * ratio * r_min * std::sin(theta1) < start_local.x)) {
              key_points.emplace_back(
                  Pose2D{x_o1 - sgn * ratio * r_min * std::sin(theta1),
                         y_o1 + sgn * ratio * r_min * std::cos(theta1), theta1});
              key_points.emplace_back(
                  Pose2D{x2, target_local.y, target_local.theta});
              key_points.emplace_back(target_local);
              success = true;
              break;
            }
          }
        }
        if (success)
          break;
      }
    }
  }
  if (success) {
    SmoothCurve(key_points, step_size, path);
  }
  for (int i = 0; i < path.size(); i++) {
    path[i] = planning_math::tf2d_inv(target, path[i]);
  }
  return success;
}

void SmoothCurve(const std::vector<Pose2D> &key_points, const double step_size,
                 std::vector<Pose2D> &path) {
  for (int i = 0; i + 1 < key_points.size(); ++i) {
    if (std::abs(key_points[i].theta - key_points[i + 1].theta) < 1e-5) {
      // straight
      path.push_back(key_points[i]);
      double seg_length = std::hypot(key_points[i + 1].x - key_points[i].x,
                                     key_points[i + 1].y - key_points[i].y);
      int seg_num = std::floor(seg_length / step_size);
      double dx = (key_points[i + 1].x - key_points[i].x) / seg_length;
      double dy = (key_points[i + 1].y - key_points[i].y) / seg_length;

      for (int j = 1; j <= seg_num; ++j) {
        path.emplace_back(key_points[i].x + j * dx * step_size,
                          key_points[i].y + j * dy * step_size,
                          key_points[i].theta);
      }
    } else {
      // curve
      path.push_back(key_points[i]);

      // judge backwoards or forwards
      planning_math::Vec2d vec_former_latter(
          key_points[i + 1].x - key_points[i].x,
          key_points[i + 1].y - key_points[i].y);
      double vec_heading = vec_former_latter.Angle();
      double turning_radius =
          std::sqrt(std::pow(vec_former_latter.Length(), 2) / 2 /
                    (1 - cos(key_points[i].theta - key_points[i + 1].theta)));
      int towards =
          std::abs(vec_heading - key_points[i].theta) < M_PI_2 ? 1 : -1;
      int is_counter = key_points[i + 1].theta > key_points[i].theta ? 1 : -1;
      int steer = towards * is_counter;

      double angular_step = std::atan(step_size / 2 / turning_radius);

      double center_x =
          key_points[i].x - steer * turning_radius * sin(key_points[i].theta);
      double center_y =
          key_points[i].y + steer * turning_radius * cos(key_points[i].theta);

      int seg_num =
          std::floor(std::abs(key_points[i + 1].theta - key_points[i].theta) /
                     angular_step);
      for (int j = 1; j <= seg_num; ++j) {
        double inter_theta =
            key_points[i].theta + is_counter * j * angular_step;
        double inter_x = center_x + steer * turning_radius * sin(inter_theta);
        double inter_y = center_y - steer * turning_radius * cos(inter_theta);

        path.emplace_back(inter_x, inter_y, inter_theta);
      }
    }
  }
  path.push_back(key_points.back());
}

} // namespace msquare