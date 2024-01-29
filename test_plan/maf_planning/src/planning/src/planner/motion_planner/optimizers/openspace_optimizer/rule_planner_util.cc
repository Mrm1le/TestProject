#include "planner/motion_planner/optimizers/openspace_optimizer/rule_planner_util.h"

namespace msquare {

planning_math::Vec2d MirrorInjection(const planning_math::Vec2d &point) {
  return planning_math::Vec2d(-point.x(), point.y());
}

planning_math::LineSegment2d
MirrorInjection(const planning_math::LineSegment2d &line) {
  return planning_math::LineSegment2d(
      planning_math::Vec2d(-line.end().x(), line.end().y()),
      planning_math::Vec2d(-line.start().x(), line.start().y()));
}

planning_math::Box2d MirrorInjection(const planning_math::Box2d &box) {
  return planning_math::Box2d(
      planning_math::Vec2d(-box.center_x(), box.center_y()),
      planning_math::NormalizeAngle(M_PI - box.heading()), box.length(),
      box.width());
}

Pose2D MirrorInjection(const Pose2D &pose) {
  return Pose2D(-pose.x, pose.y,
                planning_math::NormalizeAngle(M_PI - pose.theta));
}

void addInflationPoints(std::vector<planning_math::Vec2d> &points,
                        const planning_math::Vec2d &p,
                        double inflation_for_points) {
  int extra_num = 8;
  double delta_theta = 2 * M_PI / extra_num;
  points.emplace_back(p);
  if (inflation_for_points < 2e-6) {
    return;
  }
  for (int i = 0; i < extra_num; i++) {
    double relative_theta = delta_theta * i;
    points.emplace_back(p.x() + inflation_for_points * cos(relative_theta),
                        p.y() + inflation_for_points * sin(relative_theta));
  }
}

void getEPCorners(const Pose2D &pose,
                  std::vector<planning_math::Vec2d> &corners,
                  std::vector<planning_math::LineSegment2d> &lines,
                  double lon_change, double lat_change) {
  // all six corners
  corners.clear();
  lines.clear();

  double half_width = VehicleParam::Instance()->width_wo_rearview_mirror / 2 +
                      CarParams::GetInstance()->lat_inflation();
  double front_to_rear = VehicleParam::Instance()->front_edge_to_center +
                         CarParams::GetInstance()->lat_inflation();
  double front_corner_width = VehicleParam::Instance()->bumper_length / 2.0 +
                              CarParams::GetInstance()->lat_inflation();
  double front_corner_length = VehicleParam::Instance()->front_edge_to_center -
                               VehicleParam::Instance()->light_to_front_edge;

  double back_to_rear = VehicleParam::Instance()->back_edge_to_center +
                        CarParams::GetInstance()->lat_inflation();
  double back_corner_length = back_to_rear - 0.4;
  double back_corner_width = half_width - 0.3;

  double temp_half_width = half_width + lat_change;
  double temp_front_corner_width = front_corner_width + lat_change;
  double temp_front_to_rear = front_to_rear + lon_change;
  double temp_back_to_rear = back_to_rear + lon_change;
  double temp_back_corner_width = back_corner_width + lat_change;
  corners.emplace_back(pose.x + temp_front_to_rear * cos(pose.theta) -
                           temp_front_corner_width * sin(pose.theta),
                       pose.y + temp_front_to_rear * sin(pose.theta) +
                           temp_front_corner_width * cos(pose.theta));
  corners.emplace_back(pose.x + front_corner_length * cos(pose.theta) -
                           temp_half_width * sin(pose.theta),
                       pose.y + front_corner_length * sin(pose.theta) +
                           temp_half_width * cos(pose.theta));

  corners.emplace_back(pose.x - back_corner_length * cos(pose.theta) -
                           temp_half_width * sin(pose.theta),
                       pose.y - back_corner_length * sin(pose.theta) +
                           temp_half_width * cos(pose.theta));
  corners.emplace_back(pose.x - temp_back_to_rear * cos(pose.theta) -
                           temp_back_corner_width * sin(pose.theta),
                       pose.y - temp_back_to_rear * sin(pose.theta) +
                           temp_back_corner_width * cos(pose.theta));
  corners.emplace_back(pose.x - temp_back_to_rear * cos(pose.theta) +
                           temp_back_corner_width * sin(pose.theta),
                       pose.y - temp_back_to_rear * sin(pose.theta) -
                           temp_back_corner_width * cos(pose.theta));
  corners.emplace_back(pose.x - back_corner_length * cos(pose.theta) +
                           temp_half_width * sin(pose.theta),
                       pose.y - back_corner_length * sin(pose.theta) -
                           temp_half_width * cos(pose.theta));

  corners.emplace_back(pose.x + front_corner_length * cos(pose.theta) +
                           temp_half_width * sin(pose.theta),
                       pose.y + front_corner_length * sin(pose.theta) -
                           temp_half_width * cos(pose.theta));
  corners.emplace_back(pose.x + temp_front_to_rear * cos(pose.theta) +
                           temp_front_corner_width * sin(pose.theta),
                       pose.y + temp_front_to_rear * sin(pose.theta) -
                           temp_front_corner_width * cos(pose.theta));

  for (int i = 0; i < corners.size(); ++i) {
    lines.emplace_back(corners.at(i), corners.at((i + 1) % (corners.size())));
  }
}

std::vector<double> calcMaxTheta(const planning_math::Vec2d &arc_center,
                                 const planning_math::LineSegment2d &line,
                                 const planning_math::Vec2d &point) {
  double dist_line = line.DistanceTo(arc_center);
  double dist = arc_center.DistanceTo(point);
  std::vector<double> theta;

  if (dist < dist_line) {
    return theta;
  }

  double x1 = line.start().x(), y1 = line.start().y();
  double x2 = line.end().x(), y2 = line.end().y();

  double A = std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2);
  double B = 2 * (x2 - arc_center.x()) * (x1 - x2) +
             2 * (y2 - arc_center.y()) * (y1 - y2);
  double C = std::pow(x2 - arc_center.x(), 2) +
             std::pow(y2 - arc_center.y(), 2) - std::pow(dist, 2);

  double lambda_1 = (-B + sqrt(std::pow(B, 2) - 4 * A * C)) / (2 * A);
  double lambda_2 = (-B - sqrt(std::pow(B, 2) - 4 * A * C)) / (2 * A);
  if (lambda_1 > 0 && lambda_1 < 1.0) {
    double lambda = lambda_1;
    double new_x = lambda * x1 + (1 - lambda) * x2;
    double new_y = lambda * y1 + (1 - lambda) * y2;

    planning_math::Vec2d vec_O_origin(point.x() - arc_center.x(),
                                      point.y() - arc_center.y());
    planning_math::Vec2d vec_O_new(new_x - arc_center.x(),
                                   new_y - arc_center.y());
    int clockwise = vec_O_origin.CrossProd(vec_O_new) > 0 ? 1 : -1;
    double cos_value = std::max(
        -1.0, std::min(vec_O_origin.InnerProd(vec_O_new) /
                           (vec_O_origin.Length() * vec_O_new.Length()),
                       1.0));
    double theta1 = clockwise * acos(cos_value);
    theta.emplace_back(theta1);
  }
  if (lambda_2 > 0 && lambda_2 < 1.0) {
    double lambda = lambda_2;
    double new_x = lambda * x1 + (1 - lambda) * x2;
    double new_y = lambda * y1 + (1 - lambda) * y2;

    planning_math::Vec2d vec_O_origin(point.x() - arc_center.x(),
                                      point.y() - arc_center.y());
    planning_math::Vec2d vec_O_new(new_x - arc_center.x(),
                                   new_y - arc_center.y());
    int clockwise = vec_O_origin.CrossProd(vec_O_new) > 0 ? 1 : -1;
    double cos_value = std::max(
        -1.0, std::min(vec_O_origin.InnerProd(vec_O_new) /
                           (vec_O_origin.Length() * vec_O_new.Length()),
                       1.0));
    double theta2 = clockwise * acos(cos_value);
    theta.emplace_back(theta2);
  }

  return theta;
}

Pose2D calcNextPose(const std::vector<planning_math::LineSegment2d> obstacles,
                    const std::vector<planning_math::Vec2d> points_of_obstacles,
                    const Pose2D init_pose, const Pose2D &start_pose,
                    int steer_direction, int travel_direction, double radius) {
  bool is_init_pose = fabs(start_pose.x - init_pose.x) < 1e-6 &&
                      fabs(start_pose.y - init_pose.y) < 1e-6 &&
                      fabs(start_pose.theta - init_pose.theta) < 1e-6;
  // get center
  double x_O = start_pose.x - steer_direction * radius * sin(start_pose.theta);
  double y_O = start_pose.y + steer_direction * radius * cos(start_pose.theta);
  planning_math::Vec2d center(x_O, y_O);

  std::vector<planning_math::Vec2d> ego_corners;
  std::vector<planning_math::LineSegment2d> ego_lines;
  getEPCorners(start_pose, ego_corners, ego_lines);
  planning_math::Polygon2d temp_polygon;
  temp_polygon.update(ego_corners);

  int is_counter_clockwise = steer_direction * travel_direction;
  double steer_central_angle = is_counter_clockwise * M_PI;
  const double kSafetyThre =
      std::max(1e-6, (CarParams::GetInstance()->lon_inflation() -
                      CarParams::GetInstance()->lat_inflation()) /
                         radius);
  // calc all obs with ego corners
  for (auto corner : ego_corners) {
    for (auto obs_line : obstacles) {
      if (is_init_pose && temp_polygon.HasOverlap(obs_line)) {
        continue;
      }
      std::vector<double> theta_vec = calcMaxTheta(center, obs_line, corner);
      for (double max_theta_to_obs : theta_vec) {
        if (is_counter_clockwise * max_theta_to_obs < 0) {
          // TODO(lizhiqiang): std::abs(max_theta_to_obs) < kSafetyThre
          // deprecated
        } else if (std::abs(max_theta_to_obs) <= kSafetyThre ||
                   std::isnan(max_theta_to_obs)) {
          steer_central_angle = 0.0;
        } else if (is_counter_clockwise > 0) {
          steer_central_angle =
              std::min(steer_central_angle, max_theta_to_obs - kSafetyThre);
        } else {
          steer_central_angle =
              std::max(steer_central_angle, max_theta_to_obs + kSafetyThre);
        }
      }
    }
  }
  // calc all ego edges with obs
  for (auto obs_point : points_of_obstacles) {
    if (is_init_pose && temp_polygon.IsPointIn(obs_point)) {
      continue;
    }
    for (auto ego_edge : ego_lines) {
      std::vector<double> theta_vec = calcMaxTheta(center, ego_edge, obs_point);
      for (double max_theta_to_obs : theta_vec) {
        max_theta_to_obs = -max_theta_to_obs;
        if (is_counter_clockwise * max_theta_to_obs < 0) {
          // TODO(lizhiqiang): std::abs(max_theta_to_obs) < kSafetyThre
          // deprecated
        } else if (std::abs(max_theta_to_obs) <= kSafetyThre ||
                   std::isnan(max_theta_to_obs)) {
          steer_central_angle = 0.0;
        } else if (is_counter_clockwise > 0) {
          steer_central_angle =
              std::min(steer_central_angle, max_theta_to_obs - kSafetyThre);
        } else {
          steer_central_angle =
              std::max(steer_central_angle, max_theta_to_obs + kSafetyThre);
        }
      }
    }
  }
  double min_step_size =
      std::max(msquare::HybridAstarConfig::GetInstance()->step_size,
               CarParams::GetInstance()->lon_inflation() - 0.2);
  steer_central_angle = (std::abs(steer_central_angle) > min_step_size / radius)
                            ? steer_central_angle
                            : 0.0;
  double next_theta = start_pose.theta + steer_central_angle;
  double next_x = x_O + steer_direction * radius * sin(next_theta);
  double next_y = y_O - steer_direction * radius * cos(next_theta);
  return Pose2D(next_x, next_y, next_theta);
}

bool checkStraightLine(
    const std::vector<planning_math::LineSegment2d> obstacles,
    const std::vector<planning_math::Vec2d> points_of_obstacles,
    const Pose2D &start_pose, const Pose2D &end_pose) {
  planning_math::Vec2d straight_line{end_pose.x - start_pose.x,
                                     end_pose.y - start_pose.y};
  planning_math::Vec2d direction =
      planning_math::Vec2d::CreateUnitVec2d(start_pose.theta);

  static planning_math::Polygon2d ego_polygon;
  static std::vector<planning_math::Vec2d> ego_corners(8);
  static std::vector<planning_math::Vec2d> start_corners(8);
  static std::vector<planning_math::Vec2d> end_corners(8);
  static std::vector<planning_math::LineSegment2d> lines(8);
  static planning_math::Polygon2d start_polygon;

  getEPCorners(start_pose, start_corners, lines);
  start_polygon.update(start_corners);
  getEPCorners(end_pose, end_corners, lines);

  if (direction.InnerProd(straight_line) > 0.0) {
    // start rear points
    ego_corners[2] = start_corners[2];
    ego_corners[3] = start_corners[3];
    ego_corners[4] = start_corners[4];
    ego_corners[5] = start_corners[5];
    // end front points
    ego_corners[0] = end_corners[0];
    ego_corners[1] = end_corners[1];
    ego_corners[6] = end_corners[6];
    ego_corners[7] = end_corners[7];
  } else {
    // start rear points
    ego_corners[2] = end_corners[2];
    ego_corners[3] = end_corners[3];
    ego_corners[4] = end_corners[4];
    ego_corners[5] = end_corners[5];
    // end front points
    ego_corners[0] = start_corners[0];
    ego_corners[1] = start_corners[1];
    ego_corners[6] = start_corners[6];
    ego_corners[7] = start_corners[7];
  }
  ego_polygon.update(ego_corners);
  for (auto &line_segment : obstacles) {
    if (start_polygon.HasOverlap(line_segment)) {
      continue;
    }
    if (ego_polygon.HasOverlap(line_segment)) {
      return false;
    }
  }
  for (auto &p : points_of_obstacles) {
    if (start_polygon.IsPointIn(p)) {
      continue;
    }
    if (ego_polygon.IsPointIn(p)) {
      return false;
    }
  }
  return true;
}

bool isRSPathSafe(const std::vector<planning_math::LineSegment2d> obstacles,
                  const std::vector<planning_math::Vec2d> points_of_obstacles,
                  const Pose2D init_pose,
                  const std::vector<Pose2D> &path_key_points) {
  if (path_key_points.size() < 2) {
    return false;
  }
  for (int i = 0; i + 1 < path_key_points.size(); ++i) {
    if (fabs(path_key_points[i].theta - path_key_points[i + 1].theta) < 1e-3) {
      // straight
      if (!checkStraightLine(obstacles, points_of_obstacles, path_key_points[i],
                             path_key_points[i + 1])) {
        return false;
      }
    } else {
      // curve
      planning_math::Vec2d vec_former_latter(
          path_key_points[i + 1].x - path_key_points[i].x,
          path_key_points[i + 1].y - path_key_points[i].y);
      double vec_heading = vec_former_latter.Angle();
      double turning_radius = sqrt(
          std::pow(vec_former_latter.Length(), 2) / 2 /
          (1 - cos(path_key_points[i].theta - path_key_points[i + 1].theta)));
      int towards =
          fabs(vec_heading - path_key_points[i].theta) < M_PI_2 ? 1 : -1;
      int is_counter =
          path_key_points[i + 1].theta > path_key_points[i].theta ? 1 : -1;
      int steer = towards * is_counter;

      Pose2D limit_pose =
          calcNextPose(obstacles, points_of_obstacles, init_pose,
                       path_key_points[i], steer, towards, turning_radius);
      if (is_counter > 0 && limit_pose.theta < path_key_points[i + 1].theta) {
        return false;
      }
      if (is_counter < 0 && limit_pose.theta > path_key_points[i + 1].theta) {
        return false;
      }
    }
  }
  return true;
}

std::vector<Pose2D> InterpolatePath(double step,
                                    const std::vector<Pose2D> key_points) {
  std::vector<Pose2D> path;

  double veh_min_r = CarParams::GetInstance()->min_turn_radius;
  double angular_step = atan(step / veh_min_r);
  for (int i = 0; i + 1 < key_points.size(); ++i) {
    if (fabs(key_points[i].theta - key_points[i + 1].theta) < 1e-3) {
      // straight
      path.push_back(key_points[i]);
      double seg_length = std::hypot(key_points[i + 1].x - key_points[i].x,
                                     key_points[i + 1].y - key_points[i].y);
      // std::cout << "seg_length: " << seg_length << std::endl;
      int seg_num = floor(seg_length / step);
      double dx = (key_points[i + 1].x - key_points[i].x) / seg_length;
      double dy = (key_points[i + 1].y - key_points[i].y) / seg_length;

      for (int j = 1; j <= seg_num; ++j) {
        path.emplace_back(key_points[i].x + j * dx * step,
                          key_points[i].y + j * dy * step, key_points[i].theta);
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
          sqrt(std::pow(vec_former_latter.Length(), 2) / 2 /
               (1 - cos(key_points[i].theta - key_points[i + 1].theta)));
      angular_step = atan(step / turning_radius);
      int towards = fabs(vec_heading - key_points[i].theta) < M_PI_2 ? 1 : -1;
      int is_counter = key_points[i + 1].theta > key_points[i].theta ? 1 : -1;
      int steer = towards * is_counter;

      double center_x =
          key_points[i].x - steer * turning_radius * sin(key_points[i].theta);
      double center_y =
          key_points[i].y + steer * turning_radius * cos(key_points[i].theta);

      int seg_num = floor(fabs(key_points[i + 1].theta - key_points[i].theta) /
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
  return path;
}

msquare::SbpResult convertToSbpResult(const std::vector<Pose2D> &traj) {
  msquare::SbpResult result = {};
  for (auto pt : traj) {
    result.x.push_back(pt.x);
    result.y.push_back(pt.y);
    result.phi.push_back(pt.theta);
  }
  return result;
}

} // namespace msquare