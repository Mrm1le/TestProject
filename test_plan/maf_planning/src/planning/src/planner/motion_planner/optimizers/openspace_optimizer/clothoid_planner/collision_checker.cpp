#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/collision_checker.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"

namespace clothoid {
using namespace msquare;

Pose2D CollisionChecker::rotateTheta(const Pose2D &start_pose,
                                     int steer_direction, int travel_direction,
                                     double radius, double theta) {
  double new_theta =
      start_pose.theta +
      (steer_direction * travel_direction > 0.0 ? 1 : -1) * theta;
  double cross_theta = (start_pose.theta + new_theta) / 2.0;

  double dis = 2 * radius * std::sin(theta / 2.0);

  double new_x = start_pose.x +
                 dis * (travel_direction > 0 ? 1 : -1) * std::cos(cross_theta);
  double new_y = start_pose.y +
                 dis * (travel_direction > 0 ? 1 : -1) * std::sin(cross_theta);

  return Pose2D(new_x, new_y, new_theta);
}

std::vector<double>
CollisionChecker::rotateMaxTheta(const planning_math::Vec2d &arc_center,
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

Pose2D CollisionChecker::rotateMaxPose(const Pose2D &ego_pose,
                                       RotateType rotate_type, ShapeType type,
                                       double radius, bool is_init, double lat,
                                       double lon) {
  int steer_direction = (rotate_type == RotateType::LEFT_FORWARD ||
                                 rotate_type == RotateType::LEFT_BCAKWARD
                             ? 1
                             : -1);
  int travel_direction = (rotate_type == RotateType::LEFT_FORWARD ||
                                  rotate_type == RotateType::RIGHT_FORWARD
                              ? 1
                              : -1);
  return rotateMaxPose(ego_pose, steer_direction, travel_direction, type,
                       radius, is_init, lat, lon);
}

Pose2D CollisionChecker::rotateMaxPose(const Pose2D &ego_pose,
                                       int steer_direction,
                                       int travel_direction, ShapeType type,
                                       double radius, bool is_init, double lat,
                                       double lon) {
  // get center
  double x_O = ego_pose.x - steer_direction * radius * sin(ego_pose.theta);
  double y_O = ego_pose.y + steer_direction * radius * cos(ego_pose.theta);
  planning_math::Vec2d center(x_O, y_O);

  std::vector<msquare::planning_math::Vec2d> corners;
  csg_.getCollisionShape(ego_pose, corners, type, lat, lon);

  planning_math::Polygon2d temp_polygon;
  temp_polygon.update(corners);

  int is_counter_clockwise = steer_direction * travel_direction;
  double steer_central_angle = is_counter_clockwise * M_PI_2;
  const double kSafetyThre = 1e-6;

  // calc all obs with ego corners
  for (auto corner : corners) {
    for (auto obs_line : obs_lines_) {
      if (is_init && temp_polygon.HasOverlap(obs_line)) {
        continue;
      }
      std::vector<double> theta_vec = rotateMaxTheta(center, obs_line, corner);
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
  std::vector<planning_math::LineSegment2d> lines;
  csg_.formatLines(corners, lines);
  for (auto obs_point : obs_pts_) {
    if (is_init && temp_polygon.IsPointIn(obs_point)) {
      continue;
    }
    for (auto ego_edge : lines) {
      std::vector<double> theta_vec =
          rotateMaxTheta(center, ego_edge, obs_point);
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

  std::vector<planning_math::Vec2d> axis_corners;
  std::vector<planning_math::LineSegment2d> axis_lines;
  csg_.getWheelBaseShape(ego_pose, axis_corners, lat, lon);
  csg_.formatLines(axis_corners, axis_lines);
  temp_polygon.update(axis_corners);

  // calc dashed obs with ego axis corners
  for (auto corner : axis_corners) {
    for (auto obs_line : step_lines_) {
      if (is_init && temp_polygon.HasOverlap(obs_line)) {
        continue;
      }
      std::vector<double> theta_vec = rotateMaxTheta(center, obs_line, corner);
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
  // calc all ego edges with dashed obs
  for (auto obs_point : step_pts_) {
    if (is_init && temp_polygon.IsPointIn(obs_point)) {
      continue;
    }
    for (auto ego_edge : axis_lines) {
      std::vector<double> theta_vec =
          rotateMaxTheta(center, ego_edge, obs_point);
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
  steer_central_angle = (std::abs(steer_central_angle) > para_.step / radius)
                            ? steer_central_angle
                            : 0.0;
  double next_theta = ego_pose.theta + steer_central_angle;
  double next_x = x_O + steer_direction * radius * sin(next_theta);
  double next_y = y_O - steer_direction * radius * cos(next_theta);
  return Pose2D(next_x, next_y, next_theta);
}

bool CollisionChecker::checkTerminalPose(const Pose2D &pose) {
  if (!checkSinglePose(pose, 0.0, 0.0, ShapeType::OCTAGON)) {
    return false;
  }

  // if(!checkSinglePoseWheelBase(pose, 0.0, 0.0, ShapeType::WHEEL_BASE)){
  //   return false;
  // }

  return true;
}

bool CollisionChecker::checkSinglePose(const Pose2D &pose, double lat,
                                       double lon, ShapeType type) {

  std::vector<msquare::planning_math::Vec2d> corners;
  csg_.getCollisionShape(pose, corners, type, lat, lon);

  ego_polygon_.update(corners);
  for (auto &l : obs_lines_) {
    if (ego_polygon_.HasOverlap(l)) {
      return false;
    }
  }

  for (auto &p : obs_pts_) {
    if (ego_polygon_.IsPointIn(p)) {
      return false;
    }
  }

  return true;
}
bool CollisionChecker::checkSinglePoseWheelBase(const Pose2D &pose, double lat,
                                                double lon, ShapeType type) {
  std::vector<msquare::planning_math::Vec2d> corners;
  csg_.getCollisionShape(pose, corners, type, lat, lon);

  ego_polygon_.update(corners);
  for (auto &l : step_lines_) {
    if (ego_polygon_.HasOverlap(l)) {
      return false;
    }
  }

  for (auto &p : step_pts_) {
    if (ego_polygon_.IsPointIn(p)) {
      return false;
    }
  }

  return true;
}
bool CollisionChecker::checkBatchPose(const std::vector<Pose2D> &pose,
                                      double lat, double lon, ShapeType type) {
  for (const auto &p : pose) {
    if (!checkSinglePose(p, lat, lon, type) ||
        !checkSinglePoseWheelBase(p, lat, lon, ShapeType::WHEEL_BASE)) {
      return false;
    }
  }
  return true;
}

bool CollisionChecker::checkBatchPoseExceptStart(
    const std::vector<Pose2D> &pose, double lat, double lon, ShapeType type) {
  for (unsigned int i = 1; i < pose.size(); i++) {
    if (!checkSinglePose(pose[i], lat, lon, type) ||
        !checkSinglePoseWheelBase(pose[i], lat, lon, ShapeType::WHEEL_BASE)) {
      return false;
    }
  }
  return true;
}
bool CollisionChecker::checkBatchPoseExceptEnd(const std::vector<Pose2D> &pose,
                                               double lat, double lon,
                                               ShapeType type) {
  for (unsigned int i = 0; i < pose.size() - 1; i++) {
    if (!checkSinglePose(pose[i], lat, lon, type) ||
        !checkSinglePoseWheelBase(pose[i], lat, lon, ShapeType::WHEEL_BASE)) {
      return false;
    }
  }
  return true;
}

// TODO: not consider wall ?
double CollisionChecker::moveForward(const Pose2D &pose, double lat, double lon,
                                     ShapeType type) {
  double forward_length = para_.straight_line;
  double max_y = csg_.half_width_ + lat;
  double min_y = -max_y;
  double max_x = csg_.front_to_rear_ + lon;

  for (auto &p : obs_pts_) {
    msquare::planning_math::Vec2d p_transformed =
        msquare::planning_math::tf2d(pose, p);
    if (p_transformed.x() < max_x || p_transformed.y() > max_y ||
        p_transformed.y() < min_y) {
      continue;
    }
    forward_length = std::min(forward_length, p_transformed.x() - max_x);
  }

  Pose2D local_ego_pose = msquare::planning_math::tf2d(pose, pose);

  // for wall
  std::vector<msquare::planning_math::Vec2d> wall_corners;
  csg_.getRectShape(local_ego_pose, wall_corners, lat, lon);

  msquare::planning_math::LineSegment2d wall_front_edge(wall_corners[2],
                                                        wall_corners[3]);

  for (auto &line : obs_lines_) {
    planning_math::LineSegment2d l_transformed =
        planning_math::tf2d(pose, line);
    if (l_transformed.HasIntersect(wall_front_edge)) {
      return 0;
    }
    if (l_transformed.max_y() < wall_front_edge.min_y() ||
        l_transformed.min_y() > wall_front_edge.max_y() ||
        l_transformed.max_x() < wall_front_edge.min_x()) {
      continue;
    }
    if (l_transformed.min_y() >= wall_front_edge.min_y() &&
        l_transformed.max_y() <= wall_front_edge.max_y()) {
      forward_length = std::min(forward_length, l_transformed.min_x() -
                                                    wall_front_edge.max_x());
    } else if (l_transformed.min_y() < wall_front_edge.min_y() &&
               l_transformed.max_y() > wall_front_edge.max_y()) {
      double lambda_1 = (wall_front_edge.max_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_1 = lambda_1 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      double lambda_2 = (wall_front_edge.min_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_2 = lambda_2 * l_transformed.end().x() +
                   (1 - lambda_2) * l_transformed.start().x();
      if (x_1 < wall_front_edge.max_x() && x_2 < wall_front_edge.max_x()) {
        continue;
      }
      forward_length = std::min(forward_length,
                                std::min(x_1, x_2) - wall_front_edge.max_x());
    } else if (l_transformed.start().y() >= wall_front_edge.min_y() &&
               l_transformed.start().y() <= wall_front_edge.max_y()) {
      double axis_y = l_transformed.end().y() > wall_front_edge.max_y()
                          ? wall_front_edge.max_y()
                          : wall_front_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x > wall_front_edge.max_x()) {
        forward_length = std::min(forward_length,
                                  std::min(axis_x, l_transformed.start().x()) -
                                      wall_front_edge.max_x());
      }
    } else {
      // if (l_transformed.end().y() >= wall_front_edge.min_y() &&
      //   l_transformed.end().y() <= wall_front_edge.max_y())
      double axis_y = l_transformed.start().y() > wall_front_edge.max_y()
                          ? wall_front_edge.max_y()
                          : wall_front_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x > wall_front_edge.max_x()) {
        forward_length =
            std::min(forward_length, std::min(axis_x, l_transformed.end().x()) -
                                         wall_front_edge.max_x());
      }
    }
    forward_length = std::max(0.0, forward_length);
  }

  // for road side
  std::vector<msquare::planning_math::Vec2d> corners;
  csg_.getWheelBaseShape(local_ego_pose, corners, lat, lon);

  double step_max_y = std::numeric_limits<double>::lowest();
  double step_min_y = std::numeric_limits<double>::max();
  double step_max_x = std::numeric_limits<double>::lowest();
  for (auto &co : corners) {
    if (co.y() > step_max_y) {
      step_max_y = co.y();
    }
    if (co.y() < step_min_y) {
      step_min_y = co.y();
    }
    if (co.x() > step_max_x) {
      step_max_x = co.x();
    }
  }

  for (auto &p : step_pts_) {
    msquare::planning_math::Vec2d p_transformed =
        msquare::planning_math::tf2d(pose, p);
    if (p_transformed.x() < step_max_x || p_transformed.y() > step_max_y ||
        p_transformed.y() < step_min_y) {
      continue;
    }
    forward_length = std::min(forward_length, p_transformed.x() - step_max_x);
  }

  msquare::planning_math::LineSegment2d front_axis(corners[2], corners[3]);

  for (auto &l : step_lines_) {
    msquare::planning_math::LineSegment2d l_transformed =
        msquare::planning_math::tf2d(pose, l);
    if (l_transformed.HasIntersect(front_axis)) {
      return 0.0;
    }
    if (l_transformed.max_y() < front_axis.min_y() ||
        l_transformed.min_y() > front_axis.max_y() ||
        l_transformed.max_x() < front_axis.min_x()) {
      continue;
    }
    if (l_transformed.min_y() >= front_axis.min_y() &&
        l_transformed.max_y() <= front_axis.max_y()) {
      forward_length =
          std::min(forward_length, l_transformed.min_x() - front_axis.max_x());
    } else if (l_transformed.min_y() < front_axis.min_y() &&
               l_transformed.max_y() > front_axis.max_y()) {
      double lambda_1 = (front_axis.max_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_1 = lambda_1 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      double lambda_2 = (front_axis.min_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_2 = lambda_2 * l_transformed.end().x() +
                   (1 - lambda_2) * l_transformed.start().x();
      if (x_1 < front_axis.max_x() && x_2 < front_axis.max_x()) {
        continue;
      }
      forward_length =
          std::min(forward_length, std::min(x_1, x_2) - front_axis.max_x());
    } else if (l_transformed.start().y() >= front_axis.min_y() &&
               l_transformed.start().y() <= front_axis.max_y()) {
      double axis_y = l_transformed.end().y() > front_axis.max_y()
                          ? front_axis.max_y()
                          : front_axis.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x > front_axis.max_x()) {
        forward_length = std::min(forward_length,
                                  std::min(axis_x, l_transformed.start().x()) -
                                      front_axis.max_x());
      }
    } else {
      // if (l_transformed.end().y() >= front_axis.min_y() &&
      //   l_transformed.end().y() <= front_axis.max_y())
      double axis_y = l_transformed.start().y() > front_axis.max_y()
                          ? front_axis.max_y()
                          : front_axis.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x > front_axis.max_x()) {
        forward_length =
            std::min(forward_length, std::min(axis_x, l_transformed.end().x()) -
                                         front_axis.max_x());
      }
    }
    forward_length = std::max(0.0, forward_length);
  }

  return forward_length;
}

double CollisionChecker::moveBackward(const Pose2D &pose, double lat,
                                      double lon, ShapeType type) {
  double back_length = para_.straight_line;
  double max_y = csg_.half_width_ + lat;
  double min_y = -max_y;
  double min_x = -csg_.back_to_rear_ - lon;

  for (auto &p : obs_pts_) {
    msquare::planning_math::Vec2d p_transformed =
        msquare::planning_math::tf2d(pose, p);
    if (p_transformed.x() > min_x || p_transformed.y() > max_y ||
        p_transformed.y() < min_y) {
      continue;
    }
    back_length = std::min(back_length, min_x - p_transformed.x());
  }

  Pose2D local_ego_pose = msquare::planning_math::tf2d(pose, pose);

  // wall
  std::vector<msquare::planning_math::Vec2d> wall_corners;
  csg_.getRectShape(local_ego_pose, wall_corners, lat, lon);

  msquare::planning_math::LineSegment2d wall_back_edge(wall_corners[0],
                                                       wall_corners[1]);
  for (auto &l : obs_lines_) {
    planning_math::LineSegment2d l_transformed = planning_math::tf2d(pose, l);
    if (l_transformed.HasIntersect(wall_back_edge)) {
      return 0;
    }
    if (l_transformed.max_y() < wall_back_edge.min_y() ||
        l_transformed.min_y() > wall_back_edge.max_y() ||
        l_transformed.min_x() > wall_back_edge.max_x()) {
      continue;
    }
    if (l_transformed.min_y() >= wall_back_edge.min_y() &&
        l_transformed.max_y() <= wall_back_edge.max_y()) {
      back_length =
          std::min(back_length, wall_back_edge.min_x() - l_transformed.max_x());
    } else if (l_transformed.min_y() < wall_back_edge.min_y() &&
               l_transformed.max_y() > wall_back_edge.max_y()) {
      double lambda_1 = (wall_back_edge.max_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_1 = lambda_1 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      double lambda_2 = (wall_back_edge.min_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_2 = lambda_2 * l_transformed.end().x() +
                   (1 - lambda_2) * l_transformed.start().x();
      if (x_1 > wall_back_edge.min_x() && x_2 > wall_back_edge.min_x()) {
        continue;
      }
      back_length =
          std::min(back_length, wall_back_edge.min_x() - std::max(x_1, x_2));
    } else if (l_transformed.start().y() >= wall_back_edge.min_y() &&
               l_transformed.start().y() <= wall_back_edge.max_y()) {
      double axis_y = l_transformed.end().y() > wall_back_edge.max_y()
                          ? wall_back_edge.max_y()
                          : wall_back_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x < wall_back_edge.min_x()) {
        back_length = std::min(back_length,
                               wall_back_edge.min_x() -
                                   std::max(axis_x, l_transformed.start().x()));
      }
    } else {
      // if (l_transformed.end().y() >= wall_back_edge.min_y() &&
      //   l_transformed.end().y() <= wall_back_edge.max_y())
      double axis_y = l_transformed.start().y() > wall_back_edge.max_y()
                          ? wall_back_edge.max_y()
                          : wall_back_edge.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x < wall_back_edge.min_x()) {
        back_length = std::min(back_length,
                               wall_back_edge.min_x() -
                                   std::max(axis_x, l_transformed.end().x()));
      }
    }
    back_length = std::max(0.0, back_length);
  }

  // road side
  std::vector<msquare::planning_math::Vec2d> corners;
  csg_.getWheelBaseShape(local_ego_pose, corners, lat, lon);

  double step_max_y = corners[0].y();
  double step_min_y = corners[0].y();
  double step_min_x = corners[0].x();
  for (auto &co : corners) {
    if (co.y() > step_max_y) {
      step_max_y = co.y();
    }
    if (co.y() < step_min_y) {
      step_min_y = co.y();
    }
    if (co.x() < step_min_x) {
      step_min_x = co.x();
    }
  }
  for (auto &p : step_pts_) {
    msquare::planning_math::Vec2d p_transformed =
        msquare::planning_math::tf2d(pose, p);
    if (p_transformed.x() > step_min_x || p_transformed.y() > step_max_y ||
        p_transformed.y() < step_min_y) {
      continue;
    }
    back_length = std::min(back_length, step_min_x - p_transformed.x());
  }

  msquare::planning_math::LineSegment2d rear_axis(corners[0], corners[1]);

  for (auto &l : step_lines_) {
    msquare::planning_math::LineSegment2d l_transformed =
        msquare::planning_math::tf2d(pose, l);
    if (l_transformed.HasIntersect(rear_axis)) {
      return 0;
    }
    if (l_transformed.max_y() < rear_axis.min_y() ||
        l_transformed.min_y() > rear_axis.max_y() ||
        l_transformed.min_x() > rear_axis.max_x()) {
      continue;
    }
    if (l_transformed.min_y() >= rear_axis.min_y() &&
        l_transformed.max_y() <= rear_axis.max_y()) {
      back_length =
          std::min(back_length, rear_axis.min_x() - l_transformed.max_x());
    } else if (l_transformed.min_y() < rear_axis.min_y() &&
               l_transformed.max_y() > rear_axis.max_y()) {
      double lambda_1 = (rear_axis.max_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_1 = lambda_1 * l_transformed.end().x() +
                   (1 - lambda_1) * l_transformed.start().x();
      double lambda_2 = (rear_axis.min_y() - l_transformed.start().y()) /
                        (l_transformed.end().y() - l_transformed.start().y());
      double x_2 = lambda_2 * l_transformed.end().x() +
                   (1 - lambda_2) * l_transformed.start().x();
      if (x_1 > rear_axis.min_x() && x_2 > rear_axis.min_x()) {
        continue;
      }
      back_length =
          std::min(back_length, rear_axis.min_x() - std::max(x_1, x_2));
    } else if (l_transformed.start().y() >= rear_axis.min_y() &&
               l_transformed.start().y() <= rear_axis.max_y()) {
      double axis_y = l_transformed.end().y() > rear_axis.max_y()
                          ? rear_axis.max_y()
                          : rear_axis.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x < rear_axis.min_x()) {
        back_length = std::min(back_length,
                               rear_axis.min_x() -
                                   std::max(axis_x, l_transformed.start().x()));
      }
    } else {
      // if (l_transformed.end().y() >= rear_axis.min_y() &&
      //   l_transformed.end().y() <= rear_axis.max_y())
      double axis_y = l_transformed.start().y() > rear_axis.max_y()
                          ? rear_axis.max_y()
                          : rear_axis.min_y();
      double lambda = (axis_y - l_transformed.start().y()) /
                      (l_transformed.end().y() - l_transformed.start().y());
      double axis_x = lambda * l_transformed.end().x() +
                      (1 - lambda) * l_transformed.start().x();
      if (axis_x < rear_axis.min_x()) {
        back_length = std::min(back_length,
                               rear_axis.min_x() -
                                   std::max(axis_x, l_transformed.end().x()));
      }
    }
    back_length = std::max(0.0, back_length);
  }

  return back_length;
}

} // namespace clothoid