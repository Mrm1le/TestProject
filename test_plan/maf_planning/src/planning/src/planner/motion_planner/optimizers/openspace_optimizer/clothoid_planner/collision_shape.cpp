#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/collision_shape.h"

namespace clothoid {
void CollisionShapeGenerator::init() {
  width_ = para_.width;
  length_ = para_.length;
  half_width_ = 0.5 * width_;
  front_to_rear_ = para_.front_to_rear;
  back_to_rear_ = length_ - front_to_rear_;
}
void CollisionShapeGenerator::getCollisionShape(
    const Pose2D &ego_pose, std::vector<planning_math::Vec2d> &polygon_points,
    ShapeType type, double lat, double lon) const {
  switch (type) {
  case ShapeType::RAW:
    return getRawShape(ego_pose, polygon_points, lat, lon);
    break;
  case ShapeType::RECT:
    return getRectShape(ego_pose, polygon_points, lat, lon);
    break;
  case ShapeType::WHEEL_BASE:
    return getWheelBaseShape(ego_pose, polygon_points, lat, lon);
    break;
  case ShapeType::ROTATE:
    return getRotateShape(ego_pose, polygon_points, lat, lon);
    break;
  case ShapeType::OCTAGON:
    return getOctagonShape(ego_pose, polygon_points, lat, lon);
  default:
    break;
  }
}

void CollisionShapeGenerator::getOctagonShape(
    const Pose2D &ego_pose, std::vector<planning_math::Vec2d> &polygon_points,
    double lat, double lon) const {
  double temp_half_width = half_width_ + lat;
  double temp_front_corner_width = para_.front_corner_width + 0.02;
  double temp_front_to_rear = front_to_rear_ + lon;
  double temp_back_to_rear = back_to_rear_ + lon;
  double temp_front_corner_length = para_.front_corner_length + lon + 0.05;
  double real_width = half_width_;
  double real_back_rear = back_to_rear_;

  double back_light_len = para_.back_light_len;
  double back_light_height = para_.back_light_height;

  polygon_points.clear();
  polygon_points.reserve(8);

  double x, y, cc, cs;
  x = ego_pose.x;
  y = ego_pose.y;
  cc = std::cos(ego_pose.theta);
  cs = std::sin(ego_pose.theta);

  polygon_points.emplace_back(
      x + temp_front_to_rear * cc - temp_front_corner_width * cs,
      y + temp_front_to_rear * cs + temp_front_corner_width * cc);
  polygon_points.emplace_back(
      x + temp_front_corner_length * cc - temp_half_width * cs,
      y + temp_front_corner_length * cs + temp_half_width * cc);

  polygon_points.emplace_back(
      x - (real_back_rear - back_light_len) * cc - temp_half_width * cs,
      y - (real_back_rear - back_light_len) * cs + temp_half_width * cc);
  polygon_points.emplace_back(
      x - temp_back_to_rear * cc - (real_width - back_light_height) * cs,
      y - temp_back_to_rear * cs + (real_width - back_light_height) * cc);
  polygon_points.emplace_back(
      x - temp_back_to_rear * cc + (real_width - back_light_height) * cs,
      y - temp_back_to_rear * cs - (real_width - back_light_height) * cc);
  polygon_points.emplace_back(
      x - (real_back_rear - back_light_len) * cc + temp_half_width * cs,
      y - (real_back_rear - back_light_len) * cs - temp_half_width * cc);

  polygon_points.emplace_back(
      x + temp_front_corner_length * cc + temp_half_width * cs,
      y + temp_front_corner_length * cs - temp_half_width * cc);
  polygon_points.emplace_back(
      x + temp_front_to_rear * cc + temp_front_corner_width * cs,
      y + temp_front_to_rear * cs - temp_front_corner_width * cc);
}

void CollisionShapeGenerator::getRectShape(
    const Pose2D &pose, std::vector<planning_math::Vec2d> &polygon_points,
    double lat, double lon) const {

  double edge_width = half_width_ + lat;
  double back_edge_length = -back_to_rear_ - lon;
  double front_edge_length = front_to_rear_ + lon;
  polygon_points.clear();
  polygon_points.reserve(4);
  polygon_points.emplace_back(pose.x - edge_width * sin(pose.theta) +
                                  back_edge_length * cos(pose.theta),
                              pose.y + edge_width * cos(pose.theta) +
                                  back_edge_length * sin(pose.theta));
  polygon_points.emplace_back(pose.x + edge_width * sin(pose.theta) +
                                  back_edge_length * cos(pose.theta),
                              pose.y - edge_width * cos(pose.theta) +
                                  back_edge_length * sin(pose.theta));
  polygon_points.emplace_back(pose.x + edge_width * sin(pose.theta) +
                                  front_edge_length * cos(pose.theta),
                              pose.y - edge_width * cos(pose.theta) +
                                  front_edge_length * sin(pose.theta));
  polygon_points.emplace_back(pose.x - edge_width * sin(pose.theta) +
                                  front_edge_length * cos(pose.theta),
                              pose.y + edge_width * cos(pose.theta) +
                                  front_edge_length * sin(pose.theta));
}

void CollisionShapeGenerator::getRawShape(
    const Pose2D &ego_pose, std::vector<planning_math::Vec2d> &polygon_points,
    double lat, double lon) const {
  double temp_half_width = half_width_ + lat;
  double temp_front_corner_width = para_.front_corner_width + lat;
  double temp_front_to_rear = front_to_rear_ + lon;
  double temp_back_to_rear = back_to_rear_ + lon;
  double temp_front_corner_length = para_.front_corner_length + lon;

  polygon_points.clear();
  polygon_points.reserve(6);

  double x, y, cc, cs;
  x = ego_pose.x;
  y = ego_pose.y;
  cc = std::cos(ego_pose.theta);
  cs = std::sin(ego_pose.theta);
  polygon_points.emplace_back(
      x + temp_front_to_rear * cc - temp_front_corner_width * cs,
      ego_pose.y + temp_front_to_rear * cs + temp_front_corner_width * cc);
  polygon_points.emplace_back(
      x + temp_front_corner_length * cc - temp_half_width * cs,
      ego_pose.y + temp_front_corner_length * cs + temp_half_width * cc);
  polygon_points.emplace_back(x - temp_back_to_rear * cc - temp_half_width * cs,
                              ego_pose.y - temp_back_to_rear * cs +
                                  temp_half_width * cc);
  polygon_points.emplace_back(x - temp_back_to_rear * cc + temp_half_width * cs,
                              ego_pose.y - temp_back_to_rear * cs -
                                  temp_half_width * cc);
  polygon_points.emplace_back(
      x + temp_front_corner_length * cc + temp_half_width * cs,
      ego_pose.y + temp_front_corner_length * cs - temp_half_width * cc);
  polygon_points.emplace_back(
      x + temp_front_to_rear * cc + temp_front_corner_width * cs,
      ego_pose.y + temp_front_to_rear * cs - temp_front_corner_width * cc);
}

void CollisionShapeGenerator::getWheelBaseShape(
    const Pose2D &ego_pose, std::vector<planning_math::Vec2d> &polygon_points,
    double lat, double lon) const {
  lon = lon / 2.0;
  double axis_width = half_width_ + lat;
  double axis_add_temp = std::sqrt((std::max(
      (2 * para_.wheel_radius - para_.max_step_height) * para_.max_step_height,
      1e-6)));
  double axis_add = std::max(axis_add_temp, 0.5 * para_.wheel_radius);
  double back_axis_length = -axis_add - lon;
  double front_axis_length = para_.wheel_base + axis_add + lon;

  polygon_points.clear();
  polygon_points.reserve(4);

  double x, y, cc, cs;
  x = ego_pose.x;
  y = ego_pose.y;
  cc = std::cos(ego_pose.theta);
  cs = std::sin(ego_pose.theta);
  polygon_points.emplace_back(x - axis_width * cs + back_axis_length * cc,
                              ego_pose.y + axis_width * cc +
                                  back_axis_length * cs);
  polygon_points.emplace_back(x + axis_width * cs + back_axis_length * cc,
                              ego_pose.y - axis_width * cc +
                                  back_axis_length * cs);
  polygon_points.emplace_back(x + axis_width * cs + front_axis_length * cc,
                              ego_pose.y - axis_width * cc +
                                  front_axis_length * cs);
  polygon_points.emplace_back(x - axis_width * cs + front_axis_length * cc,
                              ego_pose.y + axis_width * cc +
                                  front_axis_length * cs);
}
// inflation
void CollisionShapeGenerator::getRotateShape(
    const Pose2D &ego_pose, std::vector<planning_math::Vec2d> &polygon_points,
    double lat, double lon) const {
  double temp_half_width = half_width_ + lat;
  double temp_front_corner_width = para_.front_corner_width + 0.02;
  double temp_front_to_rear = front_to_rear_ + lon;
  double temp_back_to_rear = back_to_rear_ + lon;
  double temp_front_corner_length = para_.front_corner_length + lon + 0.05;
  double real_width = half_width_;
  double real_back_rear = back_to_rear_;

  polygon_points.clear();
  polygon_points.reserve(8);

  double x, y, cc, cs;
  x = ego_pose.x;
  y = ego_pose.y;
  cc = std::cos(ego_pose.theta);
  cs = std::sin(ego_pose.theta);

  polygon_points.emplace_back(
      x + temp_front_to_rear * cc - temp_front_corner_width * cs,
      y + temp_front_to_rear * cs + temp_front_corner_width * cc);
  polygon_points.emplace_back(
      x + temp_front_corner_length * cc - temp_half_width * cs,
      y + temp_front_corner_length * cs + temp_half_width * cc);

  polygon_points.emplace_back(x - real_back_rear * cc - temp_half_width * cs,
                              y - real_back_rear * cs + temp_half_width * cc);
  polygon_points.emplace_back(x - temp_back_to_rear * cc - real_width * cs,
                              y - temp_back_to_rear * cs + real_width * cc);
  polygon_points.emplace_back(x - temp_back_to_rear * cc + real_width * cs,
                              y - temp_back_to_rear * cs - real_width * cc);
  polygon_points.emplace_back(x - real_back_rear * cc + temp_half_width * cs,
                              y - real_back_rear * cs - temp_half_width * cc);

  polygon_points.emplace_back(
      x + temp_front_corner_length * cc + temp_half_width * cs,
      y + temp_front_corner_length * cs - temp_half_width * cc);
  polygon_points.emplace_back(
      x + temp_front_to_rear * cc + temp_front_corner_width * cs,
      y + temp_front_to_rear * cs - temp_front_corner_width * cc);
}

void CollisionShapeGenerator::formatLines(
    const std::vector<planning_math::Vec2d> &pts,
    std::vector<planning_math::LineSegment2d> &lines) {
  lines.clear();
  lines.reserve(pts.size());
  for (unsigned int i = 0; i < pts.size(); ++i) {
    lines.emplace_back(pts.at(i), pts.at((i + 1) % (pts.size())));
  }
}

} // namespace clothoid
