#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"
#include <iostream>

namespace msquare {

namespace parking {

inline planning_math::Box2d getEgoBox(const TrajectoryPoint &ego_state) {
  double xx = ego_state.path_point.x +
              VehicleParam::Instance()->center_to_geometry_center *
                  cos(ego_state.path_point.theta);
  double yy = ego_state.path_point.y +
              VehicleParam::Instance()->center_to_geometry_center *
                  sin(ego_state.path_point.theta);
  return planning_math::Box2d(
      planning_math::Vec2d(xx, yy), ego_state.path_point.theta,
      VehicleParam::Instance()->length, VehicleParam::Instance()->width);
}

inline planning_math::LineSegment2d
getUpperTline(double min_pt_x, double max_pt_x, double min_upper_bound_height,
              const std::vector<planning_math::Vec2d> &local_points,
              const planning_math::Box2d &local_map_boundary,
              std::vector<planning_math::Box2d> local_boxes,
              std::vector<planning_math::Box2d> &lower_boxes) {
  // TODO@lizhiqiang: if need, add default distance param into apa.yaml
  double middle_height = 3.0;
  std::vector<planning_math::Box2d> upper_boxes;
  for (auto &box : local_boxes) {
    if (box.min_y() > middle_height) {
      upper_boxes.push_back(box);
      continue;
    }
    lower_boxes.push_back(box);
  }

  double upper_line_height = local_map_boundary.max_y();
  for (auto &box : upper_boxes) {
    upper_line_height = std::min(upper_line_height, box.min_y());
  }

  for (auto &pt : local_points) {
    if (pt.x() > min_pt_x && pt.x() < max_pt_x && pt.y() > middle_height) {
      upper_line_height = std::min(upper_line_height, pt.y());
    }
  }
  upper_line_height = std::max(upper_line_height, min_upper_bound_height);
  return planning_math::LineSegment2d(
      planning_math::Vec2d(local_map_boundary.min_x(), upper_line_height),
      planning_math::Vec2d(local_map_boundary.max_x(), upper_line_height));
}

} // namespace parking
} // namespace msquare