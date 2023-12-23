#include "common/angle_parking_slot.h"
#include "common/math/math_utils.h"
#include "common/planning_config.h"
#include "common/slot_util.h"
#include "planning/common/common.h"

namespace msquare {
namespace parking {

using namespace planning_math;

AngleParkingSlot::AngleParkingSlot(
    const std::string cfg_file, std::vector<Point3D> lot_corners,
    bool is_relative_left, std::vector<planning_math::Vec2d> original_corners)
    : BaseParkingSlot(cfg_file, lot_corners, is_relative_left),
      original_corners_(original_corners) {}

AngleParkingSlot::~AngleParkingSlot() {}

std::vector<planning_math::LineSegment2d>
AngleParkingSlot::genFrontWings(double span) {
  std::vector<planning_math::LineSegment2d> res;
  Vec2d left_vector = original_corners_.at(1) - original_corners_.at(2);
  Vec2d left_wing_vector = left_vector * span / left_vector.Length();
  res.push_back(LineSegment2d(original_corners_.at(0),
                              original_corners_.at(0) + left_wing_vector));
  res.push_back(LineSegment2d(original_corners_.at(3),
                              original_corners_.at(3) - left_wing_vector));
  return res;
}

Pose2D AngleParkingSlot::getParkingInPose(double front_edge_to_rear,
                                          double back_edge_to_rear,
                                          double brake_buffer,
                                          const Pose2D &init_pose,
                                          double wheel_stop_depth) {
  double relative_heading =
      planning_math::NormalizeAngle(box_.heading() - init_pose.theta);
  bool is_init_pose_invalid =
      (std::fabs(init_pose.x) < 1e-8 && std::fabs(init_pose.y) < 1e-8 &&
       std::fabs(init_pose.theta) < 1e-8);
  // if (std::cos(relative_heading) > 0 || is_init_pose_invalid) {
  if (true) {
    return BaseParkingSlot::getParkingInPose(front_edge_to_rear,
                                             back_edge_to_rear, brake_buffer,
                                             init_pose, wheel_stop_depth);
  } else {
    // reverse back_edge_to_rear and front_edge_to_rear
    Pose2D res_tmp = BaseParkingSlot::getParkingInPose(
        back_edge_to_rear, front_edge_to_rear, brake_buffer, init_pose,
        wheel_stop_depth);
    // reverse heading
    res_tmp.theta = planning_math::NormalizeAngle(res_tmp.theta + M_PI);
    return res_tmp;
  }
}

std::vector<planning_math::LineSegment2d> AngleParkingSlot::getTshapedAreaLines(
    const TrajectoryPoint &ego_state, const planning_math::Box2d &map_boundary,
    std::vector<ObstacleBox> nearby_boxes,
    const std::vector<planning_math::Vec2d> &points) {
  Vec2d slot_front_left = original_corners_[0];
  Vec2d slot_front_right = original_corners_[3];
  Vec2d slot_front_center = (slot_front_left + slot_front_right) / 2;
  Pose2D local_frame_pose{slot_front_center.x(), slot_front_center.y(),
                          (slot_front_right - slot_front_left).Angle()};
  Vec2d local_front_left =
      planning_math::tf2d(local_frame_pose, slot_front_left);
  Vec2d local_front_right =
      planning_math::tf2d(local_frame_pose, slot_front_right);

  planning_math::Box2d ego_box = getEgoBox(ego_state);
  planning_math::Box2d local_ego_box =
      planning_math::tf2d(local_frame_pose, ego_box);

  double side_radius = VehicleParam::Instance()->min_turn_radius +
                       VehicleParam::Instance()->width / 2;
  double corner_radius =
      std::hypot(side_radius, VehicleParam::Instance()->front_edge_to_center);
  double radius_diff = corner_radius - side_radius;
  double min_upper_bound_height = local_ego_box.max_y() + radius_diff;
  double max_lower_bound_height =
      std::min(local_ego_box.length() / 2.0, min_upper_bound_height);

  planning_math::Box2d local_map_boundary =
      planning_math::tf2d(local_frame_pose, map_boundary);
  std::vector<planning_math::Box2d> local_boxes;
  std::vector<planning_math::Vec2d> local_points;
  // for (auto &box : nearby_boxes) {
  //   local_boxes.push_back(planning_math::tf2d(local_frame_pose, box));
  // }
  for (auto &p : points) {
    local_points.push_back(planning_math::tf2d(local_frame_pose, p));
  }
  std::vector<planning_math::Box2d> local_lower_boxes;

  std::vector<planning_math::LineSegment2d> local_T_lines, T_lines;
  const double DEFAULT_OPPOSITE_POINTS_ROI = 3.0;
  planning_math::LineSegment2d upper_line =
      getUpperTline(local_front_left.x() - DEFAULT_OPPOSITE_POINTS_ROI,
                    local_front_right.x() + DEFAULT_OPPOSITE_POINTS_ROI,
                    min_upper_bound_height, local_points, local_map_boundary,
                    local_boxes, local_lower_boxes);
  // std::cout << "lower_boxes size: " << local_lower_boxes.size() << std::endl;
  local_T_lines.push_back(upper_line);

  Box2d local_slot = planning_math::tf2d(local_frame_pose, box_);
  Box2d slot_box_on_left(local_slot);
  slot_box_on_left.SetLength(slot_box_on_left.length() + 2.0);
  Box2d slot_box_on_right(local_slot);
  slot_box_on_right.SetLength(slot_box_on_right.length() + 2.0);
  slot_box_on_right.Shift(local_front_right - local_front_left);
  slot_box_on_left.Shift(local_front_left - local_front_right);

  Vec2d slot_back_left = original_corners_[1];
  Vec2d local_back_left = planning_math::tf2d(local_frame_pose, slot_back_left);
  double local_heading = atan2(local_front_left.y() - local_back_left.y(),
                               local_front_left.x() - local_back_left.x());
  double left_lower_height = local_map_boundary.min_y();
  double right_lower_height = local_map_boundary.min_y();
  double left_x = local_map_boundary.min_x();
  double right_x = local_map_boundary.max_x();

  for (const Box2d &box : local_lower_boxes) {
    if (box.max_y() < -VehicleParam::Instance()->front_edge_to_center) {
      continue;
    }
    if (box.HasOverlap(slot_box_on_left)) {
      // process left box
      left_lower_height = std::max(box.max_y(), left_lower_height);
      for (auto pt : box.GetAllCorners()) {
        left_x = std::max(pt.x() - pt.y() / tan(local_heading), left_x);
      }
    } else if (box.HasOverlap(slot_box_on_right)) {
      // process right box
      right_lower_height = std::max(box.max_y(), right_lower_height);
      for (auto pt : box.GetAllCorners()) {
        right_x = std::min(pt.x() - pt.y() / tan(local_heading), right_x);
      }
    }
  }
  for (auto &p : local_points) {
    if (p.y() < -VehicleParam::Instance()->front_edge_to_center) {
      continue;
    }
    if (slot_box_on_left.IsPointIn(p)) {
      left_lower_height = std::max(p.y(), left_lower_height);
      left_x = std::max(p.x() - p.y() / tan(local_heading), left_x);
    } else if (slot_box_on_right.IsPointIn(p)) {
      right_lower_height = std::max(p.y(), right_lower_height);
      right_x = std::min(p.x() - p.y() / tan(local_heading), right_x);
    }
  }

  const double DEFAULT_RULE_SLOT_WIDTH = 3.1 / sin(local_heading);
  if (is_relative_left_) {
    right_x =
        std::min(right_x, std::max(DEFAULT_RULE_SLOT_WIDTH / 2, right_x - 0.5));
    right_lower_height = -0.5;
    left_x =
        std::max(left_x, std::min(-DEFAULT_RULE_SLOT_WIDTH / 2, left_x + 0.5));
    left_lower_height = std::max(-2.0 * VehicleParam::Instance()->length / 3.0,
                                 left_lower_height);
  } else {
    left_x =
        std::max(left_x, std::min(-DEFAULT_RULE_SLOT_WIDTH / 2, left_x + 0.5));
    left_lower_height = -0.5;
    right_x =
        std::min(right_x, std::max(DEFAULT_RULE_SLOT_WIDTH / 2, right_x - 0.5));
    right_lower_height = std::max(-2.0 * VehicleParam::Instance()->length / 3.0,
                                  right_lower_height);
  }
  left_x = std::max(left_x, -DEFAULT_RULE_SLOT_WIDTH / 2);
  right_x = std::min(right_x, DEFAULT_RULE_SLOT_WIDTH / 2);

  // ensure that lower lines don't cross map_boundary
  left_lower_height = std::min(left_lower_height, max_lower_bound_height);
  right_lower_height = std::min(right_lower_height, max_lower_bound_height);
  left_lower_height = std::max(left_lower_height, local_map_boundary.min_y());
  right_lower_height = std::max(right_lower_height, local_map_boundary.min_y());
  left_x = std::max(left_x, local_map_boundary.min_x());
  right_x = std::min(right_x, local_map_boundary.max_x());

  local_T_lines.push_back(
      LineSegment2d(Vec2d(local_map_boundary.min_x(), left_lower_height),
                    Vec2d(left_lower_height / tan(local_heading) + left_x,
                          left_lower_height)));
  local_T_lines.push_back(LineSegment2d(
      Vec2d(left_lower_height / tan(local_heading) + left_x, left_lower_height),
      Vec2d(local_map_boundary.min_y() / tan(local_heading) + left_x,
            local_map_boundary.min_y())));
  local_T_lines.push_back(LineSegment2d(
      Vec2d(local_map_boundary.min_y() / tan(local_heading) + right_x,
            local_map_boundary.min_y()),
      Vec2d(right_lower_height / tan(local_heading) + right_x,
            right_lower_height)));
  local_T_lines.push_back(
      LineSegment2d(Vec2d(right_lower_height / tan(local_heading) + right_x,
                          right_lower_height),
                    Vec2d(local_map_boundary.max_x(), right_lower_height)));

  for (auto &line : local_T_lines) {
    T_lines.push_back(tf2d_inv(local_frame_pose, line));
  }
  return T_lines;
}

double AngleParkingSlot::getOpeningHeading() const {
  LineSegment2d center_line(original_corners_.at(0), original_corners_.at(3));
  return NormalizeAngle(center_line.heading() + M_PI / 2);
}

} // namespace parking
} // namespace msquare