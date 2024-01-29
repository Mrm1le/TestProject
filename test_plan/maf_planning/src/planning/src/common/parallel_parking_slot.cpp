#include "common/parallel_parking_slot.h"
#include "common/math/math_utils.h"
#include "common/planning_config.h"
#include "common/slot_util.h"
#include "planning/common/common.h"
#include <exception>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace msquare {

namespace parking {

using namespace planning_math;

ParallelParkingSlot::ParallelParkingSlot(
    const std::string cfg_file, std::vector<Point3D> lot_corners,
    std::vector<planning_math::Vec2d> physical_corners, bool is_relative_left)
    : BaseParkingSlot(cfg_file, lot_corners, is_relative_left),
      physical_corners_(physical_corners) {}

Pose2D ParallelParkingSlot::getParkingInPose(double front_edge_to_rear,
                                             double back_edge_to_rear,
                                             double brake_buffer,
                                             const Pose2D &init_pose,
                                             double wheel_stop_depth) {
  Pose2D lot_front_pose;
  Pose2D lot_back_pose;
  // ego is heading right by default
  lot_front_pose.x = (corners_[2].x + corners_[3].x) / 2;
  lot_front_pose.y = (corners_[2].y + corners_[3].y) / 2;
  lot_back_pose.x = (corners_[1].x + corners_[0].x) / 2;
  lot_back_pose.y = (corners_[1].y + corners_[0].y) / 2;

  MSD_LOG(ERROR, "getParkingInPose is_relative_left_=%d", is_relative_left_);

  if (is_relative_left_) {
    std::swap(lot_front_pose, lot_back_pose);
  }

  MSD_LOG(ERROR, "getParkingInPose enable_get_heading_with_front_corners_=%d",
          enable_get_heading_with_front_corners_);

  if (enable_get_heading_with_front_corners_) {
    double front_edge_heading = std::atan2(corners_[0].y - corners_[3].y,
                                           corners_[0].x - corners_[3].x);
    lot_front_pose.theta = NormalizeAngle(front_edge_heading - M_PI_2);
  } else {
    lot_front_pose.theta = lot_back_pose.theta = std::atan2(
        lot_front_pose.y - lot_back_pose.y, lot_front_pose.x - lot_back_pose.x);
  }

  double lot_length = std::hypot(lot_front_pose.y - lot_back_pose.y,
                                 lot_front_pose.x - lot_back_pose.x);

  double target2back =
      lot_length / 2 - (front_edge_to_rear - back_edge_to_rear) / 2;

  Pose2D target_in_lot_front_frame;
  target_in_lot_front_frame.x = -(lot_length - target2back);

  MSD_LOG(ERROR, "getParkingInPose target_in_lot_front_frame.x=%f",
          target_in_lot_front_frame.x);

  return planning_math::tf2d_inv(lot_front_pose, target_in_lot_front_frame);
}

std::vector<planning_math::LineSegment2d>
ParallelParkingSlot::genLotWalls(double vehicle_width) {
  return {};
}

planning_math::LineSegment2d ParallelParkingSlot::genBottomWall() {
  LineSegment2d bottom_wall_raw = BaseParkingSlot::genBottomWall();
  // extend for safety
  const double EXTEND_LENGTH = 10.0;
  Vec2d extend_vec = bottom_wall_raw.unit_direction() * EXTEND_LENGTH;

  std::vector<Vec2d> lot_corners_vec2d = box_.GetAllCorners();
  Vec2d bottom_vec = lot_corners_vec2d[2] - lot_corners_vec2d[1];
  bottom_vec.Normalize();

  return LineSegment2d(bottom_wall_raw.start() - extend_vec + bottom_vec * 1.0,
                       bottom_wall_raw.end() + extend_vec + bottom_vec * 1.0);
}

std::vector<planning_math::LineSegment2d>
ParallelParkingSlot::genFrontWings(double span) {
  std::vector<LineSegment2d> res = BaseParkingSlot::genFrontWings(span);
  // add wing which is in front
  Pose2D box_center_pose{box_.center().x(), box_.center().y(), box_.heading()};

  const double PASSWAY_WIDTH = 4.5;
  double wing_x = box_.length() / 2 + PASSWAY_WIDTH;
  LineSegment2d wing_local(Vec2d(wing_x, -span), Vec2d(wing_x, span));
  res.push_back(tf2d_inv(box_center_pose, wing_local));
  return res;
}

std::vector<TrajectoryPoint> ParallelParkingSlot::getCenterLine(
    double front_edge_to_rear, double back_edge_to_rear, double brake_buffer,
    double wheel_stop_depth) {
  std::vector<TrajectoryPoint> center_line_traj;
  return center_line_traj;
}

bool ParallelParkingSlot::isLateralInside(
    const Pose2D &pose, const FootprintModelPtr footprint_model) {
  if (!box_.IsPointIn(Vec2d(pose.x, pose.y))) {
    return false;
  }

  std::vector<LineSegment2d> box_edges = box_.GetAllEdges();
  std::vector<LineSegment2d> box_edges_to_check;
  box_edges_to_check.push_back(box_edges[0]);
  box_edges_to_check.push_back(box_edges[2]);
  if (footprint_model->checkOverlap(pose, box_edges_to_check)) {
    return false;
  }

  return true;
}

bool ParallelParkingSlot::isInside(const Pose2D &pose,
                                   const FootprintModelPtr footprint_model) {
  if (!box_.IsPointIn(Vec2d(pose.x, pose.y))) {
    return false;
  }
  std::vector<LineSegment2d> box_edges = box_.GetAllEdges();
  if (footprint_model->checkOverlap(pose, box_edges)) {
    return false;
  }

  return true;
}

bool ParallelParkingSlot::isAlongsideLot(const Pose2D &pose, double thres) {
  Vec2d heading_vec;
  if (is_relative_left_) {
    heading_vec = Vec2d(corners_.at(0).x - corners_.at(3).x,
                        corners_.at(0).y - corners_.at(3).y);
  } else {
    heading_vec = Vec2d(corners_.at(3).x - corners_.at(0).x,
                        corners_.at(3).y - corners_.at(0).y);
  }
  double slot_heading = heading_vec.Angle();

  return std::fabs(NormalizeAngle(pose.theta - slot_heading)) < thres;
}

bool ParallelParkingSlot::isVehicleReached(
    const VehicleParam *vehicle_param, const Pose2D &pose,
    const FootprintModelPtr footprint_model, double wheel_stop_depth) {
  Pose2D target_pose = getParkingInPose(
      vehicle_param->front_edge_to_center, vehicle_param->back_edge_to_center,
      vehicle_param->brake_distance_buffer, pose, wheel_stop_depth);
  const double lon_ending_thres_parallel =
      CarParams::GetInstance()
          ->car_config.ending_check_config.lon_ending_thres_parallel;
  const double lat_ending_thres_parallel =
      CarParams::GetInstance()
          ->car_config.ending_check_config.lat_ending_thres_parallel;
  const double angle_ending_thres_parallel =
      CarParams::GetInstance()
          ->car_config.ending_check_config.angle_ending_thres_parallel;
  const Pose2D POSE_THRES{lon_ending_thres_parallel, lat_ending_thres_parallel,
                          angle_ending_thres_parallel};
  if (!isEgoInPlace(pose, target_pose, vehicle_param->wheel_base, POSE_THRES)) {
    MSD_LOG(ERROR, "isVehicleReached isEgoInPlace fail");
    return false;
  }

  // check if longitudinal feasible
  std::vector<LineSegment2d> box_edges = box_.GetAllEdges();
  LineSegment2d bottom_edge = box_edges[1];
  LineSegment2d top_edge = box_edges[3];
  if (is_relative_left_) {
    std::swap(bottom_edge, top_edge);
  }
  const double MAX_CENTER_OFFSET_FORWARD =
      CarParams::GetInstance()
          ->car_config.ending_check_config.max_center_offset_parallel;
  const double MAX_CENTER_OFFSET_BACKWARD =
      -CarParams::GetInstance()
           ->car_config.ending_check_config.max_center_offset_parallel;
  double center_offset = (bottom_edge.DistanceTo(Vec2d(pose.x, pose.y)) -
                          vehicle_param->back_edge_to_center) -
                         (top_edge.DistanceTo(Vec2d(pose.x, pose.y)) -
                          vehicle_param->front_edge_to_center);
  bool is_center_offset_too_much = center_offset > MAX_CENTER_OFFSET_FORWARD ||
                                   center_offset < MAX_CENTER_OFFSET_BACKWARD;
  if (is_center_offset_too_much) {
    MSD_LOG(ERROR, "isVehicleReached center_offset = %f", center_offset);
    return false;
  }
  return true;
}

void ParallelParkingSlot::customizePlanningParams(
    CarParams *car_params, HybridAstarConfig *hybridastar_config,
    StrategyParams *strategy_params) {}

bool ParallelParkingSlot::isInsidePhysicalSlot(
    const Pose2D &pose, const FootprintModelPtr footprint_model) {
  Polygon2d origin_polygon(physical_corners_);
  const double LON_TOLERANCE = 0.5;
  // lateral tolerance does not count
  origin_polygon.ExpandByDistance(LON_TOLERANCE);
  if (!origin_polygon.IsPointIn(Vec2d(pose.x, pose.y))) {
    return false;
  }
  for (const LineSegment2d &line : origin_polygon.line_segments()) {
    if (footprint_model->checkOverlap(pose, line)) {
      return false;
    }
  }

  return true;
}

std::vector<planning_math::LineSegment2d>
ParallelParkingSlot::getTshapedAreaLines(
    const TrajectoryPoint &ego_state, const planning_math::Box2d &map_boundary,
    std::vector<ObstacleBox> nearby_boxes,
    const std::vector<planning_math::Vec2d> &points) {
  Vec2d slot_front_left(corners_[0].x, corners_[0].y);
  Vec2d slot_front_right(corners_[3].x, corners_[3].y);
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
  Box2d slot_box_on_right(local_slot);
  slot_box_on_right.Shift(local_front_right - local_front_left);
  slot_box_on_left.Shift(local_front_left - local_front_right);

  bool left_slot_empty = true, right_slot_empty = true;
  double left_lower_height = local_map_boundary.min_y();
  double right_lower_height = local_map_boundary.min_y();
  double left_x = local_map_boundary.min_x();
  double right_x = local_map_boundary.max_x();

  for (const Box2d &box : local_lower_boxes) {
    if (box.HasOverlap(slot_box_on_left)) {
      // process left box
      left_slot_empty = false;
      left_lower_height = std::max(box.max_y(), left_lower_height);
      left_x = std::max(box.max_x(), left_x);
    } else if (box.HasOverlap(slot_box_on_right)) {
      // process right box
      right_slot_empty = false;
      right_lower_height = std::max(box.max_y(), right_lower_height);
      right_x = std::min(box.min_x(), right_x);
    }
  }
  for (auto &p : local_points) {
    if (slot_box_on_left.IsPointIn(p)) {
      left_slot_empty = false;
      left_lower_height = std::max(p.y(), left_lower_height);
      left_x = std::max(p.x(), left_x);
    } else if (slot_box_on_right.IsPointIn(p)) {
      right_slot_empty = false;
      right_lower_height = std::max(p.y(), right_lower_height);
      right_x = std::min(p.x(), right_x);
    }
  }

  // TODO@lizhiqiang: if need, add default distance param into apa.yaml
  if (left_slot_empty) {
    // std::cout << "left slot is empty!!!!!" << std::endl;
    left_lower_height = -0.05 * std::hypot(VehicleParam::Instance()->length,
                                           VehicleParam::Instance()->width);
    left_x = -(slot_front_left - slot_front_right).Length();
  }
  if (right_slot_empty) {
    // std::cout << "right slot is empty!!!!!" << std::endl;
    right_lower_height = -0.05 * std::hypot(VehicleParam::Instance()->length,
                                            VehicleParam::Instance()->width);
    right_x = (slot_front_left - slot_front_right).Length();
  }

  // ensure that lower lines don't cross map_boundary
  left_lower_height = std::min(left_lower_height, max_lower_bound_height);
  right_lower_height = std::min(right_lower_height, max_lower_bound_height);
  left_lower_height = std::max(left_lower_height, local_map_boundary.min_y());
  right_lower_height = std::max(right_lower_height, local_map_boundary.min_y());
  left_x = std::max(left_x, local_map_boundary.min_x());
  right_x = std::min(right_x, local_map_boundary.max_x());

  // avoid lower region being too wide for parallel slot
  const double max_adjust_width =
      CarParams::GetInstance()
          ->car_config.slot_config.extra_max_rule_slot_width_parallel;
  double max_region_width = max_adjust_width + VehicleParam::Instance()->length;
  if (right_x - left_x > max_region_width) {
    if (left_x > -max_region_width / 2) {
      right_x = left_x + max_region_width;
    } else if (right_x < max_region_width / 2) {
      left_x = right_x - max_region_width;
    } else {
      left_x = -max_region_width / 2;
      right_x = max_region_width / 2;
    }
  }

  local_T_lines.push_back(
      LineSegment2d(Vec2d(local_map_boundary.min_x(), left_lower_height),
                    Vec2d(left_x, left_lower_height)));
  local_T_lines.push_back(
      LineSegment2d(Vec2d(left_x, left_lower_height),
                    Vec2d(left_x, local_map_boundary.min_y())));
  local_T_lines.push_back(
      LineSegment2d(Vec2d(right_x, local_map_boundary.min_y()),
                    Vec2d(right_x, right_lower_height)));
  local_T_lines.push_back(
      LineSegment2d(Vec2d(right_x, right_lower_height),
                    Vec2d(local_map_boundary.max_x(), right_lower_height)));

  for (auto &line : local_T_lines) {
    T_lines.push_back(tf2d_inv(local_frame_pose, line));
  }
  return T_lines;
}

} // namespace parking

} // namespace msquare
