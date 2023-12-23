#include "common/math/math_utils.h"
#include "common/parking_lot.h"
#include "common/planning_config.h"
#include "planning/common/common.h"
#include <exception>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace msquare {

namespace parking {

using namespace planning_math;

BaseParkingSlot::BaseParkingSlot(const std::string cfg_file,
                                 std::vector<Point3D> lot_corners,
                                 bool is_relative_left)
    : corners_(lot_corners), is_relative_left_(is_relative_left) {
  initConfig(cfg_file);
  initBox();
}

void BaseParkingSlot::initConfig(const std::string cfg_file) {
  YAML::Node yaml_node = YAML::LoadFile(cfg_file);
  lot_entrance_extra_gap_ = yaml_node["lot_entrance_extra_gap"].as<double>();
  side_safety_lot_entrance_extra_gap_ =
      yaml_node["side_safety_lot_entrance_extra_gap"].as<double>();
  lot_side_entrance_clip_ratio_ =
      yaml_node["lot_side_entrance_clip_ratio"].as<double>();
  side_safety_lot_side_entrance_clip_ratio_ =
      yaml_node["side_safety_lot_side_entrance_clip_ratio"].as<double>();

  narrow_road_width_threshold_ =
      yaml_node["narrow_road_width_threshold"].as<double>();
  narrow_road_lot_entrance_extra_gap_ =
      yaml_node["narrow_road"]["lot_entrance_extra_gap"].as<double>();
  narrow_road_lot_side_entrance_clip_ratio_ =
      yaml_node["narrow_road"]["lot_side_entrance_clip_ratio"].as<double>();

  back_distance_buffer_ = yaml_node["back_distance_buffer"].as<double>();
  distance_wheel_to_wheelstop_ =
      yaml_node["distance_wheel_to_wheelstop"].as<double>();
  distance_back_to_wheelstop_ =
      yaml_node["distance_back_to_wheelstop"].as<double>();
  enable_get_heading_with_front_corners_ =
      yaml_node["enable_get_heading_with_front_corners"].as<bool>();

  lot_side_entrance_clip_ratio_left_ = lot_side_entrance_clip_ratio_right_ =
      lot_side_entrance_clip_ratio_;
  lot_entrance_extra_gap_left_ = lot_entrance_extra_gap_right_ =
      lot_entrance_extra_gap_;
  enable_get_heading_with_front_corners_ =
      yaml_node["enable_get_heading_with_front_corners"].as<bool>();

  default_depth_ = yaml_node["default_depth"].as<double>();
}

void BaseParkingSlot::updateCorners(const std::vector<Point3D> lot_corners) {
  if (lot_corners.size() != 4) {
    // throw std::logic_error("updateCorners:: not 4 corners is provided.");
    MSD_LOG(WARN, "Parking slot corners size is not 4! Size is %d!",
            lot_corners.size());
    return;
  }

  std::vector<Point3D> corners(lot_corners);
  Vec2d heading_vec_current(corners_.at(3).x - corners_.at(2).x,
                            corners_.at(3).y - corners_.at(2).y);
  Vec2d heading_vec_new;
  for (size_t i = 0; i < 3; ++i) {
    heading_vec_new = Vec2d(corners.at(3).x - corners.at(2).x,
                            corners.at(3).y - corners.at(2).y);
    if (heading_vec_current.InnerProd(heading_vec_new) < std::cos(M_PI_4)) {
      corners.insert(corners.begin(), corners.back());
      corners.pop_back();
    } else {
      break;
    }
  }
  corners_ = corners;
  initBox();
}

Pose2D BaseParkingSlot::getParkingInPose(double front_edge_to_rear,
                                         double back_edge_to_rear,
                                         double brake_buffer,
                                         const Pose2D &init_pose) {
  Pose2D lot_front_pose;
  lot_front_pose.x = (corners_[0].x + corners_[3].x) / 2;
  lot_front_pose.y = (corners_[0].y + corners_[3].y) / 2;
  Pose2D lot_back_pose;
  lot_back_pose.x = (corners_[1].x + corners_[2].x) / 2;
  lot_back_pose.y = (corners_[1].y + corners_[2].y) / 2;

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
  // PlanningContext::Instance()->mutable_parking_behavior_planner_output()->debug_stringstream
  //    << "hzmdebug-pld-lot_length = " << lot_length << std::endl;

  double target2back = back_edge_to_rear + back_distance_buffer_;
  target2back = std::max(back_edge_to_rear, lot_length - front_edge_to_rear);
  double target2back_wrt_wheelstop =
      distance_back_to_wheelstop_ + distance_wheel_to_wheelstop_;
  target2back = std::max(target2back, target2back_wrt_wheelstop);

  Pose2D target_in_lot_front_frame;
  target_in_lot_front_frame.x = -(
      lot_length -
      (target2back + brake_buffer)); // with buffer related to control precision
  return planning_math::tf2d_inv(lot_front_pose, target_in_lot_front_frame);
}

void BaseParkingSlot::processLinesAroundLot(
    std::vector<planning_math::LineSegment2d> &lines) {
  double left_gap, right_gap;
  planning_math::Box2d box(box_);
  left_gap = lot_entrance_extra_gap_left_;
  right_gap = lot_entrance_extra_gap_right_;

  const double max_distance = box.length() / 2.0;
  const msquare::planning_math::Box2d lot_extend1(
      box.center(), box.heading(), box.length(), box.width() + left_gap * 2);
  const msquare::planning_math::Box2d lot_extend2(
      box.center(), box.heading(), box.length(), box.width() + right_gap * 2);
  std::vector<msquare::planning_math::LineSegment2d> line_result;
  const std::vector<planning_math::Vec2d> corners = box.GetAllCorners();
  // const planning_math::Vec2d left_corner = box.GetAllCorners()[0];
  // const planning_math::Vec2d right_corner = box.GetAllCorners()[1];
  const planning_math::Vec2d left_corner_extend =
      lot_extend1.GetAllCorners()[1];
  const planning_math::Vec2d right_corner_extend =
      lot_extend2.GetAllCorners()[0];
  msquare::planning_math::LineSegment2d leftCorner_to_rightCorner(
      left_corner_extend, right_corner_extend);

  int sizeOfLine = lines.size();
  std::vector<int> line_in;

  for (int i = 0; i < sizeOfLine; i++) {
    double dist1 = lines[i].DistanceTo(left_corner_extend);
    double dist2 = lines[i].DistanceTo(right_corner_extend);
    double dist3 = leftCorner_to_rightCorner.DistanceTo(lines[i].start());
    double dist4 = leftCorner_to_rightCorner.DistanceTo(lines[i].end());
    if (dist1 < max_distance || dist2 < max_distance || dist3 < max_distance ||
        dist4 < max_distance) {
      if (!((lines[i].DistanceTo(left_corner_extend) ==
                 left_corner_extend.DistanceTo(lines[i].start()) &&
             lines[i].DistanceTo(right_corner_extend) ==
                 right_corner_extend.DistanceTo(lines[i].start())) ||
            (lines[i].DistanceTo(left_corner_extend) ==
                 left_corner_extend.DistanceTo(lines[i].end()) &&
             lines[i].DistanceTo(right_corner_extend) ==
                 right_corner_extend.DistanceTo(lines[i].end())))) {
        line_in.push_back(i);
      } else {
        line_result.push_back(lines[i]);
      }
    } else {
      line_result.push_back(lines[i]);
    }
  }
  if (line_in.empty()) {
    return;
  } else {
    for (size_t i = 0; i < line_in.size(); i++) {
      int numOfLine = line_in[i];
      if ((lines[numOfLine].DistanceTo(left_corner_extend) ==
               left_corner_extend.DistanceTo(lines[numOfLine].start()) &&
           lines[numOfLine].DistanceTo(right_corner_extend) ==
               right_corner_extend.DistanceTo(lines[numOfLine].end())) ||
          (lines[numOfLine].DistanceTo(left_corner_extend) ==
               left_corner_extend.DistanceTo(lines[numOfLine].end()) &&
           lines[numOfLine].DistanceTo(right_corner_extend) ==
               right_corner_extend.DistanceTo(lines[numOfLine].start()))) {
        continue;
      }

      else if (std::abs(planning_math::InnerProd(
                   lines[numOfLine].unit_direction().x(),
                   lines[numOfLine].unit_direction().y(), box.cos_heading(),
                   box.sin_heading())) > std::cos(M_PI_4))

      {
        // std::cout << "hzmdebug-process line find vertical one: " << numOfLine
        // << ", len: " <<  lines[numOfLine].length() << std::endl;
        if (lot_extend2.HasOverlap(lines[numOfLine])) {
          continue;
        } else {
          line_result.push_back(lines[numOfLine]);
        }
      }
      {
        const planning_math::Vec2d per_unit_direction(
            lines[numOfLine].unit_direction().y(),
            -lines[numOfLine].unit_direction().x());
        auto lines_close = lines[numOfLine];
        planning_math::Vec2d line_center = lines[numOfLine].center();
        // extend
        double extend_length =
            lines_close.length() + box.width() * 2 + left_gap + right_gap;
        planning_math::Vec2d unit_direction_extend(
            lines_close.unit_direction().x(), lines_close.unit_direction().y());

        planning_math::Vec2d line_pt1(
            line_center.x() + unit_direction_extend.x() * extend_length,
            line_center.y() + unit_direction_extend.y() * extend_length);
        planning_math::Vec2d line_pt2(
            line_center.x() - unit_direction_extend.x() * extend_length,
            line_center.y() - unit_direction_extend.y() * extend_length);
        planning_math::LineSegment2d lines_close_extend(line_pt1, line_pt2);

        const double left_dist =
            lines_close_extend.DistanceTo(left_corner_extend);
        const double right_dist =
            lines_close_extend.DistanceTo(right_corner_extend);

        const planning_math::Vec2d left_intersection_one(
            left_corner_extend.x() + left_dist * per_unit_direction.x(),
            left_corner_extend.y() + left_dist * per_unit_direction.y());
        const planning_math::Vec2d left_intersection_two(
            left_corner_extend.x() - left_dist * per_unit_direction.x(),
            left_corner_extend.y() - left_dist * per_unit_direction.y());
        const planning_math::Vec2d left_intersection =
            lines_close_extend.IsPointIn(left_intersection_one) == true
                ? left_intersection_one
                : left_intersection_two;
        // std::cout << lines_close_extend.IsPointIn(left_intersection_one) <<
        // "," << lines_close_extend.IsPointIn(left_intersection_two) <<
        // std::endl;
        const planning_math::Vec2d right_intersection_one(
            right_corner_extend.x() + right_dist * per_unit_direction.x(),
            right_corner_extend.y() + right_dist * per_unit_direction.y());
        const planning_math::Vec2d right_intersection_two(
            right_corner_extend.x() - right_dist * per_unit_direction.x(),
            right_corner_extend.y() - right_dist * per_unit_direction.y());
        // std::cout << lines_close_extend.IsPointIn(right_intersection_one) <<
        // "," << lines_close_extend.IsPointIn(right_intersection_two) <<
        // std::endl;
        const planning_math::Vec2d right_intersection =
            lines_close_extend.IsPointIn(right_intersection_one) == true
                ? right_intersection_one
                : right_intersection_two;

        std::vector<planning_math::LineSegment2d>::iterator it =
            lines.begin() + numOfLine;
        planning_math::Vec2d log_dir(std::cos(box.heading()),
                                     std::sin(box.heading()));
        planning_math::LineSegment2d log_line(box.center(),
                                              box.center() + log_dir * 5.0);
        bool left_intersected = false, right_intersected = false;
        if (lines_close.IsPointIn(left_intersection)) {
          planning_math::LineSegment2d tmp(left_intersection,
                                           lines_close.start());
          planning_math::LineSegment2d left_to_right(left_intersection,
                                                     right_intersection);
          planning_math::LineSegment2d tmp2(left_intersection,
                                            lines_close.end());
          // planning_math::LineSegment2d tmp3(left_intersection +
          // tmp.unit_direction() * left_gap, lines_close.start());
          // planning_math::LineSegment2d tmp4(left_intersection +
          // tmp2.unit_direction() * left_gap, lines_close.end());
          left_intersected = true;
          if (left_to_right.unit_direction().x() *
                      lines_close.unit_direction().x() +
                  left_to_right.unit_direction().y() *
                      lines_close.unit_direction().y() <
              0) {
            line_result.push_back(tmp2);
          } else {
            line_result.push_back(tmp);
          }
        }
        if (lines_close.IsPointIn(right_intersection)) {
          planning_math::LineSegment2d tmp(right_intersection,
                                           lines_close.start());
          planning_math::LineSegment2d left_to_right(left_intersection,
                                                     right_intersection);
          planning_math::LineSegment2d tmp2(right_intersection,
                                            lines_close.end());
          // planning_math::LineSegment2d tmp3(right_intersection +
          // tmp.unit_direction() * right_gap, lines_close.start());
          // planning_math::LineSegment2d tmp4(right_intersection +
          // tmp2.unit_direction() * right_gap, lines_close.end());
          right_intersected = true;
          if (left_to_right.unit_direction().x() *
                      lines_close.unit_direction().x() +
                  left_to_right.unit_direction().y() *
                      lines_close.unit_direction().y() >
              0) {
            line_result.push_back(tmp2);
          } else {

            line_result.push_back(tmp);
          }
        }
        if ((!left_intersected) && (!right_intersected))
          line_result.push_back(lines_close);
      }
    }
  }
  lines = line_result;
}

std::vector<planning_math::LineSegment2d>
BaseParkingSlot::genLotWalls(double vehicle_width) {

  using namespace planning_math;
  Box2d lot_box(box_);

  // format lot_corners
  std::vector<Vec2d> lot_corners_vec2d = lot_box.GetAllCorners();
  Vec2d left_front_corner, right_front_corner;

  left_front_corner =
      lot_side_entrance_clip_ratio_left_ * lot_corners_vec2d[2] +
      (1 - lot_side_entrance_clip_ratio_left_) * lot_corners_vec2d[1];
  right_front_corner =
      lot_side_entrance_clip_ratio_right_ * lot_corners_vec2d[3] +
      (1 - lot_side_entrance_clip_ratio_right_) * lot_corners_vec2d[0];
  lot_corners_vec2d.at(1) = left_front_corner;
  lot_corners_vec2d.at(0) = right_front_corner;

  // result
  std::vector<LineSegment2d> walls;
  walls.emplace_back(lot_corners_vec2d.at(1), lot_corners_vec2d.at(2));
  walls.emplace_back(lot_corners_vec2d.at(2), lot_corners_vec2d.at(3));
  walls.emplace_back(lot_corners_vec2d.at(3), lot_corners_vec2d.at(0));

  return walls;
}

std::vector<planning_math::LineSegment2d>
BaseParkingSlot::genFrontWings(double span) {
  using namespace planning_math;

  std::vector<LineSegment2d> walls;
  if (span < lot_entrance_extra_gap_) {
    return walls;
  }

  Box2d lot_box(box_);
  Pose2D box_center_pose{lot_box.center().x(), lot_box.center().y(),
                         lot_box.heading()};

  double wing_start_x = lot_box.length() / 2;
  LineSegment2d left_wing(
      Vec2d(wing_start_x, lot_box.half_width() + lot_entrance_extra_gap_left_),
      Vec2d(wing_start_x, span));
  LineSegment2d right_wing(
      Vec2d(wing_start_x,
            -lot_box.half_width() - lot_entrance_extra_gap_right_),
      Vec2d(wing_start_x, -span));
  walls.push_back(left_wing);
  walls.push_back(right_wing);

  for (size_t i = 0; i < walls.size(); ++i) {
    walls[i] = tf2d_inv(box_center_pose, walls[i]);
  }

  return walls;
}

void BaseParkingSlot::initBox() {
  Pose2D lot_front_pose;
  lot_front_pose.x = (corners_[0].x + corners_[3].x) / 2;
  lot_front_pose.y = (corners_[0].y + corners_[3].y) / 2;
  Pose2D lot_back_pose;
  lot_back_pose.x = (corners_[1].x + corners_[2].x) / 2;
  lot_back_pose.y = (corners_[1].y + corners_[2].y) / 2;
  Pose2D lot_center_pose;
  lot_center_pose.x = (lot_front_pose.x + lot_back_pose.x) / 2;
  lot_center_pose.y = (lot_front_pose.y + lot_back_pose.y) / 2;
  lot_center_pose.theta = atan2(lot_front_pose.y - lot_back_pose.y,
                                lot_front_pose.x - lot_back_pose.x);
  double lot_length = std::hypot(lot_front_pose.y - lot_back_pose.y,
                                 lot_front_pose.x - lot_back_pose.x);
  double lot_width =
      std::hypot(corners_[0].x - corners_[3].x, corners_[0].y - corners_[3].y);

  box_ = planning_math::Box2d(
      planning_math::Vec2d{lot_center_pose.x, lot_center_pose.y},
      lot_center_pose.theta, lot_length, lot_width);
}

void BaseParkingSlot::matchRoadWidth(double width) {
  if (width <= narrow_road_width_threshold_) {
    // std::cout << "BaseParkingSlot narrow road: " << width << std::endl;
    lot_entrance_extra_gap_ = narrow_road_lot_entrance_extra_gap_;
    lot_side_entrance_clip_ratio_ = narrow_road_lot_side_entrance_clip_ratio_;
    lot_side_entrance_clip_ratio_left_ = lot_side_entrance_clip_ratio_right_ =
        lot_side_entrance_clip_ratio_;
  }
}

void BaseParkingSlot::setSafeMode(bool left_safe, bool right_safe) {
  if (left_safe) {
    lot_side_entrance_clip_ratio_left_ =
        side_safety_lot_side_entrance_clip_ratio_;
    lot_entrance_extra_gap_left_ = side_safety_lot_entrance_extra_gap_;
  }

  if (right_safe) {
    lot_side_entrance_clip_ratio_right_ =
        side_safety_lot_side_entrance_clip_ratio_;
    lot_entrance_extra_gap_right_ = side_safety_lot_entrance_extra_gap_;
  }

  if (!left_safe && !right_safe) {
    // std::cerr << "BaseParkingSlot::setSafeMode is called in vain" <<
    // std::endl;
  }
}

planning_math::LineSegment2d BaseParkingSlot::genBottomWall() {
  std::vector<Vec2d> lot_corners_vec2d = box_.GetAllCorners();

  double default_length_interp = std::max(1.0, default_depth_ / box_.length());
  double dual_default_length_interp = 1 - default_length_interp;
  Vec2d default_left_bottom(default_length_interp * lot_corners_vec2d.at(2) +
                            dual_default_length_interp *
                                lot_corners_vec2d.at(1));
  Vec2d default_right_bottom(default_length_interp * lot_corners_vec2d.at(3) +
                             dual_default_length_interp *
                                 lot_corners_vec2d.at(0));

  return LineSegment2d(default_left_bottom, default_right_bottom);
}

std::vector<TrajectoryPoint>
BaseParkingSlot::getCenterLine(double front_edge_to_rear,
                               double back_edge_to_rear, double brake_buffer,
                               double wheel_stop_depth) {
  std::vector<TrajectoryPoint> center_line_traj;
  const double STEP_SIZE = 0.25;
  Vec2d front_center((corners_[0].x + corners_[3].x) / 2,
                     (corners_[0].y + corners_[3].y) / 2);
  Pose2D parking_in_pose = getParkingInPose(
      front_edge_to_rear, back_edge_to_rear, brake_buffer, Pose2D{0, 0, 0});
  Vec2d back_center(parking_in_pose.x, parking_in_pose.y);
  LineSegment2d center_line(back_center, front_center);
  double heading = center_line.heading();
  for (double s = 0; s < center_line.length(); s += STEP_SIZE) {
    Vec2d point = center_line.getPoint(s);
    TrajectoryPoint tp;
    tp.path_point.theta = heading;
    tp.path_point.x = point.x();
    tp.path_point.y = point.y();
    tp.v = -0.5;
    center_line_traj.push_back(tp);
  }
  Vec2d point = center_line.getPoint(center_line.length());
  TrajectoryPoint tp;
  tp.path_point.theta = heading;
  tp.path_point.x = point.x();
  tp.path_point.y = point.y();
  tp.v = -0.5;
  center_line_traj.push_back(tp);

  std::reverse(center_line_traj.begin(), center_line_traj.end());
  return center_line_traj;
}

bool BaseParkingSlot::isLateralInside(const Pose2D &pose,
                                      const FootprintModelPtr footprint_model) {
  // TODO@all: what the hell is reached for prependicular slot?
  return false;

  if (!box_.IsPointIn(Vec2d(pose.x, pose.y))) {
    return false;
  }

  std::vector<LineSegment2d> box_edges = box_.GetAllEdges();
  std::vector<LineSegment2d> box_edges_to_check;
  box_edges_to_check.push_back(box_edges[1]);
  box_edges_to_check.push_back(box_edges[3]);
  if (footprint_model->checkOverlap(pose, box_edges_to_check)) {
    return false;
  }

  return true;
}

bool BaseParkingSlot::isInside(const Pose2D &pose,
                               const FootprintModelPtr footprint_model) {
  // TODO@all: what the hell is reached for prependicular slot?
  return false;

  if (!box_.IsPointIn(Vec2d(pose.x, pose.y))) {
    return false;
  }
  std::vector<LineSegment2d> box_edges = box_.GetAllEdges();
  if (footprint_model->checkOverlap(pose, box_edges)) {
    return false;
  }

  return true;
}

bool BaseParkingSlot::isAlongsideLot(const Pose2D &pose, double thres) {
  // TODO@all: what the hell is reached for prependicular slot?
  return false;

  Vec2d heading_vec;
  heading_vec = Vec2d(corners_.at(0).x - corners_.at(1).x,
                      corners_.at(0).y - corners_.at(1).y);
  double slot_heading = heading_vec.Angle();

  return std::fabs(NormalizeAngle(pose.theta - slot_heading)) < thres;
}

bool BaseParkingSlot::isVehicleReached(const VehicleParam *vehicle_param,
                                       const Pose2D &pose,
                                       const FootprintModelPtr footprint_model,
                                       double wheel_stop_depth) {
  // TODO@all: what the hell is reached for prependicular slot?
  return false;

  // check if inside
  if (!isInside(pose, footprint_model)) {
    return false;
  }
  // // check if longitudinal feasible
  // std::vector<LineSegment2d> box_edges = box_.GetAllEdges();
  // LineSegment2d bottom_edge = box_edges[2];
  // LineSegment2d top_edge = box_edges[0];
  // if ((bottom_edge.DistanceTo(Vec2d(pose.x, pose.y)) -
  //      vehicle_param->back_edge_to_center) -
  //         (top_edge.DistanceTo(Vec2d(pose.x, pose.y)) -
  //          vehicle_param->front_edge_to_center) >
  //     0.2) {
  //   return false;
  // }
  return true;
}

} // namespace parking

} // namespace msquare
