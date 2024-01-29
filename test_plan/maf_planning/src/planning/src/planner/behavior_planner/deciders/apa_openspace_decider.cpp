#include "planner/behavior_planner/deciders/apa_openspace_decider.h"
#include "common/math/math_utils.h"
#include "common/planning_config.h"
#include "common/planning_context.h"
#include "common/utils/yaml_utils.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <fstream>

#include "planner/behavior_planner/deciders/openspace_utils.h"
#include "planning/common/common.h"

namespace msquare {

using namespace planning_math;
using namespace parking;

ApaOpenspaceDecider::ApaOpenspaceDecider(
    const std::shared_ptr<WorldModel> &world_model)
    : BaseOpenspaceDecider(world_model) {
  log_file_prefix_ =
      PlanningConfig::Instance()->config_files().log_path + "apa_decider";

  parking_lot_ = PlanningContext::Instance()
                     ->mutable_parking_behavior_planner_output()
                     ->parking_lot;

  lot_box_ = parking_lot_->getBox();
  // std::cout << "hzmdebug-lot-" << lot_box_.length() << ", " <<
  // lot_box_.width() << std::endl;
  lot_box_on_left_ = lot_box_on_right_ = lot_box_;
  const std::vector<Vec2d> &lot_corners = lot_box_.GetAllCorners();
  lot_box_on_right_.Shift(lot_corners.at(0) - lot_corners.at(1));
  lot_box_on_left_.Shift(lot_corners.at(1) - lot_corners.at(0));
}

ApaOpenspaceDecider::~ApaOpenspaceDecider() { ; }

void ApaOpenspaceDecider::reset() {
  boundary_lines_.clear();
  BaseOpenspaceDecider::reset();
}

bool ApaOpenspaceDecider::isNarrowChannelScenario() {
  std::vector<planning_math::Vec2d> local_points{};
  for (const auto &pt : points_) {
    local_points.push_back(planning_math::tf2d(local_frame_pose_, pt));
  }

  Pose2D ego_pose;
  ego_pose.x = ego_state_.path_point.x;
  ego_pose.y = ego_state_.path_point.y;
  ego_pose.theta = ego_state_.path_point.theta;
  Pose2D ego_pose_local = planning_math::tf2d(local_frame_pose_, ego_pose);

  double max_x = ego_pose_local.x + 10.0;
  double min_x = lot_box_.length() / 2;
  double max_y = 5.0;
  double min_y = -5.0;

  for (const auto &local_pt : local_points) {
    if (local_pt.y() < max_y && local_pt.y() > min_y) {
      if (local_pt.x() > ego_pose_local.x) {
        max_x = std::min(max_x, local_pt.x());
      } else {
        min_x = std::max(min_x, local_pt.x());
      }
    }
  }
  const double narrow_scenario_channel_width =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.narrow_channel_width;
  return max_x - min_x < narrow_scenario_channel_width;
}

void ApaOpenspaceDecider::InitLocalFrame() {
  if (!is_pre_apa_) {
    auto &input =
        PlanningContext::Instance()->parking_behavior_planner_output();
    ego_state_ = input.init_traj_point;
    target_state_ = input.target_traj_point;
  }

  Pose2D target_state_pose, ego_pose;
  target_state_pose.x = target_state_.path_point.x;
  target_state_pose.y = target_state_.path_point.y;
  target_state_pose.theta = target_state_.path_point.theta;
  ego_pose.x = ego_state_.path_point.x;
  ego_pose.y = ego_state_.path_point.y;
  ego_pose.theta = ego_state_.path_point.theta;

  local_frame_pose_ = Pose2D(lot_box_.center_x(), lot_box_.center_y(),
                             parking_lot_->getOpeningHeading());

  Pose2D ego_pose_local = planning_math::tf2d(local_frame_pose_, ego_pose);
  Pose2D target_pose_local =
      planning_math::tf2d(local_frame_pose_, target_state_pose);
  target_state_local_.path_point.x = target_pose_local.x;
  target_state_local_.path_point.y = target_pose_local.y;
  target_state_local_.path_point.theta = target_pose_local.theta;
  ego_state_local_.path_point.x = ego_pose_local.x;
  ego_state_local_.path_point.y = ego_pose_local.y;
  ego_state_local_.path_point.theta = ego_pose_local.theta;
}

void ApaOpenspaceDecider::get_openspace_boundary() {
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  const CarParams *car_params = CarParams::GetInstance();
  const double u_turn_envelope_radius = std::hypot(
      car_params->min_turn_radius + VehicleParam::Instance()->width / 2,
      VehicleParam::Instance()->front_edge_to_center);
  const double x_buffer_near_lot = 1.0;
  const double y_buffer_near_lot = car_params->min_turn_radius / 2;

  Box2d box_lot_with_buffer = tf2d(local_frame_pose_, lot_box_);
  box_lot_with_buffer.Shift(u_turn_envelope_radius / 2 *
                            Vec2d(box_lot_with_buffer.cos_heading(),
                                  box_lot_with_buffer.sin_heading()));
  box_lot_with_buffer.LongitudinalExtend(u_turn_envelope_radius +
                                         2 * x_buffer_near_lot);
  box_lot_with_buffer.LateralExtend(y_buffer_near_lot * 2);
  AABox2d map_aabox_local = box_lot_with_buffer.GetAABox();

  double nearby_buffer = u_turn_envelope_radius;
  map_aabox_local.MergeFrom(
      Vec2d(ego_state_local_.path_point.x + nearby_buffer,
            ego_state_local_.path_point.y + nearby_buffer));
  map_aabox_local.MergeFrom(
      Vec2d(ego_state_local_.path_point.x - nearby_buffer,
            ego_state_local_.path_point.y - nearby_buffer));
  Vec2d virtual_gear_switch_point_local(car_params->min_turn_radius,
                                        -car_params->min_turn_radius);
  if (std::sin(ego_state_local_.path_point.theta) > 0) {
    virtual_gear_switch_point_local =
        Vec2d(car_params->min_turn_radius, car_params->min_turn_radius);
  }
  map_aabox_local.MergeFrom(virtual_gear_switch_point_local +
                            Vec2d(nearby_buffer, nearby_buffer));
  map_aabox_local.MergeFrom(virtual_gear_switch_point_local +
                            Vec2d(-nearby_buffer, -nearby_buffer));
  // map_aabox_local.MergeFrom(Vec2d(target_state_local_.path_point.x +
  // nearby_buffer, target_state_local_.path_point.y + nearby_buffer));
  // map_aabox_local.MergeFrom(Vec2d(target_state_local_.path_point.x -
  // nearby_buffer, target_state_local_.path_point.y - nearby_buffer));

  Box2d local_map_box(map_aabox_local);
  MSD_LOG(INFO, "hzmdebug-local_map_box [width, length] = [%lf, %lf].\n",
          local_map_box.width(), local_map_box.length());
  // MSD_LOG(INFO, "hzmdebug-local_map_box_old [width, length] = [%lf, %lf].\n",
  //   local_map_box_old.width(), local_map_box_old.length());

  map_boundary_ = planning_math::tf2d_inv(local_frame_pose_, local_map_box);

  auto squaremap_info = PlanningContext::Instance()->mutable_square_map();
  auto corners = map_boundary_.GetAllCorners();
  for (size_t i = 0; i < corners.size(); i++) {
    squaremap_info->box_corners[i].x = corners[i].x();
    squaremap_info->box_corners[i].y = corners[i].y();
  }
  squaremap_info->box_height = world_model_->get_ego_state().ego_enu.position.z;
}

bool ApaOpenspaceDecider::gather_map_info() {
  using planning_math::Box2d;
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  reset();
  if (world_model_->is_parking_lvp() || world_model_->is_parking_apa()) {
    // construct parking lot box
    bool parking_lot_box_available =
        (PlanningContext::Instance()
             ->mutable_parking_behavior_planner_output()
             ->parking_lot != nullptr);
    planning_math::Box2d parking_lot_box;
    planning_math::Box2d parking_lot_box_bottom;
    if (parking_lot_box_available) {
      const planning_math::Box2d parking_lot_box_origin =
          PlanningContext::Instance()
              ->parking_behavior_planner_output()
              .parking_lot->getBox();
      parking_lot_box =
          planning_math::Box2d(parking_lot_box_origin.center() +
                                   0.5 * planning_math::Vec2d::CreateUnitVec2d(
                                             parking_lot_box_origin.heading()),
                               parking_lot_box_origin.heading(),
                               parking_lot_box_origin.length() + 1.2,
                               parking_lot_box_origin.width());
      parking_lot_box_bottom = planning_math::Box2d(
          parking_lot_box_origin.center() -
              0.5 * parking_lot_box_origin.length() *
                  planning_math::Vec2d::CreateUnitVec2d(
                      parking_lot_box_origin.heading()),
          parking_lot_box_origin.heading(), parking_lot_box_origin.length(),
          parking_lot_box_origin.width() * 2.0);
    }
    // auto points = world_model_->obstacle_manager().get_uss_points().Items();
    // auto points = world_model_->obstacle_manager().get_points().Items();
    // // std::cout << "hzmdebug-merge: points nubmer =" << points.size() <<
    // // std::endl;
    // for (auto &point : points) {
    //   // remove the uss points which in parking lot
    //   if (parking_lot_box_available &&
    //       (parking_lot_box.IsPointIn(point->PerceptionBoundingBox().center())
    //       ||
    //        parking_lot_box_bottom.IsPointIn(
    //            point->PerceptionBoundingBox().center()))) {
    //     continue;
    //   }
    //   points_.emplace_back(point->PerceptionBoundingBox().center());
    // }
    // uss groundline
    const auto &slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
    bool not_remove_any_pts =
        (slot_info.type.value == ParkingSlotType::PARALLEL);
    for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
      if (obs.id != PlanningContext::Instance()
                        ->parking_behavior_planner_output()
                        .parking_slot_info.id) {
        continue;
      }
      if (obs.type == GroundLineType::GROUND_LINE_TYPE_USS_REALTIME_OBSTACLE) {
        continue;
      }
      if (!(obs.type == GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN ||
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_WALL ||
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_PILLAR ||
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_FENCE ||
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP ||
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SPECIAL ||
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_VEHICLE ||
            obs.type ==
                GroundLineType::GROUND_LINE_USS_TYPE_ONLY_USS_UNKNOWN)) {
        for (int i = 0; i < obs.pts.size(); i++) {
          Vec2d point{obs.pts[i].x, obs.pts[i].y};
          // remove the uss points which in parking lot
          if (!not_remove_any_pts && parking_lot_box_available &&
              (parking_lot_box.IsPointIn(point) ||
               (slot_info.type.value == ParkingSlotType::PERPENDICULAR &&
                parking_lot_box_bottom.IsPointIn(point)))) {
            if (!parking_lot_box.IsPointOnBoundarySoft(point)) {
              continue;
            }
          }
          points_.emplace_back(point);
        }
      } else if (obs.pts.size() % 2 == 0 &&
                 PlanningContext::Instance()
                         ->parking_behavior_planner_output()
                         .parking_slot_info.type.value ==
                     ParkingSlotType::PARALLEL) {
        if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP) {
          for (int i = 0; i < obs.pts.size(); i = i + 2) {
            lines_.push_back(
                ObstacleLine(Vec2d(obs.pts[i].x, obs.pts[i].y),
                             Vec2d(obs.pts[i + 1].x, obs.pts[i + 1].y),
                             OpenspaceObstacleType::PARKINGLINE));
          }
        } else {
          for (int i = 0; i < obs.pts.size(); i = i + 2) {
            lines_.push_back(
                ObstacleLine(Vec2d(obs.pts[i].x, obs.pts[i].y),
                             Vec2d(obs.pts[i + 1].x, obs.pts[i + 1].y),
                             OpenspaceObstacleType::WALL));
          }
        }
      }
    }
    // get fusion slots
    if (world_model_->is_parking_lvp() || world_model_->is_parking_apa()) {
      const ParkingMapInfo &parking_map_info =
          world_model_->get_parking_map_info();
      for (auto &parking_lot :
           parking_map_info.parking_lots_detection_fusion_results) {
        if (parking_lot.id != PlanningContext::Instance()
                                  ->parking_behavior_planner_output()
                                  .parking_slot_info.id &&
            !parking_lot.corners.empty()) {
          // ignore all slots besides
          Pose2D target_parking_lot_pose(parking_lot_box.center().x(),
                                         parking_lot_box.center().y(),
                                         parking_lot_box.heading());
          Vec2d slot_corner_0(parking_lot.corners.at(0).position.x,
                              parking_lot.corners.at(0).position.y);
          if (tf2d(target_parking_lot_pose, slot_corner_0).x() <
              parking_lot_box.length()) {
            continue;
          }

          std::vector<planning_math::LineSegment2d> tmp_slot_lines(3);
          for (int i = 0; i + 1 < parking_lot.corners.size(); ++i) {
            const Point3D &pt1 = parking_lot.corners.at(i).position;
            const Point3D &pt2 = parking_lot.corners.at(i + 1).position;
            tmp_slot_lines.at(i) = planning_math::LineSegment2d(
                planning_math::Vec2d(pt1.x, pt1.y),
                planning_math::Vec2d(pt2.x, pt2.y));
          }
          lines_.push_back(ObstacleLine(
              planning_math::LineSegment2d(tmp_slot_lines.at(0).center(),
                                           tmp_slot_lines.at(2).center()),
              OpenspaceObstacleType::PARKINGLINE));
        }
      }
    }
    return true;
  }

  if (world_model_->get_square_map_response().road_borders.size() == 0 &&
      world_model_->get_square_map_response().obstacles.pillar.size() == 0) {
    return false;
  }

  for (size_t i = 0;
       i < world_model_->get_square_map_response().road_borders.size(); i++) {
    // if border is physical, add directly. Otherwise, save to tmp and process
    // afterwards
    if (world_model_->get_square_map_response().road_borders[i].type ==
        LaneLineType::PHYSICAL) {
      for (size_t j = 1;
           j <
           world_model_->get_square_map_response().road_borders[i].pts.size();
           j++) {
        auto &pt1 =
            world_model_->get_square_map_response().road_borders[i].pts[j - 1];
        auto &pt2 =
            world_model_->get_square_map_response().road_borders[i].pts[j];
        lines_.push_back(ObstacleLine(Vec2d(pt1.x, pt1.y), Vec2d(pt2.x, pt2.y),
                                      OpenspaceObstacleType::WALL));
        // std::cout << "line_in: " << pt1.x << "  " << pt1.y << "  " << pt2.x
        // << "   " << pt2.y << std::endl;
      }
    } else {
      for (size_t j = 1;
           j <
           world_model_->get_square_map_response().road_borders[i].pts.size();
           j++) {
        auto &pt1 =
            world_model_->get_square_map_response().road_borders[i].pts[j - 1];
        auto &pt2 =
            world_model_->get_square_map_response().road_borders[i].pts[j];
        boundary_lines_.push_back(
            planning_math::LineSegment2d(planning_math::Vec2d(pt1.x, pt1.y),
                                         planning_math::Vec2d(pt2.x, pt2.y)));
        // std::cout << "line_in: " << pt1.x << "  " << pt1.y << "  " << pt2.x
        // << "   " << pt2.y << std::endl;
      }
    }
  }

  // get pillar
  for (size_t i = 0;
       i < world_model_->get_square_map_response().obstacles.pillar.size();
       i++) {
    for (size_t j = 1;
         j <
         world_model_->get_square_map_response().obstacles.pillar[i].pts.size();
         j++) {
      auto &pt1 =
          world_model_->get_square_map_response().obstacles.pillar[i].pts[j -
                                                                          1];
      auto &pt2 =
          world_model_->get_square_map_response().obstacles.pillar[i].pts[j];
      lines_.push_back(ObstacleLine(
          planning_math::LineSegment2d(planning_math::Vec2d(pt1.x, pt1.y),
                                       planning_math::Vec2d(pt2.x, pt2.y)),
          OpenspaceObstacleType::WALL));
    }
    auto &pt1 =
        world_model_->get_square_map_response().obstacles.pillar[i].pts[3];
    auto &pt2 =
        world_model_->get_square_map_response().obstacles.pillar[i].pts[0];
    lines_.push_back(ObstacleLine(
        planning_math::LineSegment2d(planning_math::Vec2d(pt1.x, pt1.y),
                                     planning_math::Vec2d(pt2.x, pt2.y)),
        OpenspaceObstacleType::WALL));
  }

  // get toll gate
  for (size_t i = 0;
       i < world_model_->get_square_map_response().obstacles.unknown.size();
       i++) {
    for (size_t j = 1; j < world_model_->get_square_map_response()
                               .obstacles.unknown[i]
                               .pts.size();
         j++) {
      auto &pt1 =
          world_model_->get_square_map_response().obstacles.unknown[i].pts[j -
                                                                           1];
      auto &pt2 =
          world_model_->get_square_map_response().obstacles.unknown[i].pts[j];
      lines_.push_back(ObstacleLine(
          planning_math::LineSegment2d(planning_math::Vec2d(pt1.x, pt1.y),
                                       planning_math::Vec2d(pt2.x, pt2.y)),
          OpenspaceObstacleType::WALL));
    }
    if (world_model_->get_square_map_response()
            .obstacles.unknown[i]
            .pts.size() > 1) {
      auto &pt1 = world_model_->get_square_map_response()
                      .obstacles.unknown[i]
                      .pts.back();
      auto &pt2 = world_model_->get_square_map_response()
                      .obstacles.unknown[i]
                      .pts.front();
      lines_.push_back(ObstacleLine(
          planning_math::LineSegment2d(planning_math::Vec2d(pt1.x, pt1.y),
                                       planning_math::Vec2d(pt2.x, pt2.y)),
          OpenspaceObstacleType::WALL));
    }
  }

  // add virtual edgs if pillar is beside
  //   auto& pillars = world_model_->get_square_map_response().obstacles.pillar;
  //   for(auto& pillar : pillars){
  //     if(pillar.pts.size() != 4)
  //     {
  //       throw std::logic_error("size of pillar is not 4, please refactor
  //       code.");
  //     }
  //     std::vector<Vec2d> corners(pillar.pts.size());
  //     std::transform(pillar.pts.begin(), pillar.pts.end(), corners.begin(),
  //     [](const Point3D& pt){ return Vec2d(pt.x, pt.y); });

  //     Vec2d center = std::accumulate(corners.begin(), corners.end(),
  //     Vec2d(0,0))/corners.size(); if(lot_box_on_left_.IsPointIn(center) ||
  //     lot_box_on_right_.IsPointIn(center))
  //     {
  // // std::cout << "hzmdebug-lot-pillar[" << pillar.id << "] is on side lot"
  // << std::endl;
  //       Vec2d lon_cross = (corners[0] + corners[3] - (corners[1] +
  //       corners[2])) / 2; Vec2d lat_cross = (corners[0] + corners[1] -
  //       (corners[2] + corners[3])) / 2; Box2d pillar_box(center,
  //       lon_cross.Angle(), lon_cross.Length(), lat_cross.Length());

  //       double dist_extend = (CarParams::GetInstance()->lat_inflation() +
  //       CarParams::GetInstance()->lat_inflation_max) / 2;
  //       pillar_box.LateralExtend(dist_extend * 2);
  //       pillar_box.LongitudinalExtend(dist_extend * 2);
  //       std::vector<LineSegment2d> edges = pillar_box.GetAllEdges();
  //       for(auto& e : edges)
  //       {
  //         lines_.push_back(ObstacleLine(e, OpenspaceObstacleType::BOUNDARY));
  //       }
  //     }
  //     else
  //     {
  //       // std::cout << "hzmdebug-lot-pillar[" << pillar.id << "] is on side
  //       lot" << std::endl;
  //     }

  //   }

  world_model_->reset_square_map_response();
  return true;
}

void ApaOpenspaceDecider::obstacle_filter() {
  // to local frame
  using planning_math::Box2d;
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  Box2d ego_car_box = world_model_->get_ego_state().ego_box;
  const auto &slot_info = PlanningContext::Instance()
                              ->parking_behavior_planner_output()
                              .parking_slot_info;
  bool not_remove_any_pts = (slot_info.type.value == ParkingSlotType::PARALLEL);
  if (not_remove_any_pts) {
    return;
  }

  planning_math::Vec2d vec_to_center(
      (VehicleParam::Instance()->front_edge_to_center -
       VehicleParam::Instance()->back_edge_to_center) /
          2.0,
      (VehicleParam::Instance()->left_edge_to_center -
       VehicleParam::Instance()->right_edge_to_center) /
          2.0);
  planning_math::Vec2d target_pos(target_state_.path_point.x,
                                  target_state_.path_point.y);
  planning_math::Vec2d target_box_center(
      target_pos + vec_to_center.rotate(target_state_.path_point.theta));
  planning_math::Vec2d ego2target_vec =
      target_box_center - ego_car_box.center();
  double ego2target_rotation = planning_math::NormalizeAngle(
      target_state_.path_point.theta - ego_car_box.heading());

  Box2d target_box(ego_car_box);
  target_box.Shift(ego2target_vec);
  target_box.RotateFromCenter(ego2target_rotation);

  for (auto iter = points_.begin(); iter != points_.end();) {
    if (target_box.IsPointIn(*iter)) {
      iter = points_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = lines_.begin(); iter != lines_.end();) {
    if (iter->is_physical() && target_box.HasOverlap(*iter)) {
      iter = lines_.erase(iter);
    } else {
      ++iter;
    }
  }

  // for (auto iter = boxes_.begin(); iter != boxes_.end();) {
  //   if (iter->HasOverlap(target_box)) {
  //     iter = boxes_.erase(iter);
  //   } else {
  //     ++iter;
  //   }
  // }
  // ignore_obstacle_box_near_target();
}

void ApaOpenspaceDecider::get_static_obstacle() {
  boxes_.clear();
  // for (auto &obj :
  //      world_model_->obstacle_manager().get_static_obstacles().Items()) {
  //   bool is_obstacle_vehicle = obj->Type() == ObjectType::COUPE ||
  //                              obj->Type() == ObjectType::BUS ||
  //                              obj->Type() == ObjectType::ENGINEER_TRUCK ||
  //                              obj->Type() == ObjectType::TRICYCLE;
  //   if (!is_obstacle_vehicle)
  //     continue;

  //   auto &obj_boundingbox = obj->PerceptionBoundingBox();
  //   boxes_.push_back(ObstacleBox(obj_boundingbox,
  //   OpenspaceObstacleType::CAR));
  // }
  // std::cout << "hzmdebug-static: boxes_.size() = " << boxes_.size() <<
  // std::endl;

  // const LeaderPair &lead_cars = PlanningContext::Instance()
  //                                   ->longitudinal_behavior_planner_output()
  //                                   .lead_cars;

  //   auto &park_in_snapshot =
  //   PlanningContext::Instance()->mutable_parking_behavior_planner_output()->park_in_snapshot;
  //   if (park_in_snapshot.is_available) {
  // // std::cout << "hzmdebug-static: park_in_snapshot.boxes.size() = " <<
  // park_in_snapshot.boxes.size() << std::endl;
  //     for (auto box : park_in_snapshot.boxes) {
  //       bool is_in_side_lot = lot_box_on_left_.IsPointIn(box.second.center())
  //           || lot_box_on_right_.IsPointIn(box.second.center());
  //       if (box.first || is_in_side_lot) {
  //         auto& obj_boundingbox = box.second;
  // 	      auto iter = std::find_if(boxes_.begin(), boxes_.end(),
  //           [obj_boundingbox](ObstacleBox& obs_box){ return
  //           obj_boundingbox.HasOverlap(obs_box);});
  //         if(iter != boxes_.end()) continue;
  //         boxes_.push_back(ObstacleBox(obj_boundingbox,
  //         OpenspaceObstacleType::CAR));
  //       }
  //     }
  //     park_in_snapshot.is_available = false;
  //     park_in_snapshot.boxes.clear();

  //   }
  //   else {
  for (auto &obj : world_model_->obstacle_manager().get_obstacles().Items()) {
    bool is_obstacle_vehicle = obj->Type() == ObjectType::COUPE ||
                               obj->Type() == ObjectType::BUS ||
                               obj->Type() == ObjectType::ENGINEER_TRUCK ||
                               obj->Type() == ObjectType::TRICYCLE;
    if (is_obstacle_vehicle) {
      bool is_object_considered = obj->IsStatic() == 1 && obj->Is_confident();

      bool is_in_side_lot =
          lot_box_on_left_.IsPointIn(obj->PerceptionBoundingBox().center()) ||
          lot_box_on_right_.IsPointIn(obj->PerceptionBoundingBox().center());

      if (is_object_considered || is_in_side_lot) {
        const Box2d &obj_boundingbox = obj->PerceptionBoundingBox();
        // auto iter = std::find_if(boxes_.begin(), boxes_.end(),
        //                          [obj_boundingbox](ObstacleBox &obs_box) {
        //                            return
        //                            obj_boundingbox.HasOverlap(obs_box);
        //                          });
        // if (iter != boxes_.end())
        //   continue;
        // if (obj->Id() == lead_cars.first.id &&
        //     lead_cars.first.is_need_fillet_cutting) {
        //   // std::cout << "hzmdebug-obs: obj[" << obj->Id() << "] need fillet
        //   // cutting" << std::endl;
        //   Box2d box_thinner(obj_boundingbox);
        //   Box2d box_shorter(obj_boundingbox);
        //   double fillet_cutting_length = obj->FilletCuttingLength();
        //   box_thinner.LateralExtend(-2 * fillet_cutting_length);
        //   box_shorter.Shift(-fillet_cutting_length / 2 *
        //                     Vec2d::CreateUnitVec2d(obj_boundingbox.heading()));
        //   box_shorter.LongitudinalExtend(-fillet_cutting_length);
        //   boxes_.push_back(
        //       ObstacleBox(box_thinner, OpenspaceObstacleType::CAR));
        //   boxes_.push_back(
        //       ObstacleBox(box_shorter, OpenspaceObstacleType::CAR));
        // } else {
        boxes_.push_back(
            ObstacleBox(obj_boundingbox, OpenspaceObstacleType::CAR));
        // }
      }
    }
  }
  // }
}

void ApaOpenspaceDecider::modify_inflation_for_car() {
  using planning_math::Box2d;

  LeaderPair lead_cars = PlanningContext::Instance()
                             ->longitudinal_behavior_planner_output()
                             .lead_cars;
  Obstacle *lead_car = world_model_->mutable_obstacle_manager().find_obstacle(
      lead_cars.first.id);
  if (lead_car == nullptr) {
    return;
  }

  const Box2d &lead_box = lead_car->PerceptionBoundingBox();
  if (lead_car->IsNeedFilletCutting()) {
    // std::cout << "hzmdebug-obs: lead_car->IsNeedFilletCutting()\n";
    Box2d box_thinner(lead_box);
    Box2d box_shorter(lead_box);
    double fillet_cutting_length = lead_car->FilletCuttingLength();
    box_thinner.LateralExtend(-2 * fillet_cutting_length);
    box_shorter.LongitudinalExtend(-2 * fillet_cutting_length);
    modify_inflation_for_obstacle(CarParams::GetInstance(),
                                  world_model_->get_ego_state().ego_box,
                                  box_thinner);
    modify_inflation_for_obstacle(CarParams::GetInstance(),
                                  world_model_->get_ego_state().ego_box,
                                  box_shorter);
  } else {
    modify_inflation_for_obstacle(CarParams::GetInstance(),
                                  world_model_->get_ego_state().ego_box,
                                  lead_box);
  }
}

void ApaOpenspaceDecider::ignore_obstacle_box_near_target() {
  ignoreCarAtInnerSide();

  Box2d target_box = parking_lot_->getBox();

  std::vector<LineSegment2d> target_edges(target_box.GetAllEdges());
  LineSegment2d &target_left_edge = target_edges.at(1);
  LineSegment2d &target_right_edge = target_edges.at(3);
  for (auto iter = boxes_.begin(); iter != boxes_.end();) {
    if (iter->HasOverlap(target_box)) {
      // if (!target_box.IsPointIn(iter->center())) {
      //   // std::cout << "hzmdebug-lot-setSafeMode\n";
      //   parking_lot_->setSafeMode(iter->HasOverlap(target_left_edge),
      //                             iter->HasOverlap(target_right_edge));
      // }
      // std::cout << "hzmdebug-lot-erase box\n";
      iter = boxes_.erase(iter);
      // ++iter;
    } else {
      ++iter;
    }
  }
}

void ApaOpenspaceDecider::hackBoxNearTarget(double lon_extend,
                                            double lat_extend) {
  for (auto iter = boxes_.begin(); iter != boxes_.end(); ++iter) {
    if (iter->HasOverlap(lot_box_on_left_) ||
        iter->HasOverlap(lot_box_on_right_)) {
      iter->LongitudinalExtend(lon_extend);
      iter->LateralExtend(lat_extend);
    }
  }
}

void ApaOpenspaceDecider::modify_inflation_for_near_obs() {
  Box2d ego_box = world_model_->get_ego_state().ego_box;

  for (const ObstacleLine &line : lines_) {
    if (line.is_physical()) {
      modify_inflation_for_obstacle(CarParams::GetInstance(), ego_box, line);
    }
  }

  for (const ObstacleBox &box : boxes_) {
    modify_inflation_for_obstacle(CarParams::GetInstance(), ego_box, box);
  }

  for (const Vec2d &point : points_) {
    modify_inflation_for_obstacle(CarParams::GetInstance(), ego_box, point);
  }
}

void ApaOpenspaceDecider::modify_inflation_for_obs_near_target() {
  Box2d ego_box = world_model_->get_ego_state().ego_box;

  planning_math::Vec2d vec_to_center(
      (VehicleParam::Instance()->front_edge_to_center -
       VehicleParam::Instance()->back_edge_to_center) /
          2.0,
      (VehicleParam::Instance()->left_edge_to_center -
       VehicleParam::Instance()->right_edge_to_center) /
          2.0);
  planning_math::Vec2d target_pos(target_state_.path_point.x,
                                  target_state_.path_point.y);
  planning_math::Vec2d target_box_center(
      target_pos + vec_to_center.rotate(target_state_.path_point.theta));
  Vec2d ego2target_vec = target_box_center - ego_box.center();
  double ego2target_rotation = planning_math::NormalizeAngle(
      target_state_.path_point.theta - ego_box.heading());

  Box2d target_box(ego_box);
  target_box.Shift(ego2target_vec);
  target_box.RotateFromCenter(ego2target_rotation);

  for (const ObstacleLine &line : lines_) {
    if (line.is_physical()) {
      modify_inflation_for_obstacle(CarParams::GetInstance(), target_box, line);
    }
  }

  for (const ObstacleBox &box : boxes_) {
    modify_inflation_for_obstacle(CarParams::GetInstance(), target_box, box);
  }

  for (const Vec2d &point : points_) {
    modify_inflation_for_obstacle(CarParams::GetInstance(), ego_box, point);
  }
}

void ApaOpenspaceDecider::requestSquareMapping() {
  InitLocalFrame();
  get_openspace_boundary();
  (void)gather_map_info();
}

void ApaOpenspaceDecider::process() {
  if (!is_pre_apa_) {
    get_static_obstacle();
  }
  // get_freespace_lead(); // in case not all points can be fetched
  obstacle_filter();
  // hackBoxNearTarget(0.6, -0.6);
  // auto &parking_slot_info = PlanningContext::Instance()
  //                               ->parking_behavior_planner_output()
  //                               .parking_slot_info;

  auto has_point_in_lot = [](std::vector<planning_math::Vec2d> points,
                             planning_math::Box2d lot) -> bool {
    for (auto pt : points) {
      if (lot.IsPointIn(pt)) {
        return true;
      }
    }
    return false;
  };
  bool point_in_left_lot = has_point_in_lot(points_, lot_box_on_left_);
  bool point_in_right_lot = has_point_in_lot(points_, lot_box_on_right_);
  parking_lot_->setSafeMode(point_in_left_lot, point_in_right_lot);
  parking_lot_->processLinesAroundLot(boundary_lines_);
  for (size_t i = 0; i < boundary_lines_.size(); i++) {
    lines_.push_back(
        ObstacleLine(boundary_lines_[i], OpenspaceObstacleType::BOUNDARY));
  }

  std::vector<LineSegment2d> lot_lines =
      parking_lot_->genLotWalls(CarParams::GetInstance()->vehicle_width_real);
  for (auto &wall : lot_lines) {
    lines_.push_back(ObstacleLine(wall, OpenspaceObstacleType::PARKINGLINE));
  }
  // to avoid forward path from bottom
  lines_.push_back(ObstacleLine(parking_lot_->genBottomWall(),
                                OpenspaceObstacleType::BOUNDARY));

  modify_inflation_for_near_obs();
  modify_inflation_for_obs_near_target();

  for (auto iter = boxes_.begin(); iter != boxes_.end();) {
    if (!iter->HasOverlap(map_boundary_)) {
      iter = boxes_.erase(iter);
    } else {
      ++iter;
    }
  }

  if (world_model_->is_parking_lvp() || world_model_->is_parking_apa()) {
    // addBlindBorder();
    // get T shaped area bourder
    std::vector<LineSegment2d> t_shaped_boarders =
        parking_lot_->getTshapedAreaLines(ego_state_, map_boundary_, boxes_,
                                          points_);
    for (auto &boarder : t_shaped_boarders) {
      lines_.push_back(ObstacleLine(boarder, OpenspaceObstacleType::ROADBOARD));
    }
  }

  feedOutput();
  reset();
}

void ApaOpenspaceDecider::get_freespace_lead() {
  using planning_math::Box2d;
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  auto &collision_freespace_line = PlanningContext::Instance()
                                       ->longitudinal_behavior_planner_output()
                                       .free_space;
  bool is_freespace_point_valid =
      collision_freespace_line.id != -1 &&
      PlanningContext::Instance()->planning_status().fs_block &&
      collision_freespace_line.d_rel < 2;
  if (is_freespace_point_valid) {
    ObstacleLine fs_line(
        Vec2d(collision_freespace_line.x, collision_freespace_line.y),
        Vec2d(collision_freespace_line.x + 0.1,
              collision_freespace_line.y + 0.1),
        OpenspaceObstacleType::FREESPACE);
    lines_.push_back(fs_line);

    MSD_LOG(INFO, "hzmdebug: OSD fs_lead id = %d.\n",
            collision_freespace_line.id);
  } else {
    MSD_LOG(INFO, "hzmdebug: OSD fs_lead is none\n");
  }
}

void ApaOpenspaceDecider::addBlindBorder() {
  Box2d ego_box(world_model_->get_ego_state().ego_box);

  // // add lot front border
  // if(parking_lot_->getType() !=  ParkingSlotType::PERPENDICULAR)
  // {
  //   throw std::runtime_error("ApaOpenspaceDecider::addBlindBorder not
  //   implemented except vertical lot");
  // }
  Vec2d lot_corner_0 = parking_lot_->getBox().GetAllCorners().at(0);
  Vec2d diff_vec = ego_box.center() - lot_corner_0;
  if (Vec2d::CreateUnitVec2d(parking_lot_->getBox().heading())
              .InnerProd(diff_vec) <= 0 &&
      !parking_lot_->getBox().HasOverlap(ego_box)) {
    // std::cout << "ApaOpenspaceDecider::addBlindBorder: can not add lot wings"
    // << std::endl;
  } else {
    std::vector<LineSegment2d> front_wings =
        parking_lot_->genFrontWings(map_boundary_.width());
    for (auto &item : front_wings) {
      if (!ego_box.HasOverlap(item)) {
        lines_.push_back(ObstacleLine(item, OpenspaceObstacleType::BOUNDARY));
      }
    }
  }
}

void ApaOpenspaceDecider::ignoreCarAtInnerSide() {
  Box2d lot_box_inner_side(lot_box_on_right_);
  if (parking_lot_->isOnLeft()) {
    lot_box_inner_side = lot_box_on_left_;
  }

  for (auto iter = boxes_.begin(); iter != boxes_.end(); ++iter) {
    if (lot_box_inner_side.IsPointIn(iter->center())) {
      // std::cout << "hzmdebug-lot-setSafeMode for car at inner side\n";
      parking_lot_->setSafeMode(parking_lot_->isOnLeft(),
                                !parking_lot_->isOnLeft());
    }
  }
}

} // namespace msquare
