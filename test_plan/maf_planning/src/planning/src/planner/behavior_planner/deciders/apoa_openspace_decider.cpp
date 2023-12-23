#include "planner/behavior_planner/deciders/apoa_openspace_decider.h"
#include "common/math/math_utils.h"
#include "common/parking_obstacle_manager.h"
#include "common/planning_config.h"
#include "common/planning_context.h"
#include "common/utils/yaml_utils.h"
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "planner/behavior_planner/deciders/openspace_utils.h"
#include "planning/common/common.h"

namespace msquare {

using namespace planning_math;
using namespace parking;

ApoaOpenspaceDecider::ApoaOpenspaceDecider(
    const std::shared_ptr<WorldModel> &world_model)
    : BaseOpenspaceDecider(world_model) {
  log_file_prefix_ =
      PlanningConfig::Instance()->config_files().log_path + "apoa_decider";

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

ApoaOpenspaceDecider::~ApoaOpenspaceDecider() { ; }

void ApoaOpenspaceDecider::reset() {
  boundary_lines_.clear();
  BaseOpenspaceDecider::reset();
}

void ApoaOpenspaceDecider::InitLocalFrame() {
  auto &input = PlanningContext::Instance()->parking_behavior_planner_output();
  ego_state_ = input.init_traj_point;
  target_state_ = input.target_traj_point;

  Pose2D target_state_pose, ego_pose;
  target_state_pose.x = target_state_.path_point.x;
  target_state_pose.y = target_state_.path_point.y;
  target_state_pose.theta = target_state_.path_point.theta;
  ego_pose.x = ego_state_.path_point.x;
  ego_pose.y = ego_state_.path_point.y;
  ego_pose.theta = ego_state_.path_point.theta;

  local_frame_pose_ =
      Pose2D(lot_box_.center_x(), lot_box_.center_y(), lot_box_.heading());

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

void ApoaOpenspaceDecider::get_openspace_boundary() {
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
  // if(VehicleParam::Instance()->getBox(ego_state_local_.path_point.x,
  // ego_state_local_.path_point.y,
  // ego_state_local_.path_point.theta).HasOverlap(Box2d(map_aabox_local)))
  // {
  map_aabox_local.MergeFrom(
      Vec2d(ego_state_local_.path_point.x + nearby_buffer,
            ego_state_local_.path_point.y + nearby_buffer));
  map_aabox_local.MergeFrom(
      Vec2d(ego_state_local_.path_point.x - nearby_buffer,
            ego_state_local_.path_point.y - nearby_buffer));
  // }
  map_aabox_local.MergeFrom(
      Vec2d(target_state_local_.path_point.x + nearby_buffer,
            target_state_local_.path_point.y + nearby_buffer));
  map_aabox_local.MergeFrom(
      Vec2d(target_state_local_.path_point.x - nearby_buffer,
            target_state_local_.path_point.y - nearby_buffer));

  const std::vector<Vec2d> &lot_box_corners = lot_box_.GetAllCorners();
  Vec2d lot_box_front_center =
      (lot_box_corners.at(0) + lot_box_corners.at(1)) / 2;
  Point2D cart_start_point{lot_box_front_center.x(), lot_box_front_center.y()};
  Point2D frenet_start_point;
  if (world_model_->get_frenet_coord()->CartCoord2FrenetCoord(
          cart_start_point, frenet_start_point) ==
      TRANSFORM_STATUS::TRANSFORM_SUCCESS) {
    (void)extend_boundary_with_refline(
        world_model_, u_turn_envelope_radius, car_params->vehicle_length / 2,
        target_state_, ego_state_, map_aabox_local, local_frame_pose_,
        frenet_start_point.x);
  }

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

bool ApoaOpenspaceDecider::gather_map_info() {
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
    }
    // auto points = world_model_->obstacle_manager().get_uss_points().Items();
    // auto points = world_model_->obstacle_manager().get_points().Items();
    // for (auto &point : points) {
    //   // remove the uss points which in parking lot
    //   if (parking_lot_box_available &&
    //       parking_lot_box.IsPointIn(point->PerceptionBoundingBox().center())) {
    //     continue;
    //   }
    //   points_.emplace_back(point->PerceptionBoundingBox().center());
    // }

    // get wall line, step line, wall points, step points
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
            obs.type == GroundLineType::GROUND_LINE_USS_TYPE_ONLY_USS_UNKNOWN)) {
        for (int i = 0; i < obs.pts.size(); i++) {
          Vec2d point{obs.pts[i].x, obs.pts[i].y};
          // remove the uss points which in parking lot
          if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_POINT_GENERAL_STEP) {
            step_points_.emplace_back(point);
          } else {
            points_.emplace_back(point);
          }
        }
      } else if (obs.pts.size() % 2 == 0) {
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
    // if (!world_model_->is_parking_svp()) {
    //   const ParkingMapInfo &parking_map_info =
    //       world_model_->get_parking_map_info();
    //   for (auto &parking_lot :
    //        parking_map_info.parking_lots_detection_fusion_results) {
    //     if (parking_lot.id != PlanningContext::Instance()
    //                               ->parking_behavior_planner_output()
    //                               .parking_slot_info.id &&
    //         !parking_lot.corners.empty()) {
    //       // ignore all slots besides
    //       Pose2D target_parking_lot_pose(parking_lot_box.center().x(),
    //                                      parking_lot_box.center().y(),
    //                                      parking_lot_box.heading());
    //       Vec2d slot_corner_0(parking_lot.corners.at(0).position.x,
    //                           parking_lot.corners.at(0).position.y);
    //       if (tf2d(target_parking_lot_pose, slot_corner_0).x() <
    //           parking_lot_box.length()) {
    //         continue;
    //       }

    //       for (int i = 1; i < parking_lot.corners.size(); ++i) {
    //         const Point3D &pt1 = parking_lot.corners.at(i - 1).position;
    //         const Point3D &pt2 = parking_lot.corners.at(i).position;
    //         lines_.push_back(
    //             ObstacleLine(planning_math::LineSegment2d(
    //                              planning_math::Vec2d(pt1.x, pt1.y),
    //                              planning_math::Vec2d(pt2.x, pt2.y)),
    //                          OpenspaceObstacleType::PARKINGLINE));
    //       }
    //     }
    //   }
    // }
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

  world_model_->reset_square_map_response();
  return true;
}

void ApoaOpenspaceDecider::obstacle_filter() {
  // to local frame
  using planning_math::Box2d;
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  Box2d ego_car_box = world_model_->get_ego_state().ego_box;
  ego_car_box.SetWidth(VehicleParam::Instance()->width_wo_rearview_mirror *
                       CarParams::GetInstance()->shrink_ratio_for_lines_min_);
  ego_car_box.SetLength(VehicleParam::Instance()->length *
                        CarParams::GetInstance()->shrink_ratio_for_lines_min_);

  for (auto iter = lines_.begin(); iter != lines_.end();) {
    if (!iter->is_physical() &&
        iter->type() != OpenspaceObstacleType::PARKINGLINE &&
        ego_car_box.HasOverlap(*iter)) {
      iter = lines_.erase(iter);
    } else {
      ++iter;
    }
  }

  ignore_obstacles_near_target();
  for (auto iter = boxes_.begin(); iter != boxes_.end();) {
    if (iter->type() == OpenspaceObstacleType::CAR &&
        iter->DistanceTo(world_model_->get_ego_state().ego_box) > 10.0) {
      iter = boxes_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void ApoaOpenspaceDecider::get_static_obstacle() {
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

  const LeaderPair &lead_cars = PlanningContext::Instance()
                                    ->longitudinal_behavior_planner_output()
                                    .lead_cars;

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

void ApoaOpenspaceDecider::modify_inflation_for_car() {
  using planning_math::Box2d;

  LeaderPair lead_cars = PlanningContext::Instance()
                             ->longitudinal_behavior_planner_output()
                             .lead_cars;
  Obstacle *lead_car = world_model_->mutable_obstacle_manager().find_obstacle(
      lead_cars.first.id);
  if (lead_car == nullptr) {
    return;
  }
  Box2d lead_box = lead_car->PerceptionBoundingBox();
  modify_inflation_for_obstacle(CarParams::GetInstance(),
                                world_model_->get_ego_state().ego_box,
                                lead_box);
}

void ApoaOpenspaceDecider::ignore_obstacles_near_target() {
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

  target_box.LateralExtend(CarParams::GetInstance()->vehicle_width -
                           target_box.width());
  target_box.LongitudinalExtend(CarParams::GetInstance()->vehicle_length -
                                target_box.length());

  for (auto iter = boxes_.begin(); iter != boxes_.end();) {
    if (iter->HasOverlap(target_box)) {
      iter = boxes_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = lines_.begin(); iter != lines_.end();) {
    if (target_box.HasOverlap(*iter)) {
      iter = lines_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void ApoaOpenspaceDecider::modify_inflation_for_near_obs() {
  Box2d ego_box = world_model_->get_ego_state().ego_box;

  for (const ObstacleLine &line : lines_) {
    if (line.is_physical()) {
      modify_inflation_for_obstacle(CarParams::GetInstance(), ego_box, line);
    }
  }

  for (const ObstacleBox &box : boxes_) {
    modify_inflation_for_obstacle(CarParams::GetInstance(), ego_box, box);
  }
}

void ApoaOpenspaceDecider::modify_inflation_for_obs_near_target() {
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
      modify_inflation_for_obstacle(CarParams::GetInstance(), ego_box, line);
    }
  }

  for (const ObstacleBox &box : boxes_) {
    modify_inflation_for_obstacle(CarParams::GetInstance(), target_box, box);
  }
}

void ApoaOpenspaceDecider::requestSquareMapping() {
  InitLocalFrame();
  get_openspace_boundary();
  (void)gather_map_info();
}

void ApoaOpenspaceDecider::process() {
  get_static_obstacle();
  obstacle_filter();
  // get_freespace_lead(); // in case not all points can be fetched
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_lot->processLinesAroundLot(boundary_lines_);
  for (size_t i = 0; i < boundary_lines_.size(); i++) {
    lines_.push_back(
        ObstacleLine(boundary_lines_[i], OpenspaceObstacleType::BOUNDARY));
  }

  modify_inflation_for_near_obs();
  modify_inflation_for_obs_near_target();

  for (auto iter = boxes_.begin(); iter != boxes_.end();) {
    if (!iter->HasOverlap(map_boundary_)) {
      iter = boxes_.erase(iter);
    } else {
      ++iter;
    }
  }

  std::vector<LineSegment2d> t_shaped_boarders =
      parking_lot_->getTshapedAreaLines(target_state_, map_boundary_, boxes_,
                                        points_);
  for (auto &boarder : t_shaped_boarders) {
    lines_.push_back(ObstacleLine(boarder, OpenspaceObstacleType::ROADBOARD));
  }

  feedOutput();
  reset();
}

void ApoaOpenspaceDecider::get_freespace_lead() {
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

} // namespace msquare
