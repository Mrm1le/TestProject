#include "planner/behavior_planner/deciders/rpa_straight_openspace_decider.h"
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

RpaStraightOpenspaceDecider::RpaStraightOpenspaceDecider(
    const std::shared_ptr<WorldModel> &world_model)
    : BaseOpenspaceDecider(world_model) {}

RpaStraightOpenspaceDecider::~RpaStraightOpenspaceDecider() { ; }

void RpaStraightOpenspaceDecider::reset() { BaseOpenspaceDecider::reset(); }

void RpaStraightOpenspaceDecider::retriveData() {
  auto &input = PlanningContext::Instance()->parking_behavior_planner_output();
  ego_state_ = input.init_traj_point;
  target_state_ = input.target_traj_point;

  Pose2D ego_pose(ego_state_.path_point.x, ego_state_.path_point.y,
                  ego_state_.path_point.theta);

  Pose2D target_pose(target_state_.path_point.x, target_state_.path_point.y,
                     target_state_.path_point.theta);

  local_frame_pose_ = ego_pose;
  local_init_pose_ = planning_math::tf2d(local_frame_pose_, ego_pose);
  local_target_pose_ = planning_math::tf2d(local_frame_pose_, target_pose);
}

void RpaStraightOpenspaceDecider::getOpenspaceBoundary() {
  const static double max_box_length = 20.0;
  const static double max_box_width = 15.0;
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  planning_math::Vec2d local_box_center(
      0.5 * (local_init_pose_.x + local_target_pose_.x),
      0.5 * (local_init_pose_.y + local_target_pose_.y));
  double dis_init_target =
      std::hypot(local_target_pose_.x - local_init_pose_.x,
                 local_target_pose_.y - local_init_pose_.y);

  double box_length = std::max(
      max_box_length, dis_init_target + VehicleParam::Instance()->length);
  double box_width =
      std::max(max_box_width, 4 * VehicleParam::Instance()->width);

  tlines_.emplace_back(msquare::planning_math::Vec2d(
                           -box_width / 2.0, VehicleParam::Instance()->width),
                       msquare::planning_math::Vec2d(
                           box_width / 2.0, VehicleParam::Instance()->width));
  tlines_.emplace_back(
      msquare::planning_math::Vec2d(-box_width / 2.0,
                                    -VehicleParam::Instance()->width),
      msquare::planning_math::Vec2d(-VehicleParam::Instance()->length,
                                    -VehicleParam::Instance()->width));
  tlines_.emplace_back(
      msquare::planning_math::Vec2d(-VehicleParam::Instance()->length,
                                    -2.0 * VehicleParam::Instance()->width),
      msquare::planning_math::Vec2d(-VehicleParam::Instance()->length,
                                    -VehicleParam::Instance()->width));
  tlines_.emplace_back(
      msquare::planning_math::Vec2d(VehicleParam::Instance()->length,
                                    -2.0 * VehicleParam::Instance()->width),
      msquare::planning_math::Vec2d(VehicleParam::Instance()->length,
                                    -VehicleParam::Instance()->width));
  tlines_.emplace_back(
      msquare::planning_math::Vec2d(VehicleParam::Instance()->length,
                                    -VehicleParam::Instance()->width),
      msquare::planning_math::Vec2d(box_width / 2.0,
                                    -VehicleParam::Instance()->width));

  planning_math::Box2d local_box(local_box_center, 0.0, box_length, box_width);
  map_boundary_ = planning_math::tf2d_inv(local_frame_pose_, local_box);

  auto squaremap_info = PlanningContext::Instance()->mutable_square_map();
  auto corners = map_boundary_.GetAllCorners();
  for (size_t i = 0; i < corners.size(); i++) {
    squaremap_info->box_corners[i].x = corners[i].x();
    squaremap_info->box_corners[i].y = corners[i].y();
  }
  squaremap_info->box_height = world_model_->get_ego_state().ego_enu.position.z;
}

bool RpaStraightOpenspaceDecider::getObstacles() {
  using planning_math::Box2d;
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  reset();
  EgoState ego_state = world_model_->get_ego_state();
  Pose2D ego_pose(ego_state.ego_pose.x, ego_state.ego_pose.y,
                  ego_state.ego_pose.theta);
  double cs = std::cos(ego_pose.theta);
  double ss = std::sin(ego_pose.theta);
  planning_math::Vec2d ego_back(
      ego_pose.x - VehicleParam::Instance()->back_edge_to_center * cs,
      ego_pose.y - VehicleParam::Instance()->back_edge_to_center * ss);
  planning_math::Vec2d ego_front(
      ego_pose.x + VehicleParam::Instance()->front_edge_to_center * cs,
      ego_pose.y + VehicleParam::Instance()->front_edge_to_center * ss);
  planning_math::Box2d ego_box(
      planning_math::LineSegment2d(ego_back, ego_front),
      VehicleParam::Instance()->width_wo_rearview_mirror);

  /* wheel stopper */
  const ParkingMapInfo &parking_map_info = world_model_->get_parking_map_info();
  for (auto &parking_lot :
       parking_map_info.parking_lots_detection_fusion_results) {
    if (!parking_lot.wheel_stop.available) {
      continue;
    }
    planning_math::LineSegment2d stopper_line(
        Vec2d(parking_lot.wheel_stop.point1.x, parking_lot.wheel_stop.point1.y),
        Vec2d(parking_lot.wheel_stop.point2.x,
              parking_lot.wheel_stop.point2.y));
    if (ego_box.HasOverlap(stopper_line)) {
      continue;
    }
    lines_.push_back(
        ObstacleLine(stopper_line, OpenspaceObstacleType::PARKINGLINE));
  }

  if (world_model_->is_parking_lvp() || world_model_->is_parking_apa()) {
    // auto points = world_model_->obstacle_manager().get_uss_points().Items();
    auto points = world_model_->obstacle_manager().get_points().Items();
    for (auto &point : points) {
      points_.emplace_back(point->PerceptionBoundingBox().center());
    }
  }

  // uss groundline
  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if (obs.pts.size() % 2 == 0) {
      if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP ||
          obs.type == GroundLineType::GROUND_LINE_TYPE_STEP ||
          obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_STEP) {
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
    } else {
      for (int i = 0; i < obs.pts.size(); i++) {
        points_.emplace_back(obs.pts[i].x, obs.pts[i].y);
      }
    }
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

  boxes_.clear();
  const LeaderPair &lead_cars = PlanningContext::Instance()
                                    ->longitudinal_behavior_planner_output()
                                    .lead_cars;
  for (auto &obj : world_model_->obstacle_manager().get_obstacles().Items()) {
    bool is_obstacle_vehicle = obj->Type() == ObjectType::COUPE ||
                               obj->Type() == ObjectType::BUS ||
                               obj->Type() == ObjectType::ENGINEER_TRUCK ||
                               obj->Type() == ObjectType::TRICYCLE;
    if (is_obstacle_vehicle) {
      bool is_object_considered = obj->IsStatic() == 1 && obj->Is_confident();
      if (is_object_considered) {
        const planning_math::Box2d &obj_boundingbox =
            obj->PerceptionBoundingBox();
        boxes_.push_back(
            ObstacleBox(obj_boundingbox, OpenspaceObstacleType::CAR));
      }
    }
  }

  for (auto &l : tlines_) {
    lines_.push_back(ObstacleLine(planning_math::tf2d_inv(local_frame_pose_, l),
                                  OpenspaceObstacleType::ROADBOARD));
  }

  world_model_->reset_square_map_response();
  return true;
}

void RpaStraightOpenspaceDecider::requestSquareMapping() {
  retriveData();
  getOpenspaceBoundary();
  (void)getObstacles();
}

void RpaStraightOpenspaceDecider::process() {

  for (auto iter = boxes_.begin(); iter != boxes_.end();) {
    if (!iter->HasOverlap(map_boundary_)) {
      iter = boxes_.erase(iter);
    } else {
      ++iter;
    }
  }

  feedOutput();
  reset();
}

} // namespace msquare
