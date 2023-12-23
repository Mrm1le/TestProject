#include "planner/behavior_planner/deciders/openspace_decider.h"
#include "common/math/math_utils.h"
#include "common/parking_obstacle_manager.h"
#include "common/planning_config.h"
#include "common/planning_context.h"
#include "common/utils/yaml_utils.h"
#include <algorithm>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "planner/behavior_planner/deciders/openspace_utils.h"
#include "planning/common/common.h"

namespace msquare {

using planning_math::Box2d;
using planning_math::LineSegment2d;
using planning_math::Vec2d;

using namespace parking;

OpenspaceDecider::OpenspaceDecider(
    const std::shared_ptr<WorldModel> &world_model)
    : BaseOpenspaceDecider(world_model) {
  default_swell_distance_ = 5;
  default_step_size_ = 3;

  log_file_prefix_ =
      PlanningConfig::Instance()->config_files().log_path + "general_decider";
}

OpenspaceDecider::~OpenspaceDecider() { ; }

void OpenspaceDecider::obstacle_filter() {
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
    if (!iter->is_physical() && ego_car_box.HasOverlap(*iter)) {
      iter = lines_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void OpenspaceDecider::get_static_obstacle() {
  boxes_.clear();
  for (auto &obj : world_model_->obstacle_manager().get_obstacles().Items()) {
    bool is_obstacle_vehicle = obj->Type() == ObjectType::COUPE ||
                               obj->Type() == ObjectType::BUS ||
                               obj->Type() == ObjectType::ENGINEER_TRUCK ||
                               obj->Type() == ObjectType::TRICYCLE;
    if (is_obstacle_vehicle && obj->IsStatic() == 1 && obj->Is_confident()) {
      auto obj_boundingbox = obj->PerceptionBoundingBox();
      boxes_.push_back(
          ObstacleBox(obj_boundingbox, OpenspaceObstacleType::CAR));
    }
  }
}

void OpenspaceDecider::get_freespace_lead() {
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

void OpenspaceDecider::modify_inflation_for_car() {
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

bool OpenspaceDecider::init_frame_AVP() {
  auto &input = PlanningContext::Instance()->parking_behavior_planner_output();

  ego_state_ = input.init_traj_point;
  target_state_ = input.target_traj_point;
  // std::cout << "end_state_openspace: " << ego_state_.path_point.x << "   "
  //           << ego_state_.path_point.y << "  "
  //           << ego_state_.path_point.theta << "  "
  //           << target_state_.path_point.x << " "
  //           << target_state_.path_point.y << " "
  //           << target_state_.path_point.theta << std::endl;

  planning_math::Box2d grid_map_bounding_box;
  GridMapBoxLocal box_local;
  box_local.s_left = -15; // enough space to fascilitate longitudinal motion
  box_local.s_right = 15; // enough space to fascilitate longitudinal motion
  box_local.s_up = 10;    // enough space to fascilitate longitudinal motion
  box_local.s_down = -10; // enough space to fascilitate longitudinal motion
  Pose2D tmp_location_zero;
  (void)get_gridmap_boundary_normal(
      world_model_, default_swell_distance_, default_step_size_, target_state_,
      ego_state_, grid_map_bounding_box, box_local, tmp_location_zero);

  local_frame_pose_ = tmp_location_zero;
  map_boundary_ = grid_map_bounding_box;
  // std::cout << "grid map localframe: " << local_frame_pose_.x << "   "
  //           << local_frame_pose_.y << "  "
  //           << local_frame_pose_.theta << std::endl;

  // std::cout << "grid map boundary: " << grid_map_bounding_box.center_x() << "
  // "
  //           << grid_map_bounding_box.center_y() << "  "
  //           << grid_map_bounding_box.heading() << " "
  //           << grid_map_bounding_box.length() << "  "
  //           << grid_map_bounding_box.width() << std::endl;

  // for(int i = 0; i < grid_map_bounding_box.GetAllCorners().size(); i++){
  //   std::cout << " 1 " << grid_map_bounding_box.GetAllCorners()[i].x() << " "
  //             << grid_map_bounding_box.GetAllCorners()[i].y() << std::endl;
  // }
  auto squaremap_info = PlanningContext::Instance()->mutable_square_map();
  auto corners = grid_map_bounding_box.GetAllCorners();
  // squaremap_info->box_corners.resize(corners.size());
  for (size_t i = 0; i < corners.size(); i++) {
    squaremap_info->box_corners[i].x = corners[i].x();
    squaremap_info->box_corners[i].y = corners[i].y();
  }
  squaremap_info->box_height = world_model_->get_ego_state().ego_enu.position.z;

  Pose2D target_state_pose, ego_pose;
  target_state_pose.x = target_state_.path_point.x;
  target_state_pose.y = target_state_.path_point.y;
  target_state_pose.theta = target_state_.path_point.theta;
  ego_pose.x = ego_state_.path_point.x;
  ego_pose.y = ego_state_.path_point.y;
  ego_pose.theta = ego_state_.path_point.theta;

  Pose2D ego_pose_local = planning_math::tf2d(local_frame_pose_, ego_pose);
  Pose2D target_pose_local =
      planning_math::tf2d(local_frame_pose_, target_state_pose);
  target_state_local_.path_point.x = target_pose_local.x;
  target_state_local_.path_point.y = target_pose_local.y;
  target_state_local_.path_point.theta = target_pose_local.theta;
  ego_state_local_.path_point.x = ego_pose_local.x;
  ego_state_local_.path_point.y = ego_pose_local.y;
  ego_state_local_.path_point.theta = ego_pose_local.theta;

  planning_math::Box2d ego_box = world_model_->get_ego_state().ego_box;
  // derive target box from ego pose box
  // TODO: consider that box center is not on rear axes
  const CarParams *car_params = CarParams::GetInstance();
  planning_math::Box2d target_box({ego_pose.x, ego_pose.y}, ego_pose.theta,
                                  car_params->vehicle_length,
                                  VehicleParam::Instance()->width);

  // check_frame_bounds(ego_box);
  // extend_frame_bounds(ego_box);
  // check_frame_bounds(target_box);
  // extend_frame_bounds(target_box);

  return true;
}

bool OpenspaceDecider::gather_map_info_avp() {
  using planning_math::Box2d;
  using planning_math::LineSegment2d;
  using planning_math::Vec2d;

  reset();
  if (world_model_->is_parking_lvp() || world_model_->is_parking_apa()) {
    auto points = world_model_->obstacle_manager().get_points().Items();
    for (auto &point : points) {
      points_.emplace_back(point->PerceptionBoundingBox().center());
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
        lines_.push_back(ObstacleLine(Vec2d(pt1.x, pt1.y), Vec2d(pt2.x, pt2.y),
                                      OpenspaceObstacleType::BOUNDARY));
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

void OpenspaceDecider::ignore_obstacle_box_near_target() {
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
}

void OpenspaceDecider::modify_inflation_for_near_obs() {
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

void OpenspaceDecider::modify_inflation_for_obs_near_target() {
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
}

void OpenspaceDecider::requestSquareMapping() {
  (void)init_frame_AVP();
  (void)gather_map_info_avp();
}

void OpenspaceDecider::process() {
  auto frenet_coor = world_model_->get_frenet_coord();
  if (frenet_coor == nullptr) {
    MSD_LOG(INFO, "hzmdebug: OSD null frenet\n");
    return;
  }
  get_static_obstacle();
  // add_freespace_obstacle();
  obstacle_filter();

  // get_freespace_lead(); // in case not all points can be fetched
  modify_inflation_for_near_obs();
  modify_inflation_for_obs_near_target();

  feedOutput();
  reset();
}

// void OpenspaceDecider::add_freespace_obstacle() {
//   const std::vector<const Obstacle *> fs_obs =
//       world_model_->obstacle_manager().get_freespace_obstacles().Items();
//   for (auto &fs_ob : fs_obs) {
//     lines_.emplace_back(fs_ob->PerceptionLine(),
//                         OpenspaceObstacleType::FREESPACE);
//   }
// }

} // namespace msquare
