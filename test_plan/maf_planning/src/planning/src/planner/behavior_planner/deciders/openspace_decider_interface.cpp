#include "planner/behavior_planner/deciders/openspace_decider_interface.h"
#include "common/math/math_utils.h"
#include "common/utils/yaml_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.h"
#include "planning/common/common.h"
#include <fstream>

namespace msquare {

using namespace parking;
using namespace planning_math;

BaseOpenspaceDecider::BaseOpenspaceDecider(
    const std::shared_ptr<WorldModel> &world_model)
    : world_model_(world_model) {}

void BaseOpenspaceDecider::feedOutput() {
  auto output = PlanningContext::Instance()->mutable_openspace_decider_output();
  auto apa_meta_state = PlanningContext::Instance()
                            ->parking_behavior_planner_output()
                            .apa_meta_state;
  output->init_state = ego_state_;
  output->target_state = target_state_;
  output->map_boundary = map_boundary_;
  output->obstacle_boxs.clear();
  output->obstacle_lines.clear();
  output->lines.clear();
  output->points.clear();
  output->step_points.clear();
  output->apa_meta_state = apa_meta_state;
  std::vector<LineSegment2d> T_lines;
  for (size_t i = 0; i < boxes_.size(); i++) {
    if (!map_boundary_.HasOverlap(boxes_[i]))
      continue;
    output->obstacle_boxs.push_back(boxes_[i]);
  }
  for (size_t i = 0; i < lines_.size(); i++) {
    if (!map_boundary_.HasOverlap(lines_[i]))
      continue;
    if (lines_[i].is_physical()) {
      output->obstacle_lines.push_back(lines_[i]);
    } else {
      if (lines_[i].type() == OpenspaceObstacleType::ROADBOARD) {
        T_lines.push_back(lines_[i]);
      } else {
        output->lines.push_back(lines_[i]);
      }
    }
  }
  if (!T_lines.empty()) {
    output->T_lines.is_inited = true;
    output->T_lines.road_upper_bound = T_lines[0];
    output->T_lines.road_lower_left_bound = T_lines[1];
    output->T_lines.slot_left_bound = T_lines[2];
    output->T_lines.slot_right_bound = T_lines[3];
    output->T_lines.road_lower_right_bound = T_lines[4];
  }

  for (size_t i = 0; i < points_.size(); i++) {
    if (!map_boundary_.IsPointIn(points_[i]))
      continue;
    output->points.push_back(points_[i]);
  }

  for (size_t i = 0; i < step_points_.size(); i++) {
    if (!map_boundary_.IsPointIn(step_points_[i]))
      continue;
    output->step_points.push_back(step_points_[i]);
  }

  using namespace planning_math;
  const Pose2D &ego_pose = world_model_->get_ego_state().ego_pose;
  // TODO@huangzhengming maintain a global ego footprint model?
  FootprintModelPtr box_model =
      std::make_shared<BoxFootprintModel>(VehicleParam::Instance(), 0, 0);
  output->lines.erase(
      std::remove_if(
          output->lines.begin(), output->lines.end(),
          [ego_pose, box_model](const planning_math::LineSegment2d &line) {
            return box_model->checkOverlap(ego_pose, line, true);
          }),
      output->lines.end());
}

bool BaseOpenspaceDecider::isReverseSearchRequired() {
  return false;
  // Vec2d ego_position = Vec2d(ego_state_.path_point.x,
  // ego_state_.path_point.y); Vec2d target_position =
  //     Vec2d(target_state_.path_point.x, target_state_.path_point.y);
  // // TODO@huangzhengming: find a better way ?
  // const auto &slot_box = PlanningContext::Instance()
  //                            ->mutable_parking_behavior_planner_output()
  //                            ->parking_lot->getBox();
  // return !slot_box.IsPointIn(ego_position);
}

void BaseOpenspaceDecider::reset() {
  lines_.clear();
  boxes_.clear();
  points_.clear();
  step_points_.clear();
}

} // namespace msquare
