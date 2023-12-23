#include "planner/behavior_planner/deciders/path_decider.h"
#include "common/obstacle_process_utils.h"
#include "planning/common/common.h"
#include "planning/common/logging.h"

namespace msquare {

PathDecider::PathDecider(const TaskConfig &config) : Decider(config) {
  // mph_assert(config.has_path_decider_config());
}

void PathDecider::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

void PathDecider::reset(const TaskConfig &config) { Task::reset(config); }

void PathDecider::unset() {}

TaskStatus PathDecider::process() {
  MLOG_PROFILING(name_.c_str());
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "world model is none!");
    return TaskStatus::STATUS_FAILED;
  }

  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }

  auto &obstacle_manager = baseline_info_->obstacle_manager();
  auto lane_status = context_->planning_status().lane_status;
  auto &lateral_output = context_->lateral_behavior_planner_output();
  auto &nudge_obstacles = lateral_output.avd_info;
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  CLEAR_LON_DECISION;

  //  ObjectNudge blocking_nudge_direction;
  //  blocking_nudge_direction.type = ObjectNudge::NO_NUDGE;

  //------------------ DDP obstacle decider overwrite --------------------//
  const auto &ddp_odc_output =
      ddp::DdpContext::Instance()->obstacle_decider_output();

  if (ddp_odc_output.odc_decision_valid) {
    // if ddp is on
    const auto &obj_decision_map = ddp_odc_output.obs_infos_map;
    for (auto obj_with_decision : obj_decision_map) {

      int obj_id = obj_with_decision.first;
      auto obj_info = obj_with_decision.second;

      ObjectDecisionType nudge_decision;
      ObjectDecisionType follow_decision;

      auto mutable_nudge_decision = nudge_decision.mutable_nudge();
      if (obj_info.lat_decision ==
          odc_interface::ObsInfo::LatDecision::LAT_IGNORE) {
        mutable_nudge_decision->type = ObjectNudge::NO_NUDGE;
      } else if (obj_info.nudge_side ==
                 odc_interface::ObsInfo::NudgeType::LEFT_NUDGE) {
        mutable_nudge_decision->type = ObjectNudge::LEFT_NUDGE;
      } else if (obj_info.nudge_side ==
                 odc_interface::ObsInfo::NudgeType::RIGHT_NUDGE) {
        mutable_nudge_decision->type = ObjectNudge::RIGHT_NUDGE;
      } else {
        mutable_nudge_decision->type = ObjectNudge::NO_NUDGE;
      }
      MSD_LOG(INFO, "ODC overwriting lateral1[%d][%d]", obj_info.id,
              mutable_nudge_decision->type);

      auto mutable_follow_decision = follow_decision.mutable_follow();
      if (obj_info.lon_decision ==
          odc_interface::ObsInfo::LonDecision::FOLLOW) {
        (void)obstacle_decision_manager.add_longitudinal_decision(
            "ddp_decider", obj_info.id, follow_decision);
        MSD_LOG(INFO, "ODC overwriting longitu[%d]", obj_info.id);
        LOG_LON_DECISION_INFO(obj_info.id, "follow", "ddp overlap");
      } else {
        mutable_nudge_decision->is_longitunidal_ignored = true;
        LOG_LON_DECISION_INFO(obj_info.id, "ignore", " ddp not overlap");
      }

      nudge_decision.setDecisionSource("ddp_decider");
      (void)obstacle_decision_manager.add_lateral_decision(
          "ddp_decider", obj_info.id, nudge_decision);
      MSD_LOG(INFO, "ODC overwriting lateral2[%d][%d]", obj_info.id,
              mutable_nudge_decision->type);
    }

  } else {
    for (auto nudge_obstacle : nudge_obstacles) {
      auto obstacle = obstacle_manager.find_obstacle(nudge_obstacle.id);
      MSD_LOG(INFO, "nudge_obstacle = %d", nudge_obstacle.id);
      if (obstacle == nullptr) {
        continue;
      }
      ObjectDecisionType nudge_decision;
      auto mutable_nudge_decision = nudge_decision.mutable_nudge();
      if (nudge_obstacle.avd_direction == "left") {
        mutable_nudge_decision->type = ObjectNudge::LEFT_NUDGE;
      } else if (nudge_obstacle.avd_direction == "right") {
        mutable_nudge_decision->type = ObjectNudge::RIGHT_NUDGE;
      } else {
        mutable_nudge_decision->type = ObjectNudge::NO_NUDGE;
      }
      mutable_nudge_decision->is_longitunidal_ignored = nudge_obstacle.ignore;
      mutable_nudge_decision->start_time = nudge_obstacle.blocked_time_begin;
      mutable_nudge_decision->time_buffer =
          nudge_obstacle.blocked_time_end - nudge_obstacle.blocked_time_begin;
      mutable_nudge_decision->priority = nudge_obstacle.avd_priority;
      mutable_nudge_decision->distance_l =
          obstacle->Type() == ObjectType::PEDESTRIAN ? 0.5 : 0.3;
      nudge_decision.setDecisionSource("path_decider");
      (void)obstacle_decision_manager.add_lateral_decision(
          "path_decider", nudge_obstacle.id, nudge_decision);
    }
  }
  //------------------ DDP obstacle decider overwrite --------------------//

  return TaskStatus::STATUS_SUCCESS;
}

} // namespace msquare
