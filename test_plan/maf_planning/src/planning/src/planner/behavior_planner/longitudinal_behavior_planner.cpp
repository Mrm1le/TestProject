#include "planner/behavior_planner/longitudinal_behavior_planner.h"
#include "common/config_context.h"
#include "common/obstacle_manager.h"
#include "common/obstacle_process_utils.h"
#include "mjson/mjson.hpp"
#include "planning/common/common.h"

// #define USE_ASTAR true

namespace msquare {

void LongitudinalBehaviorPlanner::reset(const TaskConfig &config) {
  Task::reset(config);
}

void LongitudinalBehaviorPlanner::unset() {}

TaskStatus
LongitudinalBehaviorPlanner::execute(ScenarioFacadeContext *context) {
  MLOG_PROFILING(name_.c_str());
  if (Task::execute(context) != TaskStatus::STATUS_SUCCESS) {
    return TaskStatus::STATUS_FAILED;
  }
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "world model is none!");
    return TaskStatus::STATUS_FAILED;
  }
  if (!baseline_info_ || !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "baseline info is invalid!");
    return TaskStatus::STATUS_FAILED;
  }
  if (context_->planning_status().planning_success) {
    return TaskStatus::STATUS_SUCCESS_BREAK;
  }
  if (calculate()) {
    return TaskStatus::STATUS_SUCCESS;
  } else {
    return TaskStatus::STATUS_FAILED;
  }
}

void LongitudinalBehaviorPlanner::init(
    std::shared_ptr<WorldModel> world_model) {
  // todo: set st_graph_generator here
  Task::init(world_model);
}

LongitudinalBehaviorPlanner::LongitudinalBehaviorPlanner(
    const TaskConfig &config)
    : Task(config) {}

bool LongitudinalBehaviorPlanner::calculate() {
  // auto &obstacle_decision_manager =
  //     context_->mutable_obstacle_decision_manager();

  // add origin lane obstalce follow decision
  // const auto &lc_status =
  // context_->lateral_behavior_planner_output().lc_status; bool is_lane_change
  // =
  //     lc_status == "left_lane_change" || lc_status == "right_lane_change";
  // auto tmp_leadone = world_model_->mutable_lateral_obstacle().tleadone();
  // if (is_lane_change && tmp_leadone != nullptr) {
  //   auto ptr_obstacle_decision =
  //       obstacle_decision_manager.find_obstacle_decision(tmp_leadone->track_id);
  //   if (ptr_obstacle_decision) {
  //     ObjectDecisionType lon_follow_decision{};
  //     auto follow_decision = lon_follow_decision.mutable_follow();
  //     follow_decision->in_origin_lane = true;
  //     ptr_obstacle_decision->AddLongitudinalDecision("New_Planner",
  //                                                    lon_follow_decision);
  //     LOG_LON_DECISION_INFO(tmp_leadone->track_id, "follow",
  //                           "follow obstacle at origin lane");
  //   }
  // }

  return true;
}

} // namespace msquare
