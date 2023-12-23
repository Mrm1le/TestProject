#include "planner/tasks/task.h"

namespace msquare {

Task::Task(const TaskConfig &config) : config_(config) {
  // name_ = TaskTypeNames[static_cast<int>(config_.task_type()) - 1];
}

const std::string &Task::name() const { return name_; }

void Task::init(std::shared_ptr<WorldModel> world_model) {
  // mph_assert(nullptr != scenario_facade);
  // scenario_facade_ = scenario_facade;
  world_model_ = world_model;
}

void Task::reset(const TaskConfig &config) {
  config_ = config;
  name_ = TaskTypeNames[static_cast<int>(config_.task_type()) - 1];
}

void Task::lat_reset(){};

void Task::unset() {}

TaskStatus Task::execute() { return TaskStatus::STATUS_SUCCESS; }

TaskStatus Task::execute(ScenarioFacadeContext *context) {
  context_ = context;
  auto &planning_status = context_->planning_status();
  int target_lane_id = planning_status.lane_status.target_lane_id;
  baseline_info_ = world_model_->get_baseline_info(target_lane_id);

  auto &obstacle_manager = baseline_info_->obstacle_manager();
  auto &obstacle_decsion_mgr = context_->mutable_obstacle_decision_manager();

  for (auto ptr_obstacle : obstacle_manager.get_obstacles().Items()) {
    if (obstacle_decsion_mgr.find_obstacle_decision(ptr_obstacle->Id()) ==
        nullptr) {
      ObstacleDecision obs_decision(ptr_obstacle->Id());
      obstacle_decsion_mgr.add_obstacle_decision(obs_decision);
    }
  }

  obstacle_decsion_mgr.set_hash_id_map(obstacle_manager.hash_id_map());

  return TaskStatus::STATUS_SUCCESS;
}

} // namespace msquare
