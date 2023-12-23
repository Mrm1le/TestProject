#include "planner/scenarios/loop_scenario_facade.h"

namespace msquare {

void LoopScenarioFacade::init() {
  if (init_) {
    return;
  }

  ScenarioFacade::init();

  if (!get_scenario_config()) {
    MSD_LOG(INFO, "fail to get scenario specific config");
    return;
  }
  init_ = true;
}

bool LoopScenarioFacade::get_scenario_config() {
  // if (!config_.fixlane_cruise_scenario_config_) {
  //   MSD_LOG(INFO, "miss scenario specific config");
  //   return false;
  // }
  return true;
}

bool LoopScenarioFacade::process(std::shared_ptr<WorldModel> world_model,
                                 std::shared_ptr<BaseLineInfo> baseline_info) {
  mph_assert(nullptr != world_model);
  mph_assert(nullptr != baseline_info);
  world_model_ = world_model;
  baseline_info_ = baseline_info;
  for (auto task_iter = tasks_.begin(); task_iter != tasks_.end();
       task_iter++) {
    task_iter->second->init(world_model_);
  }

  size_t task_index = 0;
  while (task_index < task_list_.size()) {
    auto *current_task = task_list_[task_index];
    auto ret = current_task->execute(scenario_facade_context_.get());
    if (ret == TaskStatus::STATUS_FORK_LOOP) {
      generate_loop_context();
      if (loop_scenario_contexts_.empty()) {
        MSD_LOG(INFO, "no sub task loop !");
        return false;
      }
      for (size_t sub_loop = 0; sub_loop < loop_scenario_contexts_.size();
           ++sub_loop) {
        size_t sub_task_index = task_index;
        auto sub_ret = TaskStatus::STATUS_FAILED;
        while (sub_ret != TaskStatus::STATUS_SUCCESS_BREAK) {
          sub_task_index++;
          if (sub_task_index >= task_list_.size()) {
            MSD_LOG(INFO, "sub task index overlap !");
            return false;
          }
          auto *current_sub_task = task_list_[sub_task_index];
          auto sub_ret =
              current_sub_task->execute(&loop_scenario_contexts_[sub_loop]);
          if (sub_ret == TaskStatus::STATUS_FAILED) {
            MSD_LOG(INFO, "failed to run sub task[%s] in loop[%d]",
                    current_sub_task->name().c_str(), sub_loop);
            loop_scenario_contexts_[sub_loop]
                .mutable_planning_status()
                ->planning_success = false;
            break;
          } else if (sub_ret == TaskStatus::STATUS_SUCCESS_BREAK) {
            loop_scenario_contexts_[sub_loop]
                .mutable_planning_status()
                ->planning_success = true;
            break;
          } else if (sub_ret == TaskStatus::STATUS_SUCCESS) {
            sub_task_index++;
          } else {
            MSD_LOG(INFO, "illegal task return value !");
            return false;
          }
        }
      } // add evaluation process
    } else if (ret == TaskStatus::STATUS_FAILED) {
      MSD_LOG(INFO, "failed to run task[%s]", current_task->name().c_str());
      return false;
    } else if (ret == TaskStatus::STATUS_SUCCESS) {
      task_index++;
    } else if (ret == TaskStatus::STATUS_SUCCESS_BREAK) {
      // debug
      if (task_index < task_list_.size() - 1) {
        MSD_LOG(INFO, "return in advance [%d]", task_index);
      }
      break;
    }
  }
  post_process();
  return true;
}

} // namespace msquare
