#include "common/hmi/hmi_manager.h"

#include "common/hmi/acc_takeover.h"

namespace msquare {
HmiManager::HmiManager() {
  time_state_machines_.clear();
  if (msquare::ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.enable_acc_takeover) {
    TimeStateMachine *acc_takeover_hmi = new AccTakeoverHmi;
    // AccTakeoverHmi acc_takeover_hmi;
    acc_takeover_hmi->Init(
        ConfigurationContext::Instance()
            ->planner_config()
            .longitudinal_motion_planner_config.acc_takeover_max_run_time,
        ConfigurationContext::Instance()
            ->planner_config()
            .longitudinal_motion_planner_config.acc_takeover_min_cooldown_time);
    time_state_machines_.push_back(acc_takeover_hmi);
  }
}

void HmiManager::Update(const std::shared_ptr<WorldModel> world_model,
                        const std::shared_ptr<BaseLineInfo> baseline_info) {
  const size_t num = time_state_machines_.size();
  for (size_t i = 0; i < time_state_machines_.size(); ++i) {
    time_state_machines_[i]->Process(world_model, baseline_info);
  }
}
} // namespace msquare
