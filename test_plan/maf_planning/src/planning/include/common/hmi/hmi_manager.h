// #pragma once
#ifndef HMI_MANAGER_H
#define HMI_MANAGER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/config_context.h"
#include "common/time_state_machine/time_state_machine.h"
#include "common/world_model.h"

namespace msquare {
class HmiManager {
public:
  HmiManager();
  ~HmiManager(){};

  void Update(const std::shared_ptr<WorldModel> world_model,
              const std::shared_ptr<BaseLineInfo> baseline_info);

private:
  std::vector<TimeStateMachine *> time_state_machines_;
};

} // namespace msquare

#endif // HMI_MANAGER_H