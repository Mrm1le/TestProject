#pragma once

#include "planner/tasks/task.h"

namespace msquare {

class PlannerPreprocessor {
public:
  PlannerPreprocessor() = default;

  virtual ~PlannerPreprocessor() = default;

  virtual void init(ScenarioFacadeContext *context,
                    const std::shared_ptr<WorldModel> world_model,
                    const std::shared_ptr<BaseLineInfo> baseline_info) {
    context_ = context;
    world_model_ = world_model;
    baseline_info_ = baseline_info;
  }

  virtual void process(){};

protected:
  ScenarioFacadeContext *context_;
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<BaseLineInfo> baseline_info_;
};

} // namespace msquare
