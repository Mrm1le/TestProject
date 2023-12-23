#ifndef MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_LONGITUDINAL_BEHAVIOR_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_LONGITUDINAL_BEHAVIOR_PLANNER_H_

#include <algorithm>
#include <iostream>

#include "common/obstacle_manager.h"
#include "planner/tasks/task.h"

namespace msquare {

class LongitudinalBehaviorPlanner : public Task {
public:
  explicit LongitudinalBehaviorPlanner(const TaskConfig &config);
  virtual ~LongitudinalBehaviorPlanner() = default;
  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

  TaskStatus execute(ScenarioFacadeContext *context);

private:
  virtual bool calculate();
};

} // namespace msquare
#endif
