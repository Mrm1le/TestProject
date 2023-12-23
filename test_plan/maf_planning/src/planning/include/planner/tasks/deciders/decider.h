#ifndef MSQUARE_DECISION_PLANNING_PLANNER_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_DECIDER_H_

#include <memory>

#include "planner/scenarios/scenario_facade.h"
#include "planner/tasks/task.h"

namespace msquare {
class Decider : public Task {
public:
  explicit Decider(const TaskConfig &config);
  virtual ~Decider() = default;

  virtual void init(std::shared_ptr<WorldModel> world_model);

  TaskStatus execute(ScenarioFacadeContext *context);

protected:
  virtual TaskStatus process() { return TaskStatus::STATUS_SUCCESS; }
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_DECIDER_H_
