#include "planner/tasks/deciders/decider.h"

namespace msquare {

Decider::Decider(const TaskConfig &config) : Task(config) {}

void Decider::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

TaskStatus Decider::execute(ScenarioFacadeContext *context) {
  if (Task::execute(context) != TaskStatus::STATUS_SUCCESS) {
    return TaskStatus::STATUS_FAILED;
  }
  if (context_->planning_status().planning_success) {
    return TaskStatus::STATUS_SUCCESS_BREAK;
  }
  return process();
}

} // namespace msquare
