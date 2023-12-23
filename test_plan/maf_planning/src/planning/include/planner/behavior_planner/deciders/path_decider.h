#ifndef MSQUARE_DECISION_PLANNING_PLANNER_PATH_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_PATH_DECIDER_H_

#include "common/obstacle_manager.h"
#include "common/world_model.h"
#include "data_driven_planner/common/ddp_context.h"
#include "planner/tasks/deciders/decider.h"

namespace msquare {

class PathDecider : public Decider {
public:
  PathDecider(const TaskConfig &config);

  virtual ~PathDecider() = default;

  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

private:
  TaskStatus process();
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_PATH_DECIDER_H_
