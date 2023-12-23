#ifndef MSQUARE_DECISION_PLANNING_PLANNER_BACK_UP_PATH_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_BACK_UP_PATH_DECIDER_H_

#include "common/obstacle_manager.h"
#include "common/world_model.h"
#include "planner/tasks/deciders/decider.h"

namespace msquare {

class BackUpPathDecider : public Decider {
public:
  BackUpPathDecider(const TaskConfig &config);

  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

private:
  TaskStatus process();
  bool check_last_published_trajectory();
  static bool
  check_trajectory_safety(const PlanningResult &planning_result,
                          std::shared_ptr<BaseLineInfo> baseline_info,
                          std::shared_ptr<ScenarioFacadeContext> context);
  void clear_longitudinal_decisions();

  void clear_lateral_decisions();
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_BACK_UP_PATH_DECIDER_H_
