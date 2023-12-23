#ifndef MSQUARE_DECISION_PLANNING_PLANNER_MOTION_PLANNER_LONGITUDINAL_MOTION_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_MOTION_PLANNER_LONGITUDINAL_MOTION_PLANNER_H_

#include <dlfcn.h>

#include "planner/message_type.h"
#include "planner/motion_planner/speed_planner_ceres/speed_planner_ceres.hpp"
#include "planner/planning_config.h"
#include "planner/tasks/task.h"

namespace msquare {

class LongitudinalMotionPlanner : public Task {
public:
  explicit LongitudinalMotionPlanner(const TaskConfig &config);
  virtual ~LongitudinalMotionPlanner() = default;
  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

  TaskStatus execute(ScenarioFacadeContext *context);

private:
  virtual bool calculate();

  speed_planner::SpeedPlanner speed_planner_;
};

} // namespace msquare
#endif
