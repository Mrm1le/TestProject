#ifndef MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_MOTION_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_LATERAL_MOTION_PLANNER_H_

#include "common/trajectory/trajectory_stitcher.h"
#include "common/utils/gauss_quad_constants.h"
#include "path_planner_ceres.hpp"
#include "planner/message_type.h"
#include "planner/tasks/task.h"

namespace msquare {

class LateralMotionPlanner : public Task {
public:
  LateralMotionPlanner(const TaskConfig &config);
  virtual ~LateralMotionPlanner() = default;
  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

  void lat_reset();

  TaskStatus execute(ScenarioFacadeContext *context);

private:
  virtual bool calculate();
  void get_init_car_state(std::array<double, 4> &init_state, double *adc_vel);

  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  LateralMotionPlannerOutput debugInfo_{};

  path_planner::PathPlanner path_planner_;
};

} // namespace msquare

#endif
