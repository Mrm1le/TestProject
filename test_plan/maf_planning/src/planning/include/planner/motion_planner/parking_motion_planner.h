#ifndef MSQUARE_DECISION_PLANNING_PLANNER_MOTION_PLANNER_MOTION_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_MOTION_PLANNER_MOTION_PLANNER_H_

#include <vector>

#include "planner/parking_planner.h"

namespace msquare {
namespace parking {
class MotionPlanner : public Planner {
public:
  MotionPlanner(const std::shared_ptr<WorldModel> &world_model);
  virtual ~MotionPlanner() = default;
};
} // namespace parking
} // namespace msquare

#endif
