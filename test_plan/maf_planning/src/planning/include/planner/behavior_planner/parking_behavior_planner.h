#ifndef MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_H_

#include "planner/parking_planner.h"

namespace msquare {
namespace parking {
class BehaviorPlanner : public Planner {
public:
  BehaviorPlanner(const std::shared_ptr<WorldModel> &world_model);
  virtual ~BehaviorPlanner();
};
} // namespace parking
} // namespace msquare

#endif
