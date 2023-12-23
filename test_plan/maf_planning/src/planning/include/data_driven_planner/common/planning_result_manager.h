#pragma once
#include <data_driven_planner/common/basic_types.h>
#include <deque>

namespace msquare {
namespace ddp {

class PlanningResultManager {
public:
  PlanningResultManager() = default;
  ~PlanningResultManager() = default;

  // results
  void add_planning_result(const PlanningResult &planning_result);

  bool get_planning_ego_pose(double timestamp, EgoPose *ego_pose);

private:
  std::deque<PlanningResult> planning_results;
};
} // namespace ddp
} // namespace msquare
