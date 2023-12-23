#pragma once

#include "common/planning_context.h" // OpenspaceDeciderOutput
#include "mtaskflow/mtaskflow.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/search_based_planner.h"
#include <iostream>

namespace msquare {
namespace parking {

class SBPlanningTask : public mtaskflow::FlowTask {
public:
  SBPlanningTask(
      mtaskflow::FlowReceiver<OpenspaceDeciderOutput> sbp_problem_receiver,
      mtaskflow::FlowPublisher<SbpResult> sbp_result_publisher,
      mtaskflow::FlowPublisher<std::string> sbp_debug_publisher = nullptr);
  ~SBPlanningTask();

  void on_running();

private:
  mtaskflow::FlowReceiver<OpenspaceDeciderOutput> sbp_problem_receriver_;
  mtaskflow::FlowPublisher<SbpResult> sbp_result_publisher_;
  mtaskflow::FlowPublisher<std::string> sbp_debug_publisher_;
  std::shared_ptr<hybrid_a_star_2::HybridAstar2> hybrid_a_star_solver_;
  std::shared_ptr<SearchBasedPlanner> solver_;
};

} // namespace parking
} // namespace msquare
