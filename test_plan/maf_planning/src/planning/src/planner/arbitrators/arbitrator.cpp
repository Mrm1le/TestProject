#include "planner/arbitrators/arbitrator.h"

namespace msquare {

Arbitrator::Arbitrator(FsmContext fsm_context, std::string name) {
  current_state_ = fsm_context.state;
  world_model_ = fsm_context.world_model;
  name_ = name;
}

void Arbitrator::init(std::vector<std::shared_ptr<Arbitrator>> arbitrators) {
  sub_arbitrators_ = arbitrators;
}

std::shared_ptr<Arbitrator> Arbitrator::get_best_solution() {
  if (best_arbitrator_ == nullptr) {
    return nullptr;
  }
  return best_arbitrator_->get_best_solution();
}

std::shared_ptr<ScenarioFacadeContext>
Arbitrator::get_optimal_scenario_context() {
  return optimal_scenario_context_;
}

} // namespace msquare
