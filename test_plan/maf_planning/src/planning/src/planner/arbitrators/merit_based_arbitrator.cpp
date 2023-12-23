#include "planner/arbitrators/merit_based_arbitrator.h"

namespace msquare {

void MeritArbitrator::arbitrate() {
  if (!cost_func_) {
    MSD_LOG(ERROR, "no cost_func for arbitrator[%s]", name_.c_str());
  }
  double min_cost = std::numeric_limits<double>::infinity();
  best_arbitrator_ = sub_arbitrators_.front();
  for (auto arbitrator : sub_arbitrators_) {
    arbitrator->arbitrate();
    // double current_arbitrator_cost = evaluate(arbitrator.get());
    double current_arbitrator_cost =
        cost_func_(arbitrator->get_optimal_scenario_context(), world_model_);
    arbitrator->set_cost(current_arbitrator_cost);
    MSD_LOG(INFO, "debug sub_arbitrator[%s] cost %f ",
            arbitrator->name().c_str(), current_arbitrator_cost);
    if (current_arbitrator_cost < min_cost) {
      min_cost = current_arbitrator_cost;
      best_arbitrator_ = arbitrator;
      optimal_scenario_context_ = arbitrator->get_optimal_scenario_context();
      if (current_arbitrator_cost <= std::numeric_limits<double>::lowest()) {
        break;
      }
    }
  }
  optimal_solution_cost_ = min_cost;
}

bool MeritArbitrator::invocation_condition() const { return true; }

bool MeritArbitrator::commitment_condition() const { return true; }

} // namespace msquare
