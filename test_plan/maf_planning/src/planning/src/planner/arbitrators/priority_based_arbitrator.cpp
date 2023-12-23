#include "planner/arbitrators/priority_based_arbitrator.h"

namespace msquare {

void PriorityArbitrator::arbitrate() {
  if (!cost_func_) {
    MSD_LOG(ERROR, "no cost_func for arbitrator[%s]", name_.c_str());
  }
  double min_cost = std::numeric_limits<double>::infinity();
  best_arbitrator_ = nullptr;
  for (auto arbitrator : sub_arbitrators_) {
    arbitrator->arbitrate();
    // double current_arbitrator_cost = evaluate(arbitrator.get());
    double current_arbitrator_cost =
        cost_func_(arbitrator->get_optimal_scenario_context(), world_model_);
    if (current_arbitrator_cost < min_cost &&
        arbitrator->higher_level_cost() < min_cost) {
      min_cost = current_arbitrator_cost;
      best_arbitrator_ = arbitrator;
      optimal_scenario_context_ = arbitrator->get_optimal_scenario_context();
      // todo: add final back
      optimal_scenario_context_->mutable_planning_status()->planning_success =
          true;
      MSD_LOG(INFO, "arbitrator priority arbitrator choose [%s] cost %f",
              arbitrator->name().c_str(), current_arbitrator_cost);
      break;
    } else {
      MSD_LOG(INFO, "arbitrator priority arbitrator [%s] failed, cost %f",
              arbitrator->name().c_str(), current_arbitrator_cost);
      if (arbitrator == sub_arbitrators_.back()) {
        MSD_LOG(INFO, "arbitrator priority final arbitrator falied, choose "
                      "first arbitrator");
        best_arbitrator_ = sub_arbitrators_.front();
        optimal_scenario_context_ =
            sub_arbitrators_.front()->get_optimal_scenario_context();
      }
    }
  }
  optimal_solution_cost_ = min_cost;
}

bool PriorityArbitrator::invocation_condition() const { return true; }

bool PriorityArbitrator::commitment_condition() const { return true; }

} // namespace msquare
