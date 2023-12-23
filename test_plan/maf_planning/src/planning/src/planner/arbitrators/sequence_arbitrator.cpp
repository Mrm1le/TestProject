#include "planner/arbitrators/sequence_arbitrator.h"

namespace msquare {

void SequenceArbitrator::arbitrate() {
  if (!cost_func_) {
    MSD_LOG(ERROR, "no cost_func for arbitrator[%s]", name_.c_str());
  }
  best_arbitrator_ = nullptr;
  for (auto arbitrator : sub_arbitrators_) {
    arbitrator->arbitrate();
    // double current_arbitrator_cost = evaluate(arbitrator.get());
    double current_arbitrator_cost =
        cost_func_(arbitrator->get_optimal_scenario_context(), world_model_);
    if (current_arbitrator_cost < std::numeric_limits<double>::infinity()) {
      optimal_solution_cost_ = current_arbitrator_cost;
      best_arbitrator_ = arbitrator;
      optimal_scenario_context_ = arbitrator->get_optimal_scenario_context();
    } else {
      break;
    }
  }
  if (!best_arbitrator_) {
    MSD_LOG(INFO, "sequence arbitrator[%s] failed", name_.c_str());
    return;
  } else {
    // run remaining tasks to get complete result for higer level arbitrator
  }
  post_process();
}

void SequenceArbitrator::post_process() {
  if (post_scenario_facade_->process(world_model_)) {
  }
}

bool SequenceArbitrator::invocation_condition() const { return true; }

bool SequenceArbitrator::commitment_condition() const { return true; }

} // namespace msquare
