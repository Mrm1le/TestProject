#pragma once

#include "planner/behavior_planner/lateral_behavior_state.h"
#include "planner/scenarios/scenario_facade.h"
#include <functional>

namespace msquare {

using ArbitratorCostFunc =
    std::function<double(const std::shared_ptr<ScenarioFacadeContext>,
                         const std::shared_ptr<WorldModel>)>;

class Arbitrator : public std::enable_shared_from_this<Arbitrator> {
public:
  Arbitrator(FsmContext fsm_context, std::string name);
  virtual ~Arbitrator() = default;
  virtual void init(std::vector<std::shared_ptr<Arbitrator>> arbitrators);
  virtual void arbitrate() = 0;
  virtual bool invocation_condition() const = 0;
  virtual bool commitment_condition() const = 0;
  // virtual double evaluate(const Arbitrator *arbitrator) const = 0;
  virtual void calculate_higher_level_cost() {
    higher_level_cost_ = optimal_solution_cost_;
  }
  virtual void post_process() {}

  virtual std::shared_ptr<Arbitrator> get_best_solution();

  std::shared_ptr<ScenarioFacadeContext> get_optimal_scenario_context();

  void set_sub_arbitrators(
      std::vector<std::shared_ptr<Arbitrator>> sub_arbitrators) {
    sub_arbitrators_ = sub_arbitrators;
  }

  void add_sub_arbitrators(std::shared_ptr<Arbitrator> arbitrator) {
    sub_arbitrators_.push_back(arbitrator);
  }

  void set_cost_function(ArbitratorCostFunc func) {
    cost_func_ = std::move(func);
  }

  double cost() { return optimal_solution_cost_; }
  void set_cost(double cost) { optimal_solution_cost_ = cost; }
  double higher_level_cost() { return higher_level_cost_; }

  const std::string &name() const { return name_; }
  const std::string &get_msg() const { return msg_; }

protected:
  std::vector<std::shared_ptr<Arbitrator>> sub_arbitrators_;
  std::shared_ptr<Arbitrator> best_arbitrator_{nullptr};
  std::shared_ptr<ScenarioFacadeContext> optimal_scenario_context_{nullptr};
  double optimal_solution_cost_{std::numeric_limits<double>::infinity()};
  double higher_level_cost_{std::numeric_limits<double>::infinity()};
  std::string name_;
  std::string msg_;
  std::shared_ptr<WorldModel> world_model_;
  ArbitratorCostFunc cost_func_;
  int current_state_;
};

} // namespace msquare