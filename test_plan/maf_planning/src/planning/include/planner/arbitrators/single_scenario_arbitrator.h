#pragma once

#include "planner/arbitrators/arbitrator.h"

namespace msquare {

class SingleScenarioArbitrator : public Arbitrator {
public:
  SingleScenarioArbitrator(FsmContext fsm_context, std::string name)
      : Arbitrator(fsm_context, name) {}
  virtual ~SingleScenarioArbitrator() = default;
  void init(const ScenarioFacadeConfig &scenario_config);
  void
  init_from_type(const ScenarioFacadeConfig::ScenarioFacadeType scenario_type);
  ScenarioFacade &get_mutable_scenario_facade() const;
  virtual void arbitrate() override;
  virtual void calculate_higher_level_cost() override;
  virtual std::shared_ptr<Arbitrator> get_best_solution() override;

  static bool
  check_trajectory_safety(const PlanningResult &planning_result,
                          std::shared_ptr<BaseLineInfo> baseline_info,
                          std::shared_ptr<ScenarioFacadeContext> context);

  virtual bool invocation_condition() const;
  virtual bool commitment_condition() const;

  // std::shared_ptr<SingleScenarioArbitrator> shared_from_this();

protected:
  std::shared_ptr<ScenarioFacade> scenario_facade_;
};

} // namespace msquare