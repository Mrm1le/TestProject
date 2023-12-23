#ifndef MSQUARE_DECISION_PLANNING_LOOP_SCENARIO_FACADE_H_
#define MSQUARE_DECISION_PLANNING_LOOP_SCENARIO_FACADE_H_

#include "planner/scenarios/scenario_facade.h"

namespace msquare {

class LoopScenarioFacade : public ScenarioFacade {
public:
  LoopScenarioFacade(const ScenarioFacadeConfig &config)
      : ScenarioFacade(config) {}

  virtual void init();

  virtual bool process(std::shared_ptr<WorldModel> world_model,
                       std::shared_ptr<BaseLineInfo> baseline_info);

  // resize loop_scenario_contexts_
  virtual void generate_loop_context() = 0;

private:
  bool get_scenario_config();

private:
  bool init_ = false;
  int current_loops_ = -1;
  std::vector<ScenarioFacadeContext> loop_scenario_contexts_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_LOOP_SCENARIO_FACADE_H_
