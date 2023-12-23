#pragma once

#include "planner/arbitrators/arbitrator.h"

namespace msquare {

class SequenceArbitrator : public Arbitrator {
public:
  SequenceArbitrator(FsmContext fsm_context, std::string name)
      : Arbitrator(fsm_context, name) {}
  virtual ~SequenceArbitrator() = default;
  void set_post_workflow(const ScenarioFacadeConfig &scenario_config);
  virtual void post_process() override;
  virtual void arbitrate() override;

  virtual bool invocation_condition() const;
  virtual bool commitment_condition() const;

protected:
  std::shared_ptr<ScenarioFacade> post_scenario_facade_;
};

} // namespace msquare