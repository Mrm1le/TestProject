#pragma once

#include "planner/arbitrators/arbitrator.h"

namespace msquare {

class PriorityArbitrator : public Arbitrator {
public:
  PriorityArbitrator(FsmContext fsm_context, std::string name)
      : Arbitrator(fsm_context, name) {}
  virtual void arbitrate() override;
  virtual ~PriorityArbitrator() = default;

  virtual bool invocation_condition() const;
  virtual bool commitment_condition() const;
};

} // namespace msquare