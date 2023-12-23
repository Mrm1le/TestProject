// #pragma once

#ifndef ACC_TAKEOVER_H
#define ACC_TAKEOVER_H

#include <memory>

#include "common/time_state_machine/time_state_machine.h"
#include "common/world_model.h"

namespace msquare {

class AccTakeoverHmi : public TimeStateMachine {
public:
  AccTakeoverHmi() = default;

  // AccTakeoverHmi(const double max_run_time,
  //   const double min_cooldown_time);

  ~AccTakeoverHmi() = default;

  virtual bool Start() override;

  virtual bool Stop() override;

  virtual void
  Process(const std::shared_ptr<WorldModel> world_model,
          const std::shared_ptr<BaseLineInfo> baseline_info) override;
};

} // namespace msquare

#endif // ACC_TAKEOVER_H