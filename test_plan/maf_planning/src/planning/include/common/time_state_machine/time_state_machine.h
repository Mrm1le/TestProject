// #pragma once
#ifndef TIME_STATE_MACHINE_H
#define TIME_STATE_MACHINE_H

#include <memory>
#include <vector>

#include "common/baseline_info.h"
#include "common/world_model.h"

namespace msquare {

enum class TState {
  NOT_SET = 0,
  COOLDOWN,
  RUNNING,
};

class TimeStateMachine {
public:
  TimeStateMachine() = default;
  // TimeStateMachine(const double max_run_time_s,
  //   const double min_cooldown_time_s);

  virtual ~TimeStateMachine() = default;

  void Init(double max_run_time, double min_cooldown_time);

  void Reset();

  TState Run();

  virtual bool Start() = 0;

  virtual bool Stop() = 0;

  virtual void Process(const std::shared_ptr<WorldModel> world_model,
                       const std::shared_ptr<BaseLineInfo> baseline_info) = 0;

  void SetMaxRunTime(const double max_run_time);

  void SetMinCoolTime(const double min_cooldown_time);

  double GetMaxRunTime() const { return max_run_time_s_; }

  double GetMinCoolTime() const { return min_cooldown_time_s_; }

  double GetRealRunTime() const { return real_run_time_s_; }

  double GetRealCoolTime() const { return real_cooldown_time_s_; }

  double GetCurTime() const { return cur_time_s_; }

  TState GetRealState() const { return real_state_; }

  int GetRealStateIndex() const {
    switch (real_state_) {
    case TState::RUNNING:
      return 2;
      break;
    case TState::COOLDOWN:
      return 1;
      break;
    default:
      return 0;
      break;
    }
  }

  int GetDesiredStateIndex() const {
    switch (desired_state_) {
    case TState::RUNNING:
      return 2;
      break;
    case TState::COOLDOWN:
      return 1;
      break;
    default:
      return 0;
      break;
    }
  }

  bool IsRunning() const { return real_state_ == TState::RUNNING; }

  bool IsCooldown() const { return real_state_ == TState::COOLDOWN; }

  bool CanStart() const {
    return real_cooldown_time_s_ >= min_cooldown_time_s_;
  }

private:
  bool MStart();

  bool MStop();

  void SetStartTime(const TState state);

  void SetRealTime(const TState state);

  void SetCurTime();

  void PrintParams();

protected:
  TState desired_state_ = TState::NOT_SET;

private:
  double max_run_time_s_ = 0.0;
  double min_cooldown_time_s_ = 0.0;
  double real_run_time_s_ = 0.0;
  double real_cooldown_time_s_ = 0.0;
  double start_run_time_s_ = 0.0;
  double start_cooldown_time_s_ = 0.0;
  double cur_time_s_ = 0.0;
  double init_time_s_ = MTIME()->timestamp().sec();
  TState real_state_ = TState::NOT_SET;
};

} // namespace msquare

#endif // TIME_STATE_MACHINE_H