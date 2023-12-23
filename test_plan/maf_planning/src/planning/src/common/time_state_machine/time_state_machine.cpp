#include "common/time_state_machine/time_state_machine.h"

#include "mlog_core/mlog.h"
#include "mlog_msg_id/mlog_msg_id.hpp"
#include "mtime_core/mtime.h"
#include "planning/common/common.h" // for MSD_LOG

using namespace msd_planning;

namespace msquare {

void TimeStateMachine::Init(double max_run_time, double min_cooldown_time) {
  max_run_time_s_ = max_run_time;
  min_cooldown_time_s_ = min_cooldown_time;
  real_run_time_s_ = 0.0;
  real_cooldown_time_s_ = min_cooldown_time_s_;
  real_state_ = TState::COOLDOWN;
  init_time_s_ = MTIME()->timestamp().sec();
  SetStartTime(real_state_);
  SetCurTime();
  MSD_LOG(INFO, "TST init param");
  PrintParams();
}

void TimeStateMachine::Reset() {
  real_run_time_s_ = 0.0;
  real_cooldown_time_s_ = min_cooldown_time_s_;
  real_state_ = TState::COOLDOWN;
  init_time_s_ = MTIME()->timestamp().sec();
  SetStartTime(real_state_);
  SetCurTime();
  MSD_LOG(INFO, "TST reset param");
  PrintParams();
}

bool TimeStateMachine::MStart() {
  MSD_LOG(INFO, "TST MStart() !!! ");
  PrintParams();
  SetCurTime();

  if (cur_time_s_ < min_cooldown_time_s_) {
    MSD_LOG(INFO, "TST force start in first colldown !!!!!!");
    real_state_ = TState::RUNNING;
    SetStartTime(real_state_);
    SetRealTime(real_state_);
    return true;
  }

  if (real_state_ == TState::RUNNING && real_run_time_s_ <= max_run_time_s_) {
    MSD_LOG(INFO, "TST process is running, do not start again !!!!!!");
    real_state_ = TState::RUNNING;
    return false;
  }

  if (real_state_ == TState::COOLDOWN &&
      real_cooldown_time_s_ < min_cooldown_time_s_) {
    MSD_LOG(INFO, "TST process is cooldown, do not start again !!!!!!");
    real_state_ = TState::COOLDOWN;
    return false;
  }

  real_state_ = TState::RUNNING;
  SetStartTime(real_state_);
  SetRealTime(real_state_);
  return true;
}

bool TimeStateMachine::MStop() {
  MSD_LOG(INFO, "TST MStop() !!! ");
  PrintParams();
  SetCurTime();
  if (real_state_ == TState::COOLDOWN) {
    MSD_LOG(INFO, "TST process is cooldown, do not start again !!!!!!");
    real_state_ = TState::COOLDOWN;
    return true;
  }
  real_state_ = TState::COOLDOWN;
  SetStartTime(real_state_);
  return true;
}

TState TimeStateMachine::Run() {
  MSD_LOG(INFO, "TST Run() !!! ");
  PrintParams();
  SetCurTime();
  if (desired_state_ == TState::RUNNING) {
    MStart();
  } else if (desired_state_ == TState::COOLDOWN) {
    MStop();
  }
  desired_state_ = TState::NOT_SET;

  if (real_state_ == TState::NOT_SET) {
    MSD_LOG(INFO, "TST current state is NOT_SET !!! ");
    return TState::NOT_SET;
  }
  
  if (real_state_ == TState::RUNNING) {
    MSD_LOG(INFO, "TST 111111");
    if (real_run_time_s_ >= max_run_time_s_) {
      real_state_ = TState::COOLDOWN;
      SetStartTime(real_state_);
    } else {
      real_state_ = TState::RUNNING;
    }
    SetRealTime(real_state_);
    PrintParams();
    return real_state_;
  }

  if (real_state_ == TState::COOLDOWN) {
    MSD_LOG(INFO, "TST 222222");
    SetRealTime(real_state_);
    PrintParams();
    return real_state_;
  }

  return TState::COOLDOWN;
}

void TimeStateMachine::SetStartTime(const TState state) {
  switch (state) {
  case TState::RUNNING:
    start_run_time_s_ = MTIME()->timestamp().sec() - init_time_s_;
    real_run_time_s_ =
        MTIME()->timestamp().sec() - start_run_time_s_ - init_time_s_;
    break;
  case TState::COOLDOWN:
    start_cooldown_time_s_ = MTIME()->timestamp().sec() - init_time_s_;
    real_cooldown_time_s_ =
        MTIME()->timestamp().sec() - start_cooldown_time_s_ - init_time_s_;
    break;
  default:
    // TODO
    break;
  }
}

void TimeStateMachine::SetRealTime(const TState state) {
  switch (state) {
  case TState::RUNNING:
    real_run_time_s_ =
        MTIME()->timestamp().sec() - start_run_time_s_ - init_time_s_;
    real_cooldown_time_s_ = 0.0;
    break;
  case TState::COOLDOWN:
    real_cooldown_time_s_ =
        MTIME()->timestamp().sec() - start_cooldown_time_s_ - init_time_s_;
    real_run_time_s_ = 0.0;
    break;
  default:
    // TODO
    break;
  }
}

void TimeStateMachine::SetCurTime() {
  cur_time_s_ = MTIME()->timestamp().sec() - init_time_s_;
}

void TimeStateMachine::PrintParams() {
  MSD_LOG(INFO, "TST init_time_s: %f", init_time_s_);
  MSD_LOG(INFO, "TST real_run_time: %f", real_run_time_s_);
  MSD_LOG(INFO, "TST real_cooldown_time: %f", real_cooldown_time_s_);
  MSD_LOG(INFO, "TST start_run_time: %f", start_run_time_s_);
  MSD_LOG(INFO, "TST start_cooldown_time: %f", start_cooldown_time_s_);
  int real_state_index = GetRealStateIndex();
  MSD_LOG(INFO, "TST real_state: %d", real_state_index);
  int desired_state_index = GetDesiredStateIndex();
  MSD_LOG(INFO, "TST desired_state: %d", desired_state_index);
}

void TimeStateMachine::SetMaxRunTime(const double max_run_time) {
  max_run_time_s_ = max_run_time;
}

void TimeStateMachine::SetMinCoolTime(const double min_cooldown_time) {
  min_cooldown_time_s_ = min_cooldown_time;
}

} // namespace msquare
