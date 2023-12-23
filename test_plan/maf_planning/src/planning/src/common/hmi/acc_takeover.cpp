#include "common/hmi/acc_takeover.h"

#include "common/config_context.h"
#include "common/planning_context.h"
#include "planning/common/common.h"

using namespace msd_planning;

namespace msquare {

// AccTakeoverHmi::AccTakeoverHmi(const double max_run_time,
//   const double min_cooldown_time) {
//   Init(max_run_time, min_cooldown_time);
// }

bool AccTakeoverHmi::Start() {
  desired_state_ = TState::RUNNING;
  return true;
}

bool AccTakeoverHmi::Stop() {
  desired_state_ = TState::COOLDOWN;
  return true;
}

void AccTakeoverHmi::Process(
    const std::shared_ptr<WorldModel> world_model,
    const std::shared_ptr<BaseLineInfo> baseline_info) {
  auto speed_planner_input =
      PlanningContext::Instance()->mutable_speed_planner_input();
  auto &acc_takeover_info = speed_planner_input->acc_takeover_info;
  auto &state_machine_param = speed_planner_input->state_machine_param;
  auto &enable_acc_takeover = acc_takeover_info.enable_acc_takeover;
  enable_acc_takeover =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.enable_acc_takeover;
  const bool dbw_status = world_model->get_vehicle_dbw_status();
  MSD_LOG(INFO, "ACCT dbw_status: %d", dbw_status);
  MSD_LOG(INFO, "ACCT enable_acc_takeover: %d", enable_acc_takeover);

  if (!dbw_status || !enable_acc_takeover) {
    MSD_LOG(INFO, "ACCT non dbw_status or takeover");
    this->Reset();
    acc_takeover_info.need_acc_takeover = false;
    acc_takeover_info.last_running = false;
    acc_takeover_info.start_flag = false;
    acc_takeover_info.stop_flag = false;

    state_machine_param.real_run_time_s = this->GetRealRunTime();
    state_machine_param.real_cooldown_time_s = this->GetRealCoolTime();
    state_machine_param.real_state = this->GetRealStateIndex();
    state_machine_param.desired_state = this->GetDesiredStateIndex();
    return;
  }

  const auto &cipv_info = speed_planner_input->cipv_info;
  const auto &ego_state =
      baseline_info->get_ego_state_manager().get_ego_state();
  const auto ego_acc = ego_state.ego_acc;

  const auto &longitudinal_motion_planner_config =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config;
  const double acc_takeover_ttc_thr =
      longitudinal_motion_planner_config.acc_takeover_ttc_thr;
  const double acc_takeover_dec_thr =
      longitudinal_motion_planner_config.acc_takeover_dec_thr;


  acc_takeover_info.start_flag =
      ego_acc < acc_takeover_dec_thr && !cipv_info.is_road_boundary &&
      cipv_info.ttc < acc_takeover_ttc_thr && this->IsCooldown();

  acc_takeover_info.stop_flag =
      (ego_acc > acc_takeover_dec_thr || cipv_info.is_road_boundary ||
       cipv_info.ttc > acc_takeover_ttc_thr) &&
      this->IsRunning();

  if (acc_takeover_info.start_flag) {
    MSD_LOG(INFO, "ACCT_state_start");
    this->Start();
  }

  if (acc_takeover_info.stop_flag) {
    MSD_LOG(INFO, "ACCT_state_stop");
    this->Stop();
  }

  this->Run();
  acc_takeover_info.need_acc_takeover = this->IsRunning() ? true : false;
  acc_takeover_info.last_running = this->IsRunning() ? true : false;
  MSD_LOG(INFO, "ACCT need_acc_takeover: %d",
          acc_takeover_info.need_acc_takeover);
  MSD_LOG(INFO, "ACCT last_running: %d", acc_takeover_info.last_running);

  state_machine_param.real_run_time_s = this->GetRealRunTime();
  state_machine_param.real_cooldown_time_s = this->GetRealCoolTime();
  state_machine_param.real_state = this->GetRealStateIndex();
  state_machine_param.desired_state = this->GetDesiredStateIndex();
  MSD_LOG(INFO, "ACCT real_run_time_s: %f",
          state_machine_param.real_run_time_s);
  MSD_LOG(INFO, "ACCT real_cooldown_time_s: %f",
          state_machine_param.real_cooldown_time_s);
  MSD_LOG(INFO, "ACCT real_state: %d", state_machine_param.real_state);
  MSD_LOG(INFO, "ACCT desired_state: %d", state_machine_param.desired_state);
}

} // namespace msquare
