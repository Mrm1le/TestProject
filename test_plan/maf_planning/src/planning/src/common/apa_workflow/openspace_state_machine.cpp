#include "common/apa_workflow/openspace_state_machine.h"
#include "common/utils/machine_logger.h"
#include "planner/behavior_planner/deciders/openspace_decider.h"
#include "planner/behavior_planner/deciders/apa_openspace_decider.h"
#include "planner/behavior_planner/deciders/apoa_openspace_decider.h"
#include "planner/behavior_planner/deciders/rpa_straight_openspace_decider.h"
#include "common/context_logger.h"

namespace msquare {
namespace parking {
namespace openspace_state_machine {

using planning_math::Vec2d;

OpenspaceStateMachine::OpenspaceStateMachine(
    const std::shared_ptr<WorldModel> &world_model,
    const std::shared_ptr<PathSampler> &path_sampler)
    : world_model_(world_model), path_sampler_(path_sampler),
      planning_run_timer_(PLANNING_RUN_DURATION),
      wlc_damping_timer_(WLC_DAMPING_DURATION) {

  openspace_decider_ = std::make_unique<OpenspaceDecider>(world_model_);

  setupCallbackFunctions();
}

OpenspaceStateMachine::~OpenspaceStateMachine() {
  destroyMachine();
}

bool OpenspaceStateMachine::update() {
  if (!openspace_state_machine_) {
    return false;
  }
  openspace_state_machine_->update();

  openspace_decider_->requestSquareMapping(); // for consistent sbp_request

  openspace_decider_->process();              // for consistent sbp_request

  return true;
}

bool OpenspaceStateMachine::changeToStandby() {
  if (!openspace_state_machine_) {
    return false;
  }
  openspace_state_machine_->changeTo<openspace_state_machine::Standby>();
  return true;
}

bool OpenspaceStateMachine::createMachineParkIn() {
#ifdef HFSM_ENABLE_LOG_INTERFACE
  openspace_state_machine_ =
      std::make_unique<hfsm::Machine<openspace_state_machine::Context>::PeerRoot<
          openspace_state_machine::COMPOSITE>>(openspace_state_machine_ctx_,
                                              HfsmContextLogger::Instance());
#else
  openspace_state_machine_ =
      std::make_unique<hfsm::Machine<openspace_state_machine::Context>::PeerRoot<
          openspace_state_machine::COMPOSITE>>(openspace_state_machine_ctx_);
#endif

  openspace_decider_ = std::make_unique<ApaOpenspaceDecider>(world_model_);

  return true;
}
bool OpenspaceStateMachine::createMachineParkOut() {
#ifdef HFSM_ENABLE_LOG_INTERFACE
  openspace_state_machine_ =
      std::make_unique<hfsm::Machine<openspace_state_machine::Context>::PeerRoot<
          openspace_state_machine::COMPOSITE>>(openspace_state_machine_ctx_,
                                              HfsmContextLogger::Instance());
#else
  openspace_state_machine_ =
      std::make_unique<hfsm::Machine<openspace_state_machine::Context>::PeerRoot<
          openspace_state_machine::COMPOSITE>>(openspace_state_machine_ctx_);
#endif

  openspace_decider_ = std::make_unique<ApoaOpenspaceDecider>(world_model_);

  return true;
}
bool OpenspaceStateMachine::createMachineRpaStraight() {
#ifdef HFSM_ENABLE_LOG_INTERFACE
  openspace_state_machine_ =
      std::make_unique<hfsm::Machine<openspace_state_machine::Context>::PeerRoot<
          openspace_state_machine::COMPOSITE>>(openspace_state_machine_ctx_,
                                              HfsmContextLogger::Instance());
#else
  openspace_state_machine_ =
      std::make_unique<hfsm::Machine<openspace_state_machine::Context>::PeerRoot<
          openspace_state_machine::COMPOSITE>>(openspace_state_machine_ctx_);
#endif

  openspace_decider_ =
      std::make_unique<RpaStraightOpenspaceDecider>(world_model_);

  return true;
}

bool OpenspaceStateMachine::destroyMachine() {
  openspace_state_machine_.reset(nullptr);
  return true;
}

openspace_state_machine::OpenspaceStateEnum
OpenspaceStateMachine::getCurrentState() {
  if (openspace_state_machine_) {
    if (openspace_state_machine_->isActive<openspace_state_machine::Init>()) {
      return openspace_state_machine::OpenspaceStateEnum::INIT;
    } else if (openspace_state_machine_
                   ->isActive<openspace_state_machine::Standby>()) {
      return openspace_state_machine::OpenspaceStateEnum::STANDBY;
    } else if (openspace_state_machine_
                   ->isActive<openspace_state_machine::Planning>()) {
      return openspace_state_machine::OpenspaceStateEnum::PLANNING;
    } else if (openspace_state_machine_
                   ->isActive<openspace_state_machine::Fallback>()) {
      return openspace_state_machine::OpenspaceStateEnum::FALLBACK;
    } else if (openspace_state_machine_
                   ->isActive<openspace_state_machine::Running>()) {
      return openspace_state_machine::OpenspaceStateEnum::RUNNING;
    } else if (openspace_state_machine_
                   ->isActive<openspace_state_machine::Finish>()) {
      return openspace_state_machine::OpenspaceStateEnum::FINISH;
    }
  }
  // invalid call
  return OpenspaceStateEnum::INVALID;
}

void OpenspaceStateMachine::setupCallbackFunctions() {
  /*** state openspace ***/
  openspace_state_machine_ctx_
      .register_entry_callback<openspace_state_machine::Standby>(
          std::bind(&OpenspaceStateMachine::onEntryOpenspaceStandby, this));
  openspace_state_machine_ctx_
      .register_entry_callback<openspace_state_machine::Planning>(
          std::bind(&OpenspaceStateMachine::onEntryOpenspacePlanning, this));
  openspace_state_machine_ctx_
      .register_entry_callback<openspace_state_machine::Running>(
          std::bind(&OpenspaceStateMachine::onEntryOpenspaceRunning, this));
  openspace_state_machine_ctx_
      .register_entry_callback<openspace_state_machine::Fallback>(
          std::bind(&OpenspaceStateMachine::onEntryOpenspaceFallback, this));
  openspace_state_machine_ctx_
      .register_entry_callback<openspace_state_machine::Finish>(
          std::bind(&OpenspaceStateMachine::onEntryOpenspaceFinish, this));

  openspace_state_machine_ctx_
      .register_update_callback<openspace_state_machine::Standby>(
          std::bind(&OpenspaceStateMachine::onUpdateOpenspaceStandby, this));
  openspace_state_machine_ctx_
      .register_update_callback<openspace_state_machine::Planning>(
          std::bind(&OpenspaceStateMachine::onUpdateOpenspacePlanning, this));
  openspace_state_machine_ctx_
      .register_update_callback<openspace_state_machine::Running>(
          std::bind(&OpenspaceStateMachine::onUpdateOpenspaceRunning, this));
  openspace_state_machine_ctx_
      .register_update_callback<openspace_state_machine::Fallback>(
          std::bind(&OpenspaceStateMachine::onUpdateOpenspaceFallback, this));
  openspace_state_machine_ctx_
      .register_update_callback<openspace_state_machine::Finish>(
          std::bind(&OpenspaceStateMachine::onUpdateOpenspaceFinish, this));

  openspace_state_machine_ctx_
      .register_transition_callback<openspace_state_machine::Standby>(
          std::bind(&OpenspaceStateMachine::onTransitionOpenspaceStandby, this,
                    std::placeholders::_1));
  openspace_state_machine_ctx_
      .register_transition_callback<openspace_state_machine::Planning>(
          std::bind(&OpenspaceStateMachine::onTransitionOpenspacePlanning, this,
                    std::placeholders::_1));
  openspace_state_machine_ctx_
      .register_transition_callback<openspace_state_machine::Running>(
          std::bind(&OpenspaceStateMachine::onTransitionOpenspaceRunning, this,
                    std::placeholders::_1));
  openspace_state_machine_ctx_
      .register_transition_callback<openspace_state_machine::Fallback>(
          std::bind(&OpenspaceStateMachine::onTransitionOpenspaceFallback, this,
                    std::placeholders::_1));
  openspace_state_machine_ctx_
      .register_transition_callback<openspace_state_machine::Finish>(
          std::bind(&OpenspaceStateMachine::onTransitionOpenspaceFinish, this,
                    std::placeholders::_1));

  openspace_state_machine_ctx_
      .register_leave_callback<openspace_state_machine::Standby>(
          std::bind(&OpenspaceStateMachine::onLeaveOpenspaceStandby, this));
  openspace_state_machine_ctx_
      .register_leave_callback<openspace_state_machine::Planning>(
          std::bind(&OpenspaceStateMachine::onLeaveOpenspacePlanning, this));
  openspace_state_machine_ctx_
      .register_leave_callback<openspace_state_machine::Running>(
          std::bind(&OpenspaceStateMachine::onLeaveOpenspaceRunning, this));
  openspace_state_machine_ctx_
      .register_leave_callback<openspace_state_machine::Fallback>(
          std::bind(&OpenspaceStateMachine::onLeaveOpenspaceFallback, this));
  openspace_state_machine_ctx_
      .register_leave_callback<openspace_state_machine::Finish>(
          std::bind(&OpenspaceStateMachine::onLeaveOpenspaceFinish, this));
}


void OpenspaceStateMachine::onEntryOpenspaceStandby() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_move_ready = false;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_moved = false;
  
  PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->update_wheel_stop = false;
  PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->is_dynamic_planning = false;

  (void)CarParams::GetInstance()->setLatInflation(
      CarParams::GetInstance()->lat_inflation_min);

  (void)CarParams::GetInstance()->setLonInflation(
      CarParams::GetInstance()->lon_inflation_min);

  const double PLANNING_RUN_DURATION =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.planning_run_duration;
  planning_run_timer_.set_timer_duration(PLANNING_RUN_DURATION);
  planning_run_timer_.reset();
  wlc_damping_timer_.reset();
}

void OpenspaceStateMachine::onUpdateOpenspaceStandby() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  // update ego state realtime
  auto output =
      PlanningContext::Instance()->mutable_parking_behavior_planner_output();
  TrajectoryPoint &init_traj_point = output->init_traj_point;

  EgoState ego_state = world_model_->get_ego_state();

  init_traj_point.path_point.x = ego_state.ego_pose.x;
  init_traj_point.path_point.y = ego_state.ego_pose.y;
  init_traj_point.path_point.theta = ego_state.ego_pose.theta;
  init_traj_point.steer =
      ego_state.ego_steer_angle / CarParams::GetInstance()->steer_ratio;
  openspace_decider_->requestSquareMapping();
  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  if (parking_slot_info.type.value == ParkingSlotType::PERPENDICULAR) { // perpendicular
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_narrow_channel = openspace_decider_->isNarrowChannelScenario();
  }
  if (PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_narrow_channel &&
      !CarParams::GetInstance()
           ->car_config.common_config.use_sop_openspace_planner) {
    const double smaller_lon_inflation =
        CarParams::GetInstance()
            ->car_config.parkin_decider_config
            .smaller_lon_inflation_narrow_perpendicular_slot;
    (void)CarParams::GetInstance()->setLonInflationMin(smaller_lon_inflation);
    (void)CarParams::GetInstance()->setLonInflation(smaller_lon_inflation);
  }
}

void OpenspaceStateMachine::onTransitionOpenspaceStandby(
    hfsm::Machine<openspace_state_machine::Context>::Control &control) {
      MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  bool is_standby_finished = world_model_->get_ego_state().is_static;
  if (PlanningContext::Instance()->planning_status().wlc_info.is_approached) {
    is_standby_finished = is_standby_finished && wlc_damping_timer_.is_timeout();
    if (is_standby_finished) {
      const WlcInfo &wlc_info_ref = PlanningContext::Instance()->planning_status().wlc_info;
      LOG_TO_CONTEXT("%s: planning at wlc offset(%f, %f)", __FUNCTION__,
        wlc_info_ref.x_offset, wlc_info_ref.y_offset);
    }
  }
  if (is_standby_finished) {
    control.changeTo<openspace_state_machine::Planning>();
  }
}

void OpenspaceStateMachine::onLeaveOpenspaceStandby() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  openspace_decider_->process();
  if (!world_model_->get_control_test_flag()) {
    StrategyParams::GetInstance()->enable_revserse_search_ =
        openspace_decider_->isReverseSearchRequired();
  } else {
    StrategyParams::GetInstance()->default_check_endsegent_len_ = 0;
    StrategyParams::GetInstance()->enable_revserse_search_ = false;
  }
}

void OpenspaceStateMachine::onEntryOpenspacePlanning() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
#ifdef SBP_THREAD_MODE
  openspace_motion_planner_ = std::unique_ptr<OpenspaceMotionPlanner>(
      new OpenspaceMotionPlanner(world_model_));
  openspace_motion_planner_->init_problem();
#else
  if (PlanningContext::Instance()
          ->mutable_openspace_decider_output()
          ->is_problem_ready) {
    MSD_LOG(WARN, "Last openspace problem is not sent");
  }
  PlanningContext::Instance()
      ->mutable_openspace_decider_output()
      ->is_problem_ready = true;
#endif
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_planned = true;
}

void OpenspaceStateMachine::onLeaveOpenspacePlanning() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
#ifdef SBP_THREAD_MODE
  openspace_motion_planner_.reset(nullptr);
#endif
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->is_fail = false;
}

void OpenspaceStateMachine::onUpdateOpenspacePlanning() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
#ifdef SBP_THREAD_MODE
  openspace_motion_planner_->calculate();
#endif
}

void OpenspaceStateMachine::onTransitionOpenspacePlanning(
    hfsm::Machine<openspace_state_machine::Context>::Control &control) {
      MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  auto odo = PlanningContext::Instance()->openspace_motion_planner_output();
  MSD_LOG(INFO, "[%s] odo.is_plan_ready:%d, odo.is_fail:%d",__FUNCTION__, odo.is_plan_ready, odo.is_fail);
  if (odo.is_plan_ready) {
    PlanningContext::Instance()
        ->mutable_openspace_motion_planner_output()
        ->is_plan_ready = false;
    PlanningContext::Instance()
        ->mutable_openspace_motion_planner_output()
        ->openspace_fallback_cnt = 0;
#ifdef SBP_THREAD_MODE
    PlanningContext::Instance()->mutable_open_space_path()->stash(
        openspace_motion_planner_->get_result());
#else
    // MSD_LOG(WARN, "%s: openspace traj size is %d", __FUNCTION__,
    // odo.traj.size());
    PlanningContext::Instance()->mutable_open_space_path()->stash(odo.traj);
#endif
    control.changeTo<openspace_state_machine::Running>();
  } else if (odo.is_fail) {
    if (PlanningContext::Instance()
            ->mutable_openspace_decider_output()
            ->is_active_replan) {
      MSD_LOG(INFO, "[%s] is_active_replan",__FUNCTION__);

      bool is_using_sop =
          (PlanningContext::Instance()
               ->parking_behavior_planner_output()
               .parking_slot_info.type.value == ParkingSlotType::PARALLEL)
              ? (CarParams::GetInstance()
                     ->car_config.common_config
                     .use_sop_openspace_planner_parallel)
              : (CarParams::GetInstance()
                     ->car_config.common_config.use_sop_openspace_planner);

      if (is_using_sop) {
        control.changeTo<openspace_state_machine::Running>();
        PlanningContext::Instance()
            ->mutable_openspace_motion_planner_output()
            ->openspace_fallback_cnt = 0;
      } else {
        if (PlanningContext::Instance()
                ->mutable_openspace_motion_planner_output()
                ->openspace_fallback_cnt > 0) {
          MSD_LOG(INFO, "[%s] openspace_fallback_cnt > 0", __FUNCTION__);
          control.changeTo<openspace_state_machine::Running>();
          PlanningContext::Instance()
              ->mutable_openspace_motion_planner_output()
              ->openspace_fallback_cnt = 0;
        } else {
          control.changeTo<openspace_state_machine::Fallback>();
        }
      }

    } else {
      PlanningContext::Instance()->mutable_open_space_path()->stash(
          std::vector<TrajectoryPoint>());
      control.changeTo<openspace_state_machine::Fallback>();
    }
  }
}

void OpenspaceStateMachine::onEntryOpenspaceFallback() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  size_t &openspace_fallback_cnt_ref =
      PlanningContext::Instance()
          ->mutable_openspace_motion_planner_output()
          ->openspace_fallback_cnt;
  openspace_fallback_cnt_ref += 1;
  if (openspace_fallback_cnt_ref == 1 &&
      std::abs(StrategyParams::GetInstance()->default_check_endsegent_len_) >
          1e-6) {
    openspace_fallback_cnt_ref = 0;
    StrategyParams::GetInstance()->default_check_endsegent_len_ = 0;
  }

  if (openspace_fallback_cnt_ref >= 1) {
    // TODO@huangzhengming#0: uniform
    CarParams::GetInstance()->shrink_ratio_for_lines_ =
        CarParams::GetInstance()->shrink_ratio_for_lines_min_;

    (void)CarParams::GetInstance()->setLatInflation(
        CarParams::GetInstance()->lat_inflation_min);

    (void)CarParams::GetInstance()->setLonInflation(
        CarParams::GetInstance()->lon_inflation_min);

    HybridAstarConfig::GetInstance()->max_iter =
        HybridAstarConfig::GetInstance()->max_iter_max;

    HybridAstarConfig::GetInstance()->footprint_model_precise_ = 6;

    HybridAstarConfig::GetInstance()->next_node_num = 5;
  }
  MSD_LOG(ERROR, "onTransitionOpenspaceFallback::fallback_cnt %d",
          openspace_fallback_cnt_ref);
}

void OpenspaceStateMachine::onTransitionOpenspaceFallback(
    const hfsm::Machine<openspace_state_machine::Context>::Control &control) {
      MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  // control.changeTo<openspace_state_machine::Standby>();
}

void OpenspaceStateMachine::onTransitionOpenspaceRunning(
    hfsm::Machine<openspace_state_machine::Context>::Control &control) {
  if (msquare::CarParams::GetInstance()->car_config.lon_config.use_sop_algorithm) {
    runTransitionOpenspaceRunningSOP(control);
  } else {
    runTransitionOpenspaceRunning(control);
  }      
}

void OpenspaceStateMachine::runTransitionOpenspaceRunning(
    hfsm::Machine<openspace_state_machine::Context>::Control &control) {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  double has_moved_min_velocity =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.has_moved_min_velocity;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_moved =
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .has_moved ||
      abs(world_model_->get_ego_state().ego_vel) > has_moved_min_velocity;
  if (world_model_->get_pause_status()) {
    planning_run_timer_.reset();
  }

  bool deviated = false;

  double MPC_ERR_THRE_IN_SLOT =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.deviated_max_dist_allowd;
  double MPC_ERR_THRE_OUT_SLOT =
      CarParams::GetInstance()
          ->car_config.parkin_decider_config.deviated_max_dist_allowd;
  if (PlanningContext::Instance()->planning_status().scenario.status_type ==
          StatusType::APA &&
      PlanningContext::Instance()->planning_status().planning_result.gear !=
          GearState::DRIVE) {
    if (world_model_->is_parking_svp()) {
      MPC_ERR_THRE_IN_SLOT = 0.15;
      MPC_ERR_THRE_OUT_SLOT = 0.25;
    } else {
      MPC_ERR_THRE_IN_SLOT =
          CarParams::GetInstance()
              ->car_config.parkin_decider_config.deviated_max_dist_allowd;
      MPC_ERR_THRE_OUT_SLOT =
          CarParams::GetInstance()
              ->car_config.parkin_decider_config.deviated_max_dist_allowd;
    }

    if (!world_model_->get_mpc_trajectory().empty()) {
      const auto &slot_box = PlanningContext::Instance()
                                 ->mutable_parking_behavior_planner_output()
                                 ->parking_lot->getBox();

      PathPoint mpc_terminal_pp = world_model_->get_mpc_trajectory().back();
      bool is_mpc_end_inside_slot =
          slot_box.IsPointIn(Vec2d(mpc_terminal_pp.x, mpc_terminal_pp.y));
      bool future_deviated_in_slot =
          !can_follow_plan_traj(MPC_ERR_THRE_IN_SLOT) && is_mpc_end_inside_slot;
      bool future_deviated_out_slot =
          !can_follow_plan_traj(MPC_ERR_THRE_OUT_SLOT) &&
          !is_mpc_end_inside_slot;
      if (future_deviated_in_slot) {
        MSD_LOG(WARN, "onTransitionOpenspaceRunning::future_deviated_in_slot!");
      }
      if (future_deviated_out_slot) {
        MSD_LOG(WARN,
                "onTransitionOpenspaceRunning::future_deviated_out_slot!");
      }

      deviated =
          deviated || future_deviated_in_slot || future_deviated_out_slot;
    }
  } else {
    deviated = deviated || !can_follow_plan_traj(MPC_ERR_THRE_OUT_SLOT);
  }

  deviated = deviated && !PlanningContext::Instance()
                              ->planning_status()
                              .planning_result.traj_pose_array.empty();
  deviated =
      deviated &&
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;

  // forbidden deviate check for control test mode
  if (world_model_->get_control_test_flag()) {
    deviated = false;
  }

  if (deviated && !world_model_->get_pause_status()) {
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->init_traj_point.v =
        PlanningContext::Instance()->planning_status().planning_result.gear ==
                GearState::REVERSE
            ? 1
            : -1;
    LOG_TO_CONTEXT("%s: replan for ego pose deviated from trajectory.",
                   __FUNCTION__);
    control.changeTo<openspace_state_machine::Standby>();
  }
  double min_finish_len = CarParams::GetInstance()->car_config.common_config.min_finish_len;
  if (path_sampler_->is_at_last_segment() &&
       PlanningContext::Instance()
               ->longitudinal_behavior_planner_output()
               .traj_length < min_finish_len && world_model_->get_ego_state().is_static) {
    control.changeTo<openspace_state_machine::Finish>();
  }
  // set is_approached in all is_finish strategy when ego is static in wlc slot
  else if (PlanningContext::Instance()
               ->parking_behavior_planner_output()
               .is_finish &&
           world_model_->get_ego_state().is_static &&
           PlanningContext::Instance()->planning_status().wlc_info.is_valid) {
    control.changeTo<openspace_state_machine::Finish>();
  }
}

void OpenspaceStateMachine::runTransitionOpenspaceRunningSOP(
  hfsm::Machine<openspace_state_machine::Context>::Control &control) {
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_moved = PlanningContext::Instance()
                        ->parking_behavior_planner_output()
                        .has_moved ||
                    abs(world_model_->get_ego_state().ego_vel) > 0.07;
  MSD_LOG(INFO, " has_moved is: [%d]", __FUNCTION__, PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_moved);
  if (world_model_->get_pause_status()) {
    planning_run_timer_.reset();
  }

  bool deviated = false;

  double MPC_ERR_THRE_IN_SLOT = 0.3;
  double MPC_ERR_THRE_OUT_SLOT = 0.3;
  if (PlanningContext::Instance()->planning_status().scenario.status_type ==
          StatusType::APA &&
      PlanningContext::Instance()->planning_status().planning_result.gear !=
          GearState::DRIVE) {
    if (world_model_->is_parking_svp()) {
      MPC_ERR_THRE_IN_SLOT = 0.15;
      MPC_ERR_THRE_OUT_SLOT = 0.25;
    } else {
      MPC_ERR_THRE_IN_SLOT = 0.3;
      MPC_ERR_THRE_OUT_SLOT = 0.3;
    }

    if (!world_model_->get_mpc_trajectory().empty()) {
      const auto &slot_box = PlanningContext::Instance()
                                 ->mutable_parking_behavior_planner_output()
                                 ->parking_lot->getBox();

      PathPoint mpc_terminal_pp = world_model_->get_mpc_trajectory().back();
      bool is_mpc_end_inside_slot =
          slot_box.IsPointIn(Vec2d(mpc_terminal_pp.x, mpc_terminal_pp.y));
      bool future_deviated_in_slot =
          !can_follow_plan_traj(MPC_ERR_THRE_IN_SLOT) && is_mpc_end_inside_slot;
      bool future_deviated_out_slot =
          !can_follow_plan_traj(MPC_ERR_THRE_OUT_SLOT) &&
          !is_mpc_end_inside_slot;
      if (future_deviated_in_slot) {
        MSD_LOG(WARN, "onTransitionOpenspaceRunning::future_deviated_in_slot!");
      }
      if (future_deviated_out_slot) {
        MSD_LOG(WARN,
                "onTransitionOpenspaceRunning::future_deviated_out_slot!");
      }

      deviated =
          deviated || future_deviated_in_slot || future_deviated_out_slot;
    }
  } else {
    deviated = deviated || !can_follow_plan_traj(MPC_ERR_THRE_OUT_SLOT);
  }

  deviated = deviated && !PlanningContext::Instance()
                              ->planning_status()
                              .planning_result.traj_pose_array.empty();
  deviated =
      deviated &&
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;

  // forbidden deviate check for control test mode
  if (world_model_->get_control_test_flag()) {
    deviated = false;
  }

  double traj_length =  PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .traj_length ;
  bool second_traj_not_replan = false;
  const auto& not_replan_scope =
      PlanningContext::Instance()->planning_status().
      planning_result.second_traj_not_replan_scope;

  if (std::fabs(not_replan_scope.first - not_replan_scope.second) > 0.01
      && traj_length > not_replan_scope.first
      && traj_length < not_replan_scope.second) {
      second_traj_not_replan = true;
  }
  // std::cout << "the not_replan_scope.first is:" << not_replan_scope.first
  //           << " not_replan_scope.second is: " << not_replan_scope.second
  //           << "traj length is:" << traj_length
  //           << " second_traj_not_replan"<< second_traj_not_replan << std::endl;
  MSD_LOG(ERROR, "the not_replan_scope.first is: %f, not_replan_scope.second is: %f, traj length is: %f, second_traj_not_replan %d, extend flag %d"
      ,not_replan_scope.first, not_replan_scope.second, traj_length, second_traj_not_replan,
      PlanningContext::Instance()
     ->planning_status().plan_control_interface.is_extended);

  if (deviated && !world_model_->get_pause_status() &&
     !(PlanningContext::Instance()
     ->planning_status().plan_control_interface.is_extended
     && (traj_length < 0.25 || second_traj_not_replan))) {
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->init_traj_point.v =
        PlanningContext::Instance()->planning_status().planning_result.gear ==
                GearState::REVERSE
            ? 1
            : -1;
    LOG_TO_CONTEXT("%s: replan for ego pose deviated from trajectory.",
                   __FUNCTION__);
    control.changeTo<openspace_state_machine::Standby>();
  }

  MSD_LOG(ERROR, "last segment is %d, tra length is:%f, is_static:%d", 
    path_sampler_->is_at_last_segment(), PlanningContext::Instance()
               ->longitudinal_behavior_planner_output()
               .traj_length, world_model_->get_ego_state().is_static);
  if (path_sampler_->is_at_last_segment() &&
       PlanningContext::Instance()
               ->longitudinal_behavior_planner_output()
               .traj_length < 0.2  
               && world_model_->get_ego_state().is_static) {
    MSD_LOG(ERROR, "last segment is finished");
    control.changeTo<openspace_state_machine::Finish>();
  } 
}

void OpenspaceStateMachine::onEntryOpenspaceRunning() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  PlanningContext::Instance()
      ->mutable_openspace_decider_output()
      ->is_active_replan = false;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_move_ready = true;
  // is_teb_ok_APOA_filter_ = FlagFilter(5);
  PlanningContext::Instance()->mutable_planning_status()->wlc_info.is_approached = false;
}

// TODO: only tracking when running
void OpenspaceStateMachine::onUpdateOpenspaceRunning() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->has_running_enough_long = planning_run_timer_.is_timeout();

  if(planning_run_timer_.is_timeout() && !world_model_->get_pause_status()) {
    PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_paused = false;
  }

  if(world_model_->get_pause_status()) {
    PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_paused = true;
  }

  // update ego state realtime
  auto output =
      PlanningContext::Instance()->mutable_parking_behavior_planner_output();
  TrajectoryPoint &init_traj_point = output->init_traj_point;

  EgoState ego_state = world_model_->get_ego_state();

  init_traj_point.path_point.x = ego_state.ego_pose.x;
  init_traj_point.path_point.y = ego_state.ego_pose.y;
  init_traj_point.path_point.theta = ego_state.ego_pose.theta;
  init_traj_point.steer =
      ego_state.ego_steer_angle / CarParams::GetInstance()->steer_ratio;

}

void OpenspaceStateMachine::onEntryOpenspaceFinish() {
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_move_ready = false;
}

void OpenspaceStateMachine::onTransitionOpenspaceFinish(
    hfsm::Machine<openspace_state_machine::Context>::Control &control) {
  if (PlanningContext::Instance()->planning_status().wlc_info.is_valid) {
    PlanningContext::Instance()->mutable_planning_status()->wlc_info.is_approached = true;
    PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
    LOG_TO_CONTEXT("%s: replan because of wireless charger offset", __FUNCTION__);
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->init_traj_point.v = 1;
    PlanningContext::Instance()->mutable_open_space_path()->stash(
      std::vector<TrajectoryPoint>());
    control.changeTo<openspace_state_machine::Standby>();
  }
}




bool OpenspaceStateMachine::can_follow_plan_traj(double dist_threshold,
                                                 bool use_average_diff) {
  double follow_error = PlanningContext::Instance()
                            ->longitudinal_behavior_planner_output()
                            .ego_lat_diff;
  if (std::isnan(follow_error)) {
    follow_error = dist_threshold + 0.1;
  }
  if (follow_error > dist_threshold) {
    MSD_LOG(INFO, "follow_error = %f > %f.", follow_error, dist_threshold);
  }
  return follow_error < dist_threshold;
}



} // namespace openspace_state_machine
} // namespace parking
} // namespace msquare