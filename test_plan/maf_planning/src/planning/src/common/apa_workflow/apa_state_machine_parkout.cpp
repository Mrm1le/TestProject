#include <cstdint>

#include "common/apa_workflow/apa_state_machine.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planning/common/common.h"
#ifdef USE_CONFIG_SERVICE
#include <mpf/core/parameter/parameter.h>
#endif
#include "common/apa_workflow/parking_config_loader.hpp"
#include "common/parking_slot_factory.h"
#include "common/context_logger.h"


namespace msquare {

namespace parking {


void ApaStateMachine::onEntryParkOut() {

//[fenix.refactor.sm] Original ParkingTaskManager::onEntryParkIn()
  MSD_LOG(INFO,"[onEntryParkOut] onEntryParkOut");
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.state_changed = true;
  current_state_ = StatusType::APOA;
  status_text = "APOA";
  current_task_++;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_slot_info.recommend_park_out_type.value = ParkOutType::INVALID;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_slot_info.available_park_out_type.value = ParkOutType::INVALID;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_slot_info.is_direction_limit = 0;
//[fenix.refactor.sm] END Original ParkingTaskManager::onEntryParkIn()

//[fenix.refactor.sm] Original ScenarioManager::onEntryApoa()
  
  auto output =
      PlanningContext::Instance()->mutable_parking_behavior_planner_output();
  output->planner_type = PlannerType::OPENSPACE;

  PlanningContext::Instance()->mutable_open_space_path()->stash(
      std::vector<TrajectoryPoint>());

  const auto &slot_type = PlanningContext::Instance()
                              ->parking_behavior_planner_output()
                              .parking_slot_info.type.value;

  (void)ParkingConfigLoader::load_config(StatusType::APOA, slot_type);

  if (slot_type == ParkingSlotType::OBLIQUE) {
    HybridAstarConfig::GetInstance()->planning_core = 11;
  }

  // get init and target of APOA
  
  TrajectoryPoint &init_traj_point = output->init_traj_point;
  TrajectoryPoint &target_traj_point = output->target_traj_point;
  APAMetaState &apa_meta_state = output->apa_meta_state;
  
  EgoState ego_state = world_model_->get_ego_state();
  init_traj_point.path_point.x = ego_state.ego_pose.x;
  init_traj_point.path_point.y = ego_state.ego_pose.y;
  init_traj_point.path_point.theta = ego_state.ego_pose.theta;
  init_traj_point.steer =
      ego_state.ego_steer_angle / CarParams::GetInstance()->steer_ratio;
  
  apa_meta_state.meta_pose_x = ego_state.ego_pose.x;
  apa_meta_state.meta_pose_y = ego_state.ego_pose.y;
  apa_meta_state.meta_pose_theta = ego_state.ego_pose.theta;
  apa_meta_state.last_v = 0.0;
  apa_meta_state.is_valid = true;

  const Pose2D &park_out_point = PlanningContext::Instance()
                              ->parking_behavior_planner_output()
                              .parking_slot_info.park_out_point.point;

  const Pose2D &error_tolerence =
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_slot_info.park_out_error_tolerence;

  target_traj_point.path_point.x = park_out_point.x;
  target_traj_point.path_point.y = park_out_point.y;
  target_traj_point.path_point.theta = park_out_point.theta;
  target_traj_point.sigma_x =  error_tolerence.x;
  target_traj_point.sigma_y =  error_tolerence.y;
  target_traj_point.sigma_yaw =  error_tolerence.theta;
  target_traj_point.a = 0.0;
  target_traj_point.steer = 0.0;
  target_traj_point.v = 0.0;


  const auto &parking_slot_info = PlanningContext::Instance()
                                      ->parking_behavior_planner_output()
                                      .parking_slot_info;
  const auto &parkout_type = parking_slot_info.park_out_type.value;
  if (parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT ||
      parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT) {
    init_traj_point.v = 1.0;
  } else if (parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR ||
             parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT ||
             parkout_type == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT) {
    init_traj_point.v = -1.0;
  } else {
    init_traj_point.v = 0.0;
  }

  if (CarParams::GetInstance()->car_config.apoa_config.use_legacy_parkout) {
    if (parkout_type == ParkOutType::PARALLEL_LEFT) {
      target_traj_point.steer = 1.0;
      apa_meta_state.parallel_direc = 1.0;
    } else if (parkout_type == ParkOutType::PARALLEL_RIGHT) {
      target_traj_point.steer = -1.0;
      apa_meta_state.parallel_direc = -1.0;
    }
  }

  std::map<std::uint8_t, std::string> parking_slot_cfg_map = {
      {ParkingSlotType::PERPENDICULAR,
       PlanningConfig::Instance()->config_files().parking_lot_config_file},
      {ParkingSlotType::OBLIQUE,
       PlanningConfig::Instance()->config_files().parking_lot_config_file},
      {ParkingSlotType::PARALLEL, PlanningConfig::Instance()
                                      ->config_files()
                                      .parallel_parking_slot_config_file}};
  ParkingSlotFactory parking_slot_factory(parking_slot_cfg_map);
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_lot = parking_slot_factory.create(
      parking_slot_info.type, parking_slot_info.corners,
      parking_slot_info.original_corners, park_out_point);
  // PlanningContext::Instance()->mutable_parking_behavior_planner_output()->parking_lot.reset(new
  // BaseParkingSlot(
  //   PlanningConfig::Instance()->config_files().parking_lot_config_file,
  //   PlanningContext::Instance()->mutable_parking_behavior_planner_output()->park_out_info.corners)
  // );
  double road_width = planning_math::interps(
      world_model_->get_refline_manager()->get_lane_width(),
      world_model_->get_refline_manager()->get_s(),
      world_model_->get_ego_state().ego_frenet.x +
          world_model_->get_distance_to_poi());
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_lot->matchRoadWidth(road_width);

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_lot->customizePlanningParams(CarParams::GetInstance(),
                                             HybridAstarConfig::GetInstance(),
                                             StrategyParams::GetInstance());

  openspace_state_machine_->createMachineParkOut();

  if (planning_math::tf2d(ego_state.ego_pose, park_out_point).y > 0) {
    PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
        maf_vehicle_status::TurnSignalType::LEFT;
  } else {
    PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
        maf_vehicle_status::TurnSignalType::RIGHT;
  }

  openspace_state_machine_->changeToStandby();
  PlanningContext::Instance()
      ->mutable_open_space_path()
      ->unstash(); // prevent path sampler from sampling

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
  PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
      false;
  is_teb_ok_APOA_filter_ = FlagFilter(5);

//[fenix.refactor.sm] END Original ScenarioManager::onEntryApoa()
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_ever_been_inside_slot = false;

// resetting outputs
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->reached_parkout_takeover_point = false;

}


void ApaStateMachine::onUpdateParkOut() {
//[fenix.refactor.sm] move to ApaStateMachine::onTransitionParkOut();
 
}


void ApaStateMachine::onTransitionParkOut(
    hfsm::Machine<Context>::Control &control) {

//[fenix.refactor.sm] Original ParkingTaskManager::onUpdateParkOut()
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  PlanningContext::Instance()->mutable_planning_status()->task_status.task =
      StatusType::APOA;
  PlanningContext::Instance()->mutable_planning_status()->task_status.status =
      TaskStatusType::RUNNING;
//[fenix.refactor.sm] END Original ParkingTaskManager::onUpdateParkOut()

//[fenix.refactor.sm] Original ParkingTaskManager::onTransitionParkOut()
  bool is_finish =
      PlanningContext::Instance()->parking_behavior_planner_output().is_finish;
  PlanningRequest planning_request = world_model_->get_planning_request();
  int plan_cnt = PlanningContext::Instance()->openspace_motion_planner_output().openspace_fallback_cnt;
  const int PLAN_FALLBACK_TH =
      CarParams::GetInstance()->car_config.apoa_config.use_legacy_parkout ? 10
                                                                          : 0;
  if (planning_request.cmd.value == ParkingCommand::STOP) {
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::SUCCEEDED;
    current_task_ = parking_task_list_.end();
    control.changeTo<Wait>();
    MSD_LOG(ERROR, "[%s] change to wait with stop command", __FUNCTION__);
  } else if (PlanningContext::Instance()->planning_status().zigzag_num > 13) {
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::FAILED;
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->task_status.failure_reason = FailureReason::ADJUEST_TOO_MANY;
    MSD_LOG(ERROR, "[%s] failed as ADJUEST_TOO_MANY", __FUNCTION__);
  } else if (plan_cnt > PLAN_FALLBACK_TH) {
    PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::PLANNING_FAILED;
    MSD_LOG(ERROR, "[%s] failed as plan too many", __FUNCTION__);
  } else if (PlanningContext::Instance()
                   ->openspace_motion_planner_output()
                   .status ==
               OpenspaceMotionPlannerOutput::PlannerStatus::START_INFEASIBLE || 
      PlanningContext::Instance()->openspace_motion_planner_output().status ==
        OpenspaceMotionPlannerOutput::PlannerStatus::END_INFEASIBLE) {
      PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::BARRIER_IN_PARKING_SLOT;
    MSD_LOG(ERROR, "[%s] failed as START_INFEASIBLE", __FUNCTION__);
  } else if (PlanningContext::Instance()
                 ->mutable_planning_status()
                 ->advanced_abandon) {
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::FAILED;
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->task_status.failure_reason = FailureReason::BARRIER_IN_PARKING_SLOT;
    MSD_LOG(ERROR, "[%s] failed as BARRIER_IN_PARKING_SLOT", __FUNCTION__);
  } else if ((current_task_ + 1) == parking_task_list_.end() ||
             (current_task_ + 1)->task_type == ParkingTaskType::WAIT) {
    if (is_finish && world_model_->get_ego_state().is_static) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::SUCCEEDED;
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = false;
      control.changeTo<Wait>();
      MSD_LOG(ERROR, "[%s] change to wait as is_finish", __FUNCTION__);
    }
  }
//[fenix.refactor.sm] END Original ParkingTaskManager::onTransitionParkOut()

//[fenix.refactor.sm] Original ScenarioManager::onUpdateApoa()
  openspace_state_machine_->update();

  bool openspace_is_running =
      openspace_state_machine_->getCurrentState() ==
      openspace_state_machine::OpenspaceStateEnum::RUNNING;
  bool openspace_is_fallback =
      openspace_state_machine_->getCurrentState() ==
      openspace_state_machine::OpenspaceStateEnum::FALLBACK;
  bool openspace_is_finish = openspace_state_machine_->getCurrentState() ==
                             openspace_state_machine::OpenspaceStateEnum::FINISH;

  PathPoint target_pp = PlanningContext::Instance()
                            ->openspace_decider_output()
                            .target_state.path_point;
  using planning_math::Vec2d;
  auto &ego_pose = world_model_->get_ego_state().ego_pose;
  auto &parking_slot_info = PlanningContext::Instance()
                                  ->parking_behavior_planner_output()
                                  .parking_slot_info;
  auto park_out_direc =parking_slot_info.park_out_type.value;


  Vec2d ego_pos(ego_pose.x, ego_pose.y);
  Vec2d target_pos(target_pp.x, target_pp.y);
  Vec2d ego_direction_vec(std::cos(ego_pose.theta), std::sin(ego_pose.theta));
  double lat_dist = abs(ego_direction_vec.CrossProd(target_pos - ego_pos));
  double lon_dist = ego_direction_vec.InnerProd(target_pos - ego_pos);
  double theta_diff =
      abs(planning_math::NormalizeAngle(target_pp.theta - ego_pose.theta));

  bool is_likely_to_grab_route =
      !PlanningContext::Instance()->planning_status().rerouting.need_rerouting
      && std::abs(world_model_->get_ego_state().ego_frenet.y) < 3.0 &&
      std::cos(theta_diff) > std::cos(0.5) &&
      std::cos(world_model_->get_frenet_coord()->GetRefCurveHeading(
                   world_model_->get_ego_state().ego_frenet.x) -
               world_model_->get_ego_state().ego_pose.theta) > std::cos(0.5);

  bool is_teb_ok = false;
  bool is_delta_f_ok = true;
  if (world_model_->is_parking_apa() || world_model_->is_parking_lvp()) {
    if (openspace_is_finish) {
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = true;
      MSD_LOG(ERROR, "[%s] finish as ego_frenet", __FUNCTION__);
      MSD_LOG(ERROR, "[%s] finish theta_diff: %f", __FUNCTION__, theta_diff);
      return;
    }
  } else {
    MSD_LOG(INFO, "%s: is_teb_ok_APOA_ = %d, %d, %d, %d \n", __FUNCTION__,
            is_teb_ok, is_teb_ok_APOA_filter_.get_flag(),
            is_likely_to_grab_route, is_delta_f_ok);
    if ((is_delta_f_ok && is_likely_to_grab_route &&
         is_teb_ok_APOA_filter_.get_flag()) ||
        openspace_is_finish) {
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = true;
      MSD_LOG(ERROR, "[%s] finish as is_likely_to_grab_route", __FUNCTION__); 
      return;   
    }
  }

  if (openspace_is_running) {
    if ((world_model_->get_collide_to_limiter_when_reverse())) {
      LOG_TO_CONTEXT("%s: abandon because of "
                     "get_collide_to_limiter_when_reverse",
                     __FUNCTION__);
      PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
          true;
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = true;
      MSD_LOG(ERROR, "[%s] advanced_abandon as get_collide_to_limiter_when_reverse", __FUNCTION__);    
      return;
    }
  }

  auto &apa_meta_state = PlanningContext::Instance()
                                   ->mutable_parking_behavior_planner_output()->apa_meta_state;
  auto &times_try_parking_in = PlanningContext::Instance()
                                   ->mutable_openspace_motion_planner_output()
                                   ->times_try_parking_in;
  if (!world_model_->get_pause_status() &&
      PlanningContext::Instance()->planning_status().blocked_timeout &&
      PlanningContext::Instance()->planning_status().blocked &&
      openspace_is_running) {
    MSD_LOG(ERROR, "[%s]blocked times_try_parking_in=%d, has_running_enough_long:%d", 
      __FUNCTION__, times_try_parking_in,PlanningContext::Instance()
              ->planning_status()
              .has_running_enough_long);    
    if (times_try_parking_in == 3) {
      LOG_TO_CONTEXT("%s: times_try_parking_in = %d.", __FUNCTION__,
                     times_try_parking_in);
      MSD_LOG(ERROR, "[%s] advanced_abandon as times_try_parking_in = 3", __FUNCTION__);    
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = true;
      PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
          true;
      times_try_parking_in = 0;
      return;
    }
    
    // set initial planning direction
    if ((PlanningContext::Instance()
            ->planning_status()
            .has_running_enough_long &&
        PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .has_moved) || 
        (PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .has_paused)) {
      times_try_parking_in += 1;
        apa_meta_state.last_v = 
            PlanningContext::Instance()
                        ->planning_status()
                        .planning_result.gear == GearState::REVERSE
                ? -1
                : 1;
      LOG_TO_CONTEXT("%s: replan for block, times_try_parking_in = %d.",
                      __FUNCTION__, times_try_parking_in);
      MSD_LOG(ERROR, "%s: replan for block, times_try_parking_in = %d. apa_meta_state.last_v = %f",
                      __FUNCTION__, times_try_parking_in, apa_meta_state.last_v);    
      openspace_state_machine_->changeToStandby();
      return;
    } else if (PlanningContext::Instance()
                  ->planning_status()
                  .has_running_enough_long) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->advanced_abandon = true;
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = true;
      LOG_TO_CONTEXT("%s: finish because of replan but not move.",
                      __FUNCTION__);
    }
  }

  if (openspace_is_fallback) {
    MSD_LOG(ERROR, "%s: transition from fallback to standby",
            __FUNCTION__);
    openspace_state_machine_->changeToStandby();
  }


#pragma region calculate_reached_parkout_takeover_point 

  const bool is_at_last_segment = path_sampler_->is_at_last_segment();
  bool reached_parkout_takeover_point =
      behavior_calculator_parkout_->calculate_reached_takeover_point(
          is_at_last_segment, openspace_is_running);

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->reached_parkout_takeover_point = reached_parkout_takeover_point;

#pragma endregion

//[fenix.refactor.sm] END Original ScenarioManager::onUpdateApoa()


//[fenix.refactor.sm] Original ScenarioManager::onTransitionApoa()

  //    transit state according to ApaStateMachine's statemachine status
  //    no need to re-write here

//[fenix.refactor.sm] END Original ScenarioManager::onTransitionApoa()


}


void ApaStateMachine::onLeaveParkOut() {
//[fenix.refactor.sm] Original ParkingTaskManager::onLeaveParkOut()
  MSD_LOG(INFO, "[%s] enter", __FUNCTION__);
  (void)parking_slot_manager_.ClearParkingSlotInfo();
  auto &last_parkin_data = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->last_parkin_data;
  last_parkin_data.is_loaded = false;
  last_parkin_data.is_valid = false;
//[fenix.refactor.sm] END Original ParkingTaskManager::onLeaveParkOut()

//[fenix.refactor.sm] Original ScenarioManager::onLeaveApoa()
  openspace_state_machine_->destroyMachine();
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->openspace_fallback_cnt = 0;
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->times_try_parking_in = 0;
  PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
      maf_vehicle_status::TurnSignalType::NONE;
  PlanningContext::Instance()->mutable_open_space_path()->stash(
      std::vector<TrajectoryPoint>());
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->apa_meta_state.is_valid = false;
  //[fenix.refactor.sm] END Original ScenarioManager::onLeaveApoa()

// resetting outputs
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->reached_parkout_takeover_point = false;

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->target_traj_point = TrajectoryPoint();
}



} // namespace parking

} // namespace msquare
