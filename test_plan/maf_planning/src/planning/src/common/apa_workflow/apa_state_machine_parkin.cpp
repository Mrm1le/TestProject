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


//header files for dynamic planning
#include "common/math/curve_join.h"
#include "common/utils/trajectory_point_utils.h"
#include "common/sbp_strategy.h"
#include "common/ego_model_manager.h"
#include "planning/common/statistic.h"


namespace msquare {

namespace parking {

using namespace planning_math;

void ApaStateMachine::onEntryParkIn() {
//[fenix.refactor.sm] Original ParkingTaskManager::onEntryParkIn()
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.state_changed = true;
  need_re_parkin_ = false;
  current_state_ = StatusType::APA;
  status_text = "APA";
  current_task_++;

  if(world_model_->is_parking_lvp()){
    CarParams::GetInstance()->car_config.parallel_config.is_min_r_priority = true;
  }

//[fenix.refactor.sm] END Original ParkingTaskManager::onEntryParkIn()

//[fenix.refactor.sm] Original ScenarioManager::onEntryApa()
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->times_tiny_slot_overlap = 0;


  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->planner_type = PlannerType::OPENSPACE;

  PlanningContext::Instance()->mutable_open_space_path()->stash(
      std::vector<TrajectoryPoint>());

  (void)ParkingConfigLoader::load_config(StatusType::APA,
                                         PlanningContext::Instance()
                                             ->parking_behavior_planner_output()
                                             .parking_slot_info.type.value);

  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.bag_recorder_filter_scenario.apa = true;
  // TODO: why shared pointer will cause compilation error?
  // std::shared_ptr<ParkingSlotInterface> parking_lot =
  // PlanningContext::Instance()->mutable_parking_behavior_planner_output()->parking_lot;
  std::map<std::uint8_t, std::string> parking_slot_cfg_map = {
      {ParkingSlotType::PERPENDICULAR,
       PlanningConfig::Instance()->config_files().parking_lot_config_file},
      {ParkingSlotType::OBLIQUE,
       PlanningConfig::Instance()->config_files().parking_lot_config_file},
      {ParkingSlotType::PARALLEL, PlanningConfig::Instance()
                                      ->config_files()
                                      .parallel_parking_slot_config_file}};
  ParkingSlotFactory parking_slot_factory(parking_slot_cfg_map);
  Pose2D aimed_poi_projection_on_route = world_model_->get_ego_state().ego_pose;
  Pose2D pose_rewrited = aimed_poi_projection_on_route;
  if (!world_model_->get_aimed_poi_projection_on_route(
          &pose_rewrited, 0)) {
    MSD_LOG(
        WARN,
        "get_aimed_poi_projection_on_route failed, use ego pose as default.");
  }
  MSD_LOG(ERROR, "pose_rewrited x = %.3f, y = %.3f theta = %.3f",pose_rewrited.x, pose_rewrited.y, pose_rewrited.theta);
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_lot = parking_slot_factory.create(
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_slot_info.type,
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_slot_info.corners,
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_slot_info.original_corners,
      aimed_poi_projection_on_route);
  // PlanningContext::Instance()->mutable_parking_behavior_planner_output()->parking_lot.reset(new
  // BaseParkingSlot(
  //   PlanningConfig::Instance()->config_files().parking_lot_config_file,
  //   PlanningContext::Instance()->mutable_parking_behavior_planner_output()->park_in_info.corners)
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

  openspace_state_machine_-> createMachineParkIn();

  // get init and target of APA
  auto output =
      PlanningContext::Instance()->mutable_parking_behavior_planner_output();
  TrajectoryPoint &init_traj_point = output->init_traj_point;
  TrajectoryPoint &target_traj_point = output->target_traj_point;

  EgoState ego_state = world_model_->get_ego_state();
  init_traj_point.path_point.x = ego_state.ego_pose.x;
  init_traj_point.path_point.y = ego_state.ego_pose.y;
  init_traj_point.path_point.theta = ego_state.ego_pose.theta;
  init_traj_point.steer =
      ego_state.ego_steer_angle / CarParams::GetInstance()->steer_ratio;
  init_traj_point.v = -0.0; // TODO: get from config file
  init_traj_point.a = 0.0;

  // get target configure of APA
  Pose2D target_pose =
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_lot->getParkingInPose(
              VehicleParam::Instance()->front_edge_to_center,
              VehicleParam::Instance()->back_edge_to_center,
              VehicleParam::Instance()->brake_distance_buffer,
              world_model_->get_ego_state().ego_pose,
              PlanningContext::Instance()
                  ->parking_behavior_planner_output()
                  .parking_slot_info.wheel_stop_info.wheel_stop_depth);
  target_traj_point.path_point.x = target_pose.x;
  target_traj_point.path_point.y = target_pose.y;
  target_traj_point.path_point.theta = target_pose.theta;

  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  // TODO: move to load_config(StatusType, ParkingSlotType);
  auto &corners = parking_slot_info.corners;
  Vec2d front_direction = planning_math::Vec2d{corners[0].x - corners[1].x,
                                               corners[0].y - corners[1].y};
  Vec2d ego_heading_vec = Vec2d::CreateUnitVec2d(ego_state.ego_pose.theta);
  double cos_relative_heading = ego_heading_vec.InnerProd(front_direction) /
                                std::max(front_direction.Length(), 1e-6);
  if (std::abs(cos_relative_heading) < std::cos(M_PI / 6) &&
      std::abs(cos_relative_heading) > std::cos(M_PI * 2 / 5) &&
      PlanningContext::Instance()
              ->parking_behavior_planner_output()
              .parking_slot_info.type.value != ParkingSlotType::PARALLEL) {
    if (CarParams::GetInstance()
            ->car_config.common_config.use_sop_openspace_planner) {
      HybridAstarConfig::GetInstance()->planning_core = 8;
    } else {
      HybridAstarConfig::GetInstance()->planning_core = 6;
    }
  }
  target_traj_point.v = (world_model_->is_parking_lvp()) ? -2 : -0;

  if (PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_lot->isOnLeft()) {
    PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
        maf_vehicle_status::TurnSignalType::LEFT;
  } else {
    PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
        maf_vehicle_status::TurnSignalType::RIGHT;
  }

  openspace_state_machine_->changeToStandby();

  PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
      false;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
  PlanningContext::Instance()
      ->mutable_openspace_decider_output()
      ->is_active_replan = false;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_narrow_channel = false;
  CarParams::GetInstance()->resetInflationAdjustFlag();
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_ever_been_inside_slot = false;

//[fenix.refactor.sm] END Original ScenarioManager::onEntryApa()



}



void ApaStateMachine::onUpdateParkIn() {

}





void ApaStateMachine::onTransitionParkIn(
    hfsm::Machine<Context>::Control &control) {

//[fenix.refactor.sm] Original ParkingTaskManager::onUpdateParkIn()
  PlanningContext::Instance()->mutable_planning_status()->task_status.task =
      StatusType::APA;
  PlanningContext::Instance()->mutable_planning_status()->task_status.status =
      TaskStatusType::RUNNING;

  bool is_using_sop =
      (PlanningContext::Instance()
           ->parking_behavior_planner_output()
           .parking_slot_info.type.value == ParkingSlotType::PARALLEL)
          ? (CarParams::GetInstance()
                 ->car_config.common_config.use_sop_openspace_planner_parallel)
          : (CarParams::GetInstance()
                 ->car_config.common_config.use_sop_openspace_planner);

  int openspace_fallback_th = is_using_sop ? 0 : 1;


  need_re_parkin_ =
      need_re_parkin_ || PlanningContext::Instance()
                                 ->openspace_motion_planner_output()
                                 .openspace_fallback_cnt > openspace_fallback_th;

#pragma region onTransitionParkIn_update_parking_slot_info
  if (world_model_->is_parking_lvp() || world_model_->is_parking_apa()) {
    (void)parking_slot_manager_.UpdateParkingSlotInfo(current_task_->poi_id);
    bool collision = /* PlanningContext::Instance()
                         ->planning_status()
                         .collide_to_sth || */
        world_model_->get_collide_to_limiter_when_reverse();
    collision = collision && PlanningContext::Instance()
                                 ->mutable_parking_behavior_planner_output()
                                 ->approaching_wheel_stop;
    (void)parking_slot_manager_.UpdateWheelStopPoint(collision);
  }
#pragma endregion
//[fenix.refactor.sm] END Original ParkingTaskManager::onUpdateParkIn()




//[fenix.refactor.sm] Original ParkingTaskManager::onTransitionParkIn
#pragma region onTransitionParkIn_finish_procedure
  bool is_finish =
      PlanningContext::Instance()->parking_behavior_planner_output().is_finish &&
      !PlanningContext::Instance()->planning_status().wlc_info.is_valid;
  MSD_LOG(ERROR, "%s: wlc_info: %d, %d, %f, %f", __FUNCTION__,
  PlanningContext::Instance()->planning_status().wlc_info.x_offset_valid,
  PlanningContext::Instance()->planning_status().wlc_info.y_offset_valid,
  PlanningContext::Instance()->planning_status().wlc_info.x_offset,
  PlanningContext::Instance()->planning_status().wlc_info.y_offset);
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      ", bp_finish" + std::to_string(PlanningContext::Instance()
                                         ->parking_behavior_planner_output()
                                         .is_finish);
  
  PlanningRequest planning_request = world_model_->get_planning_request();
  if (planning_request.cmd.value == ParkingCommand::STOP) {
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::SUCCEEDED;
    current_task_ = parking_task_list_.end();
    control.changeTo<Wait>();
  } else if (!world_model_->get_pause_status()) {
    if (need_re_parkin_) {
      // TODO: disable when use config
      unavailable_parking_lot_list_.emplace_back(current_task_->poi_id);
     
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->planning_result.bag_recorder_filter_scenario.reparkin = true;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      // for park in, we reverse the start & end
      if (PlanningContext::Instance()
              ->openspace_motion_planner_output()
              .status ==
          OpenspaceMotionPlannerOutput::PlannerStatus::END_INFEASIBLE) {
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.failure_reason = FailureReason::START_INFEASIBLE;
      } else if (PlanningContext::Instance()
                     ->openspace_motion_planner_output()
                     .status == OpenspaceMotionPlannerOutput::PlannerStatus::
                                    START_INFEASIBLE) {
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.failure_reason =
            FailureReason::BARRIER_IN_PARKING_SLOT;
      } else {
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.failure_reason = FailureReason::PLANNING_FAILED;
      }
    } else if (PlanningContext::Instance()->planning_status().zigzag_num >
               CarParams::GetInstance()
                   ->car_config.common_config.zigzag_limit) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::ADJUEST_TOO_MANY;
    } else if (PlanningContext::Instance()
                   ->mutable_planning_status()
                   ->advanced_abandon) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::BARRIER_IN_PARKING_SLOT;
    } else if ((current_task_ + 1) == parking_task_list_.end() ||
               (current_task_ + 1)->task_type == ParkingTaskType::WAIT) {
      if (world_model_->get_ego_state().is_static && is_finish) {
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.status = TaskStatusType::SUCCEEDED;
        unavailable_parking_lot_list_.clear();
        PlanningContext::Instance()
            ->mutable_parking_behavior_planner_output()
            ->is_finish = false;
        control.changeTo<Wait>();
      }
    } else {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::UNKNOW_FAILED;
      
    }
  }
#pragma endregion
//[fenix.refactor.sm] END Original ParkingTaskManager::onTransitionParkIn


//[fenix.refactor.sm] Original ScenarioManager::onUpdateApa()
  openspace_state_machine_->update();

#pragma region onTransitionParkIn_precalculate_results_for_decider 

  //all results are const during onTransitionParkIn cycle
  const bool openspace_is_running =
      openspace_state_machine_->getCurrentState() ==
      openspace_state_machine::OpenspaceStateEnum::RUNNING;
  const bool openspace_is_fallback =
      openspace_state_machine_->getCurrentState() ==
      openspace_state_machine::OpenspaceStateEnum::FALLBACK;
  const bool openspace_is_finish = openspace_state_machine_->getCurrentState() ==
                             openspace_state_machine::OpenspaceStateEnum::FINISH;

  const Pose2D &ego_pose = world_model_->get_ego_state().ego_pose;

  const Pose2D target_pose =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_lot->getParkingInPose(
              VehicleParam::Instance()->front_edge_to_center,
              VehicleParam::Instance()->back_edge_to_center,
              VehicleParam::Instance()->brake_distance_buffer,
              world_model_->get_ego_state().ego_pose,
              PlanningContext::Instance()
                  ->parking_behavior_planner_output()
                  .parking_slot_info.wheel_stop_info.wheel_stop_depth);

  const bool vehicle_reached_no_wheelstop =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_lot->isVehicleReached(VehicleParam::Instance(), ego_pose,
                                          box_model_);

  const bool vehicle_reached_with_wheelstop =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_lot->isVehicleReached(
              VehicleParam::Instance(), ego_pose, box_model_,
              PlanningContext::Instance()
                  ->parking_behavior_planner_output()
                  .parking_slot_info.wheel_stop_info.wheel_stop_depth);

  const double distance_to_end =
      std::hypot(target_pose.x - ego_pose.x, target_pose.y - ego_pose.y);

  const bool is_at_last_segment = path_sampler_->is_at_last_segment();

#pragma endregion


#pragma region onTransitionParkIn_output_target_pathpoint

  PathPoint &target_pp = PlanningContext::Instance()
                             ->mutable_parking_behavior_planner_output()
                             ->target_traj_point.path_point;
  target_pp.x = target_pose.x;
  target_pp.y = target_pose.y;
  target_pp.theta = target_pose.theta;

#pragma endregion

#pragma region onTransitionParkIn_calculate_and_output_approaching_wheelstop

  bool approaching_wheel_stop =
      behavior_calculator_parkin_->calculate_approaching_wheel_stop(
          distance_to_end, openspace_is_running);
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->approaching_wheel_stop = approaching_wheel_stop;

#pragma endregion

#pragma region onTransitionParkIn_strategy_when_reverse_collide_to_sth
  APABehaviorDeciderParkIn::DeciderResult decider_result;
  decider_result = behavior_decider_parkin_->strategy_hit_wheelstop(
      openspace_is_running, vehicle_reached_no_wheelstop);

  if (decider_result.strategy == APAStrategyType::FINISH) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;
  } else if (decider_result.strategy == APAStrategyType::REPLAN) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->init_traj_point.v = 1;
    openspace_state_machine_->changeToStandby();
  } else {
    // do nothing
  }
#pragma endregion

#pragma region onTransitionParkIn_finish_strategy_when_wheelstop_in_parallel
  decider_result = behavior_decider_parkin_->strategy_wheelstop_in_parallel_slot(
      openspace_is_running);

  if (decider_result.strategy == APAStrategyType::ABANDON) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
          true;
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;
  } else {
    // do nothing
  }
#pragma endregion

#pragma region onTransitionParkIn_finish_strategy_when_blocked_and_reached
  decider_result = behavior_decider_parkin_->strategy_blocked_and_reached(
      vehicle_reached_with_wheelstop);

  if (decider_result.strategy == APAStrategyType::FINISH) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;
  } else if (decider_result.strategy == APAStrategyType::WAIT_FOR_WLC) {
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->wlc_info.is_approached = true;
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = false;
  }
#pragma endregion

#pragma region onTransitionParkIn_strategy_slot_length_type_3
  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;

  if (!CarParams::GetInstance()->isInflationAdjusted()) {
    if (parking_slot_info.slot_length_type == 3 &&
        !CarParams::GetInstance()
             ->car_config.common_config.use_sop_openspace_planner_parallel) {
      double cur_lon_expand = CarParams::GetInstance()->lon_inflation();
      double deta_lon_expand = CarParams::GetInstance()->car_config.expand_param_config.apa_lon_adjust;
      double new_lon_expand = cur_lon_expand + deta_lon_expand;
      CarParams::GetInstance()->setLonInflationForce(new_lon_expand, deta_lon_expand);
      MSD_LOG(ERROR, "[%s] update apa lon_inflation: %f", __FUNCTION__, new_lon_expand);
      
      double cur_lat_expand = CarParams::GetInstance()->lat_inflation();
      double deta_lat_expand = CarParams::GetInstance()->car_config.expand_param_config.apa_lat_adjust;
      double new_lat_expand = cur_lat_expand + deta_lat_expand;
      CarParams::GetInstance()->setLatInflationForce(new_lat_expand, deta_lat_expand);
      MSD_LOG(ERROR, "[%s] update apa lat_inflation: %f", __FUNCTION__, new_lat_expand);
    }
  }
#pragma endregion

#pragma region onTransitionParkIn_strategy_when_tiny_perpendicular_slot
  // abandon when slot width is tiny
  decider_result = behavior_decider_parkin_->strategy_tiny_perpendicular_and_oblique_slot(
      is_at_last_segment);
  if (decider_result.strategy == APAStrategyType::ABANDON) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
        true;
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;
  }

#pragma endregion


  PlanningContext::Instance()->mutable_planning_status()->wlc_info.is_approaching =
    PlanningContext::Instance()->mutable_planning_status()->wlc_info.is_valid;

//[fenix.refactor.sm] END Original ScenarioManager::onUpdateApa()

//[fenix.refactor.sm] Original ScenarioManager::onTransitionApa()

  //    transit state according to ApaStateMachine's statemachine status
  //    no need to re-write here

//[fenix.refactor.sm] END Original ScenarioManager::onTransitionApa()

//[fenix.refactor.sm] Original ScenarioManager::onUpdateApaSimpleScene()

  // it not on svp mode for sure, see onTransitionApaSimpleScene
  static bool is_dynamic_planning_activated = false;
  static std::vector<TrajectoryPoint> curve_traj;

  Pose2D target_pose_lot =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_lot->getParkingInPose(
              VehicleParam::Instance()->front_edge_to_center,
              VehicleParam::Instance()->back_edge_to_center,
              VehicleParam::Instance()->brake_distance_buffer,
              world_model_->get_ego_state().ego_pose,
              PlanningContext::Instance()
                  ->parking_behavior_planner_output()
                  .parking_slot_info.wheel_stop_info.wheel_stop_depth);

#pragma region onTransitionParkIn_update_parking_lot
  //
  // 1. onTransitionParkIn_update_parking_lot
  bool need_update_slot_corners =
      behavior_calculator_parkin_->calculate_need_update_parking_slot_corners(
          target_pose_lot);

  if (need_update_slot_corners) {
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->parking_lot->updateCorners(
            PlanningContext::Instance()
                ->mutable_parking_behavior_planner_output()
                ->parking_slot_info.corners);
  }
#pragma endregion

  MSD_LOG(
      ERROR, "[wlc.if@%s,%d] PlanningContext::Instance()\
            ->parking_behavior_planner_output()\
            .has_moved(%d) AND\
        path_sampler_->is_at_last_segment()(%d)",
      __FUNCTION__, __LINE__,
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved,
      path_sampler_->is_at_last_segment());

#pragma region onTransitionParkin_strategy_dynamic_plan_adjust_tail
  // [fenix] TODO: re-write this block
  // 2. else set is_dynamic_planning_activated_ as false
  // 3. onTransitionParkin_strategy_dynamic_plan_adjust_tail
  bool need_check_mpc_collide =
      behavior_calculator_parkin_->calculate_need_check_mpc_collide(
          need_update_slot_corners, is_at_last_segment);
  bool need_dynamic_plan_adjust_tail =
      behavior_calculator_parkin_->calculate_need_dynamic_plan_adjust_tail(
          need_check_mpc_collide);
  decider_result = behavior_decider_parkin_->strategy_dynamic_plan_adjust_tail(
      need_dynamic_plan_adjust_tail);
  
  double max_dis_for_adjust_target = msquare::CarParams::GetInstance()->car_config.lon_config.max_dis_for_adjust_target;
  double dis_to_target = std::hypot(ego_pose.x - target_pose_lot.x, ego_pose.y - target_pose_lot.y);
  bool adjust_target_allowed = dis_to_target > max_dis_for_adjust_target;
  if (decider_result.strategy == APAStrategyType::DYNAMIC_PLAN_ADJUST_TAIL && adjust_target_allowed) {
    dynamic_planning_adjust_trajectory_tail();
  }
#pragma endregion

#pragma region onTransitionParkin_strategy_mpc_collide
  need_check_mpc_collide =
      behavior_calculator_parkin_->calculate_need_check_mpc_collide(
          need_update_slot_corners, is_at_last_segment);
  decider_result = behavior_decider_parkin_->strategy_mpc_collide(need_check_mpc_collide);

  if (decider_result.strategy == APAStrategyType::REPLAN) {
    auto &init_traj_point = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->init_traj_point.v = 1.0;
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    openspace_state_machine_->changeToStandby();
  }
#pragma endregion


#pragma region DYNAMIC_PLAN_SIMPLE_MPC
  decider_result = behavior_decider_parkin_->strategy_dynamic_plan_simple_mpc(
      distance_to_end, is_at_last_segment, openspace_is_running);

  bool dynamic_plan_simple_mpc_enabled = false;
  bool dynamic_plan_simple_mpc_traj_success = false;
  bool dynamic_plan_simple_mpc_traj_collide = false;

  if (decider_result.strategy == APAStrategyType::DYNAMIC_PLAN_SIMPLE_MPC) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    // do dynamic planning
    dynamic_plan_simple_mpc_enabled = true;
    DynamicPlanSimpleMpcResult simple_mpc_result =
        dynamic_planning_simple_mpc(ego_pose, target_pose_lot, curve_traj);

    dynamic_plan_simple_mpc_traj_success = simple_mpc_result.traj_success;
    dynamic_plan_simple_mpc_traj_collide = simple_mpc_result.traj_collide;

    // collide = curvejoin success && traj collide
  }

  decider_result =
      behavior_decider_parkin_->strategy_dynamic_plan_simple_mpc_collide_timeout(
          dynamic_plan_simple_mpc_enabled, dynamic_plan_simple_mpc_traj_success,
          dynamic_plan_simple_mpc_traj_collide);

  if (decider_result.strategy == APAStrategyType::REPLAN) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    auto &init_traj_point = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->init_traj_point.v = 1.0;
    curve_traj.clear();
    openspace_state_machine_->changeToStandby();
  }
#pragma endregion


#pragma region onTransitionParkIn_replan_strategy_when_openspace_fallback
  decider_result = behavior_decider_parkin_->strategy_openspace_fallback(
      openspace_is_fallback);
  if (decider_result.strategy == APAStrategyType::REPLAN) {
    curve_traj.clear();
    openspace_state_machine_->changeToStandby();
  }
#pragma endregion

#pragma region onTransitionParkIn_calculate_behaviors_relating_block
//calculate times_try_park_in
  bool blocked_base_scene =
      behavior_calculator_parkin_->calculate_blocked_base_scene(
          openspace_is_running);
  (void)behavior_calculator_parkin_->calculate_times_try_parkin(
      blocked_base_scene, is_at_last_segment,
      &PlanningContext::Instance()
           ->mutable_openspace_motion_planner_output()
           ->times_try_parking_in);

// calculate times_try_parking_in_about_hit_sth
  bool hit_base_scene =
      behavior_calculator_parkin_->calculate_hit_sth_base_scene(
          openspace_is_running);
  (void)behavior_calculator_parkin_->calculate_times_try_parkin(
      hit_base_scene, is_at_last_segment,
      &PlanningContext::Instance()
           ->mutable_openspace_motion_planner_output()
           ->times_try_parking_in_about_hit_sth);

#pragma endregion

#pragma region onTransitionParkIn_replan_or_finish_strategy_when_blocked

  bool is_blocked_by_obstacle_behind_in_slot =
      behavior_calculator_parkin_->calculate_blocked_by_obstacle_behind_in_slot(
          blocked_base_scene, target_pose_lot);

  PlanningContext::Instance()
      ->mutable_longitudinal_behavior_planner_output()
      ->is_blocked_by_obstacle_behind_in_slot =
      is_blocked_by_obstacle_behind_in_slot;

  decider_result =
      behavior_decider_parkin_->strategy_blocked(blocked_base_scene);
  if (decider_result.strategy == APAStrategyType::FINISH) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;

    //[fenix.refactor]  this is a unclean-approach of modifying
    //times_try_parking_in
    // only keep it because we want to keep features
    // TODO: remove it
    PlanningContext::Instance()
        ->mutable_openspace_motion_planner_output()
        ->times_try_parking_in = 0;
    PlanningContext::Instance()
        ->mutable_openspace_motion_planner_output()
        ->times_try_parking_in_about_hit_sth = 0;

  } else if (decider_result.strategy == APAStrategyType::ABANDON) {

    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;
    PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
        true;

    //[fenix.refactor] this is a unclean-approach of modifying
    //times_try_parking_in
    // only keep it because we want to keep features
    // TODO: remove it
    PlanningContext::Instance()
        ->mutable_openspace_motion_planner_output()
        ->times_try_parking_in = 0;
    PlanningContext::Instance()
        ->mutable_openspace_motion_planner_output()
        ->times_try_parking_in_about_hit_sth = 0;
  } else if (decider_result.strategy == APAStrategyType::REPLAN) {

    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());

    const auto &g_planning_gear =
        PlanningContext::Instance()->planning_status().planning_result.gear;

    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->init_traj_point.v = g_planning_gear == GearState::REVERSE ? 1 : -1;

    curve_traj.clear();
    openspace_state_machine_->changeToStandby();
  }
#pragma endregion

#pragma region onTransitionParkIn_strategy_when_traj_following_finish
  decider_result = behavior_decider_parkin_->strategy_traj_following_finish(
      openspace_is_finish, vehicle_reached_with_wheelstop);

  if (decider_result.strategy == APAStrategyType::FINISH) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;
  } else if (decider_result.strategy == APAStrategyType::REPLAN) {
    LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
    StrategyParams::GetInstance()->default_check_endsegent_len_ = 0;
    HybridAstarConfig::GetInstance()->max_zigzag_allowd = 2;
    HybridAstarConfig::GetInstance()->holonomic_with_obs_heuristic = 0;
    HybridAstarConfig::GetInstance()->non_holonomic_without_obs_heuristic = 1;
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->init_traj_point.v =
        PlanningContext::Instance()->planning_status().planning_result.gear ==
                GearState::REVERSE
            ? 1
            : -1;
    if (PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .parking_slot_info.type.value == ParkingSlotType::PARALLEL) {
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->init_traj_point.v = 0;
    }
    curve_traj.clear();
    openspace_state_machine_->changeToStandby();
  } else {
    // do nothing
  }
#pragma endregion

#pragma region onTransitionParkIn_active_replan_strategy_when_first_reverse_gear_switch_nonparallel
  // check if active replan for gear switch
  if (!msquare::CarParams::GetInstance()->car_config.lon_config.use_sop_algorithm) {
    if (PlanningContext::Instance()
            ->planning_status()
            .planning_result.gear_changing &&
        PlanningContext::Instance()->planning_status().zigzag_num == 1 &&
        !path_sampler_->is_at_last_segment() &&
        PlanningContext::Instance()
                ->parking_behavior_planner_output()
                .parking_slot_info.type.value != ParkingSlotType::PARALLEL &&
        !isEgoParallelInChannel(ego_pose, target_pose_lot) &&
        !world_model_->get_pause_status() && openspace_is_running) {
      PlanningContext::Instance()
          ->mutable_openspace_decider_output()
          ->is_active_replan = true;
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->init_traj_point.v =
          PlanningContext::Instance()->planning_status().planning_result.gear ==
                  GearState::REVERSE
              ? 1
              : -1;
      LOG_TO_CONTEXT("%s: active replan, init_v = %f", __FUNCTION__,
                    PlanningContext::Instance()
                        ->mutable_parking_behavior_planner_output()
                        ->init_traj_point.v);
      curve_traj.clear();
      openspace_state_machine_->changeToStandby();
    }
  }
#pragma endregion

#pragma region onTransitionParkIn_active_replan_strategy_when_assumed_blocked

  if (!msquare::CarParams::GetInstance()
           ->car_config.lon_config.use_sop_algorithm &&
      !is_using_sop) {
    decider_result = behavior_decider_parkin_->strategy_assumed_blocked();

    if (decider_result.strategy == APAStrategyType::REPLAN) {
      LOG_TO_CONTEXT((*PlanningContext::Instance()->mutable_now_time_seq_str() +
                    decider_result.reason)
                       .c_str());
      PlanningContext::Instance()
          ->mutable_openspace_decider_output()
          ->is_active_replan = true;
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->init_traj_point.v = -1;
      curve_traj.clear();
      openspace_state_machine_->changeToStandby();
    } else {
      // do nothing
    }    
  }
#pragma endregion

  // Jinwei: update parking_lot if ready for replan
  // cannot pass openspace_state_machine_ into
  // calculate_need_update_parking_slot_corners cause it's unique ptr
  if (openspace_state_machine_->getCurrentState() ==
      openspace_state_machine::OpenspaceStateEnum::STANDBY) {
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->parking_lot->updateCorners(
            PlanningContext::Instance()
                ->mutable_parking_behavior_planner_output()
                ->parking_slot_info.corners);
  }

#pragma region onTransitionParkIn_planning_strategy_during_pause
  const auto &longitudinal_output =
      PlanningContext::Instance()->longitudinal_behavior_planner_output();
  const auto &g_has_moved =
      PlanningContext::Instance()->parking_behavior_planner_output().has_moved;

  std::string debug_string;

  decider_result = behavior_decider_parkin_->strategy_during_pause(
      world_model_->get_pause_status(), longitudinal_output.traj_length,
      path_sampler_->is_at_last_segment(), vehicle_reached_with_wheelstop, g_has_moved, 
      longitudinal_output.is_need_pause_,
      longitudinal_output.remain_dist_info_, &debug_string);

  if (decider_result.strategy == APAStrategyType::REPLAN) {
    LOG_TO_CONTEXT(decider_result.reason.c_str());
    PlanningContext::Instance()
        ->mutable_openspace_decider_output()
        ->is_active_replan = true;
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->init_traj_point.v =
        PlanningContext::Instance()->planning_status().planning_result.gear ==
                GearState::REVERSE
            ? 1
            : -1;
    curve_traj.clear();
    openspace_state_machine_->changeToStandby();
  } else {
    // do nothing
  }

  *PlanningContext::Instance()->mutable_planning_debug_info() += debug_string;

#pragma endregion

#pragma region get const decider input for debug info
  auto print_debug_string = [&]() {
    std::string debug_string;
    debug_string =
        "\n[sm]const (rnw" + std::to_string(vehicle_reached_no_wheelstop) +
        ",rww" + std::to_string(vehicle_reached_with_wheelstop) + ",aws" +
        std::to_string(approaching_wheel_stop) + ",b_to" +
        std::to_string(
            PlanningContext::Instance()->planning_status().blocked_timeout);
    return debug_string;
  };
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      print_debug_string();
# pragma endregion

//[fenix.refactor.sm] END Original ScenarioManager::onUpdateApaSimpleScene()


//[fenix.refactor.sm] Original ScenarioManager::onTransitionApaSimpleScene()

  //NULL

//[fenix.refactor.sm] END Original ScenarioManager::onTransitionApaSimpleScene()

}


void ApaStateMachine::onLeaveParkIn() {
//[fenix.refactor.sm] Original ParkingTaskManager::onLeaveParkIn()

  if (msquare::CarParams::GetInstance()->car_config.apoa_config.use_last_parkin_data) {
    saveParkinData();
  }

//[fenix.refactor.sm] END Original ParkingTaskManager::onLeaveParkIn()

//[fenix.refactor.sm] Original ScenarioManager::onLeaveApa()
  PlanningContext::Instance()->mutable_open_space_path()->stash(
      std::vector<TrajectoryPoint>());

  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.bag_recorder_filter_scenario.apa = false;
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->openspace_fallback_cnt = 0;
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->times_try_parking_in = 0;
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->times_try_parking_in_about_hit_sth = 0;
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->times_tiny_slot_overlap = 0;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_lot.reset();
  PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
      maf_vehicle_status::TurnSignalType::NONE;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->approaching_wheel_stop = false;
  
  PlanningContext::Instance()->mutable_planning_status()->wlc_info.is_approaching = false;

  STAT_DISPLAY();

//[fenix.refactor.sm] END Original ScenarioManager::onLeaveApa()

}

bool ApaStateMachine::dynamic_planning_adjust_trajectory_tail() {
  const auto &ego_pose = world_model_->get_ego_state().ego_pose;
  auto tmp_curve_traj =
      PlanningContext::Instance()->mutable_open_space_path()->trajPoints();
  std::vector<TrajectoryPoint> last_back_seg;
  extract_last_segment(tmp_curve_traj, last_back_seg);

  if (tmp_curve_traj.empty()) {
    // unexpected, but may continue
    return true;
  }

  planning_math::Vec2d stopper_pt1(
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_slot_info.wheel_stop_info.vision_point1.x,
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_slot_info.wheel_stop_info.vision_point1.y);
  planning_math::Vec2d stopper_pt2(
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_slot_info.wheel_stop_info.vision_point2.x,
      PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_slot_info.wheel_stop_info.vision_point2.y);
  double wheel_stop_buffer =
      CarParams::GetInstance()
          ->car_config.target_pose_config.rear_axis_to_stopper;
  MSD_LOG(ERROR, "%s: wheel stop buffer is %d\n", __FUNCTION__,
          wheel_stop_buffer);
  auto &wireless_charger_report_data =
      world_model_->get_wireless_charger_report_data();
  bool is_wlc_signal_of_lon_valid =
      wireless_charger_report_data.link_state.value == 3;
  MSD_LOG(ERROR, "%s: link_state.value = %d\n", __FUNCTION__,
          wireless_charger_report_data.link_state.value);
  wheel_stop_buffer = is_wlc_signal_of_lon_valid ? (wheel_stop_buffer - 0.1)
                                                 : wheel_stop_buffer;
  if (msquare::CarParams::GetInstance()->car_config.lon_config.use_sop_algorithm) {
    wheel_stop_buffer = wheel_stop_buffer + 0.05;
  }
  lengthen_or_shorten(last_back_seg, stopper_pt1, stopper_pt2,
                      wheel_stop_buffer);

  regulateVelocity(StrategyParams::GetInstance(), last_back_seg);
  apply_trajpoints_curvature(last_back_seg);
  PlanningContext::Instance()->mutable_open_space_path()->stash(last_back_seg);
  PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->update_wheel_stop = true;

  return true;
}

bool ApaStateMachine::checkTwoSides(bool &is_left_car, bool &is_right_car) {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->parking_behavior_planner_output()
                                .parking_slot_info;
  auto &corners = parking_slot_info.corners;
  if (corners.size() != 4) {
    return false;
  }
  double fx = 0.5 * (corners[0].x + corners[3].x);
  double fy = 0.5 * (corners[0].y + corners[3].y);
  double bx = 0.5 * (corners[1].x + corners[2].x);
  double by = 0.5 * (corners[1].y + corners[2].y);
  double cx = 0.5 * (fx + bx);
  double cy = 0.5 * (fy + by);
  double ctheta = atan2(fy - by, fx - bx);
  double lot_length = std::hypot(fy - by, fx - bx);
  double lot_width =
      std::hypot(corners[0].x - corners[3].x, corners[0].y - corners[3].y);

  planning_math::Box2d slot_box = planning_math::Box2d(
      planning_math::Vec2d(cx, cy), ctheta, 0.5 * lot_length, 0.5 * lot_width);
  planning_math::Box2d slot_left(slot_box);
  planning_math::Box2d slot_right(slot_box);

  planning_math::Vec2d left_vector(corners[0].x - corners[3].x,
                                   corners[0].y - corners[3].y);
  slot_left.Shift(left_vector);
  slot_right.Shift(-1.0 * left_vector);

  for (auto &obj : world_model_->obstacle_manager().get_obstacles().Items()) {
    bool is_obstacle_vehicle = obj->Type() == ObjectType::COUPE ||
                               obj->Type() == ObjectType::BUS ||
                               obj->Type() == ObjectType::ENGINEER_TRUCK ||
                               obj->Type() == ObjectType::TRICYCLE;
    if (!is_obstacle_vehicle) {
      continue;
    }

    const Box2d &car_box = obj->PerceptionBoundingBox();

    if (!is_left_car) {
      is_left_car = slot_left.HasOverlap(car_box);
    }
    if (!is_right_car) {
      is_right_car = slot_right.HasOverlap(car_box);
    }
  }

  return true;
}


bool isEgoParallelInChannel(const Pose2D &ego_pose, Pose2D &target_pose) {
  planning_math::Vec2d target_vec (cos(target_pose.theta), sin(target_pose.theta));
  planning_math::Vec2d ego_vec (cos(ego_pose.theta), sin(ego_pose.theta));
  return fabs(target_vec.InnerProd(ego_vec)) < cos(M_PI / 180 * 75);
}

ApaStateMachine::DynamicPlanSimpleMpcResult
ApaStateMachine::dynamic_planning_simple_mpc(
    const Pose2D &ego_pose, const Pose2D &target_pose_lot,
    std::vector<TrajectoryPoint> &curve_traj) {

  DynamicPlanSimpleMpcResult result;
  result.traj_success = false;
  result.traj_collide = false;

  std::vector<Pose2D> curve;
  if (CurveJoin(ego_pose, target_pose_lot,
                CarParams::GetInstance()->min_turn_radius, 0.25, curve)) {
    static bool reset_dynamic_mpc_time_once;
    if (!reset_dynamic_mpc_time_once) {
      behavior_decider_parkin_->reset_mpc_collide_timeout();
      reset_dynamic_mpc_time_once = true;
    }
    const Box2d &slot_box = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_lot->getBox();
    const Box2d parking_lot_box_bottom = Box2d(
        slot_box.center() - 0.5 * slot_box.length() *
                                Vec2d::CreateUnitVec2d(slot_box.heading()),
        slot_box.heading(), slot_box.length(), slot_box.width() * 2.0);

    bool is_curve_collide = false;
    auto groundline_pts = world_model_->get_parking_ground_line_fusion();
    auto checker =
        parking_longitudinal_behavior_planner_->get_collision_checker();
    double obs_to_ego_square;
    double collision_threshold;
    double extra_thres;
    double ratio;
    bool use_secure;
    ObsPtsWithId obs_pts;
    for (const auto &obs : groundline_pts) {
      if (obs.id != PlanningContext::Instance()
                        ->parking_behavior_planner_output()
                        .parking_slot_info.id ||
          ((int)obs.type > 100 && (int)obs.type < 150)) {
        continue;
      }
      for (int i = 0; i < obs.pts.size(); i++) {
        if (slot_box.IsPointIn(Vec2d(obs.pts[i].x, obs.pts[i].y)) ||
            parking_lot_box_bottom.IsPointIn(
                Vec2d(obs.pts[i].x, obs.pts[i].y))) {
          continue;
        }
        planning_math::Vec2d obs_p(obs.pts[i].x, obs.pts[i].y, obs.id);
        collision_threshold = CarParams::GetInstance()->lat_inflation() * 0.5;
        extra_thres =
            std::min(CarParams::GetInstance()->lat_inflation() * 0.5,
                     CarParams::GetInstance()->lon_inflation_min - 0.1);
        obs_pts.emplace_back(collision_threshold, extra_thres,
                             CollisionCheckStatus(), obs_p, 1.0,
                             Pose2D(0.0, 0.0, 0.0, 0.0));
      }
    }
    // // obs_pts is obstacle points
    FreespacePoint lead_point;
    std::string debug_str;
    std::vector<std::pair<double, double>> old_mpc_sl_points;
    checker.remainDisCheck(obs_pts, {}, curve, true, true, ego_pose,
                           &lead_point, &debug_str, &old_mpc_sl_points);
    for (const auto &obs_result : obs_pts) {
      const CollisionCheckStatus &result = obs_result.result;
      if (!result.is_collision || !result.is_valid) {
        continue;
      }
      if (result.is_collision) {
        is_curve_collide = true;
        // printf("is_curve_collide\n");
        break;
      }
    }
    result.traj_success = true;
    result.traj_collide = is_curve_collide;
    if (!is_curve_collide) {
      curve_traj.clear();
      curve_traj.resize(curve.size());
      std::transform(curve.begin(), curve.end(), curve_traj.begin(),
                     [](const Pose2D &pose) -> TrajectoryPoint {
                       TrajectoryPoint tp;
                       tp.path_point.x = pose.x;
                       tp.path_point.y = pose.y;
                       tp.path_point.theta = pose.theta;
                       tp.v = -0.5;
                       return tp; // parasoft-suppress AUTOSAR-A8_5_0 "f-drop"
                     });
      regulateVelocity(StrategyParams::GetInstance(), curve_traj);
      apply_trajpoints_curvature(curve_traj);
      PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->is_dynamic_planning = true;
      PlanningContext::Instance()->mutable_open_space_path()->stash(curve_traj);
    }
  }
  return result;
}

} // namespace parking

} // namespace msquare
