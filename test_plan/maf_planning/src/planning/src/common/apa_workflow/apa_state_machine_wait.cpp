#include "common/apa_workflow/apa_state_machine.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planning/common/common.h"
#ifdef USE_CONFIG_SERVICE
#include <mpf/core/parameter/parameter.h>
#endif

namespace msquare {

namespace parking {

void ApaStateMachine::onEntryWait() {
  //[fenix.refactor.sm] Original ParkingTaskManager::onEntryWait()
  plan_times_ = 0;
  check_park_out_direction_ = false;
  if (msquare::CarParams::GetInstance()
          ->car_config.apoa_config.use_last_parkin_data) {
    loadParkinData();
  }

  (void)parking_slot_manager_.ClearParkingSlotKeyPoint();
  // unset_key_point();
  // clear_perception_snapshot();
  if (world_model_->get_planning_request().cmd.value != ParkingCommand::STOP) {
    (void)world_model_->clear_planning_request();
  }
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
  PlanningContext::Instance()->mutable_reference_line()->ClearReflinePoints();
  // PlanningContext::Instance()->mutable_parking_behavior_planner_output()->park_in_info.is_key_point_available
  // = false;
  // PlanningContext::Instance()->mutable_parking_behavior_planner_output()->park_out_info.is_key_point_available
  // = false;
  current_state_ = StatusType::WAIT;
  status_text = "WAIT";
  if (current_task_ != parking_task_list_.end()) {
    current_task_++;
  }
  //[fenix.refactor.sm] END Original ParkingTaskManager::onEntryWait()

  //[fenix.refactor.sm] Original ScenarioManager::onEntryWait()
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_lot = nullptr;

  //[fenix.refactor.sm] END Original ScenarioManager::onEntryWait()

  // record sbp_request count
  *PlanningContext::Instance()->mutable_sbp_request_count() = 0;
}

void ApaStateMachine::onUpdateWait() {
  //[fenix.refactor.sm] move to ApaStateMachine::onTransitionWait()
}

void ApaStateMachine::onTransitionWait(
    hfsm::Machine<Context>::Control &control) {

  const bool is_in_slot = is_in_parking_slot();

  //[fenix.refactor.sm] Original ParkingTaskManager::onUpdateWait()
  PlanningContext::Instance()->mutable_planning_status()->task_status.task =
      StatusType::WAIT;
  PlanningRequest planning_request = world_model_->get_planning_request();
  MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
  if (planning_request.cmd.value != ParkingCommand::NONE &&
      (current_task_ == parking_task_list_.end() ||
       (current_task_ + 1) == parking_task_list_.end())) {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    // parking_task_list_.clear();
    world_model_->set_control_test_flag(false);
    if (planning_request.cmd.value == ParkingCommand::DESIGNATE_PARKIN) {
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      if (!is_in_slot || is_request_to_ego_slot()) {
        MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
        (void)generate_designate_parkin_tasks();
        MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      }
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      (void)world_model_->clear_planning_request();
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    } else if (planning_request.cmd.value == ParkingCommand::PARK_OUT) {
      if (is_in_slot) {
        (void)generate_parkout_tasks();
      }
      (void)world_model_->clear_planning_request();
    } else if (planning_request.cmd.value == ParkingCommand::PARK_OUT_STANDBY) {
      check_park_out_direction_ = true;
      MSD_LOG(ERROR, "[onUpdateWait] APA PARK_OUT_STANDBY");
    } else if (planning_request.cmd.value ==
               ParkingCommand::RPA_STRAIGHT_STANDBY) { // do nothing
    } else {
      // std::cout << "Request invalid! Stop and wait for further request!"
      // << std::endl;
    }
  }

#pragma region calculate_available_and_recommended_parkout_direction

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->parking_slot_info.is_direction_limit = 0;
  if (is_in_slot && check_park_out_direction_) {
    parking_slot_manager_.UpdateEgoParkingSlotInfo();

    const auto &parking_slot_info = PlanningContext::Instance()
                                        ->parking_behavior_planner_output()
                                        .parking_slot_info;
    ParkoutSceneType parkout_scene_type =
        behavior_calculator_parkout_->calc_parkout_scene_type(
            parking_slot_info);

    uint32_t available_parkout_directions =
        behavior_calculator_parkout_->calculate_available_parkout_directions(
            parkout_scene_type);

    uint32_t recommended_parkout_direction =
        behavior_calculator_parkout_->calculate_recommended_parkout_direction(
            available_parkout_directions);

    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->parking_slot_info.available_park_out_type.value =
        available_parkout_directions;

    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->parking_slot_info.recommend_park_out_type.value =
        recommended_parkout_direction;

    if (!parkout_scene_type.is_front_collision) {
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_slot_info.is_direction_limit = 1;
    }
  } else {
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->parking_slot_info.recommend_park_out_type.value =
        ParkOutType::INVALID;

    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->parking_slot_info.available_park_out_type.value =
        ParkOutType::INVALID;
  }

//[fenix.refactor.sm] END Original ParkingTaskManager::onUpdateWait()
#pragma endregion

  //[fenix.refactor.sm] Original ParkingTaskManager::onTransitionWait()

  const int kPlanningTimeLimit = 10;
  if (planning_request.cmd.value == ParkingCommand::STOP) {
    plan_times_ = 0;
    current_task_ = parking_task_list_.end();
    control.changeTo<Wait>();
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::SUCCEEDED;
  } else if (planning_request.cmd.value ==
             ParkingCommand::RPA_STRAIGHT_STANDBY) { // switch to rpa standby
    control.changeTo<RpaStraightStandby>();
  } else if (current_task_ == parking_task_list_.end() ||
             (current_task_ + 1) == parking_task_list_.end()) {
    // do nothing and wait for further request
  } else if ((current_task_ + 1)->task_type == ParkingTaskType::WAIT) {
    current_task_++;
  } else if ((current_task_ + 1)->task_type == ParkingTaskType::PARKIN) {
    // PlanningContext::Instance()->mutable_aimed_poi()->id = (current_task_ +
    // 1)->poi_id; PlanningContext::Instance()->mutable_aimed_poi()->type =
    // (current_task_ + 1)->poi_type;
    // PlanningContext::Instance()->mutable_parking_out()->id = (current_task_ +
    // 1)->poi_id; PlanningContext::Instance()->mutable_parking_out()->type =
    // (current_task_ + 1)->poi_type;
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    if (world_model_->get_ego_state().is_static) {
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      plan_times_++;
    }

    if (parking_slot_manager_.UpdateParkingSlotInfo(
            (current_task_ + 1)->poi_id) &&
        world_model_->get_ego_state().is_static) {
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::SUCCEEDED;
      control.changeTo<ParkIn>();
    }
    if (plan_times_ > kPlanningTimeLimit) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::PLANNING_FAILED;
    }
  } else if ((current_task_ + 1)->task_type == ParkingTaskType::PARKOUT) {

    if (world_model_->get_ego_state().is_static) {
      plan_times_++;
    }
    bool success = true;
    success = is_in_slot && parking_slot_manager_.UpdateEgoParkingSlotInfo();
    uint32_t direction = (current_task_ + 1)->park_out_direction.value;
    if (direction == ParkOutDirectionType::AUTO_SELECT) {
      success &= autoSelectParkOutDirection();
    } else {
      success &= manualSelectParkoutDirection(direction);
    }
    success &= setParkOutInitAndTarget();
    if (success) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::SUCCEEDED;
      control.changeTo<ParkOut>();
    } else {
      if (plan_times_ > kPlanningTimeLimit) {
        MSD_LOG(ERROR, "%s: plan_times_ is over\n", __FUNCTION__);
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.status = TaskStatusType::FAILED;
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.failure_reason = FailureReason::PLANNING_FAILED;
      }
    }

  } else {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::FAILED;
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->task_status.failure_reason = FailureReason::UNKNOW_FAILED;
    // std::cout << "Next task invalid! Stop and wait for further request!" <<
    // std::endl;
    // current_task_ = parking_task_list_.end();
    // control.changeTo<Wait>();
  }
  //[fenix.refactor.sm] END Original ParkingTaskManager::onTransitionWait()

  //[fenix.refactor.sm] Original ScenarioManager::onUpdateWait()
  // NULL
  //[fenix.refactor.sm] END Original ScenarioManager::onUpdateWait()

  //[fenix.refactor.sm] Original ScenarioManager::onTransitionWait()
  //      transit accordingto ApaStateMachine::status
  //      no need to re-write here
  //[fenix.refactor.sm] END Original ScenarioManager::onTransitionWait()
}

void ApaStateMachine::onLeaveWait() { check_park_out_direction_ = false; }

bool ApaStateMachine::manualSelectParkoutDirection(uint32_t direction) {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;

  parking_slot_info.park_out_type.value = ParkOutType::INVALID;

  bool is_valid_direction =
      direction == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT ||
      direction == ParkOutType::PERPENDICULAR_OBLIQUE_REAR ||
      direction == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_LEFT ||
      direction == ParkOutType::PERPENDICULAR_OBLIQUE_FRONT_RIGHT ||
      direction == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_LEFT ||
      direction == ParkOutType::PERPENDICULAR_OBLIQUE_REAR_RIGHT ||
      direction == ParkOutType::PARALLEL_LEFT ||
      direction == ParkOutType::PARALLEL_RIGHT ||
      direction == ParkOutType::PARALLEL_LEFT_FRONT ||
      direction == ParkOutType::PARALLEL_RIGHT_FRONT;

  if (!is_valid_direction) {
    MSD_LOG(ERROR,
            "[manualSelectParkoutDirection] invalid parkout direction %u",
            direction);
    return false;
  }

  const uint32_t available_parkout_direction =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_slot_info.available_park_out_type.value;

  if (available_parkout_direction & direction) {
    // direction validity check finished, set;
    parking_slot_info.park_out_type.value = direction;
  } else {
    // direction is not in available direction
    MSD_LOG(ERROR,
            "[manualSelectParkoutDirection] parkout direction %u not available",
            direction);
    return false;
  }

  return true;
}

bool ApaStateMachine::autoSelectParkOutDirection() {
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;

  const uint32_t recommended_parkout_direction =
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->parking_slot_info.recommend_park_out_type.value;

  parking_slot_info.park_out_type.value = recommended_parkout_direction;

  return recommended_parkout_direction != ParkOutType::INVALID;
}

bool ApaStateMachine::setParkOutInitAndTarget() {
  Pose2D target_pose;
  Pose2D error_tolerence;

  bool success = behavior_calculator_parkout_->calculate_parkout_target(
      target_pose, error_tolerence);
  if (!success) {
    MSD_LOG(ERROR, "[setParkOutTarget] failed to get parkout point");
    return false;
  }

  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  parking_slot_info.park_out_point.point.x = target_pose.x;
  parking_slot_info.park_out_point.point.y = target_pose.y;
  parking_slot_info.park_out_point.point.theta = target_pose.theta;
  parking_slot_info.park_out_point.available = true;
  parking_slot_info.park_out_error_tolerence = error_tolerence;

  parking_slot_info.park_out_init_pose = world_model_->get_ego_state().ego_pose;

  MSD_LOG(INFO, "setParkOutTarget succeeded!");
  return true;
}

} // namespace parking

} // namespace msquare
