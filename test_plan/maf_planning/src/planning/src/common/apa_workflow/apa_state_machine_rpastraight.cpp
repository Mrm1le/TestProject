#include "common/apa_workflow/apa_state_machine.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planning/common/common.h"
#ifdef USE_CONFIG_SERVICE
#include <mpf/core/parameter/parameter.h>
#endif
#include "common/apa_workflow/parking_config_loader.hpp"
#include "common/context_logger.h"

namespace msquare {

namespace parking {



// rpa standby, do something like onEntryWait
void ApaStateMachine::onEntryRpaStraightStandby() {

//[fenix.refactor.sm] Original ParkingTaskManager::onEntryRpaStraightStandby()

  world_model_->feed_pause_status(false); // clear pause status because pause is
                                          // treated as standby for rpa straight
  plan_times_ = 0;
  if (world_model_->get_planning_request().cmd.value != ParkingCommand::STOP) {
    (void)world_model_->clear_planning_request();
  }
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
  PlanningContext::Instance()->mutable_reference_line()->ClearReflinePoints();
  // clear parkout direction for rpa
  auto &parking_slot_info = PlanningContext::Instance()
                                ->mutable_parking_behavior_planner_output()
                                ->parking_slot_info;
  parking_slot_info.available_park_out_type.value = ParkOutType::INVALID;
  parking_slot_info.recommend_park_out_type.value = ParkOutType::INVALID;

  current_state_ = StatusType::RPA_STRAIGHT_STANDBY;
  status_text = "RPA_STRAIGHT_STANDBY";
  if (current_task_ != parking_task_list_.end()) {
    current_task_++;
  }
//[fenix.refactor.sm] END Original ParkingTaskManager::onEntryRpaStraightStandby()

//[fenix.refactor.sm] Original ScenarioManager::onEntryRpaStraightStandby()
  (void)ParkingConfigLoader::load_config(StatusType::RPA_STRAIGHT, 0);

  // init collision checker
  para_.width = VehicleParam::Instance()->width_wo_rearview_mirror;
  para_.length = VehicleParam::Instance()->length;
  para_.width_with_rearview = VehicleParam::Instance()->width;
  para_.obs_inflation =
      std::max(HybridAstarConfig::GetInstance()->inflation_for_points_, 0.0);
  para_.lat = std::max(0.0, CarParams::GetInstance()->lat_inflation());
  para_.lon = std::max(0.0, CarParams::GetInstance()->lon_inflation());
  para_.min_radius = CarParams::GetInstance()->min_turn_radius;
  para_.max_steering_angle = CarParams::GetInstance()->max_steer_angle;
  para_.step = msquare::HybridAstarConfig::GetInstance()->step_size;
  para_.wheel_radius = VehicleParam::Instance()->wheel_rolling_radius;
  para_.wheel_base = VehicleParam::Instance()->wheel_base;
  para_.front_to_rear = VehicleParam::Instance()->front_edge_to_center;
  para_.back_to_rear = VehicleParam::Instance()->back_edge_to_center;
  para_.rear_to_center =
      0.5 * std::abs(para_.front_to_rear - para_.back_to_rear);
  para_.straight_lat = para_.lat;
  para_.straight_lon = para_.lon;
  para_.straight_line = 15.0;    // max straight move distance
  para_.front_corner_width = 0.595;
  para_.front_corner_length = 3.645;
  csg_ = clothoid::CollisionShapeGenerator(para_);
  clo_checker_ = clothoid::CollisionChecker(csg_);
//[fenix.refactor.sm] END Original ScenarioManager::onEntryRpaStraightStandby()

}

void ApaStateMachine::onUpdateRpaStraightStandby() {
  //[fenix.refactor.sm] moved to ApaStateMachine::onTransitionRpaStraightStandb
}

void ApaStateMachine::onTransitionRpaStraightStandby(
    hfsm::Machine<Context>::Control &control) {
//[fenix.refactor.sm] Original ParkingTaskManager::onUpdateRpaStraightStandby()
PlanningContext::Instance()->mutable_planning_status()->task_status.task =
      StatusType::RPA_STRAIGHT_STANDBY;

  PlanningRequest planning_request = world_model_->get_planning_request();
  if (parking_task_config_.enable_config) {
  } else {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    if (planning_request.cmd.value != ParkingCommand::NONE &&
        (current_task_ == parking_task_list_.end() ||
         (current_task_ + 1) == parking_task_list_.end())) {
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      world_model_->set_control_test_flag(false);
      if (planning_request.cmd.value == ParkingCommand::RPA_STRAIGHT) {
        (void)generate_rpa_straight_tasks();
        (void)world_model_->clear_planning_request();
      }
    }
  }
//[fenix.refactor.sm] END Original ParkingTaskManager::onUpdateRpaStraightStandby()

//[fenix.refactor.sm] Original ParkingTaskManager::onTransitionRpaStraightStandby()
  const int kPlanningTimeLimit = 10;
  if (planning_request.cmd.value == ParkingCommand::STOP) {
    plan_times_ = 0;
    current_task_ = parking_task_list_.end();
    control.changeTo<Wait>();
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::SUCCEEDED;
  } else if (current_task_ == parking_task_list_.end() ||
             (current_task_ + 1) == parking_task_list_.end()) {
    // do nothing and wait for further request
  } else if ((current_task_ + 1)->task_type == ParkingTaskType::RPA_STRAIGHT) {
    if (world_model_->get_ego_state().is_static) {
      plan_times_++;
    }

    if (world_model_->get_ego_state().is_static) {
      MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::RUNNING;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->rpa_straight_direction.value =
          (current_task_ + 1)->rpa_straight_direction.value;
      control.changeTo<RpaStraight>();
    }
    if (plan_times_ > kPlanningTimeLimit) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::PLANNING_FAILED;
    }
  } else {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::FAILED;
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->task_status.failure_reason = FailureReason::UNKNOW_FAILED;
  }
//[fenix.refactor.sm] END Original ParkingTaskManager::onTransitionRpaStraightStandby()



//[fenix.refactor.sm] Original ScenarioManager::onUpdateRpaStraightStandby
double forward_dis, backward_dis;
  getRPARemainDis(forward_dis, backward_dis);
  auto output = PlanningContext::Instance()->mutable_parking_behavior_planner_output();
  output->rpa_forward_dis = forward_dis;
  output->rpa_backward_dis = backward_dis;
//[fenix.refactor.sm] END Original ScenarioManager::onUpdateRpaStraightStandby

//[fenix.refactor.sm] Original ScenarioManager::onTransitionRpaStraightStandby()

  //    transit state according to ApaStateMachine's statemachine status
  //    no need to re-write here

//[fenix.refactor.sm] END Original ScenarioManager::onTransitionRpaStraightStandby()



}

void ApaStateMachine::onLeaveRpaStraightStandby() {

//[fenix.refactor.sm] Original ScenarioManager::onLeaveRpaStraightStandby()
    auto output = PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output();
    output->rpa_forward_dis = -1.0;
    output->rpa_backward_dis = -1.0;
//[fenix.refactor.sm] END Original ScenarioManager::onLeaveRpaStraightStandby()
}


void ApaStateMachine::onEntryRpaStraight() {
//[fenix.refactor.sm] Original ParkingTaskManager::onEntryRpaStraight()
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.state_changed = true;
  current_state_ = StatusType::RPA_STRAIGHT;
  status_text = "RPA_STRAIGHT";
  current_task_++;
//[fenix.refactor.sm] END Original ParkingTaskManager::onEntryRpaStraight()

//[fenix.refactor.sm] Original ScenarioManager::onEntryRpaStraight()

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->planner_type = PlannerType::OPENSPACE;

  PlanningContext::Instance()->mutable_open_space_path()->stash(
      std::vector<TrajectoryPoint>());

  // get init and target of APOA
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
  init_traj_point.v = 0.0;
  init_traj_point.a = 0.0;

  const double rpa_straight_distance = CarParams::GetInstance()
          ->car_config.rpa_config.rpa_straight_distance;
  double sign = 1.0;
  if (PlanningContext::Instance()
          ->mutable_planning_status()
          ->rpa_straight_direction.value ==
      RpaStraightDirectionType::RPA_STRAIGHT_BACKWARD) {
    sign = -1.0;
  }
  target_traj_point = init_traj_point;
  target_traj_point.path_point.x +=
      (sign * rpa_straight_distance * std::cos(ego_state.ego_pose.theta));
  target_traj_point.path_point.y +=
      (sign * rpa_straight_distance * std::sin(ego_state.ego_pose.theta));

  openspace_state_machine_->createMachineRpaStraight();

  PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
      maf_vehicle_status::TurnSignalType::EMERGENCY_FLASHER;

  openspace_state_machine_->changeToStandby();

  PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
      false;
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->is_finish = false;
//[fenix.refactor.sm] END Original ScenarioManager::onEntryRpaStraight()
  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->has_ever_been_inside_slot = false;


}

void ApaStateMachine::onUpdateRpaStraight() {
  //[fenix.refactor.sm] moved to ApaStateMachine::onTransitionRpaStraight()
}

void ApaStateMachine::onTransitionRpaStraight(
    hfsm::Machine<Context>::Control &control) {

//[fenix.refactor.sm] Original ParkingTaskManager::onUpdateRpaStraight()
  PlanningContext::Instance()->mutable_planning_status()->task_status.task =
      StatusType::RPA_STRAIGHT;
  PlanningContext::Instance()->mutable_planning_status()->task_status.status =
      TaskStatusType::RUNNING;

//[fenix.refactor.sm] END Original ParkingTaskManager::onUpdateRpaStraight()

//[fenix.refactor.sm] Original ParkingTaskManager::onTransitionRpaStraight()
  bool is_finish =
      PlanningContext::Instance()->parking_behavior_planner_output().is_finish;
  PlanningRequest planning_request = world_model_->get_planning_request();
  if (planning_request.cmd.value == ParkingCommand::STOP) {
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::SUCCEEDED;
    current_task_ = parking_task_list_.end();
    control.changeTo<Wait>();
  } else if (planning_request.cmd.value ==
             ParkingCommand::RPA_STRAIGHT_STANDBY) {
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::RUNNING;
    current_task_ = parking_task_list_.end();
    control.changeTo<RpaStraightStandby>();
  } else if (planning_request.cmd.value == ParkingCommand::PARK_OUT_STANDBY) {
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::RUNNING;
    current_task_ = parking_task_list_.end();
    control.changeTo<RpaStraightStandby>();
  } else if ((current_task_ + 1) == parking_task_list_.end() ||
             (current_task_ + 1)->task_type ==
                 ParkingTaskType::RPA_STRAIGHT_STANDBY) {
    if (is_finish && world_model_->get_ego_state().is_static) {
      if (PlanningContext::Instance()
              ->mutable_planning_status()
              ->advanced_abandon) {
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.status = TaskStatusType::FAILED;
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.failure_reason = FailureReason::RPA_COMPLETE_FAILED;
      } else {
        PlanningContext::Instance()
            ->mutable_planning_status()
            ->task_status.status = TaskStatusType::SUCCEEDED;
      }
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = false;
      control.changeTo<RpaStraightStandby>();
    } else if (PlanningContext::Instance()
                   ->openspace_motion_planner_output()
                   .openspace_fallback_cnt > 1) {
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.status = TaskStatusType::FAILED;
      PlanningContext::Instance()
          ->mutable_planning_status()
          ->task_status.failure_reason = FailureReason::PLANNING_FAILED;
    }
  } else {
    MSD_LOG(ERROR, "%s: %d\n", __FUNCTION__, __LINE__);
    PlanningContext::Instance()->mutable_planning_status()->task_status.status =
        TaskStatusType::FAILED;
    PlanningContext::Instance()
        ->mutable_planning_status()
        ->task_status.failure_reason = FailureReason::UNKNOW_FAILED;
  }

  //[fenix.refactor.sm] END Original ParkingTaskManager::onTransitionRpaStraight()

  //[fenix.refactor.sm] Original ScenarioManager::onUpdateRpaStraight()


  openspace_state_machine_->update();

  bool openspace_is_fallback =
      openspace_state_machine_->getCurrentState() ==
      openspace_state_machine::OpenspaceStateEnum::FALLBACK;

  bool openspace_is_running =
      openspace_state_machine_->getCurrentState() ==
      openspace_state_machine::OpenspaceStateEnum::RUNNING;

  bool openspace_is_finish = openspace_state_machine_->getCurrentState() ==
                             openspace_state_machine::OpenspaceStateEnum::FINISH;

  if (openspace_is_fallback) {
    openspace_state_machine_->changeToStandby();
  }

  if (openspace_is_finish) {
    PlanningContext::Instance()
                        ->mutable_parking_behavior_planner_output()
                        ->is_finish = true;
  }

  if (PlanningContext::Instance()->planning_status().planning_result.gear ==
          GearState::REVERSE &&
      openspace_is_running) {
    if (world_model_->get_collide_to_limiter_when_reverse()) {
      PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
          true;
      PlanningContext::Instance()
          ->mutable_parking_behavior_planner_output()
          ->is_finish = true;
    }
  }

  if (!is_finish &&
      PlanningContext::Instance()->planning_status().blocked_timeout &&
      PlanningContext::Instance()->planning_status().blocked) {
    PlanningContext::Instance()->mutable_planning_status()->advanced_abandon =
        true;
    PlanningContext::Instance()
        ->mutable_parking_behavior_planner_output()
        ->is_finish = true;
  }
//[fenix.refactor.sm] END Original ScenarioManager::onUpdateRpaStraight()


//[fenix.refactor.sm] Original ScenarioManager::onTransitionRpaStraight()

  //    transit state according to ApaStateMachine's statemachine status
  //    no need to re-write here

//[fenix.refactor.sm] END Original ScenarioManager::onTransitionRpaStraight()

}

void ApaStateMachine::onLeaveRpaStraight() {
//[fenix.refactor.sm] Original ScenarioManager::onLeaveRpaStraight()
  openspace_state_machine_.reset(nullptr);
  PlanningContext::Instance()
      ->mutable_openspace_motion_planner_output()
      ->openspace_fallback_cnt = 0;
  PlanningContext::Instance()->mutable_turn_signal_cmd()->value =
      maf_vehicle_status::TurnSignalType::NONE;
  PlanningContext::Instance()->mutable_open_space_path()->stash(
      std::vector<TrajectoryPoint>());
//[fenix.refactor.sm] END Original ScenarioManager::onLeaveRpaStraight()

}




void ApaStateMachine::getRPARemainDis(double &forward_dis, double &backward_dis){
  std::vector<planning_math::LineSegment2d> obs_lines;
  std::vector<planning_math::Vec2d> obs_pts;
  std::vector<planning_math::LineSegment2d> step_lines_;
  std::vector<planning_math::Vec2d> step_pts;

  EgoState ego_state = world_model_->get_ego_state();
  Pose2D ego_pose(
    ego_state.ego_pose.x,
    ego_state.ego_pose.y,
    ego_state.ego_pose.theta
  );

  std::vector<planning_math::Vec2d> ego_pts;
  csg_.getRawShape(ego_pose, ego_pts, 0.0, 0.0);

  planning_math::Polygon2d ego_polygon;
  ego_polygon.update(ego_pts);

  double cs = std::cos(ego_pose.theta);
  double ss = std::sin(ego_pose.theta);

  planning_math::Vec2d ego_center(ego_pose.x + para_.rear_to_center * cs, ego_pose.y + para_.rear_to_center * ss);
  planning_math::Vec2d ego_front(ego_pose.x + para_.front_to_rear * cs, ego_pose.y + para_.front_to_rear * ss);
  planning_math::Box2d wheel_base_box(
    planning_math::LineSegment2d(ego_center, ego_front),
    para_.width
  );

  double wheel_stopper_forward = para_.straight_line;
  double wheel_stopper_backward = para_.straight_line;
  /* wheel stopper */
  const ParkingMapInfo &parking_map_info = world_model_->get_parking_map_info();
  for (auto &parking_lot : parking_map_info.parking_lots_detection_fusion_results){
    if(!parking_lot.wheel_stop.available){
      continue;
    }
    planning_math::LineSegment2d stopper_line(
      Vec2d(parking_lot.wheel_stop.point1.x, parking_lot.wheel_stop.point1.y), 
      Vec2d(parking_lot.wheel_stop.point2.x, parking_lot.wheel_stop.point2.y)
    );
    if(wheel_base_box.HasOverlap(stopper_line)){
      wheel_stopper_forward = 0.0;
      continue;
    }
    if(ego_polygon.HasOverlap(stopper_line)){
      wheel_stopper_backward = 0.0;
      continue;
    }

    step_lines_.push_back(stopper_line);
  }

  // set obstacles
  auto points = world_model_->obstacle_manager().get_points().Items();
  for (auto &point : points) {
    obs_pts.emplace_back(point->PerceptionBoundingBox().center());
  }

  // uss groundline
  for (auto &obs : world_model_->get_parking_ground_line_fusion()) {
    if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP || 
      obs.type == GroundLineType::GROUND_LINE_TYPE_STEP ||
      obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_STEP 
    ){
      for (int i = 0; i < obs.pts.size(); i++) {
        step_pts.emplace_back(obs.pts[i].x, obs.pts[i].y);
      }
    }else{
      for (int i = 0; i < obs.pts.size(); i++) {
        obs_pts.emplace_back(obs.pts[i].x, obs.pts[i].y);
      }
    }
    
    if (obs.pts.size() % 2 == 0) {
      if (obs.type == GroundLineType::GROUND_LINE_USS_TYPE_STEP || 
      obs.type == GroundLineType::GROUND_LINE_TYPE_STEP ||
      obs.type == GroundLineType::GROUND_LINE_USS_TYPE_SIDE_POINT_STEP ) {
        for (int i = 0; i < obs.pts.size(); i = i + 2) {
          step_lines_.emplace_back(Vec2d(obs.pts[i].x, obs.pts[i].y), Vec2d(obs.pts[i + 1].x, obs.pts[i + 1].y));
        }
      } else {
        for (int i = 0; i < obs.pts.size(); i = i + 2) {
          obs_lines.emplace_back(Vec2d(obs.pts[i].x, obs.pts[i].y), Vec2d(obs.pts[i + 1].x, obs.pts[i + 1].y));
        }
      }
    }
  }

  for (auto &obj : world_model_->obstacle_manager().get_obstacles().Items()) {
      auto box_edges =obj->PerceptionBoundingBox().GetAllEdges();
      obs_lines.insert(obs_lines.end(),box_edges.begin(), box_edges.end());
  }

  clo_checker_.setData(obs_lines, obs_pts, step_lines_, step_pts);

  forward_dis = clo_checker_.moveForward(
    ego_pose, CarParams::GetInstance()->lat_inflation(),
    CarParams::GetInstance()->lon_inflation(), clothoid::ShapeType::RAW
  );
  backward_dis = clo_checker_.moveBackward(
    ego_pose, CarParams::GetInstance()->lat_inflation(),
    CarParams::GetInstance()->lon_inflation(), clothoid::ShapeType::RAW
  );

  forward_dis = std::min(forward_dis, wheel_stopper_forward);
  backward_dis = std::min(backward_dis, wheel_stopper_backward);
  
}


} // namespace parking

} // namespace msquare
