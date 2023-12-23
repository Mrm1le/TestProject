#include "planner/motion_planner/parking_longitudinal_motion_planner.h"
#include "common/math/math_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planning/common/common.h"
#include "planner/motion_planner/optimizers/parking_speed_optimizer/st_pieceewise_jerk_speed_optimizer.h"
#include "common/math/linear_interpolation.h"
#include "planner/trajectory_point.h"
#include "planner/message_type.h"
#include "common/speed/speed_data.h"
#include <chrono>
#include <ctime>
#include <vector>

namespace msquare {

namespace parking {

ParkingLongitudinalMotionPlanner::ParkingLongitudinalMotionPlanner(
    const std::shared_ptr<WorldModel> &world_model)
    : MotionPlanner(world_model), blocker_timer_(5.0),
      parking_stopper_timer_(8.0) {
  world_model_ = world_model;
  //! frenet parameters
}

bool ParkingLongitudinalMotionPlanner::clear_each_loop() {
  vec_target_speed_debug_.clear();
  current_v_count_debug_ = 0;
  return true;
}

bool ParkingLongitudinalMotionPlanner::calculate() {
  clear_each_loop();
  return run_calculate();
}

bool ParkingLongitudinalMotionPlanner::run_calculate() {
  const LongitudinalBehaviorPlannerOutput &lbp_output =
      PlanningContext::Instance()->longitudinal_behavior_planner_output();
  const LeaderPair &lead_cars = lbp_output.lead_cars;
  const FreespacePoint &free_space = lbp_output.free_space;
  const FreespaceLine &fs_line = lbp_output.fs_line;
  const std::vector<MultiDirectionalCars> &multidirectional_cars =
      lbp_output.multidirectional_cars;
  const std::vector<MultiDirectionalHuman> &multidirectional_human =
      lbp_output.multidirectional_human;
  const std::vector<IntentionStatusObstacles> &intention_status_obstacles =
      lbp_output.intention_status_obstacles;
  const std::vector<int> &prediction_obstacles =
      lbp_output.prediction_obstacles;
  const std::vector<double> &planning_mpc_diff = lbp_output.planning_mpc_diff;
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  // double calc_start = MTIME()->timestamp().sec();

  v_target_ = v_cruise;
  blocked_ = false;
  fs_blocked_ = false;
  pullover_enable_ = false;
  lateral_planning_failed_ = false;
  block_timeout_duration_ = 1.0; // TODO: recover to 3.0;

  const EgoState &ego_state = world_model_->get_ego_state();
  // const MapInfo &map_info = world_model_->get_map_info();

  const bool is_entrance = PlanningContext::Instance()
                               ->planning_status()
                               .planning_result.is_entrance;
  auto scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;

  (void)compute(lead_cars, free_space, fs_line, multidirectional_cars,
                multidirectional_human, intention_status_obstacles,
                prediction_obstacles, ego_state.ego_vel,
                ego_state.ego_steer_angle, status_type, is_entrance, scene_avp,
                planning_mpc_diff);
  const auto& sv_config2 = msquare::CarParams::GetInstance()->car_config.sv_config;
  if(!sv_config2.use_sv_speed_generator){
    fs_blocked_ = checkBlock(lead_cars, free_space, fs_line);
  }
  *PlanningContext::Instance()->mutable_planning_debug_info() +=
      "[block](" +
      std::to_string(PlanningContext::Instance()
                         ->planning_status()
                         .control_block_feedback) +
      "," + std::to_string(fs_blocked_) + "),";
  if (!PlanningContext::Instance()->planning_status().control_block_feedback &&
      !fs_blocked_) {
    block_count_ = std::max(0, block_count_ - 1);
  } else {
    block_count_ = 10;
  }
  if (block_count_ <= 0 || !ego_state.is_static ||
      gear_ !=
          PlanningContext::Instance()->planning_status().planning_result.gear ||
      (PlanningContext::Instance()->has_scene(scene_avp,
                                              ParkingSceneType::SCENE_RAMP) &&
       !lateral_planning_failed_) ||
      (PlanningContext::Instance()
               ->parking_behavior_planner_output()
               .planner_type == PlannerType::OPENSPACE &&
       !PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .is_move_ready)) {
    blocker_timer_.reset();
    block_count_ = 0;
  }

  if (world_model_->get_pause_status() || status_type != StatusType::APA ||
      std::abs(v_target_) <= VehicleParam::Instance()->velocity_deadzone ||
      std::abs(ego_state.ego_vel) >=
          VehicleParam::Instance()->velocity_deadzone ||
      !is_in_parking_slot() ||
      PlanningContext::Instance()->planning_status().planning_result.gear !=
          GearState::REVERSE) {
    parking_stopper_timer_.reset();
  }
  gear_ = PlanningContext::Instance()->planning_status().planning_result.gear;

  auto& lon_config = msquare::CarParams::GetInstance()->car_config.lon_config;
  if(lon_config.use_margin_speed || lon_config.use_osqp_speed){
    set_planning_resultV2(ego_state.ego_vel);
  }else{
    set_planning_result(ego_state.ego_vel);
  }
  return true;
}

bool ParkingLongitudinalMotionPlanner::compute(
    const LeaderPair lead_cars, const FreespacePoint freespace,
    const FreespaceLine fs_line,
    const std::vector<MultiDirectionalCars> multidirectional_cars,
    const std::vector<MultiDirectionalHuman> multidirectional_human,
    const std::vector<IntentionStatusObstacles> intention_status_obstacles,
    const std::vector<int> prediction_obstacles, const double v_ego,
    const double angle_steers, const StatusType status_type,
    const bool is_entrance, const int scene_avp,
    const std::vector<double> planning_mpc_diff){
    
    auto& lon_config = msquare::CarParams::GetInstance()->car_config.lon_config;

    if(lon_config.use_margin_speed || lon_config.use_osqp_speed){
    // use sop version speed planner
      computeVelV2(lead_cars, freespace, fs_line, multidirectional_cars,
                multidirectional_human, intention_status_obstacles,
                prediction_obstacles, v_ego,
                angle_steers, status_type, is_entrance, scene_avp,
                planning_mpc_diff);
      fs_blocked_ = checkBlockV2(lead_cars, freespace, fs_line);
      return true;
    }

    //  use old version
    return computeVel(lead_cars, freespace, fs_line, multidirectional_cars,
                multidirectional_human, intention_status_obstacles,
                prediction_obstacles, v_ego,
                angle_steers, status_type, is_entrance, scene_avp,
                planning_mpc_diff);

}

bool ParkingLongitudinalMotionPlanner::computeVel(
    const LeaderPair lead_cars, const FreespacePoint freespace,
    const FreespaceLine fs_line,
    const std::vector<MultiDirectionalCars> multidirectional_cars,
    const std::vector<MultiDirectionalHuman> multidirectional_human,
    const std::vector<IntentionStatusObstacles> intention_status_obstacles,
    const std::vector<int> prediction_obstacles, const double v_ego,
    const double angle_steers, const StatusType status_type,
    const bool is_entrance, const int scene_avp,
    const std::vector<double> planning_mpc_diff) {
  // const bool isAPA = (status_type == StatusType::APA || status_type ==
  // StatusType::APOA || status_type == StatusType::AVP);//TODO@tianbo: refactor
  // const std::string &scenario_type =
  // PlanningContext::Instance()->planning_status().scenario.scenario_type;
  auto &parking_behavior_planner_output =
      PlanningContext::Instance()->parking_behavior_planner_output();
  const bool is_openspace =
      (parking_behavior_planner_output.planner_type == PlannerType::OPENSPACE);
  bool force_stop = false;
  (void)calc_cruise_accel_limits(v_ego);
  (void)limit_accel_velocity_in_turns(v_ego, angle_steers);
  a_target_objective_ = a_target_;
  if (status_type == StatusType::WAIT || status_type == StatusType::DONE ||
      status_type == StatusType::RPA_STRAIGHT_STANDBY) {
    v_target_ = 0.0;
    force_stop = true;
    return true;
  }
  if (PlanningContext::Instance()->planning_status().rerouting.need_rerouting &&
      !is_openspace) {
    v_target_ = 0.0;
    force_stop = true;
    return true;
  }
  if (PlanningContext::Instance()
          ->planning_status()
          .planning_result.gear_changing ||
      ((int)world_model_->get_gear_report().gear_status.value) !=
          ((int)PlanningContext::Instance()
               ->planning_status()
               .planning_result.gear +
           1)) {
    v_target_ = 0.0;
    force_stop = true;
    // return true;
  }

  if (world_model_->get_pause_status() || world_model_->get_brake_override()) {
    v_target_ = 0.0;
    force_stop = true;
  }

  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .is_finish) {
    v_target_ = 0.0;
    force_stop = true;
  }

  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .approaching_wheel_stop) {
    if (PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .parking_slot_info.wheel_stop_info.available) {
      v_target_ = std::min(
          std::max(VehicleParam::Instance()->velocity_deadzone + 0.01, 0.2),
          v_target_);
    } else {
      v_target_ = std::min(
          std::max(VehicleParam::Instance()->velocity_deadzone + 0.01, 0.4),
          v_target_);
    }
  }
  vec_target_speed_debug_.emplace_back(v_target_);

  // if(world_model_->is_parking_svp()){
  //   if(!PlanningContext::Instance()->has_scene(scene_avp,
  //   ParkingSceneType::SCENE_RAMP)){
  //     limit_speed_before_turns(v_ego, scene_avp);
  //   }
  // }
  if (parking_behavior_planner_output.planner_type != PlannerType::OPENSPACE) {
    (void)limit_speed_for_large_curvature(v_ego, scene_avp);
    // std::cout << "compute_speed_for_large_curvature: "<< v_target_
    // <<std::endl;
  }
  (void)compute_speed_with_leads(lead_cars, v_ego, is_openspace);
  vec_target_speed_debug_.emplace_back(v_target_);

  // std::cout << "compute_speed_with_leads: "<< v_target_ <<std::endl;
  // std::cout << "is_entrance: " << is_entrance <<std::endl;
  // if (is_entrance){
  //   v_target_ = std::min(1.39, v_target_);
  // }
  // else
  // {
  if (!PlanningContext::Instance()->has_scene(scene_avp,
                                              ParkingSceneType::SCENE_HACK) &&
      !PlanningContext::Instance()->has_scene(scene_avp,
                                              ParkingSceneType::SCENE_RAMP) &&
      !PlanningContext::Instance()->has_scene(
          scene_avp, ParkingSceneType::SCENE_ENTRANCE)) {
    (void)compute_speed_for_freespace(freespace, v_ego);
    // std::cout<<"compute_speed_for_fs_point = "<<v_target_<<std::endl;
  }
  vec_target_speed_debug_.emplace_back(v_target_);

  (void)compute_speed_for_freespace(fs_line, v_ego);
  vec_target_speed_debug_.emplace_back(v_target_);
  
  // std::cout<<"compute_speed_for_obs_line = "<<v_target_<<std::endl;
  // std::cout<<"scene_avp = "<<scene_avp<<std::endl;
  // }
  // std::cout << "compute_speed_for_freespace: "<< v_target_ <<std::endl;

  (void)compute_speed_for_near_obstacle(v_ego, scene_avp);
  vec_target_speed_debug_.emplace_back(v_target_);
  
  // std::cout<<"compute_speed_for_near_obstacle: "<< v_target_ <<std::endl;
  v_target_ = std::max(v_target_, 0.0);

  (void)limit_speed_for_avoid_obstacle(v_ego);
  vec_target_speed_debug_.emplace_back(v_target_);
  
  // std::cout<<"compute_speed_for_avoid_obstacle: "<< v_target_ <<std::endl;

  if (status_type == StatusType::AVP && !is_openspace) {
    const ParkingMapInfo &parking_map_info_ =
        world_model_->get_parking_map_info();
    // std::cout << "apa distance: " << parking_map_info_.aimed_poi_info.id << "
    // " << world_model_->get_distance_to_poi() << std::endl;
    const double dist = world_model_->get_distance_to_poi() + static_distance_;
    if (world_model_->get_distance_to_poi() < 15.0) {
      v_target_ = std::min(v_target_, 1.39);
    }
    (void)compute_speed_for_parking(dist, v_ego, is_openspace);
  }
  vec_target_speed_debug_.emplace_back(v_target_);
  

  MSD_LOG(INFO, "%s: traj_length = %f", __FUNCTION__,
          PlanningContext::Instance()
              ->longitudinal_behavior_planner_output()
              .traj_length);
  if (PlanningContext::Instance()
              ->longitudinal_behavior_planner_output()
              .traj_length < 0.1 &&
      !is_openspace) {
    blocked_ = true;
    lateral_planning_failed_ = true;
  }

  double traj_length = PlanningContext::Instance()
                           ->longitudinal_behavior_planner_output()
                           .traj_length +
                       static_distance_;
  (void)compute_speed_for_parking(traj_length, v_ego);
  vec_target_speed_debug_.emplace_back(v_target_);
  
  if (PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .deviated &&
      !(world_model_->is_parking_lvp() ||
        world_model_->is_parking_apa() && status_type == StatusType::APA)) {
    v_target_ = std::min(v_target_, 0.5);
  }

  // std::cout << "compute_speed_for_parking: "<< v_target_ <<std::endl;

  if (!PlanningContext::Instance()->has_scene(scene_avp,
                                              ParkingSceneType::SCENE_RAMP)) {
    (void)compute_speed_for_multidirectional_cars_frenet(multidirectional_cars,
                                                         v_ego);
    // std::cout<<"compute_speed_for_multicars_frenet: "<<v_target_<<std::endl;
    (void)compute_speed_for_multidirectional_cars_ego(multidirectional_cars,
                                                      v_ego);
    // std::cout<<"compute_speed_for_multicars_ego: "<<v_target_<<std::endl;
    (void)compute_speed_for_multidirectional_human(multidirectional_human,
                                                   v_ego);
    // std::cout<<"compute_speed_for_multihuman: "<<v_target_<<std::endl;
  }
  vec_target_speed_debug_.emplace_back(v_target_);
  
  // compute_speed_for_intention_obstacle(intention_status_obstacles, v_ego);
  // std::cout<<"compute_speed_for_intention_obs: "<<v_target_<<std::endl;
  (void)compute_speed_for_prediction_obstacle(prediction_obstacles, v_ego);
  vec_target_speed_debug_.emplace_back(v_target_);
  
  // std::cout<<"compute_speed_for_prediction_obs: "<<v_target_<<std::endl;
  if (world_model_->is_parking_svp()) {
    (void)compute_speed_for_gate(v_ego);
    // std::cout<<"compute_speed_for_gate: "<<v_target_<<std::endl;
    if (status_type == StatusType::APA) {
      (void)limit_speed_for_radical_cars_while_APA();
      // std::cout<<"compute_speed_for_radical_cars: "<<v_target_<<std::endl;
    }
  }
  a_target_.second = std::min(a_target_.second, 0.5);
  if (is_openspace) {
    (void)compute_speed_for_apa();
  }
  vec_target_speed_debug_.emplace_back(v_target_);
  
  // std::cout << "compute_speed_for_apa: "<< v_target_ <<std::endl;

  // std::cout<<"v_target_final_: "<<v_target_<<std::endl;

  if (PlanningContext::Instance()->planning_status().planning_result.gear ==
          GearState::REVERSE ||
      parking_behavior_planner_output.planner_type == PlannerType::OPENSPACE) {
    a_target_.second = std::min(a_target_.second, 0.25);
  }

  compute_speed_for_uxe(force_stop);
  vec_target_speed_debug_.emplace_back(v_target_);
  
  MSD_LOG(ERROR, "compute_speed_for_uxe: v_target_ = %.3f", v_target_);

  return true;
}

bool ParkingLongitudinalMotionPlanner::checkBlock(const LeaderPair &lead_cars, const FreespacePoint &fs_point, const FreespaceLine &fs_line){
  bool use_new_fs_block = msquare::CarParams::GetInstance()->car_config.common_config.use_new_fs_block;
  if(!use_new_fs_block){
    return fs_blocked_;
  }
  if(fs_blocked_){
    return fs_blocked_;
  }

  double lon_inflation_min = CarParams::GetInstance()->lon_inflation_min;
  double remaining_distance = 10.0;
  auto &lead_one = lead_cars.first;

  if (fs_point.id >= 0) {
    remaining_distance = std::min(fs_point.d_rel - lon_inflation_min, remaining_distance);
  }
  if (lead_one.id >= 0 && lead_one.type == ObjectType::PEDESTRIAN) {
    remaining_distance = std::min(lead_one.d_rel - lon_inflation_min, remaining_distance);
  }
  if (lead_one.id >= 0 && lead_one.type == ObjectType::COUPE && !lead_one.is_static) {
    remaining_distance = std::min(lead_one.d_rel - lon_inflation_min, remaining_distance);
  }
  if (fs_line.id >= 0) {
    remaining_distance = std::min(fs_line.d_rel - lon_inflation_min, remaining_distance);
  }
  MSD_LOG(ERROR, "remaining_distance in fs_block: %.3f", remaining_distance);

  double min_finish_len = CarParams::GetInstance()->car_config.common_config.min_finish_len;
  bool is_gear_changing = PlanningContext::Instance()->planning_status().planning_result.gear_changing;
  bool is_block = (remaining_distance < min_finish_len) && (!is_gear_changing) && world_model_->get_ego_state().is_static;
  MSD_LOG(ERROR, "is_gear_changing: %d  is_static:%d",   is_gear_changing, world_model_->get_ego_state().is_static);

  return remaining_distance < min_finish_len ;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_uxe(bool force_stop){
  const auto& sv_config = msquare::CarParams::GetInstance()->car_config.sv_config;
  const ParkingBehaviorPlannerOutput &parking_behavior_planner_output =
      PlanningContext::Instance()->parking_behavior_planner_output();
  if(!sv_config.use_sv_speed_generator){
    return true;
  }

  if(std::abs(v_target_) < 0.1){
    v_target_ = 0.0;
    return true;
  }

  if (!parking_behavior_planner_output.is_move_ready) {
    v_target_ = 0.0;
    return true;
  }
  

  const auto& planning_result =
    PlanningContext::Instance()->planning_status().planning_result;
  const auto &pbpo = PlanningContext::Instance()->parking_behavior_planner_output();
  bool is_reverse = (planning_result.gear == GearState::REVERSE);
  const auto& status_type = PlanningContext::Instance()->planning_status().scenario.status_type;

  // sv speed for UXE
  sv_speed_param_.has_ever_been_inside_slot =  pbpo.has_ever_been_inside_slot;
  sv_speed_param_.is_reverse = is_reverse;
  sv_speed_param_.force_stop = force_stop; 
  sv_speed_param_.no_slot = (status_type == StatusType::RPA_STRAIGHT);
  sv_speed_param_.out_slot_coeff = sv_config.out_slot_coeff;
  sv_speed_param_.large_curv_coeff = sv_config.large_curv_coeff;
  sv_speed_param_.in_slot_coeff = sv_config.in_slot_coeff;
  sv_speed_param_.max_v = msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward;
  sv_speed_param_.width_wo_rearview_mirror = msquare::VehicleParam::Instance()->width_wo_rearview_mirror;
  sv_speed_param_.length = msquare::VehicleParam::Instance()->length;
  sv_speed_param_.curv_limit = 1.0 / CarParams::GetInstance()->min_turn_radius * sv_config.min_radius_coeff;
  bool is_in_slot =false ;
  const Pose2D & ego_pose = world_model_->get_ego_state().ego_pose;
  const planning_math::Box2d& slot_box = pbpo.parking_lot->getBox();
  v_target_ = SvSpeedGenerator::getSpeed(slot_box, ego_pose, planning_result.traj_curvature, sv_speed_param_, is_in_slot);
  MSD_LOG(ERROR, "compute_speed_for_uxe: is_in_slot = %d", is_in_slot);
  PlanningContext::Instance()->mutable_parking_behavior_planner_output()->has_ever_been_inside_slot = is_in_slot;
  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_with_leads(
    const LeaderPair lead_cars, const double v_ego, const bool isAPA) {
  double a_lead_p = 0.0;
  double d_des = 0.0;
  double v_target_lead = 40.0;
  double a_lead_p_2 = 0.0;
  double d_des_2 = 0.0;
  double v_target_lead_2 = 40.0;
  std::pair<double, double> a_target = a_target_objective_;
  double dist_to_poi = world_model_->get_distance_to_poi();
  if (lead_cars.first.id != -1) {
    // std::cout << "lead_cars.first.d_rel   " << lead_cars.first.d_rel <<
    // std::endl;
    a_lead_p = process_a_lead(lead_cars.first.a_lon);
    d_des = calc_desired_distance(lead_cars.first.v_lon, v_ego,
                                  isAPA || lead_cars.first.is_sidepass_obj ||
                                      (dist_to_poi < 0.0),
                                  false, lead_cars.first.is_static);
    v_target_lead = calc_desired_speed(lead_cars.first.d_rel, d_des,
                                       lead_cars.first.v_lon, a_lead_p);
    if (lead_cars.second.id != -1) {
      // std::cout << "lead_cars.sec.d_rel   " << lead_cars.second.d_rel <<
      // std::endl;
      a_lead_p_2 = process_a_lead(lead_cars.second.a_lon);
      d_des_2 = calc_desired_distance(
          lead_cars.second.v_lon, v_ego,
          isAPA || lead_cars.second.is_sidepass_obj || (dist_to_poi < 0.0),
          false, lead_cars.second.is_static);
      v_target_lead_2 = calc_desired_speed(lead_cars.second.d_rel, d_des_2,
                                           lead_cars.second.v_lon, a_lead_p_2);
    }
    if (v_target_lead > v_target_lead_2) {
      (void)calc_acc_accel_limits(lead_cars.second.d_rel, d_des_2, v_ego,
                                  lead_cars.second.v_lon, a_lead_p_2,
                                  v_target_lead_2, a_target);
      v_target_lead = v_target_lead_2;
      if (lead_cars.second.type != ObjectType::PEDESTRIAN){
        if (!(lead_cars.second.type == ObjectType::COUPE && !lead_cars.second.is_static
            && VehicleParam::Instance()->car_type == "C03")) {
          v_target_lead = std::max(
              VehicleParam::Instance()->velocity_deadzone * 2, v_target_lead);
        }
      }

      //if (lead_cars.second.type != ObjectType::PEDESTRIAN) {
       // v_target_lead = std::max(
      //      VehicleParam::Instance()->velocity_deadzone * 2, v_target_lead);
      //}

      // if (v_target_lead <= VehicleParam::Instance()->velocity_deadzone &&
      //     world_model_->get_ego_state().is_static &&
      //     !lead_cars.second.is_approaching_gate && lead_cars.second.is_static
      //     && lead_cars.second.type != ObjectType::PEDESTRIAN) {
      //   pullover_enable_ = (lead_cars.second.direction < 0) ||
      //                      (lead_cars.second.is_apa_status);
      //   if (pullover_enable_) {
      //     block_timeout_duration_ = 1.0;
      //   }
      //   if (lead_cars.second.type == ObjectType::PEDESTRIAN) {
      //     block_timeout_duration_ = 10.0;
      //   }
      // }
    } else {
      (void)calc_acc_accel_limits(lead_cars.first.d_rel, d_des, v_ego,
                                  lead_cars.first.v_lon, a_lead_p,
                                  v_target_lead, a_target);
      //if (lead_cars.first.type != ObjectType::PEDESTRIAN) {
       // v_target_lead = std::max(
         //   VehicleParam::Instance()->velocity_deadzone * 2, v_target_lead);
      //}
      if (lead_cars.first.type != ObjectType::PEDESTRIAN){
        if (!(lead_cars.first.type == ObjectType::COUPE && !lead_cars.first.is_static
             && VehicleParam::Instance()->car_type == "C03")) {
          v_target_lead = std::max(
              VehicleParam::Instance()->velocity_deadzone * 2, v_target_lead);
        }
      }

      // if (v_target_lead <= VehicleParam::Instance()->velocity_deadzone &&
      //     world_model_->get_ego_state().is_static &&
      //     !lead_cars.first.is_approaching_gate && lead_cars.first.is_static
      //     && lead_cars.first.type != ObjectType::PEDESTRIAN) {
      //   blocked_ = true;
      //   pullover_enable_ =
      //       (lead_cars.first.direction < 0) ||
      //       (lead_cars.first.is_apa_status);
      //   if (pullover_enable_) {
      //     block_timeout_duration_ = 1.0;
      //   }
      //   if (lead_cars.first.type == ObjectType::PEDESTRIAN) {
      //     block_timeout_duration_ = 10.0;
      //   }
      // }
    }
  }
  a_target_.first = std::min(a_target_.first, a_target.first);
  a_target_.second = std::min(a_target_.second, a_target.second);
  v_target_ = std::max(std::min(v_target_, v_target_lead), 0.0);
  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_freespace(
    const FreespacePoint lead_point, const double v_ego) {
  double n_des = 100.0;
  double d_des = 0.0;
  double v_target_lead_point = 40.0;
  std::pair<double, double> a_target = a_target_objective_;

  if (lead_point.id != -1) {
    n_des = std::max(0.0, lead_point.d_rel);
    d_des = calc_desired_distance(0.0, v_ego, true, true);
    v_target_lead_point = calc_desired_speed(lead_point.d_rel, d_des, 0.0, 0.0);
    (void)calc_acc_accel_limits(lead_point.d_rel, d_des, v_ego, 0.0, 0.0,
                                v_target_lead_point, a_target);
  }
  a_target_.first = std::min(a_target_.first, a_target.first);
  a_target_.second = std::min(a_target_.second, a_target.second);
  v_target_ = std::min(v_target_, v_target_lead_point);
  if (v_target_lead_point <= VehicleParam::Instance()->velocity_deadzone &&
      world_model_->get_ego_state().is_static) {
    fs_blocked_ = true;
  }

  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_near_obstacle(
    const double v_ego, const int scene_avp) {
  const double PI = 3.141592653589793238463;
  auto planning_result =
      PlanningContext::Instance()->planning_status().planning_result;
  std::vector<Pose2D> traj = PlanningContext::Instance()
                                 ->longitudinal_behavior_planner_output()
                                 .trajectory;
  std::vector<Pose2D> mpc_traj = PlanningContext::Instance()
                                     ->longitudinal_behavior_planner_output()
                                     .mpc_trajectory;
  std::vector<float> traj_curvature = planning_result.traj_curvature;
  bool is_in_turn = 0;
  if (traj_curvature.size() >= 20) {
    for (int i = 0; i < 20; i++) {
      if (traj_curvature[i] > 0.1) {
        is_in_turn = 1;
        break;
      }
    }
  }

  double v_limit_pre_brake = v_cruise;
  double dis_hi = is_in_turn ? 1.5 : 1.0;
  double dis_lo = is_in_turn ? 1.0 : 0.6;
  double dis_max = 6.0;
  double min_vel = 7.0 / 3.6;
  for (auto &obstacle :
       world_model_->obstacle_manager().get_obstacles().Items()) {
    auto lon_decision = obstacle->LongitudinalDecision();
    if (lon_decision.has_lead()) {
      if (obstacle->IsLonOppositeWrtFrenet()) {
        dis_hi = 1.5;
        dis_lo = 1.0;
        dis_max = 15.0;
      }
      CollisionCheckStatus result, result_mpc, result_traj;
      EgoModelManager &ego_model = collision_checker_.get_ego_model();
      const bool reverse =
          PlanningContext::Instance()->planning_status().planning_result.gear ==
          GearState::REVERSE;
      double back_comp_scaler = reverse ? -1.0 : 1.0;
      double back_comp_length =
          reverse ? 0.0
                  : -VehicleParam::Instance()->front_edge_to_center +
                        VehicleParam::Instance()->length;
      double comp_length = VehicleParam::Instance()->front_edge_to_center -
                           VehicleParam::Instance()->length / 2.0 +
                           back_comp_length * back_comp_scaler / 2.0;
      double ego_x = world_model_->get_ego_state().ego_pose.x;
      double ego_y = world_model_->get_ego_state().ego_pose.y;
      double ego_theta = world_model_->get_ego_state().ego_pose.theta;
      double center_x = ego_x + cos(ego_theta) * (comp_length);
      double center_y =
          ego_y +
          sin(world_model_->get_ego_state().ego_pose.theta) * (comp_length);
      (void)collision_checker_
          .set_deviation_length(VehicleParam::Instance()->front_edge_to_center -
                                VehicleParam::Instance()->length / 2.0)
          .set_cut_length(back_comp_length)
          .set_reverse(reverse);
      planning_math::Box2d ego_box(
          planning_math::Vec2d(center_x, center_y), ego_theta,
          VehicleParam::Instance()->length - back_comp_length,
          VehicleParam::Instance()->width);
      double collision_threshold = std::max(
          std::min(ego_box.DistanceTo(obstacle->PerceptionBoundingBox()), 0.1),
          0.05);
      (void)ego_model.set_model_type(EgoModelType::PENTAGON);
      result_mpc = collision_checker_.collision_check(
          mpc_traj, obstacle->PerceptionBoundingBox(), collision_threshold);
      result_traj = collision_checker_.collision_check(
          traj, obstacle->PerceptionBoundingBox(), collision_threshold);
      if (result_traj.min_distance < result_mpc.min_distance)
        result = result_traj;
      else
        result = result_mpc;

      if (!result.is_collision && result.min_distance < dis_hi) {
        if (result.s < dis_max / 2.0) {
          if (result.min_distance > dis_lo) {
            v_limit_pre_brake = (v_cruise + min_vel) / 2.0;
          } else if (result.min_distance > dis_lo / 2.0) {
            v_limit_pre_brake = min_vel * 0.8;
            a_target_.second = std::min(a_target_.second, 0.25);
          } else {
            v_limit_pre_brake = 0.5;
            a_target_.second = std::min(a_target_.second, 0.2);
          }
        } else if (result.s < dis_max) {
          if (result.min_distance < dis_lo) {
            v_limit_pre_brake = min_vel;
            a_target_.second = std::min(a_target_.second, 0.25);
          }
        }
        // v_limit_pre_brake = (clip(lon_decision.lead().min_distance, dis_lo,
        // dis_hi) - dis_lo) / (dis_hi - dis_lo) * (v_cruise - min_vel) +
        // min_vel;
        // std::cout << "near: " <<obstacle->Id()<<" "<< v_limit_pre_brake << "
        // " << lon_decision.lead().min_distance << "
        // "<<lon_decision.lead().distance_s<<" "  <<
        // (clip(lon_decision.lead().min_distance, dis_lo, dis_hi) - dis_lo) <<
        // " " << (v_cruise - min_vel) << std::endl;
        v_limit_pre_brake = std::max(v_limit_pre_brake, v_ego - 1.0);
        v_target_ = std::min(v_limit_pre_brake, v_target_);
      }
    }
  }

  auto &points = world_model_->obstacle_manager().get_all_points().Items();
  double min_vel_thre = 0.3;
  double dist_thre =
      std::abs(world_model_->get_ego_state().ego_steer_angle) > 3.5 ? 0.5 : 0.1;
  for (auto &point : points) {
    auto lon_decision = point->LongitudinalDecision();
    if (lon_decision.has_lead()) {
      dis_lo = 0.5;
      dis_max = 3.0;
      if (!lon_decision.lead().is_collision &&
          lon_decision.lead().min_distance < dis_lo) {
        if (lon_decision.lead().distance_s < dis_max) {
          a_target_.second = std::min(a_target_.second, 0.25);
          v_limit_pre_brake =
              std::max(min_vel_thre,
                       std::max(lon_decision.lead().min_distance - 0.05,
                                lon_decision.lead().distance_s - dist_thre));
          v_target_ = std::min(v_limit_pre_brake, v_target_);
        }
      }
    }
  }

  std::vector<Pose2D> traj_pose = planning_result.traj_pose_array;
  std::vector<float> traj_relative_s = planning_result.traj_relative_s;
  std::vector<float> traj_curv_radius = planning_result.traj_curvature_radius;
  if (traj_pose.empty())
    return true;

  for (size_t i = 0; i < traj_pose.size(); i++) {
    if (traj_pose[i].theta < 0.0)
      traj_pose[i].theta += 2 * PI;
  }
  std::vector<double> traj_steer_angle;
  std::vector<double> traj_s;
  for (size_t i = 0; i < traj_curv_radius.size(); i++) {
    if (traj_relative_s[i] < 0.0 || traj_relative_s[i] > 10.0)
      continue;
    if (std::isnan(traj_curv_radius[i]))
      continue;
    int gear =
        PlanningContext::Instance()->planning_status().planning_result.gear ==
                GearState::REVERSE
            ? -1
            : 1;
    int sgn = traj_curv_radius[i] > 0 ? 1 : -1;
    traj_steer_angle.emplace_back(
        std::atan2(VehicleParam::Instance()->wheel_base,
                   std::abs(traj_curv_radius[i])) *
        VehicleParam::Instance()->steer_ratio * sgn * 180.0 / PI);
    traj_s.emplace_back(traj_relative_s[i]);
  }
  // for(size_t i=0;i<traj_steer_angle.size();i++){
  //   std::cout<<"traj_steer_angle = "<<traj_steer_angle[i]<<std::endl;
  // }
  auto find_max_steering_angle = [&traj_steer_angle,
                                  &traj_s](double s) -> double {
    double max_steering_angle = 0.0;
    for (size_t i = 0; i < traj_steer_angle.size(); i++) {
      if ((s > traj_s[i] - 3) && (s <= traj_s[i])) {
        max_steering_angle =
            std::max(max_steering_angle, std::abs(traj_steer_angle[i]));
      }
    }
    return max_steering_angle;
  };

  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .planner_type == PlannerType::OPENSPACE) {

    auto remain_s = PlanningContext::Instance()
                        ->longitudinal_behavior_planner_output()
                        .traj_length;
    if (remain_s < 1.5) {
      v_target_ = std::min(0.2, v_target_);
    } else {
      v_target_ =
          std::min(sqrt(std::pow(0.2, 2) + 0.6 * (remain_s - 1.5)), v_target_);
    }

    auto find_nearest_steering_angle = [&traj_steer_angle,
                                        &traj_s](double s) -> double {
      double steering_angle = 0.0;
      double nearest_distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < traj_steer_angle.size(); i++) {
        if (std::abs(s - traj_s[i]) < nearest_distance) {
          nearest_distance = std::abs(s - traj_s[i]);
          steering_angle = traj_steer_angle[i];
        }
      }
      return steering_angle;
    };

    auto calc_nearest_steering_change_dist =
        [&traj_steer_angle, &traj_s](double s,
                                     double steer_diff_ratio_thr) -> double {
      double steering_angle = 0.0;
      double nearest_distance = std::numeric_limits<double>::infinity();
      int min_index = -1;
      for (size_t i = 0; i < traj_steer_angle.size(); i++) {
        if (std::abs(s - traj_s[i]) < nearest_distance) {
          nearest_distance = std::abs(s - traj_s[i]);
          steering_angle = traj_steer_angle[i];
          min_index = i;
        }
      }
      if (min_index < 0) {
        return 10.0;
      }
      int j = min_index;
      for (; j < traj_steer_angle.size() - 1; ++j) {
        if (std::abs(traj_steer_angle[j] - steering_angle) >
            steer_diff_ratio_thr *
                CarParams::GetInstance()->get_max_steer_angle()) {
          break;
        }
      }
      return traj_s[j] - traj_s[min_index];
    };

    auto &ego_pose = world_model_->get_ego_state().ego_pose;
    auto get_current_s = [&ego_pose, &traj_pose, &traj_relative_s]() -> double {
      double nearest_distance = std::numeric_limits<double>::infinity();
      double current_s = 0.0;
      for (size_t i = 0; i < traj_pose.size(); i++) {
        if (std::hypot(traj_pose[i].x - ego_pose.x,
                       traj_pose[i].y - ego_pose.y) < nearest_distance) {
          nearest_distance = std::hypot(traj_pose[i].x - ego_pose.x,
                                        traj_pose[i].y - ego_pose.y);
          current_s = traj_relative_s[i];
        }
      }
      return current_s;
    };

    double preview_distance = 1.5;
    double current_s = get_current_s();
    const double PI = 3.141592653589793238463;
    double rad_to_degree = 180.0 / PI;

    bool is_parallel_slot =
        PlanningContext::Instance()
            ->parking_behavior_planner_output()
            .parking_slot_info.type.value == ParkingSlotType::PARALLEL;

    if (is_parallel_slot) {
      double steer_diff_ratio_thr = 0.8;
      double steer_change_dist =
          calc_nearest_steering_change_dist(current_s, steer_diff_ratio_thr);
      if (steer_change_dist > preview_distance) {
        v_target_ = std::min(sqrt(std::pow(0.2, 2) +
                                  0.6 * (steer_change_dist - preview_distance)),
                             v_target_);
      }

      if (std::abs(find_nearest_steering_angle(current_s) -
                   find_nearest_steering_angle(current_s + preview_distance)) >
              steer_diff_ratio_thr *
                  CarParams::GetInstance()->get_max_steer_angle() ||
          std::abs(world_model_->get_ego_state().ego_steer_angle *
                       rad_to_degree -
                   find_nearest_steering_angle(current_s + preview_distance)) >
              steer_diff_ratio_thr *
                  CarParams::GetInstance()->get_max_steer_angle()) {
        v_target_ = std::min(0.2, v_target_);
      }
    } else {
      // 保持530版本的逻辑
      double steer_diff_ratio_thr = 1.1;
      double low_velocity = 0.3;
      double steer_change_dist =
          calc_nearest_steering_change_dist(current_s, steer_diff_ratio_thr);
      if (steer_change_dist > preview_distance) {
        v_target_ = std::min(sqrt(std::pow(low_velocity, 2) +
                                  0.6 * (steer_change_dist - preview_distance)),
                             v_target_);
      }

      if (std::abs(find_nearest_steering_angle(current_s) -
                   find_nearest_steering_angle(current_s + preview_distance)) >
              steer_diff_ratio_thr *
                  CarParams::GetInstance()->get_max_steer_angle() ||
          std::abs(world_model_->get_ego_state().ego_steer_angle *
                       rad_to_degree -
                   find_nearest_steering_angle(current_s + preview_distance)) >
              steer_diff_ratio_thr *
                  CarParams::GetInstance()->get_max_steer_angle()) {
        v_target_ = std::min(low_velocity, v_target_);
      }

      // 保持530版本的逻辑
      steer_diff_ratio_thr = 0.8;
      low_velocity = 0.5;
      steer_change_dist =
          calc_nearest_steering_change_dist(current_s, steer_diff_ratio_thr);
      if (steer_change_dist > preview_distance) {
        v_target_ = std::min(sqrt(std::pow(low_velocity, 2) +
                                  0.6 * (steer_change_dist - preview_distance)),
                             v_target_);
      }

      if (std::abs(find_nearest_steering_angle(current_s) -
                   find_nearest_steering_angle(current_s + preview_distance)) >
              steer_diff_ratio_thr *
                  CarParams::GetInstance()->get_max_steer_angle() ||
          std::abs(world_model_->get_ego_state().ego_steer_angle *
                       rad_to_degree -
                   find_nearest_steering_angle(current_s + preview_distance)) >
              steer_diff_ratio_thr *
                  CarParams::GetInstance()->get_max_steer_angle()) {
        v_target_ = std::min(low_velocity, v_target_);
      }
    }
  }

  const std::vector<double> dis_hi_fp{1.0, 1.5};
  const std::vector<double> dis_lo_fp{0.5, 1.0};
  const std::vector<double> steer_xp{135.0, 270.0};
  dis_max = 7.0;
  // std::cout<<"dis hi lo = "<<max_steer<<" "<<min_steer<<" "<<dis_hi<<"
  // "<<dis_lo<<std::endl;

  auto &pillars = world_model_->obstacle_manager().get_pillars().Items();
  auto &road_borders =
      world_model_->obstacle_manager().get_road_borders().Items();
  auto map_obstacle = pillars;
  map_obstacle.insert(map_obstacle.end(), road_borders.begin(),
                      road_borders.end());

  // std::vector<int> obs_id_considered;
  // if(!PlanningContext::Instance()->mutable_parking_lateral_behavoir_planner_output()->lateral_planning_info.empty()){
  //   obs_id_considered =
  //   PlanningContext::Instance()->mutable_parking_lateral_behavoir_planner_output()->lateral_planning_info.at("normal").sidepass_static_obj;
  //   obs_id_considered.insert(obs_id_considered.end(),PlanningContext::Instance()->mutable_parking_lateral_behavoir_planner_output()->lateral_planning_info.begin()->second.sidepass_dynamic_obj.begin(),PlanningContext::Instance()->mutable_parking_lateral_behavoir_planner_output()->lateral_planning_info.begin()->second.sidepass_dynamic_obj.end());
  // }
  // bool has_sidepass_obs = false;
  // for (auto &id : obs_id_considered){
  //   if(world_model_->obstacle_manager().find_obstacle(id)==nullptr)
  //     continue;
  //   if(world_model_->obstacle_manager().find_obstacle(id)->IsBesideRoad())
  //       continue;
  //   has_sidepass_obs = true;
  // }
  // dis_hi = has_sidepass_obs?1.5:1.0;
  // dis_lo = has_sidepass_obs?1.0:0.6;
  // dis_max = 10.0;

  for (auto &obstacle : map_obstacle) {
    auto lon_decision = obstacle->LongitudinalDecision();
    if (lon_decision.has_lead()) {
      double max_steering_angle_abs =
          find_max_steering_angle(lon_decision.lead().min_distance);
      dis_hi = interp(max_steering_angle_abs, steer_xp, dis_hi_fp);
      dis_lo = interp(max_steering_angle_abs, steer_xp, dis_lo_fp);
      if (!lon_decision.lead().is_collision &&
          lon_decision.lead().min_distance < dis_hi) {
        if (lon_decision.lead().distance_s < dis_max / 2.0) {
          if (lon_decision.lead().min_distance > dis_lo) {
            v_limit_pre_brake = (v_cruise + min_vel) / 2.0;
          } else if (lon_decision.lead().min_distance > dis_lo / 2.0) {
            v_limit_pre_brake = min_vel * 0.8;
            a_target_.second = std::min(a_target_.second, 0.25);
          } else {
            v_limit_pre_brake = 0.5;
            a_target_.second = std::min(a_target_.second, 0.2);
          }
        } else if (lon_decision.lead().distance_s < dis_max) {
          if (lon_decision.lead().min_distance < dis_lo) {
            v_limit_pre_brake = min_vel;
            a_target_.second = std::min(a_target_.second, 0.25);
          }
        }
        // v_limit_pre_brake = (clip(lon_decision.lead().min_distance, dis_lo,
        // dis_hi) - dis_lo) / (dis_hi - dis_lo) * (v_cruise - min_vel) +
        // min_vel;
        // std::cout << "near: " <<obstacle->Id()<<" "<< v_limit_pre_brake << "
        // " << lon_decision.lead().min_distance << "
        // "<<lon_decision.lead().distance_s<<" "  <<
        // (clip(lon_decision.lead().min_distance, dis_lo, dis_hi) - dis_lo) <<
        // " " << (v_cruise - min_vel) << std::endl;
        v_limit_pre_brake = std::max(v_limit_pre_brake, v_ego - 0.5);
        v_target_ = std::min(v_limit_pre_brake, v_target_);
      }
    }
  }

  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_freespace(
    const FreespaceLine &fs_line, const double v_ego) {
  double n_des = 100.0;
  double d_des = 0.0;
  double v_target_fs = 40.0;
  std::pair<double, double> a_target = a_target_objective_;

  if (fs_line.id != -1) {
    n_des = std::max(0.0, fs_line.d_rel);
    d_des = calc_desired_distance(0.0, v_ego, true, true);
    v_target_fs = calc_desired_speed(fs_line.d_rel, d_des, 0.0, 0.0);
    (void)calc_acc_accel_limits(fs_line.d_rel, d_des, v_ego, 0.0, 0.0,
                                v_target_fs, a_target);
  }
  a_target_.first = std::min(a_target_.first, a_target.first);
  a_target_.second = std::min(a_target_.second, a_target.second);
  v_target_ = std::min(v_target_, v_target_fs);

  if (v_target_fs <= VehicleParam::Instance()->velocity_deadzone &&
      world_model_->get_ego_state().is_static) {
    fs_blocked_ = true;
  }

  return true;
}

bool ParkingLongitudinalMotionPlanner::
    compute_speed_for_multidirectional_cars_frenet(
        const std::vector<MultiDirectionalCars> multidirectional_cars,
        const double v_ego) {
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  std::vector<double> v_target_multicars_vector;
  std::vector<int> id_vector;
  double v_target_multicars_min = 40.0;
  int id_v_target_multicars_min = -1;
  if (multidirectional_cars.size() == 0) {
    return false;
  }

  // std::cout << "size of multicars =
  // "<<multidirectional_cars.size()<<std::endl;
  for (auto multicars : multidirectional_cars) {
    // std::cout << "multicars_struct = <"
    //           <<multicars.id<<" : "
    //           <<multicars.TTC_pwj<<" : "
    //           <<multicars.TTC_ego<<" : "
    //           <<multicars.s_prediction<<" : "
    //           <<multicars.decay_rate<<" : "
    //           <<multicars.coordinate<<std::endl;
    if (multicars.coordinate != 1) {
      continue;
    }
    double n_des = 0.0;
    double d_des = 0.0;
    double v_target_multicars = 40.0;
    n_des = std::max(0.0, multicars.s_prediction);
    d_des = calc_desired_distance(0.0, v_ego);
    v_target_multicars = calc_desired_speed(n_des, d_des, 0.0, 0.0);
    double delta_v = 40.0;
    delta_v = v_target_ - v_target_multicars;
    delta_v *= multicars.decay_rate;
    v_target_multicars += delta_v;
    v_target_multicars_vector.push_back(v_target_multicars);
    id_vector.push_back(multicars.id);
  }
  if (v_target_multicars_vector.empty())
    return false;
  v_target_multicars_min = *std::min_element(v_target_multicars_vector.begin(),
                                             v_target_multicars_vector.end());
  vector<double>::iterator iter =
      std::find(v_target_multicars_vector.begin(),
                v_target_multicars_vector.end(), v_target_multicars_min);
  id_v_target_multicars_min =
      id_vector[std::distance(v_target_multicars_vector.begin(), iter)];
  // std::cout << "id : v_target_multicars : s_prediction : TTC_pwj : TTC_ego :
  // decay_rate = "
  // << id_v_target_multicars_min << " : "
  // << v_target_multicars_min << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].s_prediction
  // << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].TTC_pwj
  // << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].TTC_ego
  // << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].decay_rate
  // <<std::endl;
  double ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
                 VehicleParam::Instance()->front_edge_to_center;
  // std::cout << "ego_s = "<<ego_s<<std::endl;
  for (auto multicars : obstacles_) {
    if (multicars->Id() == id_v_target_multicars_min) {
      // std::cout << "static tuple =
      // <"<<multicars->IsLatStaticWrtFrenet()<<","<<multicars->IsLonStaticWrtFrenet()<<">"<<std::endl;
      auto sl_boundary = multicars->PerceptionSLBoundaryPlanning();
      double speed_yaw = multicars->Speed_yaw_relative_frenet();
      double obs_v_lat = multicars->speed() * sin(speed_yaw);
      double obs_v_lon = multicars->speed() * cos(speed_yaw);
      // std::cout << "id : sl_boundary : speed = "<< multicars->Id()<<" : <"
      // <<
      // sl_boundary.start_s<<","<<sl_boundary.end_s<<","<<sl_boundary.start_l<<","<<sl_boundary.end_l<<">
      // , <"
      // << multicars->speed()<<","<<obs_v_lat<<","<<obs_v_lon<<">"<<std::endl;
    }
  }
  // v_target_ = std::min(v_target_, std::max(std::max(v_ego - 0.1,
  // 0.0),v_target_multicars_min));

  v_target_ = std::min(v_target_, std::max(0.0, v_target_multicars_min));
  if (v_ego - v_target_ > 1.0) { // release restriction of a_target_min when
                                 // emergency brake is needed
    a_target_.first = std::min(-0.6, a_target_.first);
  }
  return true;
}

bool ParkingLongitudinalMotionPlanner::
    compute_speed_for_multidirectional_cars_ego(
        const std::vector<MultiDirectionalCars> multidirectional_cars,
        const double v_ego) {
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  std::vector<double> v_target_multicars_vector;
  std::vector<int> id_vector;
  double v_target_multicars_min = 40.0;
  int id_v_target_multicars_min = -1;
  if (multidirectional_cars.size() == 0)
    return false;
  // std::cout<<"size of multicars vector = "<<multidirectional_cars.size()
  // <<std::endl;

  for (auto multicars : multidirectional_cars) {
    // std::cout << "multicars_struct = <"
    //           <<multicars.id<<" : "
    //           <<multicars.TTC_pwj<<" : "
    //           <<multicars.TTC_ego<<" : "
    //           <<multicars.s_prediction<<" : "
    //           <<multicars.decay_rate<<" : "
    //           <<multicars.coordinate<<std::endl;
    if (multicars.coordinate != 2) {
      continue;
    }
    double n_des = 0.0;
    double d_des = 0.0;
    double v_target_multicars = 40.0;
    n_des = std::max(0.0, multicars.s_prediction);
    d_des = calc_desired_distance(0.0, v_ego);
    v_target_multicars = calc_desired_speed(n_des, d_des, 0.0, 0.0);
    double delta_v = 40.0;
    delta_v = v_target_ - v_target_multicars;
    delta_v *= multicars.decay_rate;
    v_target_multicars += delta_v;
    v_target_multicars_vector.push_back(v_target_multicars);
    id_vector.push_back(multicars.id);
  }
  if (v_target_multicars_vector.size() == 0)
    return false;
  v_target_multicars_min = *std::min_element(v_target_multicars_vector.begin(),
                                             v_target_multicars_vector.end());
  vector<double>::iterator iter =
      std::find(v_target_multicars_vector.begin(),
                v_target_multicars_vector.end(), v_target_multicars_min);
  id_v_target_multicars_min =
      id_vector[std::distance(v_target_multicars_vector.begin(), iter)];
  // std::cout << "id : v_target_multicars : s_prediction : TTC_pwj : TTC_ego :
  // decay_rate = "
  // << id_v_target_multicars_min << " : "
  // << v_target_multicars_min << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].s_prediction
  // << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].TTC_pwj
  // << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].TTC_ego
  // << " : "
  // <<multidirectional_cars[std::distance(v_target_multicars_vector.begin(),iter)].decay_rate
  // <<std::endl;
  // for (auto multicars : obstacles_){
  //   if (multicars->Id() == id_v_target_multicars_min){
  //     auto sl_boundary = multicars->PerceptionSLBoundaryPlanning();
  //     double speed_yaw = multicars->Speed_yaw_relative_planning_frenet();
  //     double obs_v_lat = multicars->speed() * sin(speed_yaw);
  //     double obs_v_lon = multicars->speed() * cos(speed_yaw);
  //     std::cout << "id : sl_boundary : speed = "<< multicars->Id()<<" : <"
  //               <<
  //               sl_boundary.start_s<<","<<sl_boundary.end_s<<","<<sl_boundary.start_l<<","<<sl_boundary.end_l<<">
  //               , <"
  //               <<
  //               multicars->speed()<<","<<obs_v_lat<<","<<obs_v_lon<<">"<<std::endl;
  //   }
  // }
  v_target_ = std::min(v_target_, std::max(0.0, v_target_multicars_min));

  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_multidirectional_human(
    const std::vector<MultiDirectionalHuman> multidirectional_human,
    const double v_ego) {
  std::vector<double> v_target_multihuman_vector;
  std::vector<int> id_vector;
  double v_target_multihuman_min = 40.0;
  int id_v_target_multihuman_min = -1;

  if (multidirectional_human.size() == 0)
    return false;

  for (auto multihuman : multidirectional_human) {
    double v_target_multihuman = 40.0;
    if (multihuman.decel_level == 1) {
      v_target_multihuman = std::max(0.5, v_target_ * 0.8);
    } else if (multihuman.decel_level == 2) {
      v_target_multihuman = std::max(0.5, v_target_ * 0.6);
    } else if (multihuman.decel_level == 3) {
      v_target_multihuman = std::max(0.5, v_target_ * 0.3);
    }
    v_target_multihuman_vector.push_back(v_target_multihuman);
    id_vector.push_back(multihuman.id);
  }
  v_target_multihuman_min = *std::min_element(
      v_target_multihuman_vector.begin(), v_target_multihuman_vector.end());
  vector<double>::iterator iter =
      std::find(v_target_multihuman_vector.begin(),
                v_target_multihuman_vector.end(), v_target_multihuman_min);
  id_v_target_multihuman_min =
      id_vector[std::distance(v_target_multihuman_vector.begin(), iter)];

  v_target_ = std::min(v_target_, std::max(0.0, v_target_multihuman_min));
  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_parking(
    const double dist, const double v_ego, const bool isAPA) {
  double v_target_parking = 40.0;
  double n_des = 0.0;
  double d_des = 0.0;
  n_des = std::max(0.0, dist);
  d_des = calc_desired_distance(0.0, v_ego);
  v_target_parking = calc_desired_speed(n_des, d_des, 0.0, 0.0);
  // if ((dist > static_distance_ + 0.25 + 0.1) && isAPA) {
  //   v_target_parking = std::max(v_target_parking,
  //   VehicleParam::Instance()->velocity_deadzone + 0.02);
  // }
  v_target_ = std::max(std::min(v_target_, v_target_parking), 0.0);
  return true;
}

bool ParkingLongitudinalMotionPlanner::limit_accel_velocity_in_turns(
    const double v_ego, const double angle_steers) {
  // *** this function returns a limited long acceleration allowed, depending on
  // the existing lateral acceleration
  //  this should avoid accelerating when losing the target in turns
  const double PI = 3.141592653589793238463;
  double deg_to_rad = PI / 180.0;

  double a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V);
  double a_y = std::pow(v_ego, 2) * angle_steers /
               (VehicleParam::Instance()->steer_ratio *
                VehicleParam::Instance()->wheel_base);
  double a_x_allowed =
      std::sqrt(std::max(std::pow(a_total_max, 2) - std::pow(a_y, 2), 0.0));

  // And limit the logitudinal velocity for a safe turn
  double a_y_max =
      interp(std::abs(angle_steers), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
  double v_limit_steering =
      std::sqrt((a_y_max * VehicleParam::Instance()->steer_ratio *
                 VehicleParam::Instance()->wheel_base) /
                std::max(std::abs(angle_steers), 0.001));

  v_target_ = std::min(v_target_, v_limit_steering);
  a_target_.second = std::min(a_target_.second, a_x_allowed);
  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_apa() {
  const EgoState &ego_state = world_model_->get_ego_state();
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const ParkingBehaviorPlannerOutput &parking_behavior_planner_output =
      PlanningContext::Instance()->parking_behavior_planner_output();
  const bool is_reverse =
      (planning_status.planning_result.gear == GearState::REVERSE);
  if (parking_behavior_planner_output.is_move_ready) {
    std::vector<Pose2D> traj_pose_array_ =
        planning_status.planning_result.traj_pose_array;
    double dist = DBL_MAX;
    double dist_min = DBL_MAX;
    size_t index = 0;
    size_t index_min = 0;
    for (const auto &point : traj_pose_array_) {
      index++;
      dist = std::sqrt((std::pow(point.x - ego_state.ego_pose.x, 2) +
                        std::pow(point.y - ego_state.ego_pose.y, 2)));
      if (dist < dist_min) {
        dist_min = dist;
        index_min = index;
      }
    }
    index = index_min;
    if (index == traj_pose_array_.size() || index == 0) {
      v_apa_ = 0;
      a_apa_ = 0;
    } else {
      index -= 1;
      std::vector<float> traj_vel_array_ =
          planning_status.planning_result.traj_vel_array;
      std::vector<float> traj_acceleration_ =
          planning_status.planning_result.traj_acceleration;
      MSD_LOG(INFO, "index & SIZE: %d %ld %ld", index, traj_vel_array_.size(),
              traj_acceleration_.size());
      MSD_LOG(INFO, "dist_min : %f", dist_min);
      MSD_LOG(INFO, "traj v a x y: %f %f %f %f", traj_vel_array_.at(index),
              traj_acceleration_.at(index), traj_pose_array_.at(index).x,
              traj_pose_array_.at(index).y);
      v_apa_ = traj_vel_array_.at(index);
      // MSD_LOG(INFO, "compute v_apa:  %f %f %f", traj_vel_array_.at(index),
      // traj_vel_array_.at(index+1), traj_vel_array_.at(index+2)); double s = 0
      // ; v_apa_ = traj_vel_array_.at(index) * 0.5; for (int i = index + 1;i <
      // traj_vel_array_.size(); i++ ){
      //   s += std::hypot(traj_pose_array_.at(i).x - traj_pose_array_.at(i -
      //   1).x, traj_pose_array_.at(i).y - traj_pose_array_.at(i - 1).y ); if
      //   (s > 1.0) {
      //     if (std::abs(traj_vel_array_.at(i)) < 0.1) {
      //       v_apa_ = traj_vel_array_.at(index) * 0.5;
      //     }
      //     else{
      //       v_apa_ = traj_vel_array_.at(index);
      //     }
      //     break;
      //   }
      // }

      a_apa_ = traj_acceleration_.at(index);
      // if (v_apa_ > 0) {
      //   v_apa_ = std::max(v_apa_, _V_MIN_APA);
      // }
      // else if (v_apa_ < 0){
      //   v_apa_ = std::min(v_apa_, -_V_MIN_APA);
      // }
      if (is_reverse) {
        v_apa_ = std::min(v_apa_, -_V_MIN_APA);
      } else {
        v_apa_ = std::max(v_apa_, _V_MIN_APA);
      }
    }
  } else {
    v_apa_ = 0.0;
    a_apa_ = 0.0;
  }
  MSD_LOG(INFO, " v_target_, v_apa_, a_apa_: %f %f %f\n", v_target_, v_apa_,
          a_apa_);
  if (std::abs(v_apa_) < v_target_) {
    v_target_ = v_apa_;
    if (v_apa_ < 0.01 && v_apa_ > -0.01 && a_apa_ < 0.01 && a_apa_ > -0.01) {
      if (is_reverse) {
        a_apa_ = 1.0;
      } else {
        a_apa_ = -1.0;
      }
    }
    if (is_reverse) {
      a_target_.first = std::min(a_target_.first, -a_apa_);
      a_target_.second = std::max(a_target_.second, -a_apa_);
    } else {
      a_target_.first = std::min(a_target_.first, a_apa_);
      a_target_.second = std::max(a_target_.second, a_apa_);
    }
  } else {
    if (std::abs(v_target_) < 0.1) {
      a_apa_ = 0.0;
    }
    if (v_apa_ < 0) {
      v_target_ = -v_target_;
    }
  }
  return true;
}

bool ParkingLongitudinalMotionPlanner::limit_speed_before_turns(
    const double v_ego, const int scene_avp) {
  const MapInfo &map_info = world_model_->get_map_info();
  double dist_to_crossing = map_info.distance_to_crossing;
  double ego_s = world_model_->get_ego_state().ego_frenet.x +
                 VehicleParam::Instance()->back_edge_to_center;
  // for (int i=0;i<25;++i){
  //   double check_s = ego_s + 0.4 * i;
  //   // std::cout << "ahead s : ego_s : is intersection =" <<check_s <<" : "<<
  //   ego_s <<" : "<< world_model_->cal_intersection(check_s) <<std::endl;
  //   if(!world_model_->cal_intersection(check_s))
  //     continue;
  //   else{
  //     double v_limit_before_turns = interp(check_s -
  //     ego_s,_DIST_TURNS_BP,_DIST_TURNS_V); v_target_ = std::min(v_target_,
  //     v_limit_before_turns); break;
  //   }
  // }
  // std::cout <<"v_target_before_intersection := " << v_target_ <<std::endl;
  // std::cout <<"dist_to_crossing = "<<dist_to_crossing<<std::endl;
  if (dist_to_crossing < -0.0)
    return true;
  double v_limit_before_turns =
      interp(dist_to_crossing, _DIST_TURNS_BP, _DIST_TURNS_V);
  v_target_ = std::min(v_target_, v_limit_before_turns);
  return true;
}

bool ParkingLongitudinalMotionPlanner::limit_speed_for_large_curvature(
    const double v_ego, const int scene_avp) {
  // const EgoState &ego_state = world_model_->get_ego_state();

  double v_limit_for_large_curvature = 10.0 / 3.6;
  auto planning_result =
      PlanningContext::Instance()->planning_status().planning_result;
  bool is_ramp = PlanningContext::Instance()->has_scene(
      scene_avp, ParkingSceneType::SCENE_RAMP);

  std::vector<float> traj_curvature = planning_result.traj_curvature;
  std::vector<float> traj_curvature_radius =
      planning_result.traj_curvature_radius;
  std::vector<float> traj_relative_s = planning_result.traj_relative_s;
  // std::cout << "lxrdebug: size of curvature : radius : s = "
  // <<traj_curvature.size() <<" : "<<traj_curvature_radius.size()<<" :
  // "<<traj_relative_s.size()<<std::endl;
  if (traj_curvature_radius.empty())
    return false;
  if (traj_curvature_radius.size() <= 10)
    return false;
  double curv_radius_thres = 10.0;
  if (traj_curvature_radius.at(10) <= curv_radius_thres) {
    v_limit_for_large_curvature = 10.0 / 3.6;
    v_target_ = std::min(v_target_, v_limit_for_large_curvature);
    return true;
  }
  int start_index = -1, end_index = -1;
  int curv_length = 0;
  // std::vector<float>::iterator it;
  // it =
  // std::find_if(traj_curvature_radius.begin(),traj_curvature_radius.end(),*traj_curvature_radius)
  for (size_t i = 10; i < traj_curvature_radius.size(); ++i) {
    if (traj_curvature_radius.at(i) < curv_radius_thres && curv_length == 0) {
      start_index = i;
      ++curv_length;
      continue;
    }
    if (traj_curvature_radius.at(i) < curv_radius_thres) {
      ++curv_length;

    } else if (curv_length != 0) {
      end_index = i;
      break;
    }
  }
  // std::cout <<"lxrdebug  start_idx:end_idx:length = "<<start_index<<" :
  // "<<end_index<<" : "<<curv_length<< std::endl;

  if (start_index == -1 || PlanningContext::Instance()
                                   ->parking_behavior_planner_output()
                                   .planner_type == PlannerType::OPENSPACE) {
    v_limit_for_large_curvature = 10.0 / 3.6;
  } else {
    // double ego_s = world_model_->get_ego_state_planning().ego_frenet.x;
    // std::cout << "lxrdebug: ego_s= " << ego_s <<std::endl;
    // std::cout << "lxrdebug: curvature start s= " <<
    // planning_result.traj_relative_s.at(start_index) <<std::endl;
    if (planning_result.traj_relative_s.size() <= start_index)
      return false;
    else {
      v_limit_for_large_curvature =
          interp(planning_result.traj_relative_s.at(start_index) -
                     planning_result.traj_relative_s.at(10),
                 is_ramp ? _DIST_TURNS_BP_RAMP : _DIST_TURNS_BP, _DIST_TURNS_V);
    }
    // std::cout << "lxrdebug: v_limit_for_large_curvature = " <<
    // v_limit_for_large_curvature <<std::endl;
  }
  v_target_ = std::min(v_target_, std::max(is_ramp ? 0.0 : (v_ego - 0.5),
                                           v_limit_for_large_curvature));
  return true;
}

bool ParkingLongitudinalMotionPlanner::limit_speed_for_avoid_obstacle(
    const double v_ego) {
  double ego_s = world_model_->get_ego_state_planning().ego_frenet.x +
                 VehicleParam::Instance()->front_edge_to_center;
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  std::vector<int> obs_id_considered;
  if (!PlanningContext::Instance()
           ->mutable_parking_lateral_behavoir_planner_output()
           ->lateral_planning_info.empty()) {
    obs_id_considered = PlanningContext::Instance()
                            ->mutable_parking_lateral_behavoir_planner_output()
                            ->lateral_planning_info.at("normal")
                            .sidepass_static_obj;
    obs_id_considered.insert(
        obs_id_considered.end(),
        PlanningContext::Instance()
            ->mutable_parking_lateral_behavoir_planner_output()
            ->lateral_planning_info.begin()
            ->second.sidepass_dynamic_obj.begin(),
        PlanningContext::Instance()
            ->mutable_parking_lateral_behavoir_planner_output()
            ->lateral_planning_info.begin()
            ->second.sidepass_dynamic_obj.end());
  }
  SLBoundary sl_boundary;
  double v_avoid_obs = 40.0;
  for (const int &obs_id : obs_id_considered) {
    auto obs = world_model_->obstacle_manager().find_obstacle(obs_id);
    if (obs != nullptr) {
      sl_boundary = obs->PerceptionSLBoundaryPlanning();
      if (std::abs((sl_boundary.start_l + sl_boundary.end_l) / 2) < 4.0 &&
          obs->IsLonOppositeWrtFrenet() == 1 &&
          (sl_boundary.start_s + sl_boundary.end_s) / 2 - ego_s > -2.0 &&
          (sl_boundary.start_s - ego_s) < 10.0) {
        v_avoid_obs = std::min(v_avoid_obs, std::max(v_ego - 0.3, 1.39));
      }
    }
  }
  obs_id_considered.clear();
  v_target_ = std::min(v_target_, v_avoid_obs);
  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_intention_obstacle(
    const std::vector<IntentionStatusObstacles> intention_status_obstacles,
    const double v_ego) {

  double v_intention_obs = 40.0;
  obstacles_ = world_model_->obstacle_manager().get_obstacles().Items();
  double ego_s = world_model_->get_ego_state().ego_frenet.x +
                 VehicleParam::Instance()->front_edge_to_center;

  for (const auto &intention_obs : intention_status_obstacles) {
    SLBoundary sl_boundary;
    double v_intention_obs_tmp = 40.0;
    if (intention_obs.type == IntentionObsType::HIGHSPEEDDECEL ||
        intention_obs.type == IntentionObsType::PULLOVER) {
      auto obs = world_model_->mutable_obstacle_manager().find_obstacle(
          intention_obs.id);
      if (obs != nullptr) {
        if (sl_boundary.start_s - ego_s > 4.0) {
          sl_boundary = obs->PerceptionSLBoundary();
          double speed_yaw = obs->Speed_yaw_relative_frenet();
          double obs_v_lat = obs->speed() * sin(speed_yaw);
          double obs_v_lon = obs->speed() * cos(speed_yaw);
          double d_des =
              calc_desired_distance(obs_v_lon, v_ego, false, false, true,
                                    IntentionObsType::HIGHSPEEDDECEL);
          v_intention_obs_tmp = calc_desired_speed(sl_boundary.start_s - ego_s,
                                                   d_des, obs_v_lon, 0.0);
          if (v_intention_obs_tmp < v_intention_obs) {
            v_intention_obs = v_intention_obs_tmp;
            // std::cout <<"id  d_des v_special = "<<intention_obs.id<<"
            // "<<d_des<<" "<<v_intention_obs_tmp<<std::endl;
          }
        }
      }
    }

    if (intention_obs.type == IntentionObsType::APA ||
        intention_obs.type == IntentionObsType::APOA) {
      auto obs = world_model_->mutable_obstacle_manager().find_obstacle(
          intention_obs.id);
      if (obs != nullptr) {
        sl_boundary = obs->PerceptionSLBoundary();
        if (sl_boundary.start_s - ego_s > 4.0) {
          double d_des = calc_desired_distance(
              0.0, v_ego, false, false, true, IntentionObsType::HIGHSPEEDDECEL);
          v_intention_obs_tmp =
              calc_desired_speed(sl_boundary.start_s - ego_s, d_des, 0.0, 0.0);

          if (v_intention_obs_tmp < v_intention_obs) {
            v_intention_obs = v_intention_obs_tmp;
            // std::cout <<" id d_des v_special = "<<intention_obs.id <<"
            // "<<d_des<<" "<<v_intention_obs_tmp<<std::endl;
          }
        }
      }
    }
  }

  v_target_ = std::max(std::min(v_intention_obs, v_target_), 0.0);

  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_prediction_obstacle(
    const std::vector<int> prediction_obstacles, const double v_ego) {
  double v_prediction_obs = 40.0;
  int prediction_obs_id = -1;
  if (prediction_obstacles.size() == 0) {
    return true;
  }
  for (int i = 0; i < prediction_obstacles.size(); i++) {

    auto obstacle_i = world_model_->mutable_obstacle_manager().find_obstacle(
        prediction_obstacles.at(i));
    if (obstacle_i == nullptr) {
      continue;
    }
    PseudoPredictionTrajectory prediction_traj =
        obstacle_i->GetPseudoPredictionTraj();
    // prebrake for opposite car sidepass
    if (prediction_traj.direction == -1) {
      double v_prediction_obs_tmp_oppo = 40.0;
      v_prediction_obs_tmp_oppo =
          interp(prediction_traj.meet_s, _PREDICTION_BP, _PREDICTION_V);
      if (v_prediction_obs_tmp_oppo < v_prediction_obs) {
        prediction_obs_id = prediction_traj.id;
        v_prediction_obs = v_prediction_obs_tmp_oppo;
        // std::cout <<"obs_id meets TTC = "<<prediction_obs_id<<"
        // "<<prediction_traj.meet_s<<" "<<prediction_traj.TTC<<std::endl;
      }
    }
    // stop for highspeed car or pedestrain that crossing frenet before egocar
    else if (prediction_traj.direction == 1) {
      double v_prediction_obs_tmp_same = 40.0;
      int idx_cross = -1;
      for (int i = 0; i < (int)prediction_traj.pose.size() - 1; i++) {
        if (prediction_traj.relative_sl.at(i).y *
                prediction_traj.relative_sl.at(i + 1).y <
            0) {
          idx_cross = i;
          break;
        }
      }
      if (idx_cross != -1) { // prediction trajectory will cross frenet at
                             // s_prediction in TTC_obs
        double TTC_obs = prediction_traj.time_series.at(idx_cross);
        double TTC_ego = prediction_traj.relative_sl.at(idx_cross).x / v_ego;
        double s_prediction = prediction_traj.relative_sl.at(idx_cross).x;

        if (s_prediction < 4.0 && TTC_obs < 2.5 &&
            std::abs(TTC_obs - TTC_ego) <
                2.0) { // currently use same strategy for car and pedestrain
          v_prediction_obs_tmp_same = 0.0;
        }
        if (v_prediction_obs_tmp_same < v_prediction_obs) {
          prediction_obs_id = prediction_traj.id;
          v_prediction_obs = v_prediction_obs_tmp_same;
        }
      }
    }
  }
  // std::cout<<" v_target_ v_prediction_obs = "<<v_target_<<"
  // "<<v_prediction_obs<<std::endl;
  v_target_ = std::max(std::min(v_target_, v_prediction_obs), 0.0);

  return true;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_gate(
    const double v_ego) {
  // const PlanningResult &planning_result = planning_status.planning_result;
  const ObstacleManager &obs_manager = world_model_->obstacle_manager();
  double ego_s = world_model_->get_ego_state().ego_frenet.x +
                 VehicleParam::Instance()->front_edge_to_center;
  double v_gate = 40.0;
  for (const auto &obs : obs_manager.get_gates().Items()) {
    // std::cout <<"lxrdebug: Gate!!!"<<std::endl;
    SLBoundary sl_boundary = obs->PerceptionSLBoundary();
    // std::cout <<"lxrdebug: ego_s slboundary"<<ego_s<<"
    // "<<sl_boundary.start_s<<" "<<sl_boundary.end_s<<"
    // "<<sl_boundary.start_l<<" "<<sl_boundary.end_l<<std::endl;
    if ((sl_boundary.end_s - ego_s > -2) && (sl_boundary.start_s - ego_s < 7) &&
        (std::abs(sl_boundary.start_l) < 5 ||
         std::abs(sl_boundary.end_l) < 5)) {
      v_gate = 1.39;
    }
  }
  v_target_ = std::min(v_target_, v_gate);

  return true;
}

bool ParkingLongitudinalMotionPlanner::
    limit_speed_for_radical_cars_while_APA() {
  auto frenet_coord_ = world_model_->get_frenet_coord();
  if (frenet_coord_ == nullptr) {
    return false;
  }
  const double PI = 3.141592653589793238463;
  double ego_s = world_model_->get_ego_state().ego_frenet.x;
  double ego_s_front = ego_s + VehicleParam::Instance()->front_edge_to_center;
  double ego_s_back = ego_s - VehicleParam::Instance()->back_edge_to_center;

  double right_border_dist = -planning_math::interps(
      world_model_->get_refline_manager()->get_right_road_border_distance(),
      world_model_->get_refline_manager()->get_s(), ego_s);
  double left_border_dist = planning_math::interps(
      world_model_->get_refline_manager()->get_left_road_border_distance(),
      world_model_->get_refline_manager()->get_s(), ego_s);
  // std::cout <<"obsdebug rigth left border = "<<right_border_dist<<"
  // "<<left_border_dist<<std::endl; std::cout <<"obsdebug offset front back =
  // "<< VehicleParam::Instance()->front_edge_to_center *
  // sin(world_model_->get_ego_state().ego_pose.theta -
  // frenet_coord_->GetRefCurveHeading(ego_s))<<" "<<
  // VehicleParam::Instance()->back_edge_to_center *
  // sin(world_model_->get_ego_state().ego_pose.theta -
  // frenet_coord_->GetRefCurveHeading(ego_s))<<std::endl;
  double right_border_dist_front =
      std::abs(right_border_dist -
               VehicleParam::Instance()->front_edge_to_center *
                   sin(world_model_->get_ego_state().ego_pose.theta -
                       frenet_coord_->GetRefCurveHeading(ego_s))) -
      VehicleParam::Instance()->width *
          cos(world_model_->get_ego_state().ego_pose.theta -
              frenet_coord_->GetRefCurveHeading(ego_s)) /
          2.0;
  double left_border_dist_front =
      std::abs(-left_border_dist +
               VehicleParam::Instance()->front_edge_to_center *
                   sin(world_model_->get_ego_state().ego_pose.theta -
                       frenet_coord_->GetRefCurveHeading(ego_s))) -
      VehicleParam::Instance()->width *
          cos(world_model_->get_ego_state().ego_pose.theta -
              frenet_coord_->GetRefCurveHeading(ego_s)) /
          2.0;
  double right_border_dist_back =
      std::abs(right_border_dist -
               VehicleParam::Instance()->back_edge_to_center *
                   sin(world_model_->get_ego_state().ego_pose.theta -
                       frenet_coord_->GetRefCurveHeading(ego_s))) -
      VehicleParam::Instance()->width *
          cos(world_model_->get_ego_state().ego_pose.theta -
              frenet_coord_->GetRefCurveHeading(ego_s)) /
          2.0;
  double left_border_dist_back =
      std::abs(-left_border_dist +
               VehicleParam::Instance()->back_edge_to_center *
                   sin(world_model_->get_ego_state().ego_pose.theta -
                       frenet_coord_->GetRefCurveHeading(ego_s))) -
      VehicleParam::Instance()->width *
          cos(world_model_->get_ego_state().ego_pose.theta -
              frenet_coord_->GetRefCurveHeading(ego_s)) /
          2.0;
  // std::cout <<"obsdebug right left = "<<right_border_dist_front<<"
  // "<<left_border_dist_front<<" "<<right_border_dist_back<<"
  // "<<left_border_dist_back<<std::endl;
  if (std::min(right_border_dist_front, right_border_dist_back) > 2.5 ||
      std::min(left_border_dist_front, left_border_dist_back) > 2.5) {
    for (const auto &obs :
         world_model_->obstacle_manager().get_obstacles().Items()) {
      if ((std::abs(std::fmod(obs->GetCartePosewrtEgo().theta, PI)) >
           PI / 6.0) ||
          (obs->Type() == ObjectType::PEDESTRIAN))
        continue;
      if ((obs->IsInRoad() || obs->IsAcrossRoadBorder()) &&
          (obs->IsLonStaticWrtFrenet() != 1 ||
           obs->IsLatStaticWrtFrenet() != 1)) {
        SLBoundary sl_boundary;
        sl_boundary = obs->PerceptionSLBoundary();
        double speed_yaw = obs->Speed_yaw_relative_frenet();
        double obs_v_lat = obs->PerceptionSpeed() * sin(speed_yaw);
        double obs_v_lon = obs->PerceptionSpeed() * cos(speed_yaw);
        if ((ego_s_back - sl_boundary.end_s < 5.0 &&
             ego_s_front > sl_boundary.start_s && obs_v_lon > 0.5) ||
            (sl_boundary.start_s - ego_s_front < 5.0 &&
             sl_boundary.end_s > ego_s_back && obs_v_lon < -0.5)) {
          v_target_ = 0.0;
        }
      }
    }
  }

  return true;
}

bool ParkingLongitudinalMotionPlanner::limit_speed_for_mpc_deviation(
    const std::vector<double> planning_mpc_diff) {
  if (planning_mpc_diff.empty())
    return true;
  double max_deviation =
      *std::max_element(planning_mpc_diff.begin(), planning_mpc_diff.end());
  double average_deviation =
      std::accumulate(planning_mpc_diff.begin(), planning_mpc_diff.end(), 0.0) /
      planning_mpc_diff.size();
  // std::cout<<"max average deviation = "<<max_deviation<<"
  // "<<average_deviation<<std::endl;
  if (max_deviation > 0.30) {
    v_target_ = 0.5;
  }
  return true;
}

double ParkingLongitudinalMotionPlanner::process_a_lead(const double a_lead) {
  double a_lead_threshold = 0.5;
  return std::min(a_lead + a_lead_threshold, 0.0);
}

double ParkingLongitudinalMotionPlanner::calc_desired_distance(
    const double v_lead, const double v_ego, const bool isAPA,
    const bool is_freespace, const bool is_static, IntentionObsType type) {
  double t_gap = interp(v_ego, _T_GAP_VEGO_BP, _T_GAP_VEGO_V);
  double safety_factor =
      is_static ? 1.0 : 3.0; // double safety dist for non-static leadcar
  double safety_distance =
      isAPA ? CarParams::GetInstance()->lon_inflation_min * safety_factor
            : std::max(CarParams::GetInstance()->lon_inflation_min, 0.2) *
                  safety_factor;
  // if (is_in_parking_slot()) {
  //   safety_distance -= 0.35;
  // }
  double d_offset = is_freespace ? safety_distance
                                 : (isAPA ? safety_distance : static_distance_);
  if (type == IntentionObsType::HIGHSPEEDDECEL ||
      type == IntentionObsType::PULLOVER) {
    d_offset = 12.0;
  } else if (type == IntentionObsType::APA || type == IntentionObsType::APOA) {
    d_offset = 7.0;
  }
  d_offset += std::max(-v_lead, 0.0) * 1.0;
  return d_offset + std::max(v_lead, 0.0) * t_gap;
}

double ParkingLongitudinalMotionPlanner::calc_desired_speed(
    const double d_lead, const double d_des, const double v_lead_unp,
    const double a_lead_p) {
  // *** compute desired speed ***
  // the desired speed curve is divided in 4 portions:
  // 1-constant
  // 2-linear to regain distance
  // 3-linear to shorten distance
  // 4-parabolic (constant decel)
  const double max_runaway_speed = -2.;

  double v_lead = std::max(v_lead_unp, 0.0);

  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);

  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));
  double v_rel_des = 0.0;

  double delta_d = d_lead - d_des;
  if (delta_d > 0.1) {
    if (std::fabs(l_slope) > 1e-5) {
      delta_d +=
          VehicleParam::Instance()      // parasoft-suppress AUTOSAR-A5_6_1
                  ->velocity_deadzone / // parasoft-suppress AUTOSAR-A5_6_1
              l_slope - // parasoft-suppress AUTOSAR-A5_6_1 "f-drop"
          0.1;
    } else {
      delta_d += VehicleParam::Instance()->velocity_deadzone / 1e-5 - 0.1;
    }
  }
  if (delta_d < 0.0) {
    double v_rel_des_1 = (-max_runaway_speed) / d_des * delta_d;
    double v_rel_des_2 = delta_d * l_slope / 3.0;
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else if (delta_d < x_linear_to_parabola) {
    v_rel_des = delta_d * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else {
    v_rel_des = std::sqrt(2 * (delta_d - x_parabola_offset) * p_slope);
  }
  double v_target = v_rel_des + v_lead;

  return v_target;
}

bool ParkingLongitudinalMotionPlanner::calc_acc_accel_limits(
    const double d_lead, const double d_des, const double v_ego,
    const double v_lead_unp, const double a_lead, const double v_target,
    std::pair<double, double> &a_target) {
  double v_lead = std::max(v_lead_unp, 0.0);
  double v_rel = v_lead - v_ego;

  v_rel *= -1;

  // this is how much lead accel we consider in assigning the desired decel
  double a_lead_contr =
      a_lead * interp(v_lead, _A_LEAD_LOW_SPEED_BP, _A_LEAD_LOW_SPEED_V) * 0.8;

  // first call of calc_positive_accel_limit is used to shape v_pid
  a_target.second = calc_positive_accel_limit(
      d_lead, d_des, v_ego, v_rel, v_target, a_lead_contr, a_target.second);
  // *** compute max decel ***
  // 1.  # assume the car is 1m/s slower
  double v_offset = 0.85;
  // assume the distance is 1m lower
  double d_offset = 0.85 + 0.2 * v_ego;
  if (v_target - v_ego > 0.5) {
    // acc target speed is above vehicle speed, so we can use the cruise limits
    // pass
  } else {
    // add small value to avoid by zero divisions
    // compute needed accel to get to 1m distance with -1m/s rel speed
    double decel_offset = interp(v_lead, _DECEL_OFFSET_BP, _DECEL_OFFSET_V);

    double critical_decel =
        calc_critical_decel(d_lead, v_rel, d_offset, v_offset);
    a_target.first =
        std::min(decel_offset + critical_decel + a_lead_contr, a_target.first);
  }
  // a_min can't be higher than a_max
  a_target.first = min(a_target.first, a_target.second);
  // final check on limits
  a_target.first = clip(a_target.first, _A_MIN, _A_MAX);
  a_target.second = clip(a_target.second, _A_MIN, _A_MAX);
  return true;
}

double ParkingLongitudinalMotionPlanner::calc_positive_accel_limit(
    const double d_lead, const double d_des, const double v_ego,
    const double v_rel, const double v_target, const double a_lead_contr,
    const double a_max_const) {
  double a_coast_min = -1.0;
  double a_max = a_max_const;
  // never coast faster then -1m/s^2
  // coasting behavior above v_coast. Forcing a_max to be negative will force
  // the pid_speed to decrease, regardless v_target
  if (v_ego > v_target + 0.5) {
    // for smooth coast we can be aggressive and target a point where car would
    // actually crash
    double v_offset_coast = 1.0;
    double d_offset_coast = d_des / 2.0 - 4.0;

    // acceleration value to smoothly coast until we hit v_target
    if (d_lead > d_offset_coast + 0.1) {
      double a_coast =
          calc_critical_decel(d_lead, v_rel, d_offset_coast, v_offset_coast);
      // if lead is decelerating, then offset the coast decel
      a_coast += a_lead_contr;
      a_max = std::max(a_coast, a_coast_min);
    } else {
      a_max = a_coast_min;
    }
  } else {
    // same as cruise accel, plus add a small correction based on relative lead
    // speed if the lead car is faster, we can accelerate more, if the car is
    // slower, then we can reduce acceleration
    a_max = a_max + interp(v_ego, _A_CORR_BY_SPEED_BP, _A_CORR_BY_SPEED_V) *
                        clip(-v_rel / 4.0, -0.5, 1.0);
  }
  return a_max;
}

double ParkingLongitudinalMotionPlanner::calc_critical_decel(
    const double d_lead, const double v_rel, const double d_offset,
    const double v_offset) {
  // this function computes the required decel to avoid crashing, given safety
  // offsets
  double a_critical = -std::pow(std::max(0.0, v_rel + v_offset), 2) /
                      std::max(2 * (d_lead - d_offset), 0.5);
  return a_critical;
}

double ParkingLongitudinalMotionPlanner::clip(const double x, const double lo,
                                              const double hi) {
  return std::max(lo, std::min(hi, x));
}

bool ParkingLongitudinalMotionPlanner::calc_cruise_accel_limits(
    const double v_ego) {
  a_target_.first = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V);
  a_target_.second = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V);
  MSD_LOG(ERROR, "a_target_.first: %.3f, a_target_.second: %.3f", a_target_.first, a_target_.second);

  return true;
}

double ParkingLongitudinalMotionPlanner::interp(double x,
                                                const std::vector<double> &xp,
                                                const std::vector<double> &fp) {
  const int N = xp.size() - 1;
  if (N < 0) {
    return 0.0;
  }

  if (x < xp[0]) {
    return fp[0];
  }
  for (int i = 0; i <= N; ++i) {
    if (x < xp[i]) {
      return ((x - xp[i - 1]) * (fp[i] - fp[i - 1]) / (xp[i] - xp[i - 1]) +
              fp[i - 1]);
    }
  }

  return fp[N];
}

bool ParkingLongitudinalMotionPlanner::is_in_parking_slot() {
  if (world_model_->is_parking_svp()) {
    return world_model_->get_map_info().current_parking_lot_id != 0;
  } else {
    const ParkingMapInfo &parking_map_info =
        world_model_->get_parking_map_info();
    std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results =
        parking_map_info.parking_lots_detection_fusion_results;
    for (auto &parking_lot : parking_lots_detection_fusion_results) {
      if (parking_lot.is_car_in) {
        return true;
      }
    }
    return false;
  }
  return false;
}

void ParkingLongitudinalMotionPlanner::set_planning_result(const double v_ego) {

  // const EgoState &ego_state = world_model_->get_ego_state();
  // const MapInfo &map_info = world_model_->get_map_info();
  // const StatusType &status_type =
  // PlanningContext::Instance()->planning_status().scenario.status_type;
  blocker_timer_.set_timer_duration(block_timeout_duration_);
  PlanningStatus *planning_status =
      PlanningContext::Instance()->mutable_planning_status();
  planning_status->blocked =
      PlanningContext::Instance()->planning_status().control_block_feedback ||
      fs_blocked_;
  planning_status->blocked_timeout = blocker_timer_.is_timeout();
  if (blocker_timer_.is_timeout()) {
    planning_status->fs_block =
        PlanningContext::Instance()->planning_status().control_block_feedback;
    planning_status->pullover_enable = pullover_enable_;
    if (planning_status->fs_block) {
      PlanningContext::Instance()
          ->mutable_longitudinal_behavior_planner_output()
          ->last_block_fs = Point2D(PlanningContext::Instance()
                                        ->longitudinal_behavior_planner_output()
                                        .free_space.x,
                                    PlanningContext::Instance()
                                        ->longitudinal_behavior_planner_output()
                                        .free_space.y);
    }
    // planning_status->pullover_enable = false;
  } else if (pullover_enable_) {
    planning_status->pullover_enable = pullover_enable_;
  }
  planning_status->collide_to_sth = parking_stopper_timer_.is_timeout();
  planning_status->block_timeout_duration = block_timeout_duration_;
  planning_status->block_time = blocker_timer_.get_count();
  planning_status->stopping =
      (std::abs(v_target_) < VehicleParam::Instance()->velocity_deadzone);
  PlanningResult &planning_result = planning_status->planning_result;

  planning_result.v_array.clear();
  planning_result.a_array.clear();
  double acc;
  // if (status_type == StatusType::APA || status_type == StatusType::APOA){
  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .planner_type == PlannerType::OPENSPACE) {
    acc = a_apa_;
  } else {
    acc = 0;
    // if (v_ego > v_target_) {
    //   acc = a_target_.first;
    // }
    // else {
    //   acc = a_target_.second;
    // }
  }
  // v_target_ = 2.8;
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.bag_recorder_filter_scenario.abrupt_brake =
      (bool)(a_target_.first < -1.0);

  if (std::abs(v_ego) <= 0.05 &&
      std::abs(v_target_) > VehicleParam::Instance()->velocity_deadzone) {
    a_target_.second = clip(abs(v_target_), a_target_.second, 0.4);
  }
  vec_target_speed_debug_.emplace_back(current_v_);
  ++current_v_count_debug_;

  if (std::abs(current_v_ - v_target_) < 1e-2) {
    current_jerk_ = 0.0;
    current_a_ = 0.0;
    current_v_ = v_target_;
  } else {
    current_jerk_ = (std::abs(v_target_) > std::abs(current_v_)) ? 1.0 : -1.0;
    current_a_ = clip(current_a_ + current_jerk_ / rate_,
                      std::min(a_target_.first, current_a_),
                      std::max(a_target_.second, current_a_));

    if (planning_result.gear == GearState::DRIVE) {
      current_v_ = std::min(std::max(v_target_, v_ego), current_v_);
      if ((current_v_ - v_target_) *
              (current_v_ + current_a_ / rate_ - v_target_) <
          0.0) {
        current_v_ = v_target_;
      } else {
        current_v_ = std::max(current_v_ + current_a_ / rate_, 0.0);
      }
    } else {
      current_v_ = std::max(std::min(v_target_, v_ego), current_v_);
      if ((current_v_ - v_target_) *
              (current_v_ - current_a_ / rate_ - v_target_) <
          0.0) {
        current_v_ = v_target_;
      } else {
        current_v_ = std::min(current_v_ - current_a_ / rate_, 0.0);
      }
    }
  }
  vec_target_speed_debug_.emplace_back(current_v_);
  ++current_v_count_debug_;
  create_planning_debug_string(a_target_);

  bool no_acc_limit = msquare::CarParams::GetInstance()->car_config.sv_config.no_acc_limit;
  if(no_acc_limit){
    current_v_ = v_target_;
  }


  planning_result.a_array.push_back(a_target_.first);
  planning_result.a_array.push_back(a_target_.second);
  planning_result.a_target_min = a_target_.first;
  planning_result.a_target_max = a_target_.second;
  planning_result.a_target = acc;

  planning_result.v_array.push_back(current_v_);
  planning_result.v_target = current_v_;
}

bool ParkingLongitudinalMotionPlanner::create_planning_debug_string(
    const std::pair<double, double> &a_target) {
  std::string debug_info;

  debug_info += "\n[sp]tgt_v{";
  for (size_t i = 0; i < vec_target_speed_debug_.size(); ++i) {
    // add current_v sign
    if (i == (vec_target_speed_debug_.size() - current_v_count_debug_))
      debug_info += "(";
    // add compute type
    if (i == 1)
      debug_info += "ld";
    if (i == 2)
      debug_info += "pt";
    if (i == 3)
      debug_info += "li";
    if (i == 4)
      debug_info += "sd";
    if (i == 7)
      debug_info += "len";
    if (i == 10)
      debug_info += "apa";
    if (i == 11)
      debug_info += "ux";
    debug_info += 
        std::to_string(vec_target_speed_debug_.at(i)).substr(0, 5) + ",";
  }
  debug_info += ")}";

  debug_info += ",tgt_a(" + std::to_string(a_target.first).substr(0, 5) +
                std::to_string(a_target.second).substr(0, 5) + ")";

  *PlanningContext::Instance()->mutable_planning_debug_info() += debug_info;
  return true;
}

} // namespace parking

} // namespace msquare
