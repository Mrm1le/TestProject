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


bool ParkingLongitudinalMotionPlanner::computeVelV2(
    const LeaderPair lead_cars, const FreespacePoint freespace,
    const FreespaceLine fs_line,
    const std::vector<MultiDirectionalCars> multidirectional_cars,
    const std::vector<MultiDirectionalHuman> multidirectional_human,
    const std::vector<IntentionStatusObstacles> intention_status_obstacles,
    const std::vector<int> prediction_obstacles, const double v_ego,
    const double angle_steers, const StatusType status_type,
    const bool is_entrance, const int scene_avp,
    const std::vector<double> planning_mpc_diff){
  auto &parking_behavior_planner_output =
      PlanningContext::Instance()->parking_behavior_planner_output();
  auto &lbpo = PlanningContext::Instance()->longitudinal_behavior_planner_output();
  const bool is_openspace =
      (parking_behavior_planner_output.planner_type == PlannerType::OPENSPACE);
  const auto& planning_result =
    PlanningContext::Instance()->planning_status().planning_result;
  auto& lon_config = msquare::CarParams::GetInstance()->car_config.lon_config;
  double min_finish_len = msquare::CarParams::GetInstance()->car_config.common_config.min_finish_len;
  bool is_reverse = (planning_result.gear == GearState::REVERSE);
  MSD_LOG(ERROR, "v_ego: %.3f", v_ego);
  MSD_LOG(ERROR, "compute v_target_= %.3f", v_target_);
  (void)calc_cruise_accel_limits(v_ego);
  MSD_LOG(ERROR, "calc_cruise_accel_limits v_target_= %.3f", v_target_);
  // (void)limit_accel_velocity_in_turns(v_ego, angle_steers);
  a_target_objective_ = a_target_;
  if (status_type == StatusType::WAIT || status_type == StatusType::DONE) {
    v_target_ = 0.0;
    return true;
  }

  bool is_stop_status = false;

  if ((PlanningContext::Instance()
          ->planning_status()
          .planning_result.gear_changing 
      && lbpo.traj_length < min_finish_len) ||
      ((int)world_model_->get_gear_report().gear_status.value) !=
          ((int)PlanningContext::Instance()
               ->planning_status()
               .planning_result.gear +
           1)) {
    v_target_ = 0.0;
    MSD_LOG(ERROR, "gear changing is: %d ,  real gear is: %d , cur gear is %d, traj_length is: %f", 
          PlanningContext::Instance()
          ->planning_status()
          .planning_result.gear_changing,
          (int)world_model_->get_gear_report().gear_status.value,
          (int)PlanningContext::Instance()
               ->planning_status()
               .planning_result.gear, lbpo.traj_length);
    is_stop_status = true;
    MSD_LOG(ERROR, "gear changing is: %d ,  real gear is: %d , cur gear is %d", 
          PlanningContext::Instance()
          ->planning_status()
          .planning_result.gear_changing,
          (int)world_model_->get_gear_report().gear_status.value,
          (int)PlanningContext::Instance()
               ->planning_status()
               .planning_result.gear,lbpo.traj_length);
    MSD_LOG(ERROR, "set 0 as gear not match");
    // return true;
  }

  if(lbpo.traj_length < 0.1){
    v_target_ = 0.0;
    MSD_LOG(ERROR, "set 0 as traj_length < 0.1");
  }

  if (world_model_->get_pause_status() || world_model_->get_brake_override()) {
    MSD_LOG(ERROR, "set 0 as get_pause_status or get_brake_override");
    is_stop_status = true;
    v_target_ = 0.0;
  }

  if (parking_behavior_planner_output.is_finish) {
    is_stop_status = true;
    v_target_ = 0.0;
    MSD_LOG(ERROR, "set 0 as is_finish");
  }

  (void)compute_speed_with_leadsV2(lead_cars, v_ego, is_openspace);
  MSD_LOG(ERROR, "compute_speed_with_leads v_target_= %.3f", v_target_);

  compute_speed_for_remain_distanceV2(lead_cars, freespace, fs_line, is_reverse);
  MSD_LOG(ERROR, "compute_speed_for_remain_distance v_target_: %f", v_target_);

  // compute_speed_for_curvature();
  // MSD_LOG(ERROR, "compute_speed_for_curvature v_target_: %f", v_target_);

  v_target_ = std::max(v_target_, 0.0);
  MSD_LOG(ERROR, "limit_speed_for_margin_limit: before v_target_ = %.3f", v_target_);

  limit_speed_for_margin_limit();
  MSD_LOG(ERROR, "limit_speed_for_margin_limit: v_target_ = %.3f", v_target_);

  compute_speed_use_osqp(lead_cars, freespace, fs_line, is_reverse, is_stop_status);
  if (PlanningContext::Instance()
          ->longitudinal_behavior_planner_output()
          .deviated &&
      !(world_model_->is_parking_lvp() ||
        world_model_->is_parking_apa() && status_type == StatusType::APA)) {
    v_target_ = std::min(v_target_, 0.5);
  }

  if (!PlanningContext::Instance()->has_scene(scene_avp,
                                              ParkingSceneType::SCENE_RAMP)) {
    (void)compute_speed_for_multidirectional_cars_frenet(multidirectional_cars,
                                                         v_ego);
    (void)compute_speed_for_multidirectional_cars_ego(multidirectional_cars,
                                                      v_ego);
    (void)compute_speed_for_multidirectional_human(multidirectional_human,
                                                   v_ego);
  }

  (void)compute_speed_for_prediction_obstacleV2(prediction_obstacles, v_ego);
  MSD_LOG(ERROR, "compute_speed_for_prediction_obstacle: v_target_ = %.3f", v_target_);
  if (is_openspace) {
    (void)compute_speed_for_apaV2();
  }
  MSD_LOG(ERROR, "compute_speed_for_apa: v_target_ = %.3f", v_target_);

  v_target_ = (is_reverse ? -1.0 : 1.0) * std::abs(v_target_);
  MSD_LOG(ERROR, "consider gear direction: v_target_ = %.3f", v_target_);

  return true;
}

bool ParkingLongitudinalMotionPlanner::checkBlockV2(const LeaderPair &lead_cars, const FreespacePoint &fs_point, const FreespaceLine &fs_line){
  double lon_inflation_min = CarParams::GetInstance()->lon_inflation_min;
  double remaining_distance = 10.0;
  auto &lead_one = lead_cars.first;

  if (fs_point.id >= 0) {
    remaining_distance = std::min(fs_point.d_rel - lon_inflation_min, remaining_distance);
  }
  // if (lead_one.id >= 0 && lead_one.type == ObjectType::PEDESTRIAN) {
  //   remaining_distance = std::min(lead_one.d_rel - lon_inflation_min, remaining_distance);
  // }
  // if (lead_one.id >= 0 && lead_one.type == ObjectType::COUPE && !lead_one.is_static) {
  //   remaining_distance = std::min(lead_one.d_rel - lon_inflation_min, remaining_distance);
  // }
  if (fs_line.id >= 0) {
    remaining_distance = std::min(fs_line.d_rel - lon_inflation_min, remaining_distance);
  }
  MSD_LOG(ERROR, "remaining_distance: %.3f", remaining_distance);
  double min_finish_len = CarParams::GetInstance()->car_config.common_config.min_finish_len;
  
  return remaining_distance < min_finish_len;
}

bool ParkingLongitudinalMotionPlanner::compute_speed_with_leadsV2(
    const LeaderPair lead_cars, const double v_ego, const bool isAPA) {
  double a_lead_p = 0.0;
  double d_des = 0.0;
  double v_target_lead = 40.0;
  double a_lead_p_2 = 0.0;
  double d_des_2 = 0.0;
  double v_target_lead_2 = 40.0;
  std::pair<double, double> a_target = a_target_objective_;
  double dist_to_poi = world_model_->get_distance_to_poi();
  if(lead_cars.first.id != -1 && lead_cars.first.is_static && lead_cars.first.type == ObjectType::COUPE){
    MSD_LOG(INFO, "return as lead_cars.first.is_static");
    return true;
  }
  if(lead_cars.first.id != -1 && lead_cars.second.id != -1 && lead_cars.second.is_static
    && lead_cars.second.type == ObjectType::COUPE){
    MSD_LOG(INFO, "return as lead_cars.second.is_static");
    return true;
  }
  if (lead_cars.first.id != -1) {
    MSD_LOG(ERROR,"lead_cars.first.d_rel = %.3f id: %d", lead_cars.first.d_rel, lead_cars.first.id);
    a_lead_p = process_a_lead(lead_cars.first.a_lon);
    d_des = calc_desired_distance(lead_cars.first.v_lon, v_ego,
                                  isAPA || lead_cars.first.is_sidepass_obj ||
                                      (dist_to_poi < 0.0),
                                  false, lead_cars.first.is_static);
    v_target_lead = calc_desired_speed(lead_cars.first.d_rel, d_des,
                                       lead_cars.first.v_lon, a_lead_p);
    if (lead_cars.second.id != -1) {
      MSD_LOG(ERROR,"lead_cars.second.d_rel = %.3f id: %d", lead_cars.second.d_rel, lead_cars.first.id);
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
  MSD_LOG(ERROR, "leads v_target_= %.3f, v_target_lead = %.3f", v_target_, v_target_lead);

  return true;
}

void ParkingLongitudinalMotionPlanner::compute_speed_for_remain_distanceV2(
  const LeaderPair& lead_cars,const FreespacePoint& lead_point,
  const FreespaceLine &fs_line, bool is_reverse) {

  double min_obs_s = 10;
  double n_des = 10;
  if (lead_point.id != -1) {
    n_des = std::max(0.0, lead_point.d_rel);
    min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
    MSD_LOG(ERROR, "lead point remain_s is: %f, the pos is:x %f, y %f", min_obs_s, lead_point.x, lead_point.y);
  }

  // if (lead_cars.first.id != -1 && !lead_cars.first.is_static) {
  //   n_des = std::max(0.0, lead_cars.first.d_rel);
  //   min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
  //   MSD_LOG(ERROR, "first lead car remain_s is: %f, id is:%d", min_obs_s, lead_cars.first.id);
  //   if (lead_cars.second.id != -1 && !lead_cars.second.is_static) {
  //     n_des = std::max(0.0, lead_cars.first.d_rel);
  //     min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
  //     MSD_LOG(ERROR, "second lead car remain_s is: %f, id is:%d", min_obs_s, lead_cars.second.id);
  //   }
  // }

  if (fs_line.id != -1) {
    n_des = std::max(0.0, fs_line.d_rel);
    min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
    MSD_LOG(ERROR, "first lead line remain_s is: %f, the pos is:x %f, y %f, x_end:%f, y_end:%f", 
    min_obs_s, fs_line.x_start, fs_line.y_start, fs_line.x_end,fs_line.y_end);
  }
  min_obs_s = min_obs_s - CarParams::GetInstance()->lon_inflation_min;
  double traj_end_speed_limit_s = msquare::CarParams::GetInstance()->car_config.lon_config.traj_end_speed_limit_s;
  double max_forward_speed = is_reverse ? TrajectoryOptimizerConfig::GetInstance()->param_max_speed_reverse : TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward;
  double max_deceleration = 0.24;
  double time_delay = 0.2;

  double consider_obstacle_max_s = msquare::CarParams::GetInstance()->car_config.lon_config.consider_obstacle_max_s;

  double control_take_over_remain_s = msquare::CarParams::GetInstance()->car_config.lon_config.control_take_over_remain_s;
  double control_take_over_speed = msquare::CarParams::GetInstance()->car_config.lon_config.control_take_over_speed;
  double control_take_over_acc = msquare::CarParams::GetInstance()->car_config.lon_config.control_take_over_acc;
  double min_velocity = msquare::CarParams::GetInstance()->car_config.lon_config.min_velocity;
  double planning_deceleration = msquare::CarParams::GetInstance()->car_config.lon_config.planning_deceleration;
  double start_speed_reduce_remain_s = std::abs(max_forward_speed * max_forward_speed - control_take_over_speed * control_take_over_speed) / (2 * planning_deceleration) + control_take_over_remain_s;

  v_target_ = std::min(max_forward_speed, v_target_);
  // double control_speed 
    // + max_forward_speed * time_delay;
  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .planner_type == PlannerType::OPENSPACE) {

    double remain_s = min_obs_s;
    MSD_LOG(ERROR, "init remain_s is: %f",remain_s);
    remaining_distance_ = remain_s;
    if(remain_s > consider_obstacle_max_s){
      return;
    }
    if (remain_s < traj_end_speed_limit_s) {
      v_target_ = 0.0;
    } else if(remain_s > traj_end_speed_limit_s &&
     remain_s < control_take_over_remain_s){
      double delta_s = control_take_over_remain_s - traj_end_speed_limit_s;
      double control_take_over_s = (control_take_over_speed * control_take_over_speed -
        min_velocity * min_velocity) / ( 2 * control_take_over_acc);
      if (delta_s > control_take_over_s) {
        double diff_s = delta_s - control_take_over_s;
        if(remain_s > traj_end_speed_limit_s && remain_s < traj_end_speed_limit_s + diff_s) {
          v_target_ = std::min(min_velocity, v_target_);
        } else {
          double speed_low_diff_s = remain_s - diff_s - traj_end_speed_limit_s;
          v_target_ = 
            std::min(std::sqrt(min_velocity * min_velocity + 
            2 * control_take_over_acc * speed_low_diff_s),v_target_);
        }
        // v_target_ = std::sqrt(min_velocity * min_velocity  + 
        // 2 * control_take_over_acc )
      }
    } else if(remain_s > control_take_over_remain_s
      && remain_s < start_speed_reduce_remain_s) {
      double delta_s = remain_s - control_take_over_remain_s;
      v_target_ = std::min(std::sqrt(control_take_over_speed * control_take_over_speed +
        2 * planning_deceleration * delta_s), v_target_);
      // v_target_ = max(v_target_, 0.15);
    }
    v_target_ = std::min(max_forward_speed, v_target_);
    MSD_LOG(ERROR, "compute_speed_for_remain_distance is: %f, remain_s is: %f",
            v_target_, remain_s);
  }
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_prediction_obstacleV2(
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

bool ParkingLongitudinalMotionPlanner::limit_speed_for_margin_limit(){
  const auto& planner_output = PlanningContext::Instance()->longitudinal_behavior_planner_output();
  bool path_segment_updated = planner_output.path_segment_updated;

  if(path_segment_updated){
    const std::vector<Pose2D> &plan_traj = planner_output.trajectory;
    const std::vector<double>& curvatures = planner_output.curvatures;
    MSD_LOG(ERROR, "update path segment: receive updated traj_size=%d", plan_traj.size());
    int cut_num = 10;
    if(plan_traj.size() < cut_num + 2){
      return false;
    }
    if(plan_traj.size() != curvatures.size()){return false;}
    
    std::vector<Pose2D> cut_traj(plan_traj.begin(), plan_traj.end()-cut_num);
    std::vector<double> cut_cur(curvatures.begin(), curvatures.end()-cut_num);
    marginVelocityLimit(cut_traj, cut_cur);

    PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->path_segment_updated = false;
    std::string& debug_string = PlanningContext::Instance()->mutable_longitudinal_behavior_planner_output()->speed_margin_debug;
    speed_margin_limiter_.getSerializeString(debug_string);
    MSD_LOG(ERROR, "received total s= %.3f", speed_margin_limiter_.getTotals());
  }

  double traj_length = planner_output.traj_length;
  double limit_v;
  if(!speed_margin_limiter_.getVConsiderLast(traj_length,last_margin_v_, limit_v, last_res_)){
    last_margin_v_ = -1.0;
    last_pose_.is_valid = false;
    return false;
  }
  last_pose_.updatePose(world_model_->get_ego_state().ego_pose);
  MSD_LOG(ERROR, "traj_length= %.3f, limit_v = %.3f", traj_length, limit_v);
  limit_v = std::max(0.0, limit_v);
  last_margin_v_ = limit_v;
  v_target_ = std::min(v_target_, limit_v);
  return true;
}

bool ParkingLongitudinalMotionPlanner::marginVelocityLimit(const Pose2DTrajectory &plan_traj, const std::vector<double>& curvatures){
  auto instance = PlanningContext::Instance();
  const auto& planning_result =instance->planning_status().planning_result;
  const EgoState &ego_state = world_model_->get_ego_state();
  bool is_reverse = (planning_result.gear == GearState::REVERSE);
  const bool is_not_parallel = instance->parking_behavior_planner_output()
              .parking_slot_info.type.value != ParkingSlotType::PARALLEL;
  const bool is_parkin = instance->planning_status().scenario.status_type == StatusType::APA;
  const bool is_at_last_segment = instance ->longitudinal_behavior_planner_output().is_at_last_segment;

  std::vector<planning_math::Box2d> obs_boxes;
  std::vector<planning_math::Vec2d> obs_pts;
  // get obstacles
  for(const auto& box_obs:world_model_->obstacle_manager().get_obstacles().Items()){
    obs_boxes.emplace_back(box_obs->PerceptionBoundingBox());
  }

  for(const auto& p_obs:world_model_->obstacle_manager().get_points().Items()){
    obs_pts.emplace_back(p_obs->PerceptionBoundingBox().center());
  }

  auto line_obs = world_model_->obstacle_manager().get_lines().Items();
  auto & pillars = world_model_->obstacle_manager().get_pillars().Items();
  auto & road_borders = world_model_->obstacle_manager().get_road_borders().Items();
  auto & gates = world_model_->obstacle_manager().get_gates().Items();
  line_obs.insert(line_obs.end(), pillars.begin(), pillars.end());
  line_obs.insert(line_obs.end(), road_borders.begin(),road_borders.end());
  line_obs.insert(line_obs.end(), gates.begin(), gates.end());
  std::vector<msquare::planning_math::LineSegment2d> line_obs_final;
  for(const auto& l_obs:line_obs){
    line_obs_final.emplace_back(l_obs->PerceptionLine());
  }
  
  SpeedMarginPara speed_margin_para;
  speed_margin_para.init(is_reverse,is_at_last_segment, obs_boxes, obs_pts, line_obs_final);
  speed_margin_para.is_apa_not_parallel = (is_not_parallel && is_parkin);
  bool update_wheel_stop =  instance->longitudinal_behavior_planner_output().update_wheel_stop;
  bool is_dynamic_planning = instance->longitudinal_behavior_planner_output().is_dynamic_planning;
  double last_delta_s = last_pose_.distance(ego_state.ego_pose);
  if(update_wheel_stop){
    speed_margin_para.setWheelStopper(last_margin_v_);
  }
  speed_margin_para.setDynamicPlanning(last_margin_v_, last_delta_s, is_dynamic_planning);
  speed_margin_para.last_res = last_res_;
  MSD_LOG(ERROR, "is_dynamic_planning = %d, update_wheel_stop = %d", is_dynamic_planning, update_wheel_stop);
  speed_margin_limiter_ = SpeedMarginLimiter(speed_margin_para, plan_traj, curvatures, false);
  speed_margin_limiter_.filterSV();
  return true;
}

  double ParkingLongitudinalMotionPlanner::compute_remain_distance_for_qp(
                                        const LeaderPair& lead_cars,
                                        const FreespacePoint& lead_point,
                                        const FreespaceLine &fs_line) {
  double min_obs_s = 50;
  double n_des = 50;
  if (lead_point.id != -1) {
    n_des = std::max(0.0, lead_point.d_rel);
    min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
    MSD_LOG(ERROR, "lead point remain_s is: %f, the pos is:x %f, y %f", min_obs_s, lead_point.x, lead_point.y);
  }

  if (lead_cars.first.id != -1 && !lead_cars.first.is_static) {
    n_des = std::max(0.0, lead_cars.first.d_rel);
    min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
    MSD_LOG(ERROR, "first lead car remain_s is: %f, id is:%d", min_obs_s, lead_cars.first.id);
    if (lead_cars.second.id != -1 && !lead_cars.second.is_static) {
      n_des = std::max(0.0, lead_cars.first.d_rel);
      min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
      MSD_LOG(ERROR, "second lead car remain_s is: %f, id is:%d", min_obs_s, lead_cars.second.id);
    }
  }

  if (fs_line.id != -1) {
    n_des = std::max(0.0, fs_line.d_rel);
    min_obs_s = min_obs_s > n_des ? n_des : min_obs_s;
    MSD_LOG(ERROR, "first lead line remain_s is: %f, the pos is:x %f, y %f, x_end:%f, y_end:%f", 
    min_obs_s, fs_line.x_start, fs_line.y_start, fs_line.x_end,fs_line.y_end);
  }
  min_obs_s = min_obs_s - CarParams::GetInstance()->lon_inflation_min;
  double traj_remain_s = PlanningContext::Instance()
                      ->longitudinal_behavior_planner_output()
                      .traj_length;
  double remain_s = traj_remain_s;
  MSD_LOG(ERROR, "init traj remain_s is: %f, min_obs is: %f",remain_s, min_obs_s);
  // std::cout << "init remain_s is:" << remain_s << std::endl;
  remain_s = remain_s > min_obs_s ? min_obs_s : remain_s;
  msquare::parking::VecST vec_st;
  double dt = 0.1;
  double min_remain_v = speed_margin_limiter_.getVByRemains(remain_s);
  speed_margin_limiter_.getSTByS(vec_st, dt, traj_remain_s);
  MSD_LOG(ERROR, "init remain_s is: %f ",remain_s);
  double traj_length_margin_v = std::numeric_limits<double>::max();
  if (!vec_st.empty()) {
    traj_length_margin_v = vec_st.front().v;
  }
  MSD_LOG(ERROR, "min_remain_v: %f, traj_length_margin_v: %f ",
      min_remain_v, traj_length_margin_v);
  if (min_remain_v > traj_length_margin_v) {
    remain_s = traj_remain_s;
  }
  MSD_LOG(ERROR, "final remain_s:%f", remain_s);
  return remain_s;
}

void ParkingLongitudinalMotionPlanner::compute_speed_use_osqp(const LeaderPair& lead_cars,
                            const FreespacePoint& lead_point,
                            const FreespaceLine &fs_line, 
                            bool is_reverse,
                            bool is_stop_status) {
  const auto& planner_output = PlanningContext::Instance()->longitudinal_behavior_planner_output();
  PlanningResult& planning_result 
      = PlanningContext::Instance()->mutable_planning_status()->planning_result;
  std::vector<TrajectoryPoint> pre_pwj_trajectory = planning_result.pwj_trajectory;
  SpeedData speed_data;
  for (const auto& traj_pt : pre_pwj_trajectory) {
    SpeedPoint speed_pt;
    speed_pt.t = traj_pt.relative_time;
    speed_pt.s = traj_pt.path_point.s;
    speed_pt.v = traj_pt.v;
    speed_pt.a = traj_pt.a;
    if (traj_pt.relative_time < -10) {
      break;
    }
    speed_data.emplace_back(std::move(speed_pt));
  }
  // std::cout << "trajectory size is: " << pre_pwj_trajectory.size() 
  //           << "the speed data size is:" << speed_data.size()
  //           << std::endl;
  const std::vector<Pose2D>& plan_traj = planner_output.trajectory;
  const std::vector<double>& curvatures = planner_output.curvatures;
  const std::vector<double>& relative_s = planner_output.relative_s;
  int cut_num = 10;
  if(plan_traj.size() < cut_num + 2){
    return;
  }
  if(plan_traj.size() != curvatures.size()){return;}
  MSD_LOG(ERROR, "update path segment: receive updated traj_size=%d", plan_traj.size());
  double dt = 0.1;
  double ego_speed = world_model_->get_ego_state().ego_vel;
  msquare::parking::SpeedPlannerCofig config(dt, is_reverse);
  std::vector<std::vector<double>> path;
  
  msquare::parking::VecST vec_st;
  std::vector<std::vector<double>> vec_st_real;
  double remain_s = compute_remain_distance_for_qp(lead_cars, lead_point, fs_line);
  speed_margin_limiter_.getSTByS(vec_st, dt, remain_s);

  msquare::parking::VecST fix_time_vec_st;
  double fix_time = 3.0;
  for (const auto& vec_st_pt : vec_st) {
    if (vec_st_pt.t > fix_time) {
      break;
    }
    fix_time_vec_st.emplace_back(vec_st_pt);
  }

  const bool isAPA =
      (PlanningContext::Instance()->parking_behavior_planner_output().planner_type == PlannerType::OPENSPACE);
  msquare::parking::VecST first_st_obs;
  msquare::parking::VecST second_st_obs;
  computeSTFromODObs(lead_cars, ego_speed, isAPA, first_st_obs, second_st_obs, config.dt);
  // std::cout << "the first_st_obs size is: " << first_st_obs.size() << std::endl;
  MSD_LOG(ERROR, "the first_st_obs size is: %d, the second_st_obs size is: %d", 
          first_st_obs.size(), second_st_obs.size());

  msquare::parking::PiecewiseJerkSpeedOptimizer piece_wise_optimizer(config);
  double total_path_length = 
      PlanningContext::Instance()
      ->longitudinal_behavior_planner_output().traj_length;
  double init_a = 0.0;
  double init_speed = getInitSpeedForQP(speed_data, std::abs(ego_speed), &init_a);
  piece_wise_optimizer.makeOptimize(
    init_speed, init_a, &path, fix_time_vec_st, first_st_obs, second_st_obs);

  std::vector<TrajectoryPoint> trajectory;
  std::vector<PathPoint> cur_path;
  for (int i = 0 ; i < plan_traj.size() - 10; i ++) {
    PathPoint path_pt;
    path_pt.x = plan_traj[i].x;
    path_pt.y = plan_traj[i].y;
    path_pt.theta = plan_traj[i].theta;
    path_pt.s = relative_s[i];
    cur_path.emplace_back(path_pt);
  }
  // for (const auto& pose : cut_traj) {
  //   PathPoint path_pt;
  //   path_pt.x = pose.x;
  //   path_pt.y = pose.y;
  //   path_pt.theta = pose.theta;
  // }
  DiscretizedPath  discretize_path(cur_path);
  // std::cout << "the qp result size is:" << path.size() 
  //           << "the total_path_length is: " 
  //           << total_path_length
  //           << std::endl;
  planning_result.pwj_trajectory.clear();
  for (int i = 0; i < path.size(); i++) {
    const auto& speed_pt = path[i];
    TrajectoryPoint  traj_pt;
    // PathPoint path_point;
    if(speed_pt.size() <= 0) {
      break;
    }
    if (speed_pt[0] > total_path_length + 2.0) {
      // std::cout << "the s is:" << speed_pt[0] << std::endl;
      break;
    }
    PathPoint path_point = discretize_path.Evaluate(speed_pt[0]);
    traj_pt.relative_time = vec_st[i].t;
    path_point.s = speed_pt[0];
    if (is_stop_status) {
      traj_pt.v = 0;
    } else {
      traj_pt.v = speed_pt[1];
    }
    traj_pt.a = speed_pt[2];
    traj_pt.path_point = path_point;
    trajectory.emplace_back(traj_pt);
    // std::cout << "the optimize is:" << path_point.x << " " << path_point.y << " "
    //           << "the t : " << traj_pt.relative_time << "  s: " 
    //           << path_point.s << " v: " << traj_pt.v << "  a: " 
    //           << traj_pt.a << std::endl;
  }
  if (path.empty() && !pre_pwj_trajectory.empty()) {
    // todo when qp fail, use pre pwj trajectory
  }
  if (!path.empty()) {
    TrajectoryPoint  tem_traj_pt;
    tem_traj_pt.relative_time = -11;
    tem_traj_pt.v = init_speed;
    tem_traj_pt.a = init_a;
    trajectory.emplace_back(tem_traj_pt);
  }
  for (const auto& data : fix_time_vec_st) {
    TrajectoryPoint  traj_pt;
    PathPoint path_point;
    path_point.s = data.s;
    traj_pt.relative_time = data.t;
    traj_pt.v = data.v;
    traj_pt.path_point = path_point;
    trajectory.emplace_back(traj_pt);
  }
  if (!path.empty()) {
    TrajectoryPoint  tem_traj_pt_1;
    tem_traj_pt_1.relative_time = -9;
    tem_traj_pt_1.v = init_speed;
    tem_traj_pt_1.a = init_a;
    trajectory.emplace_back(tem_traj_pt_1);
  }
  for (const auto& data : first_st_obs) {
    TrajectoryPoint  traj_pt;
    PathPoint path_point;
    path_point.s = data.s;
    traj_pt.relative_time = data.t;
    traj_pt.v = data.v;
    traj_pt.path_point = path_point;
    trajectory.emplace_back(traj_pt);
  }
  if (!path.empty()) {
    TrajectoryPoint  tem_traj_pt_2;
    tem_traj_pt_2.relative_time = -7;
    tem_traj_pt_2.v = init_speed;
    tem_traj_pt_2.a = init_a;
    trajectory.emplace_back(tem_traj_pt_2);
  }
  for (const auto& data : second_st_obs) {
    TrajectoryPoint  traj_pt;
    PathPoint path_point;
    path_point.s = data.s;
    traj_pt.relative_time = data.t;
    traj_pt.v = data.v;
    traj_pt.path_point = path_point;
    trajectory.emplace_back(traj_pt);
  }
  planning_result.pwj_trajectory = trajectory;
  return;  
}

bool ParkingLongitudinalMotionPlanner::compute_speed_for_apaV2() {
  const EgoState &ego_state = world_model_->get_ego_state();
  const PlanningStatus &planning_status =
      PlanningContext::Instance()->planning_status();
  const ParkingBehaviorPlannerOutput &parking_behavior_planner_output =
      PlanningContext::Instance()->parking_behavior_planner_output();
  const bool is_reverse =
      (planning_status.planning_result.gear == GearState::REVERSE);
  MSD_LOG(ERROR, " is_move_ready : %d\n", parking_behavior_planner_output.is_move_ready);
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
    // if (index == traj_pose_array_.size() || index == 0) {
    //   v_apa_ = 0;
    //   a_apa_ = 0;
    // } else {
    //   index -= 1;
    //   std::vector<float> traj_vel_array_ =
    //       planning_status.planning_result.traj_vel_array;
    //   std::vector<float> traj_acceleration_ =
    //       planning_status.planning_result.traj_acceleration;
    //   MSD_LOG(INFO, "index & SIZE: %d %ld %ld", index, traj_vel_array_.size(),
    //           traj_acceleration_.size());
    //   MSD_LOG(INFO, "dist_min : %f", dist_min);
    //   MSD_LOG(INFO, "traj v a x y: %f %f %f %f", traj_vel_array_.at(index),
    //           traj_acceleration_.at(index), traj_pose_array_.at(index).x,
    //           traj_pose_array_.at(index).y);
    //   v_apa_ = traj_vel_array_.at(index);

    //   a_apa_ = traj_acceleration_.at(index);
    //   if (is_reverse) {
    //     v_apa_ = std::min(v_apa_, -_V_MIN_APA);
    //   } else {
    //     v_apa_ = std::max(v_apa_, _V_MIN_APA);
    //   }
    // }
  } else {
    v_apa_ = 0.0;
    a_apa_ = 0.0;
    v_target_ = v_apa_;
  }
  MSD_LOG(ERROR, " v_target_, v_apa_, a_apa_: %f %f %f\n", v_target_, v_apa_,
          a_apa_);
  // if (std::abs(v_apa_) < v_target_) {
  //   v_target_ = v_apa_;
  //   if (v_apa_ < 0.01 && v_apa_ > -0.01 && a_apa_ < 0.01 && a_apa_ > -0.01) {
  //     if (is_reverse) {
  //       a_apa_ = 1.0;
  //     } else {
  //       a_apa_ = -1.0;
  //     }
  //   }
  //   if (is_reverse) {
  //     a_target_.first = std::min(a_target_.first, -a_apa_);
  //     a_target_.second = std::max(a_target_.second, -a_apa_);
  //   } else {
  //     a_target_.first = std::min(a_target_.first, a_apa_);
  //     a_target_.second = std::max(a_target_.second, a_apa_);
  //   }
  // } else {
  //   if (std::abs(v_target_) < 0.1) {
  //     a_apa_ = 0.0;
  //   }
  //   if (v_apa_ < 0) {
  //     v_target_ = -v_target_;
  //   }
  // }
  return true;
}

bool ParkingLongitudinalMotionPlanner::computeSTFromODObs(
                const LeaderPair lead_cars, const double v_ego, 
                const bool isAPA,msquare::parking::VecST& first_st_obs, 
                msquare::parking::VecST& second_st_obs, double delta_t) {
  double a_lead_p = 0.0;
  double d_des = 0.0;
  double v_target_lead = 40.0;
  double a_lead_p_2 = 0.0;
  double d_des_2 = 0.0;
  double v_target_lead_2 = 40.0;
  std::pair<double, double> a_target = a_target_objective_;
  double dist_to_poi = world_model_->get_distance_to_poi();
  if (lead_cars.first.id != -1) {
    // std::cout << "---------------> first lead car and the v is: " << lead_cars.first.v_lon  << std::endl;
    a_lead_p = process_a_lead(lead_cars.first.a_lon);
    for (double t = 0; t < 3.0; t = t + delta_t) {
      double temp_t = 1.0;
      d_des = calc_desired_distance(lead_cars.first.v_lon, v_ego,
                                    isAPA || lead_cars.first.is_sidepass_obj ||
                                        (dist_to_poi < 0.0),
                                    false, lead_cars.first.is_static, IntentionObsType::NONE);
      v_target_lead = calc_desired_speed(lead_cars.first.d_rel, d_des,
                                        lead_cars.first.v_lon, a_lead_p);
      // std::cout << "the first obs t is: " << t << " " << v_target_lead << std::endl; 
      // first_st_obs.emplace_back(t, 0.0, v_target_lead, 0.0, 0.0);
      first_st_obs.emplace_back(t, 0.0, std::fabs(v_target_), 0.0, 0.0);

    }
  }
  if (lead_cars.first.id != -1 && lead_cars.second.id != -1) {
    a_lead_p_2 = process_a_lead(lead_cars.second.a_lon);
    for (double t = 0; t < 3.0; t = t + delta_t) {
      double temp_t = 1.0;
      d_des_2 = calc_desired_distance(lead_cars.second.v_lon, v_ego,
                                    isAPA || lead_cars.second.is_sidepass_obj ||
                                        (dist_to_poi < 0.0),
                                    false, lead_cars.second.is_static, IntentionObsType::NONE);
      v_target_lead_2 = calc_desired_speed(lead_cars.second.d_rel, d_des_2,
                                        lead_cars.second.v_lon, a_lead_p_2);
      second_st_obs.emplace_back(t, 0.0, std::fabs(v_target_), 0.0, 0.0);
    }
  }
  return true;

}

void ParkingLongitudinalMotionPlanner::compute_speed_for_curvature() {
  auto planning_result =
    PlanningContext::Instance()->planning_status().planning_result;
  std::vector<float> traj_curvature = planning_result.traj_curvature;
  std::vector<float> traj_relative_s = planning_result.traj_relative_s;
  const auto &ego_pose = world_model_->get_ego_state().ego_pose;
  std::vector<Pose2D> traj_pose = planning_result.traj_pose_array;
  std::vector<double> traj_s;
  double curvature_change_threshold = 0.3;
  std::vector<std::pair<double, double>> curvature_change_scope;
  if (traj_curvature.size() < 2
      || traj_curvature.size() != traj_relative_s.size()) {
    return;
  }
  for (size_t i = 0; i < traj_curvature.size() - 1; i++) {
    if (traj_relative_s[i] < 0.0 || traj_relative_s[i] > 10.0)
      continue;
    if (std::isnan(traj_curvature[i]))
      continue;
    if(traj_curvature[i] * traj_curvature[i + 1] < 0 &&
       std::fabs(traj_curvature[i] - traj_curvature[i + 1]) >
       curvature_change_threshold) {
      curvature_change_scope.emplace_back(traj_relative_s[i] - 0.3, traj_relative_s[i] + 0.3);
    }
  }

  if (traj_pose.empty()) {
    return;
  }
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
  double current_s = get_current_s();
  for (const auto scope : curvature_change_scope) {
    // std::cout << "the scope first:" << scope.first << " " << scope.second << std::endl;
    if (current_s > scope.first && current_s < scope.second) {
      v_target_ = std::min(0.2, v_target_);
      return;
    }
  }
}

void ParkingLongitudinalMotionPlanner::set_planning_resultV2(const double v_ego) {

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
  double dec = msquare::CarParams::GetInstance()->car_config.lon_config.dec;
  double delta_v = dec /rate_;
  double control_take_over_remain_s = msquare::CarParams::GetInstance()->car_config.lon_config.control_take_over_remain_s;
  double control_take_over_speed = msquare::CarParams::GetInstance()->car_config.lon_config.control_take_over_speed;
  
  PlanningContext::Instance()
      ->mutable_planning_status()
      ->planning_result.bag_recorder_filter_scenario.abrupt_brake =
      (bool)(a_target_.first < -1.0);

  if (std::abs(v_ego) <= 0.05 &&
      std::abs(v_target_) > VehicleParam::Instance()->velocity_deadzone) {
    a_target_.second = clip(abs(v_target_), a_target_.second, 0.4);
  }

  if (std::abs(v_target_) > std::abs(current_v_) || remaining_distance_ < control_take_over_remain_s || std::abs(current_v_ - v_target_) < 1e-2) {
    current_v_ = v_target_;
  }
  if(std::abs(v_target_) < std::abs(current_v_)){
    if (planning_result.gear == GearState::DRIVE){
      if(current_v_ - delta_v > v_target_){
        current_v_ -= delta_v;
      }else{
        current_v_ = v_target_;
      }
    }else{
      if(current_v_ + delta_v < v_target_){
        current_v_ += delta_v;
      }else{
        current_v_ = v_target_;
      }
    }
  }
  current_v_ = v_target_;
  
  planning_result.a_array.push_back(a_target_.first);
  planning_result.a_array.push_back(a_target_.second);
  planning_result.a_target_min = a_target_.first;
  planning_result.a_target_max = a_target_.second;
  planning_result.a_target = dec;

  planning_result.v_array.push_back(current_v_);
  planning_result.v_target = current_v_;
  MSD_LOG(ERROR, "final v_target_= %.3f", current_v_);

}

double ParkingLongitudinalMotionPlanner::getInitSpeedForQP(const SpeedData& speed_data, 
            double ego_speed, double* init_a) {
  SpeedPoint target_speed_data;
  double target_speed = 0.0;
  double target_acc = 0.0;
  if (speed_data.empty()) {
    return ego_speed;
  }
  auto pre_planning_time 
      = PlanningContext::Instance()->planning_status().planning_result.pre_planning_time;
  auto cur_time  = MTIME()->timestamp().us();
  double delta_time = double(cur_time - pre_planning_time) / 1000000;
  if (pre_planning_time < 0.1) {
    delta_time = 0.0;
  }
  speed_data.EvaluateByTime(speed_data.front().t + delta_time + 0.05, &target_speed_data);
  MSD_LOG(ERROR, "the ego speed is: %f, the target v is:%f, the target a is:%f", ego_speed, target_speed_data.v, target_speed_data.a);
  if (std::fabs(ego_speed) < 0.1) {
    // std::cout << "the ego velocity has big error with target velocity, replan speed planner" << std::endl;
    double delay_time = 0.65;
    target_speed = 0.2;
    SpeedPoint target_speed_data_for_acc;
    speed_data.EvaluateByTime(delay_time, &target_speed_data_for_acc);
    target_acc = target_speed_data_for_acc.a;
    // piece_wise_optimizer.makeOptimize(ego_speed, &path, vec_st);
  } else {
    target_speed = target_speed_data.v;
    target_acc = target_speed_data.a;
  }
  // target_speed = target_speed_data.v;
  *init_a = target_acc;
  MSD_LOG(ERROR, "final target v is:%f, final target a is:%f", target_speed, *init_a);
  return target_speed;
}


} // namespace parking

} // namespace msquare
