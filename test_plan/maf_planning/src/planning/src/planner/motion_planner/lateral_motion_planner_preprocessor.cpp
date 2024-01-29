#include "planner/motion_planner/lateral_motion_planner_preprocessor.h"
#include "common/obstacle_process_utils.h"
#include "planner/behavior_planner/lateral_behavior_state.h"
#include "planner/motion_planner/path_planner_ceres/path_planner_constants.hpp"
#include "planner/motion_planner/planner_cubic_spline/planner_cubic_spline_utility.hpp"
#include "planning/common/inner_snapshot.h"

namespace msquare {

using namespace msquare::planning_math;
using namespace path_planner;
using namespace planner_spline;

void LateralMotionPlannerPreprocessor::gerenate_s_at_control_quad_points() {
  const double path_planner_time = 6;
  const double frenet_length_protect = 0.1;
  const double min_total_length = 10.;

  const auto &planning_init_point =
      baseline_info_->get_ego_state().planning_init_point;
  double total_length = min_total_length;
  if (!cache_data_.lon_s_pair_list.empty()) {
    total_length =
        std::fmax(min_total_length,
                  std::fmin(cache_data_.lon_s_pair_list.back().second -
                                cache_data_.lon_s_pair_list.front().second,
                            planning_init_point.v * path_planner_time));
  }

  total_length =
      std::fmin(total_length, baseline_info_->get_frenet_coord()->GetLength() -
                                  planning_init_point.path_point.s -
                                  frenet_length_protect);
  double delta_s = total_length / double(NUM_PATH_SEGMENTS);

  // control point
  for (size_t segment_index = 0; segment_index < NUM_PATH_CONTROL_POINTS;
       segment_index++) {
    s_at_control_points_.at(segment_index) =
        delta_s * segment_index + planning_init_point.path_point.s;
  }

  // quad point
  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    s_at_quad_points_[index.segment_index][index.quad_index] =
        s_at_control_points_[index.segment_index] +
        (s_at_control_points_[index.segment_index + 1] -
         s_at_control_points_[index.segment_index]) *
            GAUSS_QUAD_5TH_POS[index.quad_index];
  }
}

void LateralMotionPlannerPreprocessor::process() {
  const auto &prev_path_planner_output =
      context_->lateral_motion_planner_output().prev_path_planner_output;
  const auto &pre_planning_result =
      context_->planning_status().pre_planning_result;
  const auto &planning_init_point =
      baseline_info_->get_ego_state().planning_init_point;

  cache_data_ = {};

  if (pre_planning_result.traj_pose_array.empty() ||
      pre_planning_result.traj_vel_array.empty() ||
      pre_planning_result.traj_acceleration.empty()) {
    set_uniform_lon_motion_input(planning_init_point);
  } else {
    set_pre_lon_result(planning_init_point);
  }

  for (const auto &p : pre_planning_result.traj_pose_array) {
    Point2D cart, fren;
    cart.x = p.position_enu.x;
    cart.y = p.position_enu.y;
    (void)baseline_info_->get_frenet_coord()->CartCoord2FrenetCoord(cart, fren);
    cache_data_.last_path_frenet_pair_list.emplace_back(fren.x,
                                                        std::move(fren));
    path_planner::PathPoint tmp;
    tmp.curvature = p.curvature;
    tmp.heading_yaw = p.heading_yaw;
    tmp.path_follow_strength = p.path_follow_strength;
    tmp.position_enu.x = p.position_enu.x;
    tmp.position_enu.y = p.position_enu.y;
    cache_data_.last_path_cart_pair_list.emplace_back(fren.x, tmp);
  }

  const auto &ddp_path_ori = ddp::DdpContext::Instance()
                                 ->get_obstacle_decider_ddp_trajectory()
                                 .trajectory;
  for (const auto &p : ddp_path_ori) {
    cache_data_.ddp_path_s_l_list.emplace_back(p.s, p.l);
  }

  generate_map_info();
  path_planner::PathPlannerPoint tmp;
  for (const auto &path_point : prev_path_planner_output.path_planner_output) {
    Point2D cart, fren;
    cart.x = path_point.x;
    cart.y = path_point.y;
    (void)baseline_info_->get_frenet_coord()->CartCoord2FrenetCoord(cart, fren);
    cache_data_.prev_path_planner_output_pair_list.emplace_back(fren.x,
                                                                path_point);
  }

  generate_output();
}

void LateralMotionPlannerPreprocessor::generate_output() {
  gerenate_s_at_control_quad_points();

  compute_planning_init_state();

  sample_ddp_path();

  compute_path_segments();

  set_init_params();

  compute_lc_decider_info();

  compute_intersection_info();

  limit_jerk();

  generate_obstacle_info();

  set_vehicle_params();

  set_intelligent_dodge_info();

  set_dlp_info();
}

path_planner::PathSampleInfo
LateralMotionPlannerPreprocessor::generate_path_sample_info(
    double quad_point_s) {
  path_planner::PathSampleInfo tmp_path_sample;
  tmp_path_sample.last_cart_traj =
      LinearInterpation::interpolate<path_planner::PathPoint>(
          cache_data_.last_path_cart_pair_list, quad_point_s);
  tmp_path_sample.sample_s = quad_point_s;
  tmp_path_sample.speed_plan =
      LinearInterpation::interpolate<path_planner::SpeedPlan>(
          cache_data_.speed_plan_pair_list, quad_point_s);

  tmp_path_sample.refline_info = generate_map_sample_info(quad_point_s);

  return tmp_path_sample;
}

void LateralMotionPlannerPreprocessor::compute_planning_init_state() {
  const auto &path_planner_input = context_->mutable_path_planner_input();
  auto &ego_state_manager = world_model_->get_cart_ego_state_manager();
  auto &cart_ego_state = ego_state_manager.get_cart_ego_state();
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  const auto &planning_init_point = ego_state.planning_init_point.path_point;
  const auto &planning_start_frenet_state = ego_state.planning_start_state;

  path_planner_input->is_replan = ego_state.flag_is_replan;
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();
  double min_ref_jerk_v =
      planner_config.lateral_motion_planner_config.min_ref_jerk_velocity;
  double min_ref_jerk =
      planner_config.lateral_motion_planner_config.min_ref_jerk;
  path_planner_input->min_ref_jerk =
      std::make_pair(min_ref_jerk_v, min_ref_jerk);

  auto &planning_init_state = path_planner_input->planning_init_state;
  planning_init_state.s = planning_init_point.s;
  planning_init_state.v = ego_state.ego_vel;
  planning_init_state.x = planning_init_point.x;
  planning_init_state.y = planning_init_point.y;
  planning_init_state.curvature = planning_init_point.kappa;
  path_planner_input->ego_theta = cart_ego_state.ego_pose.theta;

  planning_init_state.l = planning_start_frenet_state.r;
  planning_init_state.dl = planning_start_frenet_state.dr_ds;
  planning_init_state.ddl = planning_start_frenet_state.ddr_dsds;

  const auto &plan_init_point_yaw = planning_init_point.theta;
  const auto frenet_yaw =
      baseline_info_->get_frenet_coord()->GetRefCurveHeading(
          planning_init_state.s);
  const double delta_yaw =
      clip(planning_math::AngleDiff(frenet_yaw, plan_init_point_yaw),
           0.45 * M_PI, -0.45 * M_PI);

  if (cache_data_.last_path_cart_pair_list.size() > 0 &&
      cache_data_.prev_path_planner_output_pair_list.size() > 0 &&
      !ego_state.flag_is_replan &&
      world_model_->get_enter_auto_drive_time() >
          1.0 / FLAGS_planning_loop_rate / 2.0) {

    auto pre_path_planner_output_point =
        LinearInterpation::interpolate<path_planner::PathPlannerPoint>(
            cache_data_.prev_path_planner_output_pair_list,
            planning_init_state.s);
    planning_init_state.dx_ds = pre_path_planner_output_point.dx_ds;
    planning_init_state.d2x_ds2 = pre_path_planner_output_point.d2x_ds2;
    planning_init_state.dy_ds = pre_path_planner_output_point.dy_ds;
    planning_init_state.d2y_ds2 = pre_path_planner_output_point.d2y_ds2;
    planning_init_state.jerk = pre_path_planner_output_point.jerk;
    planning_init_state.sample_info = pre_path_planner_output_point.sample_info;

    if (std::fabs(planning_init_state.dx_ds -
                  std::cos(plan_init_point_yaw) / std::cos(delta_yaw)) > 0.3 ||
        std::fabs(planning_init_state.dy_ds -
                  std::sin(plan_init_point_yaw) / std::cos(delta_yaw)) > 0.3) {
      MSD_LOG(INFO,
              "xzj lat_planner debug, pre_dx_ds : %.f, pre_dy_ds : %.f, "
              "plan_init_point_yaw : %.f, plan_init_point_x : %.f, "
              "plan_init_point_y : %.f",
              pre_path_planner_output_point.dx_ds,
              pre_path_planner_output_point.dy_ds, plan_init_point_yaw,
              pre_path_planner_output_point.x, pre_path_planner_output_point.y);
      MSD_LOG(INFO,
              "xzj lat_planner debug, prev_path_planner_output_pair_list size "
              ": %.f ",
              cache_data_.prev_path_planner_output_pair_list.size());

      for (const auto &pre_point :
           cache_data_.prev_path_planner_output_pair_list) {
        MSD_LOG(INFO,
                "xzj lat_planner debug, pre_point s: %.f, x : %.f, y : %.f, "
                "dx_ds : %.f, dy_ds : %.f",
                pre_point.first, pre_point.second.x, pre_point.second.y,
                pre_point.second.dx_ds, pre_point.second.dy_ds);
      }

      planning_init_state.dx_ds =
          std::cos(plan_init_point_yaw) / std::cos(delta_yaw);
      planning_init_state.dy_ds =
          std::sin(plan_init_point_yaw) / std::cos(delta_yaw);
    }
  } else {
    planning_init_state.dx_ds =
        std::cos(plan_init_point_yaw) / std::cos(delta_yaw);
    planning_init_state.dy_ds =
        std::sin(plan_init_point_yaw) / std::cos(delta_yaw);
  }
}

void LateralMotionPlannerPreprocessor::sample_ddp_path() {
  const auto &path_planner_input = context_->mutable_path_planner_input();
  path_planner_input->ddp_info.valid =
      world_model_->use_eftp() && world_model_->lateral_use_eftp();

  Point2D fren;
  const auto &refline_end_point = world_model_->get_current_last_enu_point();
  (void)baseline_info_->get_frenet_coord()->CartCoord2FrenetCoord(
      refline_end_point, fren);

  path_planner_input->ddp_info.ddp_path.clear();
  path_planner_input->ddp_info.ddp_path.reserve(NUM_PATH_SEGMENTS);
  for (int i = 0; i < NUM_PATH_SEGMENTS; i++) {
    std::vector<path_planner::Point2d> segment;
    for (int j = 0; j < QUADRATURE_ORDER; j++) {
      double s = s_at_quad_points_[i][j];
      double l = LinearInterpation::interpolate<double>(
          cache_data_.ddp_path_s_l_list, s);
      if (s < fren.x)
        segment.emplace_back(path_planner::Point2d{s, l});
    }
    path_planner_input->ddp_info.ddp_path.emplace_back(segment);
  }
}

void LateralMotionPlannerPreprocessor::compute_path_segments() {
  const auto &path_planner_input = context_->mutable_path_planner_input();
  path_planner_input->path_segments.clear();
  path_planner_input->path_segments.reserve(NUM_PATH_SEGMENTS);

  for (int i = 0; i < NUM_PATH_SEGMENTS; i++) {
    path_planner::PathSegmentInfo path_segment;
    for (int j = 0; j < QUADRATURE_ORDER; j++) {
      double quad_point_s = s_at_quad_points_[i][j];
      auto quad_point_info = generate_path_sample_info(quad_point_s);
      quad_point_info.refline_info.time =
          LinearInterpation::interpolate<double>(cache_data_.s_t_pair_list,
                                                 quad_point_s);
      path_segment.quad_point_info.emplace_back(std::move(quad_point_info));
    }
    double end_segment_control_point_s = s_at_control_points_[i + 1];

    path_segment.end_segment_control_point_s = end_segment_control_point_s;
    path_segment.end_segment_refline_info =
        generate_map_sample_info(end_segment_control_point_s);
    path_segment.end_segment_refline_info.time =
        LinearInterpation::interpolate<double>(cache_data_.s_t_pair_list,
                                               end_segment_control_point_s);

    path_planner_input->path_segments.emplace_back(std::move(path_segment));
  }
}

void LateralMotionPlannerPreprocessor::set_init_params() {
  // set int path_planner_params
  const auto &path_planner_input = context_->mutable_path_planner_input();
  path_planner_input->path_tuning_params.max_num_iterations =
      MAX_NUM_ITERATIONS;
  path_planner_input->path_tuning_params.soft_boundary_scale =
      SOFT_BOUNDARY_SCALE;
  path_planner_input->path_tuning_params.hard_boundary_scale =
      HARD_BOUNDARY_SCALE;
  path_planner_input->path_tuning_params.init_curvature_scale =
      INIT_CURVATURE_SCALE;
  path_planner_input->path_tuning_params.curvature_limit_scale =
      CURVATURE_RATE_SCALE;
  path_planner_input->path_tuning_params.lat_accel_scale = LAT_ACCEL_SCALE;
  path_planner_input->path_tuning_params.heading_scale = HEADING_SCALE;
  path_planner_input->path_tuning_params.ref_centering_scale =
      REF_CENTERING_SCALE;
  path_planner_input->path_tuning_params.prev_plan_centering_scale =
      PREV_PLAN_CENTERING_SCALE;
  path_planner_input->path_tuning_params.obstacle_constrain_scale =
      OBSTACLE_CONSTRAIN_SCALE;
  path_planner_input->path_tuning_params.obstacle_desire_scale =
      OBSTACLE_DESIRE_SCALE;
  path_planner_input->path_tuning_params.obstacle_inflation_scale =
      OBSTACLE_INFLATION_SCALE;
}

void LateralMotionPlannerPreprocessor::update_lc_status(const int lc_status) {
  const auto &path_planner_input = context_->mutable_path_planner_input();
  path_planner_input->lc_decider_info.lc_status = lc_status;

  auto &planning_result = context_->mutable_planning_status()->planning_result;
  bool lc_end = (lc_status == path_planner::LaneChangeInfo::LaneKeep &&
                 (planning_result.prev_lc_status ==
                      path_planner::LaneChangeInfo::LeftLaneChange ||
                  planning_result.prev_lc_status ==
                      path_planner::LaneChangeInfo::RightLaneChange));
  path_planner_input->lc_decider_info.lc_end = false;
  if (lc_end) {
    path_planner_input->lc_decider_info.lc_end = true;
  }
  if (planning_result.prev_lc_status == lc_status) {
    path_planner_input->lc_decider_info.lc_status_time =
        std::min(++(planning_result.lc_status_loop) * 0.1, 10.);
  } else if (lc_end) {
    planning_result.lc_status_loop = -10;
    path_planner_input->lc_decider_info.lc_status_time =
        planning_result.lc_status_loop * 0.1;
  } else {
    planning_result.lc_status_loop = 1;
    path_planner_input->lc_decider_info.lc_status_time = 0.1;
  }
  planning_result.prev_lc_status = lc_status;
}

void LateralMotionPlannerPreprocessor::compute_lc_decider_info() {
  // set lane change decision info
  const auto &path_planner_input = context_->mutable_path_planner_input();
  const auto &lc_status = context_->lateral_behavior_planner_output().lc_status;
  const auto &lc_request =
      context_->lateral_behavior_planner_output().lc_request;
  const auto &behavior_state = context_->state_machine_output().curr_state;

  if (lc_status == "left_lane_change") {
    update_lc_status(path_planner::LaneChangeInfo::LeftLaneChange);
  } else if (lc_status == "right_lane_change") {
    update_lc_status(path_planner::LaneChangeInfo::RightLaneChange);
  } else if (behavior_state == ROAD_LC_LBACK) {
    update_lc_status(path_planner::LaneChangeInfo::LaneChangeLeftBack);
  } else if (behavior_state == ROAD_LC_RBACK) {
    update_lc_status(path_planner::LaneChangeInfo::LaneChangeRightBack);
  } else {
    update_lc_status(path_planner::LaneChangeInfo::LaneKeep);
  }

  MSD_LOG(INFO, "zyl-debug  lc_request = %s  lc_status = %s",
          lc_request.c_str(), lc_status.c_str());

  MSD_LOG(INFO, "zyl-debug  lc_wait_dir = %s",
          path_planner_input->lc_decider_info.lc_wait_dir.c_str());
  if (path_planner_input->lc_decider_info.lc_wait_dir != "none") {
    path_planner_input->lc_decider_info.pre_lc_wait = true;
    MSD_LOG(INFO, "zyl-debug  pre_lc_wait-1 = %d",
            path_planner_input->lc_decider_info.pre_lc_wait);
  } else {
    path_planner_input->lc_decider_info.pre_lc_wait = false;
    MSD_LOG(INFO, "zyl-debug  pre_lc_wait-2 = %d",
            path_planner_input->lc_decider_info.pre_lc_wait);
  }
  MSD_LOG(INFO, "zyl-debug  pre_lc_wait = %d",
          path_planner_input->lc_decider_info.pre_lc_wait);

  path_planner_input->lc_decider_info.lc_wait_dir = "none";
  if (lc_request != "none") {
    if (lc_status == "left_lane_change_wait") {
      path_planner_input->lc_decider_info.lc_wait_dir = "left";
    } else if (lc_status == "right_lane_change_wait") {
      path_planner_input->lc_decider_info.lc_wait_dir = "right";
    } else {
      path_planner_input->lc_decider_info.lc_wait_dir = "wait_none";
    }
  }
  MSD_LOG(INFO, "zyl-debug lc_status = %s",
          path_planner_input->lc_decider_info.lc_wait_dir.c_str());

  path_planner_input->lc_decider_info.is_lane_change =
      lc_status == "left_lane_change" || lc_status == "right_lane_change";
  if (path_planner_input->lc_decider_info.is_lane_change) {
    const double TPredMax = 0.5;
    const double DecreaseRate = 1.;
    double t_pred = std::max(
        TPredMax -
            DecreaseRate * path_planner_input->lc_decider_info.lc_status_time,
        0.);
    path_planner_input->lc_decider_info.t_pred = t_pred;
  } else {
    path_planner_input->lc_decider_info.t_pred = 0.0;
  }

  if ((lc_status == "left_lane_change" || lc_status == "right_lane_change") &&
      context_->planning_status().planning_result.lc_status_loop > 1) {
    const auto &prev_lc_end_point =
        context_->lateral_motion_planner_output().prev_lc_end_point;
    Point2D cart, fren;
    cart.x = prev_lc_end_point.x;
    cart.y = prev_lc_end_point.y;
    (void)baseline_info_->get_frenet_coord()->CartCoord2FrenetCoord(cart, fren);
    path_planner_input->lc_decider_info.prev_lc_end_point_fren.x = fren.x;
    path_planner_input->lc_decider_info.prev_lc_end_point_fren.y = fren.y;
    const auto &gmp_output =
        PlanningContext::Instance()->general_motion_planner_output();
    double behavoir_planner_lc_s_remain = gmp_output.lc_remain_dist;
    if (behavoir_planner_lc_s_remain > 0) {
      path_planner_input->lc_decider_info.prev_lc_end_point_fren.x =
          path_planner_input->planning_init_state.s +
          behavoir_planner_lc_s_remain;
      path_planner_input->lc_decider_info.prev_lc_end_point_fren.y = 0.;
    }
  } else {
    path_planner_input->lc_decider_info.prev_lc_end_point_fren.x = 0;
    path_planner_input->lc_decider_info.prev_lc_end_point_fren.y = 0;
  }

  if (world_model_->mutable_lateral_obstacle().tleadone() != nullptr) {
    path_planner_input->lc_decider_info.origin_lane_front_obs_id =
        world_model_->mutable_lateral_obstacle().tleadone()->track_id;
  } else {
    path_planner_input->lc_decider_info.origin_lane_front_obs_id = -1000;
  }
}

void LateralMotionPlannerPreprocessor::generate_obstacle_info() {
  const auto &path_planner_input = context_->mutable_path_planner_input();
  path_planner_input->lat_safety_improved =
      ConfigurationContext::Instance()
          ->planner_config()
          .lateral_motion_planner_config.lat_safety_improved;
  path_planner_input->obs_list.clear();
  clear_obstacle_histroy_info_without_nudge();

  std::array<std::array<double, QUADRATURE_ORDER>,
             path_planner::NUM_PATH_SEGMENTS>
      time_at_quad_point;

  Index<NUM_PATH_CONTROL_POINTS> index;
  while (index.advance()) {
    time_at_quad_point[index.segment_index][index.quad_index] =
        LinearInterpation::interpolate<double>(
            cache_data_.s_t_pair_list,
            s_at_quad_points_[index.segment_index][index.quad_index]);
  }

  ObstacleDecision lat_decision(0);
  ObjectDecisionType lat_decision_type{};
  lat_decision_type.mutable_nudge();
  lat_decision.AddLateralDecision("path_planner_preprocessor",
                                  lat_decision_type);

  auto obs_list = generate_obstacle_info_with_decision<NUM_PATH_SEGMENTS>(
      baseline_info_, context_, time_at_quad_point, lat_decision, false);

  update_obstacle_history_info(obs_list);
  path_planner_input->obs_list = std::move(obs_list);
}

void LateralMotionPlannerPreprocessor::
    clear_obstacle_histroy_info_without_nudge() {
  auto &lat_avoid_obstacle_history_info_map =
      context_->mutable_lateral_motion_planner_output()
          .lat_avoid_obstacle_history_info_map;
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();

  std::vector<int> erase_obstacles{};
  for (const auto &item : lat_avoid_obstacle_history_info_map) {
    int obstacle_id = item.first;
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(obstacle_id);
    if (ptr_obstacle_decision == nullptr)
      continue;
    if (!ptr_obstacle_decision->LateralDecision().has_nudge()) {
      erase_obstacles.push_back(obstacle_id);
    }
  }

  for (const auto &erase_id : erase_obstacles) {
    lat_avoid_obstacle_history_info_map.erase(erase_id);
  }
}

void LateralMotionPlannerPreprocessor::update_obstacle_history_info(
    std::vector<path_planner::ObsInfo> &obs_list) {
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();

  for (auto &obstacle_info : obs_list) {
    const auto *ptr_obstacle = obstacle_manager.find_obstacle(obstacle_info.id);
    if (ptr_obstacle == nullptr) {
      continue;
    }
    const auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(ptr_obstacle->Id());
    if (ptr_obstacle_decision == nullptr || ptr_obstacle == nullptr)
      continue;

    auto &lat_avoid_obstacle_history_info_map =
        context_->mutable_lateral_motion_planner_output()
            .lat_avoid_obstacle_history_info_map;
    auto &obstacle_history_info =
        lat_avoid_obstacle_history_info_map[obstacle_info.id];
    const auto &path_planner_input = context_->path_planner_input();
    if (!path_planner_input.path_segments.empty() &&
        !path_planner_input.path_segments.front().quad_point_info.empty()) {
      double left_lane_line_frenet_l = path_planner_input.path_segments.front()
                                           .quad_point_info.front()
                                           .refline_info.left_lane_border;
      double right_lane_line_frenet_l =
          -path_planner_input.path_segments.front()
               .quad_point_info.front()
               .refline_info.right_lane_border;

      // out of lane: distance_to_lane_line > 0
      double distance_to_lane_line =
          obstacle_info.nudge_side == ObsInfo::NudgeType::LEFT_NUDGE
              ? right_lane_line_frenet_l -
                    ptr_obstacle->PerceptionSLBoundary().end_l
              : ptr_obstacle->PerceptionSLBoundary().start_l -
                    left_lane_line_frenet_l;

      obstacle_history_info.insert(obstacle_history_info.begin(),
                                   distance_to_lane_line);

      if (obstacle_history_info.size() > OBSTACLE_HISTORY_INFO_SIZE) {
        obstacle_history_info.pop_back();
      }
    } else {
      if (obstacle_history_info.size()) {
        obstacle_history_info.pop_back();
      }
    }
    obstacle_info.distance_to_lane_line_history = obstacle_history_info;
  }
}

void LateralMotionPlannerPreprocessor::compute_intersection_info() {
  // set intersection info
  const auto &path_planner_input = context_->mutable_path_planner_input();
  const auto &map_info = world_model_->get_map_info();
  const auto &refline_condition = world_model_->get_refline_condition();
  const auto &prev_refline_condition =
      world_model_->get_refline_last_condition();

  auto is_in_intersection_func = [](const pass_intersection_planner::
                                        ReflineCondition
                                            &refline_condition_input) {
    return refline_condition_input.reflinecondition ==
               pass_intersection_planner::ReflineCondition::
                   FIRST_HALF_INTERSECTION ||
           refline_condition_input.reflinecondition ==
               pass_intersection_planner::ReflineCondition::IN_INTERSECTION ||
           refline_condition_input.reflinecondition ==
               pass_intersection_planner::ReflineCondition::
                   LAST_HALF_INTERSECTION;
  };

  auto inner_snapshot = msd_planning::PlanInnerSnapshot::GetInst();
  auto inner_ctx = inner_snapshot->get_mutable_data();

  if (inner_snapshot->get_snap_mode() !=
      msd_planning::PlanInnerSnapshot::M_PLAYBACK) {
    inner_ctx->real_intersection_time = MTIME()->timestamp().sec();
  }

  // add keep in intersection for 5s
  static double keep_in_intersection_time_start =
      inner_ctx->real_intersection_time;

  if (!is_in_intersection_func(refline_condition) &&
      is_in_intersection_func(prev_refline_condition)) {
    path_planner_input->intersec_info.keep_in_intersection = true;
    keep_in_intersection_time_start = inner_ctx->real_intersection_time;
    path_planner_input->intersec_info.keep_in_intersection_timer = 0.0;
  }

  if (path_planner_input->intersec_info.keep_in_intersection) {
    path_planner_input->intersec_info.keep_in_intersection_timer =
        inner_ctx->real_intersection_time - keep_in_intersection_time_start;
  }

  if (path_planner_input->intersec_info.keep_in_intersection_timer >
      KEEP_IN_INTERSECTION_TIME) {
    path_planner_input->intersec_info.keep_in_intersection = false;
    path_planner_input->intersec_info.keep_in_intersection_timer = 0.0;
  }

  if (map_info.is_in_intersection() ||
      is_in_intersection_func(refline_condition)) {
    path_planner_input->intersec_info.is_in_intersection = true;
    path_planner_input->intersec_info.dist_to_intersection = 0;
    path_planner_input->intersec_info.dist_to_nonintersection =
        map_info.dist_to_last_intsect();
  } else {
    path_planner_input->intersec_info.is_in_intersection = false;
    path_planner_input->intersec_info.dist_to_intersection =
        map_info.dist_to_intsect();
    path_planner_input->intersec_info.dist_to_nonintersection =
        std::numeric_limits<double>::max();
  }
}

void LateralMotionPlannerPreprocessor::limit_jerk() {
  // limit jerk when enter auto drive or at intersection
  const auto &path_planner_input = context_->mutable_path_planner_input();
  const auto &path_planner_input_last = context_->path_planner_input();
  const auto &enter_auto_drive_time = world_model_->get_enter_auto_drive_time();
  const auto &vehicle_dbw_status = world_model_->get_vehicle_dbw_status();
  if (vehicle_dbw_status) {
    if (path_planner_input->intersec_info.is_in_intersection) {
      path_planner_input->path_tuning_params.lat_jerk_scale =
          JERK_IN_INTERSECTION;
    } else if (path_planner_input->intersec_info.keep_in_intersection) {
      path_planner_input->path_tuning_params.lat_jerk_scale =
          (NORMAL_JERK - JERK_IN_INTERSECTION) / KEEP_IN_INTERSECTION_TIME *
              std::fmin(std::fmax(0.0, path_planner_input->intersec_info
                                           .keep_in_intersection_timer),
                        KEEP_IN_INTERSECTION_TIME) +
          JERK_IN_INTERSECTION;
    } else {
      bool is_msim = false;
      auto env = std::getenv("RealitySimulation");
      if (env != nullptr) {
        std::string platform(env);
        if (std::strcmp(env, "simulation") == 0) {
          is_msim = true;
        }
      }
      if (!is_msim) {
        path_planner_input->path_tuning_params.lat_jerk_scale =
            (NORMAL_JERK - MAX_JERK_FIRST_START_AUTO_DRIVE) / JERK_LIMIT_TIME *
                std::fmin(std::fmax(0.0, enter_auto_drive_time),
                          JERK_LIMIT_TIME) +
            MAX_JERK_FIRST_START_AUTO_DRIVE;

        const auto &planner_config =
            ConfigurationContext::Instance()->planner_config();
        double max_curv_rate =
            MAX_CURVATURE_RATE_FIRST_START_AUTO_DRIVE *
            planner_config.lateral_motion_planner_config.max_curv_rate_ratio;
        path_planner_input->path_tuning_params.curvature_limit_scale =
            (CURVATURE_RATE_SCALE - max_curv_rate) / CURVATURE_RATE_LIMIT_TIME *
                std::fmin(std::fmax(0.0, enter_auto_drive_time),
                          CURVATURE_RATE_LIMIT_TIME) +
            max_curv_rate;

        double ref_jerk_ratio_min =
            std::max(std::min(planner_config.lateral_motion_planner_config
                                  .ref_jerk_auto_start_ratio,
                              1.0),
                     0.1);
        double time_limit =
            std::max(planner_config.lateral_motion_planner_config
                         .ref_jerk_auto_start_time_limit,
                     0.1);
        double ref_jerk_ratio =
            (1.0 - ref_jerk_ratio_min) / time_limit *
                std::min(std::max(0.0, enter_auto_drive_time), time_limit) +
            ref_jerk_ratio_min;
        path_planner_input->min_ref_jerk.second *= ref_jerk_ratio;
      } else {
        path_planner_input->path_tuning_params.lat_jerk_scale = NORMAL_JERK;
      }
    }

    // limit jerk when center line kimp
    double jitter = std::max(0., std::min(world_model_->get_baseline_jitter(),
                                          CENTER_JUMP_RANGE_MAX));
    double jump_jerk =
        NORMAL_JERK + (CENTER_JUMP_JERK_MAX - NORMAL_JERK) /
                          (CENTER_JUMP_RANGE_MAX - CENTER_JUMP_RANGE_MIN) *
                          (jitter - CENTER_JUMP_RANGE_MIN);
    double jump_jerk_decay =
        path_planner_input_last.path_tuning_params.lat_jerk_scale -
        JERK_DECAY_RATE * 0.1;
    jump_jerk = std::max(jump_jerk_decay, jump_jerk);
    path_planner_input->path_tuning_params.lat_jerk_scale = std::max(
        jump_jerk, path_planner_input->path_tuning_params.lat_jerk_scale);

  } else {
    path_planner_input->path_tuning_params.lat_jerk_scale =
        MAX_JERK_FIRST_START_AUTO_DRIVE;
  }
}

void LateralMotionPlannerPreprocessor::generate_map_info() {
  // update map info
  auto target_lane_id = context_->planning_status().lane_status.target_lane_id;
  static std::vector<RefPointFrenet> cur_lane, left_lane, right_lane;
  cur_lane.clear();
  left_lane.clear();
  right_lane.clear();
  world_model_->get_map_lateral_info(cur_lane, left_lane, right_lane,
                                     target_lane_id);

  cache_data_.current_lane_map_info_pair_list.clear();
  cache_data_.current_lane_map_info_pair_list.reserve(cur_lane.size());
  cache_data_.left_lane_width_pair_list.clear();
  cache_data_.left_lane_width_pair_list.reserve(left_lane.size());
  cache_data_.right_lane_width_pair_list.clear();
  cache_data_.right_lane_width_pair_list.reserve(right_lane.size());

  for (const auto &p : cur_lane) {
    cache_data_.current_lane_map_info_pair_list.emplace_back(p.s, p);
  }
  for (const auto &p : left_lane) {
    cache_data_.left_lane_width_pair_list.emplace_back(p.s, p.lane_width);
  }
  for (const auto &p : right_lane) {
    cache_data_.right_lane_width_pair_list.emplace_back(p.s, p.lane_width);
  }
}

path_planner::RefPointInfo
LateralMotionPlannerPreprocessor::generate_map_sample_info(const double s) {
  const auto &planning_init_point =
      baseline_info_->get_ego_state().planning_init_point;

  path_planner::RefPointInfo refline_info;
  RefPointFrenet curr_lane_info =
      LinearInterpation::interpolate<RefPointFrenet>(
          cache_data_.current_lane_map_info_pair_list, s);
  refline_info.current_lane_width = curr_lane_info.lane_width;
  refline_info.left_lane_border = curr_lane_info.left_lane_border;
  refline_info.right_lane_border = curr_lane_info.right_lane_border;
  refline_info.left_road_border = curr_lane_info.left_road_border;
  refline_info.right_road_border = curr_lane_info.right_road_border;
  refline_info.left_road_border_type = curr_lane_info.left_road_border_type;
  refline_info.right_road_border_type = curr_lane_info.right_road_border_type;
  refline_info.left_lane_width = LinearInterpation::interpolate<double>(
      cache_data_.left_lane_width_pair_list, s);
  refline_info.right_lane_width = LinearInterpation::interpolate<double>(
      cache_data_.right_lane_width_pair_list, s);

  Point2D cart;
  Point2D fren(s, 0);
  (void)baseline_info_->get_frenet_coord()->FrenetCoord2CartCoord(fren, cart);
  refline_info.x = cart.x;
  refline_info.y = cart.y;

  refline_info.theta_ref = planning_math::NormalizeAngle(
      baseline_info_->get_frenet_coord()->GetRefCurveHeading(s));
  refline_info.cos_theta_ref = std::cos(refline_info.theta_ref);
  refline_info.sin_theta_ref = std::sin(refline_info.theta_ref);
  refline_info.curvature =
      baseline_info_->get_frenet_coord()->GetRefCurveCurvature(s);

  const auto &map_info = world_model_->get_map_info();
  if (map_info.left_boundary_info().empty()) {
    refline_info.left_lane_border_type = LaneBoundaryForm::DASH;
  } else {
    double lane_boundary_length = map_info.left_boundary_info().front().length;
    for (int i = 0; i < map_info.left_boundary_info().size(); ++i) {
      if (i > 0) {
        lane_boundary_length += map_info.left_boundary_info().at(i).length;
      }
      if (s - planning_init_point.path_point.s < lane_boundary_length)
        refline_info.left_lane_border_type =
            map_info.left_boundary_info().at(i).type.value.value;
    }
  }
  if (map_info.right_boundary_info().empty()) {
    refline_info.right_lane_border_type = LaneBoundaryForm::DASH;
  } else {
    double lane_boundary_length = map_info.right_boundary_info().front().length;
    for (int i = 0; i < map_info.right_boundary_info().size(); ++i) {
      if (i > 0) {
        lane_boundary_length += map_info.right_boundary_info().at(i).length;
      }
      if (s - planning_init_point.path_point.s < lane_boundary_length)
        refline_info.right_lane_border_type =
            map_info.right_boundary_info().at(i).type.value.value;
    }
  }

  return refline_info;
}

void LateralMotionPlannerPreprocessor::set_vehicle_params() {
  const auto &path_planner_input = context_->mutable_path_planner_input();
  path_planner_input->vehicle_param.length =
      msquare::ConfigurationContext::Instance()->get_vehicle_param().length;
  path_planner_input->vehicle_param.width =
      msquare::ConfigurationContext::Instance()->get_vehicle_param().width;
}

void LateralMotionPlannerPreprocessor::set_uniform_lon_motion_input(
    const TrajectoryPoint &planning_init_point) {
  const double path_planner_time = 5;
  const double deta_time = 0.2;
  const double deta_s = std::max(planning_init_point.v, 1.0) * 0.2;
  double frenet_s = planning_init_point.path_point.s;
  for (double t = 0.; t < path_planner_time; t += deta_time) {
    frenet_s += deta_s;
    path_planner::SpeedPlan speed_point{std::max(planning_init_point.v, 1.0),
                                        0.0, 0.0};
    cache_data_.speed_plan_pair_list.emplace_back(frenet_s,
                                                  std::move(speed_point));
    cache_data_.lon_s_pair_list.emplace_back(
        frenet_s, frenet_s - planning_init_point.path_point.s);
    cache_data_.s_t_pair_list.emplace_back(frenet_s, t);
  }
}

void LateralMotionPlannerPreprocessor::set_pre_lon_result(
    const TrajectoryPoint &planning_init_point) {
  const auto &planning_result = context_->planning_status().planning_result;
  const auto &pre_planning_result =
      context_->planning_status().pre_planning_result;
  double loop_relative_time =
      (planning_result.next_timestamp_sec - pre_planning_result.timestamp_sec);
  if (baseline_info_->get_ego_state_manager().get_ego_state().flag_is_replan) {
    size_t match_index = QueryNearestPoint(
        pre_planning_result.traj_pose_array,
        {planning_init_point.path_point.x, planning_init_point.path_point.y},
        1.0e-6);
    if (match_index < pre_planning_result.traj_vel_array.size()) {
      loop_relative_time =
          pre_planning_result.traj_vel_array[match_index].relative_time;
    } else {
      loop_relative_time =
          pre_planning_result.traj_vel_array.back().relative_time;
    }
  }
  double start_s = 0;
  for (size_t i = 0; i < std::min(pre_planning_result.traj_vel_array.size(),
                                  pre_planning_result.traj_acceleration.size());
       i++) {
    const auto &vel_point = pre_planning_result.traj_vel_array[i];
    if (vel_point.relative_time < loop_relative_time) {
      start_s = vel_point.distance;
      continue;
    }
    double sample_s =
        vel_point.distance - start_s + planning_init_point.path_point.s;
    path_planner::SpeedPlan speed_point{
        std::max(vel_point.target_velocity, 1.0),
        pre_planning_result.traj_acceleration[i], 0.0};
    cache_data_.speed_plan_pair_list.emplace_back(sample_s,
                                                  std::move(speed_point));
    cache_data_.lon_s_pair_list.emplace_back(sample_s, vel_point.distance);
    cache_data_.s_t_pair_list.emplace_back(sample_s, vel_point.relative_time);
  }
}

size_t LateralMotionPlannerPreprocessor::QueryNearestPoint(
    const std::vector<maf_planning::PathPoint> &pre_path,
    const planning_math::Vec2d &position, const double buffer) {
  double dist_sqr_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < pre_path.size(); ++i) {
    const planning_math::Vec2d curr_point(pre_path[i].position_enu.x,
                                          pre_path[i].position_enu.y);

    const double dist_sqr = curr_point.DistanceSquareTo(position);
    if (dist_sqr < dist_sqr_min + buffer) {
      dist_sqr_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void LateralMotionPlannerPreprocessor::set_intelligent_dodge_info() {
  const auto &pre_dodge_info =
      context_->lateral_motion_planner_output().pre_dodge_info;
  const auto &path_planner_input = context_->mutable_path_planner_input();
  if (world_model_->is_acc_mode() || !world_model_->get_vehicle_dbw_status()) {
    path_planner_input->dodge_info.reset();
    path_planner_input->dodge_info.lc_count = pre_dodge_info.lc_count;
    path_planner_input->dodge_info.cp_count = 0;
    path_planner_input->dodge_info.lc_end_clear_count = false;
  } else if (path_planner_input->lc_decider_info.lc_status !=
                 path_planner::LaneChangeInfo::LaneKeep ||
             path_planner_input->lc_decider_info.lc_wait_dir != "none" ||
             path_planner_input->lc_decider_info.pre_lc_wait) {
    path_planner_input->dodge_info.reset();
    path_planner_input->dodge_info.lc_count = 0;
    path_planner_input->dodge_info.cp_count = pre_dodge_info.cp_count;
    path_planner_input->dodge_info.lc_end_clear_count = true;
  } else {
    path_planner_input->dodge_info = pre_dodge_info;
    path_planner_input->dodge_info.lc_end_clear_count = false;
  }
}

void LateralMotionPlannerPreprocessor::set_dlp_info() {
  const auto &pre_dlp_info =
      context_->lateral_motion_planner_output().pre_dlp_info;
  const auto &path_planner_input = context_->mutable_path_planner_input();
  path_planner_input->pre_dlp_info = pre_dlp_info;
}

} // namespace msquare
