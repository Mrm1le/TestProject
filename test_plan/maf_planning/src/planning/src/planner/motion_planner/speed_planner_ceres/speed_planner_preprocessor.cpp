#include "planner/motion_planner/speed_planner_ceres/speed_planner_preprocessor.h"
#include "acc_curve.hpp"
#include "common/config_context.h"
#include "common/obstacle_process_utils.h"
#include "common/trajectory/bounded_constant_jerk_trajectory1d.h"
#include "common/trajectory/smooth_brake_trajectory.h"
#include "common/world_model.h"
#include "data_driven_planner/common/ddp_context.h"
#include "linear_interpolation_utility.hpp"
#include "path_planner_constants.hpp"
#include "planner/behavior_planner/deciders/st_graph_generator.h"
#include "planner/motion_planner/planner_cubic_spline/planner_cubic_spline_utility.hpp"
#include "planner/motion_planner/speed_planner_ceres/obstacle_headway_preprocessor.h"
#include "planner/motion_planner/speed_planner_ceres/speed_planner_constants.hpp"
#include "planning/common/logging.h"
#include <iomanip>
#include <mtime_core/time.h>

using namespace path_planner;
using namespace speed_planner;
using namespace planner_spline;
using namespace msquare::planning_math;

namespace msquare {
const double K_cycle_s = 0.1;
const double K_kph2mps = 1 / 3.6;
const double K_postpone_stop_dist = 1.5;
const double K_stop_entry_CIPV_spd_mps = 2 * K_kph2mps;
const double K_stop_entry_ego_spd_mps = 10 * K_kph2mps;
const double K_stop_rolling_dist = 1.5;
const double K_stop_standstill_spd_mps = 2 * K_kph2mps;
const double K_stop_standstill_dist = 5.5;
const double K_stop_slip_back_spd_mps = 5 * K_kph2mps;
const double K_stop_throttle_go = 3;
const double K_stop_exit_CIPV_spd_mps = 2;
const double K_stop_exit_relv_mps = 1;
const double K_stop_ego_hold_spd_mps = 0.5 * K_kph2mps;
const double K_KickStart_spd_mps = 1.5 * K_kph2mps;

const double K_stop_go_CIPV_spd_mps = 0.75;
const double K_stop_go_CIPV_dist_offset = 2;
const double K_stop_autogo_time_s = std::numeric_limits<double>::infinity();

const double K_kick_start_time_s = 10.0;

inline std::vector<double>
get_a_max_limit_according_driving_style(const DrivingStyle &driving_style) {
  switch (driving_style) {
  case DrivingStyle::DRIVING_STYLE_AGGRESSIVE:
    return AGGRESSIVE_A_MAX_VALUE_PT;
  case DrivingStyle::DRIVING_STYLE_NORMAL:
    return NORMAL_A_MAX_VALUE_PT;
  case DrivingStyle::DRIVING_STYLE_CONSERVATIVE:
    return CONSERVATIVE_A_MAX_VALUE_PT;
  default:
    return NORMAL_A_MAX_VALUE_PT;
  }
}

inline std::vector<double>
get_a_max_limit_according_style(const bool is_lane_change,
                                const LaneChangingStyle &lane_changing_style,
                                const DrivingStyle &driving_style) {
  if (is_lane_change) {
    switch (lane_changing_style) {
    case LaneChangingStyle::LANECHANGING_STYLE_AGGRESSIVE:
      return AGGRESSIVE_A_MAX_VALUE_PT;
    case LaneChangingStyle::LANECHANGING_STYLE_NORMAL:
      return get_a_max_limit_according_driving_style(driving_style);
    case LaneChangingStyle::LANECHANGING_STYLE_CONSERVATIVE:
      return CONSERVATIVE_A_MAX_VALUE_PT;
    default:
      return get_a_max_limit_according_driving_style(driving_style);
    }
  } else {
    return get_a_max_limit_according_driving_style(driving_style);
  }
}

void LongitudinalPlannerPreprocessor::process() {
  update_last_speed_planner_result();

  generate_lon_obstacles_info();

  ProcessCipvFn();

  gerenate_s_at_control_quad_points();

  compute_planning_init_state();

  set_vaj_bound_and_reason();

  set_speed_planner_init_params();

  select_cipv();

  update_stop_mode();

  stop_flag_decision();

  go_flag_decision();

  update_start_state();

  calc_accel_limit();

  generate_radar_info();

  set_vehicle_params();

  compute_stopping_point_info();

  calc_stop_ref();

  compute_overlap_info();
}

void LongitudinalPlannerPreprocessor::update_last_speed_planner_result() {
  const auto &pre_speed_planner_output = context_->speed_planner_output();
  cache_data_.prev_speed_planner_output_pair_list.clear();

  for (const auto &point : pre_speed_planner_output.path_planner_output) {
    cache_data_.prev_speed_planner_output_pair_list.emplace_back(point.t,
                                                                 point);
  }
}

void LongitudinalPlannerPreprocessor::gerenate_s_at_control_quad_points() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const double SPEED_PLANNER_TIME = 5.0;
  const double MAX_LENGTH = 120.0;
  const double MIN_LENGTH = 5.0;
  const auto &planning_init_point =
      baseline_info_->get_ego_state().planning_init_point;

  double total_length =
      std::fmin(std::fmax(15.0, planning_init_point.v * SPEED_PLANNER_TIME),
                baseline_info_->get_frenet_coord()->GetLength() -
                    planning_init_point.path_point.s - fast_math::MathEpsilon);
  total_length = std::fmax(MIN_LENGTH, std::fmin(total_length, MAX_LENGTH));

  if (speed_planner_input->stop_point_info.has_stop_point) {
    total_length =
        std::fmin(total_length,
                  std::fmax(5.0, speed_planner_input->stop_point_info.stop_s +
                                     5.0 - planning_init_point.path_point.s));
  }

  double delta_s = total_length / double(NUM_SPEED_SEGMENTS);

  // control point
  for (size_t segment_index = 0; segment_index < NUM_SPEED_CONTROL_POINTS;
       segment_index++) {
    s_at_control_points_.at(segment_index) =
        delta_s * segment_index + planning_init_point.path_point.s;
  }

  // quad point
  Index<NUM_SPEED_CONTROL_POINTS> index;
  while (index.advance()) {
    s_at_quad_points_[index.segment_index][index.quad_index] =
        s_at_control_points_[index.segment_index] +
        (s_at_control_points_[index.segment_index + 1] -
         s_at_control_points_[index.segment_index]) *
            GAUSS_QUAD_5TH_POS[index.quad_index];
  }

  speed_planner_input->s_at_control_points = s_at_control_points_;
}

void LongitudinalPlannerPreprocessor::set_curve_speed_request() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  auto &speed_segments =
      context_->mutable_speed_planner_input()->speed_segments;
  const std::array<double, 2> centri_acc_xp{60 / 3.6, 100 / 3.6};
  const std::array<double, 2> centri_acc_fp{0.504, 0.7};
  const double K_CurveAxMax = 2.5;
  const double K_CurveAxMin = -2.5;
  const double K_CurveJerkMax = 2.5;
  const double K_CurveJerkMin = -2.5;
  const auto &ego_vel =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
  const auto &dbw_status = world_model_->get_vehicle_dbw_status();
  double centri_acc_decay_ratio = interp(ego_vel, centri_acc_xp, centri_acc_fp);

  double v_curv = world_model_->get_v_curv();
  double a_curv = world_model_->get_a_curv();
  speed_planner_input->v_curv = v_curv;
  speed_planner_input->a_curv = a_curv;
  speed_planner_input->planning_init_v =
      baseline_info_->get_ego_state().planning_init_point.v;
  speed_planner_input->max_curv_s = world_model_->get_max_curv_distance();
  speed_planner_input->max_curv = world_model_->get_max_curv();
  // set curvature speed limit
  Index<NUM_SPEED_CONTROL_POINTS> index;
  while (index.advance()) {
    auto &quad_point_info =
        speed_segments[index.segment_index][index.quad_index].quad_point_info;

    // hack, receive from original curv limit
    double distance =
        std::max(quad_point_info.sample_s -
                     baseline_info_->get_ego_state().planning_start_state.s,
                 0.0);
    double planning_init_v =
        baseline_info_->get_ego_state().planning_init_point.v;
    double v_max_smooth = v_curv;

    if (v_curv < planning_init_v) {
      double curv_distance =
          (v_curv * v_curv - planning_init_v * planning_init_v) /
          (2 * std::min(a_curv, -0.01));
      if (distance < curv_distance) {
        v_max_smooth = std::sqrt(std::max(
            2 * a_curv * distance + planning_init_v * planning_init_v, 0.0));
      } else {
        double acc_distance = distance - curv_distance;
        auto lane_status = context_->planning_status().lane_status;
        bool is_lane_change =
            lane_status.status == LaneStatus::Status::LANE_CHANGE;
        const auto a_max_value_pt = get_a_max_limit_according_style(
            is_lane_change, world_model_->get_lane_changing_style(),
            world_model_->get_driving_style());

        double a_max = LinearInterpation::interpolation(
            planning_init_v, A_MAX_SPEED_PT, a_max_value_pt);
        v_max_smooth = std::sqrt(
            std::max(2 * a_max * acc_distance + v_curv * v_curv, 0.0));
      }

      MSD_LOG(INFO,
              "curv_speed: v_curv: %f, a_curv: %f, curv_distance: %f, "
              "v_max_smooth: %f, distance: %f",
              v_curv, a_curv, curv_distance, v_max_smooth, distance);
    }

    // set v_max for mrc inlane stop
    bool is_mrc_inlane_brake = world_model_->is_mrc_inlane_brake();
    if (is_mrc_inlane_brake) {
      double mrc_distance = planning_init_v * planning_init_v / (2 * 1.0);
      if (distance < mrc_distance) {
        v_max_smooth = std::sqrt(std::max(
            2 * -1.0 * distance + planning_init_v * planning_init_v, 0.0));
      } else {
        v_max_smooth = 0.0;
      }
      if (planning_init_v < 1.0) {
        v_max_smooth = 0.0;
      }
    }
    MSD_LOG(INFO,
            "quad_point_info.v_max = %.2f, quad_point_info.v_ref = %.2f, "
            "v_max_smooth = %.2f",
            quad_point_info.v_max, quad_point_info.v_ref, v_max_smooth);

    if (quad_point_info.v_max > v_max_smooth) {
      quad_point_info.v_max = v_max_smooth;
      quad_point_info.v_max_reason = "curvature_limit";
      if (is_mrc_inlane_brake) {
        quad_point_info.v_max_reason = "mrc_inlane_brake";
      }
    }

    if (quad_point_info.v_ref > v_max_smooth) {
      quad_point_info.v_ref = v_max_smooth;
      quad_point_info.v_ref_reason = "curvature_limit";
      if (is_mrc_inlane_brake) {
        quad_point_info.v_ref_reason = "mrc_inlane_brake";
      }
    }
  }
}

void LongitudinalPlannerPreprocessor::set_env_speed_limit_request() {
  auto &lon_obstacle_decision_info_map =
      context_->mutable_lon_decison_output()->obstacle_decision_info_map;
  const auto normal_obstacles_items =
      baseline_info_->obstacle_manager().get_obstacles().Items();
  const EgoState &ego_state = baseline_info_->get_ego_state();
  const std::vector<double> K_LatDistLine_m = {-1.75, -0.85, -0.2, 0, 0.1};
  const std::vector<double> K_RelVLim_kph = {200, 30, 10, 5, 0};
  auto &speed_segments =
      context_->mutable_speed_planner_input()->speed_segments;
  double ego_vx = ego_state.ego_vel;
  double env_speed_lim = 200 / 3.6;
  double avg_ax_min = 2.0;
  double env_lim_dist = 200.0;
  MSD_LOG(INFO, "ENV:TIME:%.3f", MTIME()->timestamp().sec());
  for (const auto &obstacle : normal_obstacles_items) {
    if (lon_obstacle_decision_info_map.find(obstacle->Id()) !=
        lon_obstacle_decision_info_map.end()) {
      const auto &lon_decision = lon_obstacle_decision_info_map[obstacle->Id()];
      double virtual_overlap = lon_decision.virtual_overlap;
      double obj_speed = obstacle->speed();
      MSD_LOG(INFO, "ENV:ID:%d:==================", obstacle->Id());
      MSD_LOG(INFO, "ENV:ID:%d, overlap:%.2f", obstacle->Id(), virtual_overlap);
      MSD_LOG(INFO, "ENV:ID:%d, speed:%.2f", obstacle->Id(), obj_speed);
      MSD_LOG(INFO, "ENV:ID:%d, obs_s:%.2f", obstacle->Id(),
              obstacle->S_frenet());
      MSD_LOG(INFO, "ENV:ID:%d, obs_r:%.2f", obstacle->Id(),
              obstacle->R_frenet());
      MSD_LOG(INFO, "ENV:ID:%d, ego_s:%.2f", obstacle->Id(),
              ego_state.ego_frenet.x);
      double half_length =
          ConfigurationContext::Instance()->get_vehicle_param().length / 2;
      if (obj_speed < 20 / 3.6 &&
          obstacle->S_frenet() - ego_state.ego_frenet.x > half_length) {
        double theta =
            planning_math::NormalizeAngle(obstacle->Yaw_relative_frenet());
        double obj_vx = obj_speed * std::cos(theta);
        double rel_v_lim = 100.0;
        double dist = obstacle->S_frenet() - ego_state.ego_frenet.x - 5.0;
        dist = std::fmax(dist, 2.0);
        MSD_LOG(INFO, "ENV:ID:%d, dist:%.2f", obstacle->Id(), dist);
        if (virtual_overlap > K_LatDistLine_m.back()) {
          rel_v_lim = K_RelVLim_kph.back();
        } else if (virtual_overlap < K_LatDistLine_m.front()) {
          rel_v_lim = K_RelVLim_kph.front();
        } else {
          for (int i = 0; i < K_LatDistLine_m.size() - 1; ++i) {
            if (virtual_overlap < K_LatDistLine_m[i + 1]) {
              rel_v_lim = (virtual_overlap - K_LatDistLine_m[i]) /
                              (K_LatDistLine_m[i + 1] - K_LatDistLine_m[i]) *
                              (K_RelVLim_kph[i + 1] - K_RelVLim_kph[i]) +
                          K_RelVLim_kph[i];
              break;
            }
          }
        }
        if (lon_decision.in_lane == true &&
            virtual_overlap > K_LatDistLine_m[1]) {
          rel_v_lim = std::fmin(rel_v_lim, K_RelVLim_kph[1]);
        }
        rel_v_lim /= 3.6;
        MSD_LOG(INFO, "ENV:ID:%d, rel_v_lim:%.2f", obstacle->Id(), rel_v_lim);
        MSD_LOG(INFO, "ENV:ID:%d, raw_lim:%.2f", obstacle->Id(),
                rel_v_lim + obj_vx);
        MSD_LOG(INFO, "ENV:ID:%d, ego_vx:%.2f", obstacle->Id(), ego_vx);

        double avg_ax = -(ego_vx - rel_v_lim - obj_vx) *
                        (ego_vx + rel_v_lim + obj_vx) / 2 / dist;
        avg_ax = std::fmin(std::fmax(avg_ax, -5), 2);
        if (avg_ax < avg_ax_min && lon_decision.is_follow == false &&
            virtual_overlap < K_LatDistLine_m.back()) {
          env_speed_lim = rel_v_lim + obj_vx;
          env_lim_dist = dist;
          avg_ax_min = avg_ax;
        }
        MSD_LOG(INFO, "ENV:ID:%d, avg_ax:%.2f", obstacle->Id(), avg_ax);
        MSD_LOG(INFO, "ENV:ID:%d, obj_vx:%.2f", obstacle->Id(), obj_vx);
      }
      MSD_LOG(INFO, "ENV:ID:%d, env_speed_lim:%.2f", obstacle->Id(),
              env_speed_lim);
      MSD_LOG(INFO, "ENV:ID:%d, planning_init_v:%.2f", obstacle->Id(),
              baseline_info_->get_ego_state().planning_init_point.v);
      MSD_LOG(INFO, "ENV:ID:%d, env_lim_dist:%.2f", obstacle->Id(),
              env_lim_dist);
      MSD_LOG(INFO, "ENV:ID:%d, avg_ax_min:%.2f", obstacle->Id(), avg_ax_min);
    }
  }
  Index<NUM_SPEED_CONTROL_POINTS> index;
  const double K_env_min_ax = -1.5;
  double a_env = std::fmax(K_env_min_ax, avg_ax_min);
  while (index.advance()) {
    auto &quad_point_info =
        speed_segments[index.segment_index][index.quad_index].quad_point_info;

    // hack, receive from original curv limit
    double distance =
        std::fmax(quad_point_info.sample_s -
                      baseline_info_->get_ego_state().planning_start_state.s,
                  0.0);
    double planning_init_v =
        baseline_info_->get_ego_state().planning_init_point.v;
    double v_max_smooth = env_speed_lim;
    MSD_LOG(INFO, "ENV:ct.s:%.2f", distance);
    if (env_speed_lim < planning_init_v) {
      if (distance < env_lim_dist) {
        v_max_smooth = std::sqrt(std::fmax(
            2 * a_env * distance + planning_init_v * planning_init_v, 0.0));
      } else {
        double acc_distance = distance - env_lim_dist;
        auto lane_status = context_->planning_status().lane_status;
        bool is_lane_change =
            lane_status.status == LaneStatus::Status::LANE_CHANGE;
        const auto a_max_value_pt = get_a_max_limit_according_style(
            is_lane_change, world_model_->get_lane_changing_style(),
            world_model_->get_driving_style());

        double a_max = LinearInterpation::interpolation(
            planning_init_v, A_MAX_SPEED_PT, a_max_value_pt);
        double env_spd_min = std::sqrt(std::fmax(
            2 * a_env * env_lim_dist + planning_init_v * planning_init_v, 0.0));
        v_max_smooth = std::sqrt(std::fmax(
            2 * a_max * acc_distance + env_spd_min * env_spd_min, 0.0));
      }
      MSD_LOG(INFO, "ENV:ct.s:%.2f,v_max_smooth:%.2f", distance, v_max_smooth);
    }
    if (quad_point_info.v_max > v_max_smooth) {
      quad_point_info.v_max = v_max_smooth;
      quad_point_info.v_max_reason = "env_limit";
    }

    if (quad_point_info.v_ref > v_max_smooth) {
      quad_point_info.v_ref = v_max_smooth;
      quad_point_info.v_ref_reason = "env_limit";
    }
  }
}

void LongitudinalPlannerPreprocessor::set_model_speed_request() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  speed_planner_input->model_trajectory.clear();
  const auto &ddp_traj = ddp::DdpContext::Instance()
                             ->get_obstacle_decider_ddp_trajectory()
                             .trajectory;
  for (int i = 0; i < ddp_traj.size(); i++) {
    ModelPoint tmp_point{};
    tmp_point.t = i * 0.2;
    tmp_point.v = ddp_traj[i].v;
    tmp_point.a = ddp_traj[i].a;
    tmp_point.s = ddp_traj[i].s;
    tmp_point.x = ddp_traj[i].x;
    tmp_point.y = ddp_traj[i].y;
    tmp_point.heading_angle = ddp_traj[i].heading_angle;
    speed_planner_input->model_trajectory.emplace_back(tmp_point);
  }

  if (ddp_traj.size() > 2 && world_model_->use_eftp()) {
    speed_planner_input->use_eftp = true;
  } else {
    speed_planner_input->use_eftp = false;
  }

  compute_overlap_info();

  speed_planner_input->b_dagger_longitudinal =
      ConfigurationContext::Instance()
          ->planner_config()
          .common_config.b_dagger_longitudinal;

  planning_math::InterpolationData<speed_planner::ModelPoint>
      model_point_pair_list;

  for (const auto &point : speed_planner_input->model_trajectory) {
    model_point_pair_list.emplace_back(point.s, point);
  }

  // set model speed reference
  if (model_point_pair_list.size()) {
    Index<NUM_SPEED_CONTROL_POINTS> index;
    while (index.advance()) {
      auto &quad_point_info =
          speed_planner_input
              ->speed_segments[index.segment_index][index.quad_index]
              .quad_point_info;
      auto model_point_interpolate = LinearInterpation::interpolate(
          model_point_pair_list, quad_point_info.sample_s);

      if (quad_point_info.v_ref > model_point_interpolate.v) {
        quad_point_info.v_ref = model_point_interpolate.v;
        quad_point_info.v_ref_reason = "model_ref_speed";
      }
    }
  }
}

void LongitudinalPlannerPreprocessor::set_select_gap_speed_request() {
  const auto &select_gap_decision_info =
      PlanningContext::Instance()->general_motion_planner_output();
  const auto &speed_planner_input = context_->mutable_speed_planner_input();

  if (select_gap_decision_info.gmp_valid == true) {
    speed_planner_input->gmp_valid = true;
  } else {
    speed_planner_input->gmp_valid = false;
  }

  // set model speed reference
  if (select_gap_decision_info.gmp_valid == true) {
    Index<NUM_SPEED_CONTROL_POINTS> index;
    while (index.advance()) {
      auto &quad_point_info =
          speed_planner_input
              ->speed_segments[index.segment_index][index.quad_index]
              .quad_point_info;
      const auto &s_v_array_out =
          select_gap_decision_info
              .s_v_array_out[index.segment_index][index.quad_index];

      quad_point_info.v_ref = s_v_array_out.v_ref;
      quad_point_info.v_ref_reason = "select_gap_decision";
      if (quad_point_info.v_max > s_v_array_out.v_max) {
        quad_point_info.v_max = s_v_array_out.v_max;
        quad_point_info.v_max_reason = "select_gap_decision";
      }
    }
  }
}

void LongitudinalPlannerPreprocessor::set_cipv_lost_speed_request() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  auto &cipv_lost_info = speed_planner_input->cipv_lost_info;
  const bool prohibit_acc = cipv_lost_info.prohibit_acc;
  const double speed_limit = cipv_lost_info.speed_limit;

  auto &speed_segments =
      context_->mutable_speed_planner_input()->speed_segments;
  if (prohibit_acc) {
    Index<NUM_SPEED_CONTROL_POINTS> index;
    while (index.advance()) {
      auto &quad_point_info =
          speed_segments[index.segment_index][index.quad_index].quad_point_info;
      if (quad_point_info.v_max > speed_limit) {
        quad_point_info.v_max = speed_limit;
        quad_point_info.v_max_reason = "cipv_lost";
      }
    }
  }
}

void LongitudinalPlannerPreprocessor::update_speed_limit_info(
    speed_planner::SpeedLimitSet &speed_limit_set, const double &v_ego) {
  const double K_CycleTime_s = 0.1;
  const double K_DeltaVmax_mps = 6;
  double delta_v = 0;
  double delta_v_max = speed_limit_set.a_max * K_CycleTime_s;
  double delta_v_min = speed_limit_set.a_min * K_CycleTime_s;
  delta_v = speed_limit_set.v_max - speed_limit_set.v_max_smooth;
  delta_v = std::fmax(delta_v, delta_v_min);
  delta_v = std::fmin(delta_v, delta_v_max);
  speed_limit_set.v_max_smooth += delta_v;
  if ((speed_limit_set.v_max_smooth - v_ego) * (speed_limit_set.v_max - v_ego) <
      0) {
    speed_limit_set.v_max_smooth = v_ego;
  } else if ((speed_limit_set.v_max_smooth - speed_limit_set.v_max) *
                 (v_ego - speed_limit_set.v_max) <
             0) {
    speed_limit_set.v_max_smooth = speed_limit_set.v_max;
  }
  speed_limit_set.v_max_smooth =
      std::fmax(speed_limit_set.v_max_smooth, v_ego - K_DeltaVmax_mps);
  speed_limit_set.v_max_smooth =
      std::fmin(speed_limit_set.v_max_smooth, v_ego + K_DeltaVmax_mps);
  MSD_LOG(INFO, "v_max:%f,v_max_smooth:%f,dv_max:%f,dv_min:%f",
          speed_limit_set.v_max, speed_limit_set.v_max_smooth, delta_v_max,
          delta_v_min);
  delta_v = speed_limit_set.v_min - speed_limit_set.v_min_smooth;
  delta_v = std::fmax(delta_v, delta_v_min);
  delta_v = std::fmin(delta_v, delta_v_max);
  speed_limit_set.v_min_smooth += delta_v;
  if ((speed_limit_set.v_min_smooth - v_ego) * (speed_limit_set.v_min - v_ego) <
      0) {
    speed_limit_set.v_min_smooth = v_ego;
  } else if ((speed_limit_set.v_min_smooth - speed_limit_set.v_min) *
                 (v_ego - speed_limit_set.v_min) <
             0) {
    speed_limit_set.v_min_smooth = speed_limit_set.v_min;
  }
}

void LongitudinalPlannerPreprocessor::set_speed_limit_request() {
  const double K_CruiseAxMax = 2.5;
  const double K_CruiseAxMin = -1.25;
  const double K_CruiseJerkMax = 1.5;
  const double K_CruiseJerkMin = -1.25;
  const double K_CruiseDeltaT_s = 0.1;
  const auto &ego_vel =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
  const auto &dbw_status = world_model_->get_vehicle_dbw_status();
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  auto &speed_segments = speed_planner_input->speed_segments;
  const auto &init_state = speed_planner_input->planning_init_state;
  const auto &v_cruise = world_model_->get_map_info().v_cruise();

  bool is_need_cruise_accelerate = v_cruise > init_state.v ? true : false;
  BoundedConstantJerkTrajectory1d smooth_traj(
      init_state.s, init_state.v, init_state.a,
      (is_need_cruise_accelerate ? K_CruiseJerkMax : K_CruiseJerkMin),
      K_CruiseDeltaT_s);
  smooth_traj.set_bound((is_need_cruise_accelerate ? init_state.v : v_cruise),
                        (is_need_cruise_accelerate ? v_cruise : init_state.v),
                        (is_need_cruise_accelerate ? 0.0 : K_CruiseAxMin),
                        (is_need_cruise_accelerate ? K_CruiseAxMax : 0.0));

  InterpolationData<double> smooth_traj_pair{};
  const double forward_time = 5.0;
  for (int i = 0; i < forward_time / K_CruiseDeltaT_s; i++) {
    smooth_traj_pair.emplace_back(
        smooth_traj.evaluate(0, i * K_CruiseDeltaT_s),
        smooth_traj.evaluate(1, i * K_CruiseDeltaT_s));
  }

  // init speed limit info
  Index<NUM_SPEED_CONTROL_POINTS> index;
  while (index.advance()) {
    auto &speed_segment_info =
        speed_segments[index.segment_index][index.quad_index];
    speed_segment_info.end_segment_control_point_s =
        s_at_control_points_[index.segment_index + 1];

    auto &cruise_speed_set =
        speed_planner_input->speed_limit_info
            .cruise_speed_set[index.segment_index][index.quad_index];
    cruise_speed_set.a_max = K_CruiseAxMax;
    cruise_speed_set.a_min = K_CruiseAxMin;
    cruise_speed_set.j_max = K_CruiseJerkMax;
    cruise_speed_set.j_min = K_CruiseJerkMin;
    const auto &sample_s =
        s_at_quad_points_[index.segment_index][index.quad_index];

    double smooth_v =
        LinearInterpation::interpolate(smooth_traj_pair, sample_s);
    speed_segment_info.quad_point_info.v_max = smooth_v;
    speed_segment_info.quad_point_info.v_max_reason = "cruise_speed";
    speed_segment_info.quad_point_info.v_ref = smooth_v;
    speed_segment_info.quad_point_info.v_ref_reason = "cruise_speed";
    speed_segment_info.quad_point_info.v_min = 0.0;
    speed_segment_info.quad_point_info.v_min_reason = "none";
    speed_segment_info.quad_point_info.sample_s = sample_s;
  }
}

void LongitudinalPlannerPreprocessor::set_vaj_bound_and_reason() {
  const auto &ego_vel =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();
  // set a bound and reason
  double max_acc_according_ego_vel =
      std::fmax(1.0, MAX_ACC - MAX_ACC_DECREASE_RATE * ego_vel);
  speed_planner_input->vaj_bound_info.a_max = max_acc_according_ego_vel;
  speed_planner_input->vaj_bound_info.a_max_reason =
      "init_acc_according_ego_vel";

  // todo tmp set -2.0
  speed_planner_input->vaj_bound_info.a_min = -5.0;
  speed_planner_input->vaj_bound_info.a_min_reason = "init";

  // set j bound and reason
  // todo tem set max element
  speed_planner_input->vaj_bound_info.j_max = 0.0;

  // todo tem set min element
  speed_planner_input->vaj_bound_info.j_min = 0.0;
  speed_planner_input->vaj_bound_info.j_min_reason = "init";

  auto lane_status = context_->planning_status().lane_status;
  bool is_lane_change = lane_status.status == LaneStatus::Status::LANE_CHANGE;
  const auto a_max_value_pt = get_a_max_limit_according_style(
      is_lane_change, world_model_->get_lane_changing_style(),
      world_model_->get_driving_style());

  speed_planner_input->vaj_bound_info.a_max_value_pt = a_max_value_pt;

  if (context_->speed_planner_input().cipv_lost_info.prohibit_acc) {
    speed_planner_input->vaj_bound_info.a_max_spd_pt = {0.0, 0.0, 0.0, 0.0,
                                                        0.0};
  } else {
    speed_planner_input->vaj_bound_info.a_max_spd_pt = A_MAX_SPEED_PT;
  }
  speed_planner_input->vaj_bound_info.a_min_value_pt =
      planner_config.longitudinal_motion_planner_config.a_min_value_pt;
  speed_planner_input->vaj_bound_info.a_min_spd_pt = A_MIN_SPEED_PT;
  speed_planner_input->vaj_bound_info.j_min_value_pt =
      planner_config.longitudinal_motion_planner_config.j_min_value_pt;
  speed_planner_input->vaj_bound_info.j_min_spd_pt = J_MIN_SPEED_PT;
  speed_planner_input->vaj_bound_info.j_max_value_pt = J_MAX_VALUE_PT;
  speed_planner_input->vaj_bound_info.j_max_spd_pt = J_MAX_SPEED_PT;

  MSD_LOG(INFO, "DBACC&JERK=================");
  MSD_LOG(INFO, "DBACC&JERK acc0: %f",
          speed_planner_input->vaj_bound_info.a_min_value_pt[0]);
  MSD_LOG(INFO, "DBACC&JERK acc1: %f",
          speed_planner_input->vaj_bound_info.a_min_value_pt[1]);
  MSD_LOG(INFO, "DBACC&JERK jerk0: %f",
          speed_planner_input->vaj_bound_info.j_min_value_pt[0]);
  MSD_LOG(INFO, "DBACC&JERK jerk1: %f",
          speed_planner_input->vaj_bound_info.j_min_value_pt[1]);

  // limit ax for intersection
  bool limit_ax = false;
  const int limit_ax_count_thres = 10;
  static int limit_ax_count = 0;
  limit_ax_count = std::max(limit_ax_count - 1, 0);
  if (world_model_->is_ddmap_intersection()) {
    limit_ax_count = limit_ax_count_thres;
    MSD_LOG(WARN, "AX_DEBUG: limit for intersection");
  }

  double ego_r = baseline_info_->get_ego_state().ego_frenet.y;
  static double last_ego_r = ego_r;
  if (lane_status.status != LaneStatus::Status::LANE_CHANGE) {
    // limit ax for lane jitter
    const double lane_jitter_thres = 0.5;
    if (std::abs(ego_r - last_ego_r) > lane_jitter_thres) {
      limit_ax_count = limit_ax_count_thres;
      MSD_LOG(WARN, "AX_DEBUG: limit for lane jitter");
    }
  }
  last_ego_r = ego_r;
  if (limit_ax_count > 0) {
    limit_ax = true;
  }
  speed_planner_input->vaj_bound_info.is_ddmap_intersection = limit_ax;

  // set v bound and reason
  compute_speed_segments();

  speed_planner_input->throttle_override =
      world_model_->get_throttle_override();
  speed_planner_input->is_ACC_mode = world_model_->is_acc_mode();
  speed_planner_input->set_hw_level = world_model_->get_navi_ttc_gear();
  speed_planner_input->set_speed = int(
      world_model_->get_vehicle_status().velocity.cruise_velocity.value_mps *
      3.6);
  speed_planner_input->dbw_status = world_model_->get_vehicle_dbw_status();
}

void LongitudinalPlannerPreprocessor::set_speed_planner_init_params() {
  // set int path_planner_params
  auto &speed_tuning_params =
      context_->mutable_speed_planner_input()->speed_tuning_params;
  auto &stop_point_params =
      context_->mutable_speed_planner_input()->stop_point_info;
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();
  const auto &cipv_info = context_->mutable_speed_planner_input()->cipv_info;

  speed_tuning_params.max_num_iterations = speed_planner::MAX_NUM_ITERATIONS;
  if (context_->speed_planner_input().cipv_lost_info.prohibit_acc) {
    constexpr double K_CipvLostAccLimitFac = 10000.0;
    speed_tuning_params.accel_limit_scale =
        ACCELERATION_LIMIT_SCALE * K_CipvLostAccLimitFac;
  } else {
    speed_tuning_params.accel_limit_scale = ACCELERATION_LIMIT_SCALE;
  }
  speed_tuning_params.accel_jerk_ratio = ACCEL_JERK_RATIO;
  speed_tuning_params.accel_scale = ACCELERATION_SCALE;
  speed_tuning_params.jerk_limit_scale = JERK_LIMIT_SCALE;
  speed_tuning_params.jerk_scale = JERK_SCALE;
  speed_tuning_params.stop_point_scale = STOP_POINT_SCALE;
  speed_tuning_params.model_ref_scale = MODEL_REF_SCALE;
  speed_tuning_params.model_safety_scale = MODEL_SAFETY_SCALE;
  speed_tuning_params.gmp_ref_scale = GMP_REF_SCALE;

  speed_tuning_params.safe_v_limit_scale = SAFE_V_LIMIE_SCALE;
  speed_tuning_params.accel_ceiling_scale = ACCEL_CEILING_SCALE;
  speed_tuning_params.cruise_scale = CRUISE_SCALE;
  speed_tuning_params.deadband_region_ratio = DEADBAND_REGION_RATIO;
  speed_tuning_params.deadband_limit_scale = DEADBAND_LIMIT_SCALE;
  speed_tuning_params.accel_ceiling_cost_scale = ACCEL_CEILING_COST_SCALE;
  speed_tuning_params.acc_decel_region_scale =
      planner_config.longitudinal_motion_planner_config.acc_decel_region_scale;
  speed_tuning_params.acc_accel_region_scale = ACC_ACCEL_REGION_SCALE;
  speed_tuning_params.acc_v_max_scale = ACC_VMAX_SCALE;

  stop_point_params.stop_target_accel =
      planner_config.longitudinal_motion_planner_config.stop_target_accel;
  stop_point_params.stop_target_accel_entry_velocity =
      planner_config.longitudinal_motion_planner_config
          .stop_target_accel_entry_velocity;
  stop_point_params.stop_target_accel_entry_distance =
      planner_config.longitudinal_motion_planner_config
          .stop_target_accel_entry_distance;
  if (planner_config.longitudinal_motion_planner_config.is_reduce_stop_offset &&
      cipv_info.type == path_planner::ObsInfo::COUPE) {
    stop_point_params.stop_offset =
        planner_config.longitudinal_motion_planner_config.couple_stop_offset -
        K_postpone_stop_dist;
  } else {
    stop_point_params.stop_offset =
        planner_config.longitudinal_motion_planner_config.couple_stop_offset;
  }
}

void LongitudinalPlannerPreprocessor::compute_planning_init_state() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  const auto &planning_init_point = ego_state.planning_init_point;
  const auto &planning_start_frenet_state = ego_state.planning_start_state;
  bool force_stop = world_model_->is_force_stop();
  auto &stop_point_info = speed_planner_input->stop_point_info;
  if (world_model_->get_throttle_override() &&
      stop_point_info.last_force_stop) {
    stop_point_info.driver_go_timer = 1;
  }
  if (!world_model_->get_throttle_override() &&
      stop_point_info.driver_go_timer > 0) {
    stop_point_info.driver_go_timer++;
  }
  if (stop_point_info.driver_go_timer > 5) {
    stop_point_info.driver_go_timer = 0;
  }
  stop_point_info.last_force_stop = force_stop;
  speed_planner_input->ego_state = {};
  speed_planner_input->is_replan = ego_state.flag_is_replan;
  speed_planner_input->ego_state.s = ego_state.ego_frenet.x;
  speed_planner_input->ego_state.l = ego_state.ego_frenet.y;
  speed_planner_input->ego_state.v = ego_state.ego_vel;

  speed_planner_input->planning_init_state = {};
  speed_planner_input->planning_init_state.s = planning_init_point.path_point.s;
  speed_planner_input->planning_init_state.l = planning_start_frenet_state.r;
  speed_planner_input->planning_init_state.v = planning_init_point.v;
  speed_planner_input->planning_init_state.a = planning_init_point.a;
  speed_planner_input->planning_init_state.x = planning_init_point.path_point.x;
  speed_planner_input->planning_init_state.y = planning_init_point.path_point.y;
  if (stop_point_info.driver_go_timer > 0) {
    speed_planner_input->planning_init_state.a = ego_state.ego_acc;
  }
  // use last speed planner output to compute da_ds and jerk
  if (cache_data_.prev_speed_planner_output_pair_list.size() &&
      !ego_state.flag_is_replan) {
    auto point_interpolate = LinearInterpation::interpolate(
        cache_data_.prev_speed_planner_output_pair_list,
        1.0 / FLAGS_planning_loop_rate);
    speed_planner_input->planning_init_state.da_ds = point_interpolate.da_ds;
    speed_planner_input->planning_init_state.jerk = point_interpolate.jerk;
  } else {
    speed_planner_input->planning_init_state.da_ds = 0.0;
    speed_planner_input->planning_init_state.jerk = 0.0;
  }
  if (ego_state.flag_is_replan) {
    speed_planner_input->planning_init_state.v = ego_state.ego_vel;
    speed_planner_input->planning_init_state.da_ds = 0.0;
    speed_planner_input->planning_init_state.jerk = 0.0;
  }
}

void LongitudinalPlannerPreprocessor::compute_speed_segments() {
  set_speed_limit_request();

  if (!ConfigurationContext::Instance()
           ->planner_config()
           .common_config.b_dagger_longitudinal) {
    set_curve_speed_request();
  }

  if (ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.enable_env_speed_limit) {
    set_env_speed_limit_request();
  }

  set_model_speed_request();

  set_select_gap_speed_request();

  set_cipv_lost_speed_request();
}

void LongitudinalPlannerPreprocessor::generate_lon_obstacles_info() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  speed_planner_input->lon_obs.clear();

  ObstacleDecision lon_decison(0);
  ObjectDecisionType lon_decision_type{};
  lon_decision_type.mutable_follow();
  lon_decison.AddLongitudinalDecision("speed_planner_preprocessor",
                                      lon_decision_type);

  // lon_decision.mutable_overtake();
  std::array<std::array<double, QUADRATURE_ORDER>, NUM_SPEED_SEGMENTS>
      time_list;

  Index<NUM_SPEED_CONTROL_POINTS> index;
  while (index.advance()) {
    time_list[index.segment_index][index.quad_index] =
        (index.segment_index * QUADRATURE_ORDER + index.quad_index) * 5.0 /
        (QUADRATURE_ORDER * NUM_SPEED_SEGMENTS - 1);
  }

  speed_planner_input->lon_obs =
      generate_obstacle_info_with_decision<NUM_SPEED_SEGMENTS>(
          baseline_info_, context_, time_list, lon_decison,
          world_model_->is_mrc_inlane_brake());

  if (speed_planner_input->lon_obs.size() > 1) {
    std::sort(speed_planner_input->lon_obs.begin(),
              speed_planner_input->lon_obs.end(),
              [](const path_planner::ObsInfo &obs1,
                 const path_planner::ObsInfo &obs2) {
                return obs1.polygon_init.s < obs2.polygon_init.s;
              });
  }
  MSD_LOG(INFO, "DEBUG_IGNORE0706");
  for (int i = 0; i < speed_planner_input->lon_obs.size(); ++i) {
    MSD_LOG(INFO, "DEBUG_IGNORE0706:%d", speed_planner_input->lon_obs[i].id);
    MSD_LOG(INFO, "DEBUG_IGNORE0706:%d",
            speed_planner_input->lon_obs[i].lon_decision);
  }

  calc_obs_ax();

  update_object_prediction(time_list);

  ObstacleHeadWayPreprocessor obstacle_headway_preprocessor;
  obstacle_headway_preprocessor.init(context_, world_model_, baseline_info_);
  obstacle_headway_preprocessor.process();

  get_obs_overlap();

  const auto &lidar_rb_id_vec = baseline_info_->get_lidar_rb_id_vec();
  for (auto &obs : speed_planner_input->lon_obs) {
    if (std::find(lidar_rb_id_vec.begin(), lidar_rb_id_vec.end(), obs.id) !=
        lidar_rb_id_vec.end()) {
      speed_planner_input->lidar_road_edge_info.has_road_edge = true;
      break;
    }
    speed_planner_input->lidar_road_edge_info.has_road_edge = false;
  }
}

void LongitudinalPlannerPreprocessor::get_obs_overlap() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &obs_info_map =
      context_->mutable_lon_decison_output()->obstacle_decision_info_map;
  for (auto &obs : speed_planner_input->lon_obs) {
    if (obs_info_map.find(obs.id) != obs_info_map.end()) {
      obs.overlap = obs_info_map.at(obs.id).overlap;
    }
  }
}

void LongitudinalPlannerPreprocessor::calc_obs_ax() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  auto &speed_memory = speed_planner_input->obstacle_ax_info.speed_memory;
  auto &obstacle_alive = speed_planner_input->obstacle_ax_info.obstacle_alive;
  auto &ax_filted = speed_planner_input->obstacle_ax_info.ax_filted;
  const int time_range = 6;
  double ax_calc = 0;
  const double cycle_time_s = 0.1;
  const double ax_fil_fac = 0.2;

  for (auto &obj : speed_memory) {
    obstacle_alive[obj.first] = false;
    if (obj.second.size() < time_range) {
      obj.second.push_back(0.0);
    } else {
      for (int i = 0; i < obj.second.size() - 1; ++i) {
        obj.second[i] = obj.second[i + 1];
      }
      obj.second.back() = 0.0;
    }
    MSD_LOG(INFO, "Ax_DEBUG:id:%d,mem_len:%d", obj.first, obj.second.size());
  }

  for (auto &obs : speed_planner_input->lon_obs) {
    obstacle_alive[obs.id] = true;
    if (speed_memory.count(obs.id) > 0) {
      speed_memory[obs.id].back() = obs.polygon_init.v_frenet;
    } else {
      speed_memory[obs.id].push_back(obs.polygon_init.v_frenet);
    }
    if (speed_memory[obs.id].size() == time_range) {
      ax_calc = (speed_memory[obs.id].back() - speed_memory[obs.id].front()) /
                ((time_range - 1) * cycle_time_s);
      ax_filted[obs.id] =
          ax_fil_fac * ax_calc + (1 - ax_fil_fac) * ax_filted[obs.id];
    } else {
      ax_filted[obs.id] = 0.0;
    }
    obs.polygon_init.a = ax_filted[obs.id];
    MSD_LOG(INFO, "Obs_DEBUG:dist[%d]:%f", obs.id,
            obs.polygon_init.rel_s - 2.5);
  }
  for (auto obs : obstacle_alive) {
    MSD_LOG(INFO, "Ax_DEBUG:id:%d,alive:%d,spd:%f,ax_filted:%f", obs.first,
            obs.second, speed_memory[obs.first].back(), ax_filted[obs.first]);
    for (int i = 0; i < speed_memory[obs.first].size(); ++i) {
      MSD_LOG(INFO, "Ax_DEBUG:spd[%d]:%f", i, speed_memory[obs.first][i]);
    }
  }
  auto obs = obstacle_alive.begin();
  while (obs != obstacle_alive.end()) {
    if (obs->second == false) {
      speed_memory.erase(obs->first);
      ax_filted.erase(obs->first);
      obs = obstacle_alive.erase(obs);
    } else {
      obs++;
    }
  }
}

void LongitudinalPlannerPreprocessor::update_object_prediction(
    const std::array<std::array<double, QUADRATURE_ORDER>,
                     speed_planner::NUM_SPEED_SEGMENTS> &time_list) {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  speed_planner_input->use_prediction =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.use_prediction;
  // for epcar
  MSD_LOG(ERROR, "config use_prediction : %d",
          speed_planner_input->use_prediction);

  Index<NUM_SPEED_CONTROL_POINTS> index;
  for (auto &obj : speed_planner_input->lon_obs) {
    obj.object_with_t = {};
    index.reset();
    while (index.advance()) {
      double time = time_list[index.segment_index][index.quad_index];
      if (index.segment_index < obj.polygon_list.size() &&
          index.quad_index < obj.polygon_list[index.segment_index].size()) {
        const auto &obj_at_t =
            obj.polygon_list[index.segment_index][index.quad_index];
        obj.object_with_t.push_back(std::make_pair(time, obj_at_t));
      }
    }

    if (obj.object_with_t.size() == 0) {
      speed_planner_input->use_prediction = false;
      MSD_LOG(ERROR, "obj prediction traj size is 0, dont use_prediction : %d",
              speed_planner_input->use_prediction);
    }
  }
}

void LongitudinalPlannerPreprocessor::compute_stopping_point_info() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &model_traj = speed_planner_input->model_trajectory;
  auto &stop_point_info = speed_planner_input->stop_point_info;
  const double &half_length =
      ConfigurationContext::Instance()->get_vehicle_param().length / 2.0;
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  const auto &lidar_rb_id_vec = baseline_info_->get_lidar_rb_id_vec();
  const auto &is_reduce_stop_offset =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.is_reduce_stop_offset;
  auto update_stop_point_info = [&](double obs_stop_s,
                                    const path_planner::ObsInfo &obs,
                                    bool is_model_stop) {
    double reduce_stop_offset = 0.0;
    if (obs.type == ObsInfo::Type::COUPE && is_reduce_stop_offset) {
      reduce_stop_offset = K_postpone_stop_dist;
    }
    MSD_LOG(INFO, "Stop_Point:is_reduce_stop_offset:%d", is_reduce_stop_offset);
    MSD_LOG(INFO, "Stop_Point:reduce_stop_offset:%.1f", reduce_stop_offset);
    double stop_offset =
        obs.polygon_init.stop_offset + half_length - reduce_stop_offset;
    if (!stop_point_info.has_stop_point ||
        obs_stop_s - stop_offset < stop_point_info.stop_s) {
      stop_point_info.has_stop_point = true;
      stop_point_info.should_start = false;
      stop_point_info.stop_id = obs.id;
      stop_point_info.stop_s = obs_stop_s - stop_offset;
      stop_point_info.is_model_stop = is_model_stop;
      if (is_model_stop) {
        stop_point_info.stop_reason =
            "model stopped because of " + std::to_string(obs.id);
      } else {
        stop_point_info.stop_reason =
            std::to_string(obs.id) + " obstacle stopped";
      }
    }
  };
  MSD_LOG(INFO, "Stop_Point:Time:%.2f==========", MTIME()->timestamp().sec());
  speed_planner_input->stop_point_info.has_stop_point = false;
  auto &stop_traj = speed_planner_input->stop_point_info.stop_traj;
  for (const auto &obs : speed_planner_input->lon_obs) {
    const auto &obs_fusion_data = obs.polygon_init;
    double obs_stop_s{}, obs_stop_v{}, obs_stop_a{};
    double velocity_threshold_use_const_jerk_predict_traj =
        planner_config.longitudinal_motion_planner_config
            .velocity_threshold_use_const_jerk_predict_traj;
    MSD_LOG(INFO, "Stop_Point:stop_offset:%.1f", obs.polygon_init.stop_offset);
    MSD_LOG(INFO, "Stop_Point:half_L:%.1f", half_length);
    MSD_LOG(INFO, "Stop_Point:ID:%d", obs.id);
    if (speed_planner_input->use_prediction &&
        obs_fusion_data.v_frenet >
            velocity_threshold_use_const_jerk_predict_traj) {
      MSD_LOG(INFO, "Stop_Point:Pred v_frenet: %.1f", obs_fusion_data.v_frenet);
      MSD_LOG(INFO, "Stop_Point:Pred a: %f", obs_fusion_data.a);
      MSD_LOG(INFO, "Stop_Point:Pred Obj_Vx_Thr: %f",
              velocity_threshold_use_const_jerk_predict_traj);
      for (const auto &pair : obs.object_with_t) {
        const auto &predict_traj = pair.second;
        if (predict_traj.v < 0.3) {
          obs_stop_s = predict_traj.s;
          update_stop_point_info(obs_stop_s, obs, false);
          MSD_LOG(INFO, "Stop_Point:Pred obj_s:%.1f", obs_stop_s);
          MSD_LOG(INFO, "Stop_Point:Pred obj_d:%.1f",
                  obs_stop_s - ego_state.ego_frenet.x);
          MSD_LOG(INFO, "Stop_Point:Pred stop_s:%.1f", stop_point_info.stop_s);
          MSD_LOG(INFO, "Stop_Point:Pred stop_d:%.1f",
                  stop_point_info.stop_s - ego_state.ego_frenet.x);
        }
      }
    } else {
      if (std::fabs(obs_fusion_data.v_frenet) < 0.3) {
        update_stop_point_info(obs_fusion_data.s, obs, false);
        MSD_LOG(INFO, "Stop_Point:ObjStop obj_s:%.1f", obs_fusion_data.s);
        MSD_LOG(INFO, "Stop_Point:ObjStop obj_d:%.1f",
                obs_fusion_data.s - ego_state.ego_frenet.x);
        MSD_LOG(INFO, "Stop_Point:ObjStop stop_s:%.1f", stop_point_info.stop_s);
        MSD_LOG(INFO, "Stop_Point:ObjStop stop_d:%.1f",
                stop_point_info.stop_s - ego_state.ego_frenet.x);
      } else if (obs_fusion_data.a < 0.0) {
        double obs_stop_time =
            std::fmax(0.0, -obs_fusion_data.v_frenet / obs_fusion_data.a);
        predict_obj_state_at_t(false, obs, obs_stop_time, obs_stop_s,
                               obs_stop_v, obs_stop_a);
        MSD_LOG(INFO, "Stop_Point:Calc v_frenet: %.1f",
                obs_fusion_data.v_frenet);
        MSD_LOG(INFO, "Stop_Point:Calc obs_stop_time: %.1f", obs_stop_time);
        MSD_LOG(INFO, "Stop_Point:Calc a: %f", obs_fusion_data.a);
        if (obs_stop_time > 6.0) {
          continue;
        }
        if (!speed_planner_input->stop_point_info.has_stop_point ||
            obs_stop_s < speed_planner_input->stop_point_info.stop_s) {
          stop_traj.clear();
          for (int i = 0; i < obs_stop_time * 5; ++i) {
            double pre_s, pre_v, pre_a;
            predict_obj_state_at_t(false, obs, i * 0.2, obs_stop_s, obs_stop_v,
                                   obs_stop_a);
            stop_traj.push_back({obs_stop_s, obs_stop_v});
          }
        }
        update_stop_point_info(obs_stop_s, obs, false);
        MSD_LOG(INFO, "Stop_Point:Calc obj_s:%.1f", obs_stop_s);
        MSD_LOG(INFO, "Stop_Point:Calc obj_d:%.1f",
                obs_stop_s - ego_state.ego_frenet.x);
        MSD_LOG(INFO, "Stop_Point:Calc stop_s:%.1f", stop_point_info.stop_s);
        MSD_LOG(INFO, "Stop_Point:Calc stop_d:%.1f",
                stop_point_info.stop_s - ego_state.ego_frenet.x);
      }
    }
  }

  if (speed_planner_input->use_eftp && ego_state.ego_vel > 6.0 ||
      speed_planner_input->enable_model_traj_modify) {
    double stop_offset =
        planner_config.longitudinal_motion_planner_config.eftp_stop_offset +
        half_length;
    double model_average_accel =
        (model_traj.back().v - model_traj.front().v) /
        std::fmax(0.01, (model_traj.back().t - model_traj.front().t));
    const auto &last_point = model_traj.back();
    const bool is_model_stop = true;

    if (model_average_accel < 0.0) {
      double pred_stop_s =
          last_point.s - last_point.v * last_point.v / 2 / model_average_accel;

      for (const auto &obs : speed_planner_input->lon_obs) {
        if (obs.id == speed_planner_input->cipv_info.cipv_id) {
          update_stop_point_info(pred_stop_s + stop_offset, obs, is_model_stop);
          MSD_LOG(INFO, "Stop_Point:DDP_Calc obj_s:%.1f",
                  pred_stop_s + stop_offset);
          MSD_LOG(INFO, "Stop_Point:DDP_Calc obj_d:%.1f",
                  pred_stop_s + stop_offset - ego_state.ego_frenet.x);
          MSD_LOG(INFO, "Stop_Point:DDP_Calc stop_s:%.1f",
                  stop_point_info.stop_s);
          MSD_LOG(INFO, "Stop_Point:DDP_Calc stop_d:%.1f",
                  stop_point_info.stop_s - ego_state.ego_frenet.x);
          break;
        }
      }
    } else if (last_point.v < 0.5) {
      double model_stop_s = last_point.s;
      for (const auto &obs : speed_planner_input->lon_obs) {
        if (obs.id == speed_planner_input->cipv_info.cipv_id) {
          update_stop_point_info(last_point.s + stop_offset, obs,
                                 is_model_stop);
          MSD_LOG(INFO, "Stop_Point:DDP_Stop obj_s:%.1f",
                  last_point.s + stop_offset);
          MSD_LOG(INFO, "Stop_Point:DDP_Stop obj_d:%.1f",
                  last_point.s + stop_offset - ego_state.ego_frenet.x);
          MSD_LOG(INFO, "Stop_Point:DDP_Stop stop_s:%.1f",
                  stop_point_info.stop_s);
          MSD_LOG(INFO, "Stop_Point:DDP_Stop stop_d:%.1f",
                  stop_point_info.stop_s - ego_state.ego_frenet.x);
          break;
        }
      }
    }
  }

  // if is rule_base stop point, disable model_ref
  speed_planner_input->enable_model_ref = speed_planner_input->use_eftp;
  if (speed_planner_input->b_dagger_longitudinal) {
    speed_planner_input->enable_model_ref = true;
  } else if (stop_point_info.has_stop_point && !stop_point_info.is_model_stop) {
    speed_planner_input->enable_model_ref = false;
  }

  if (!speed_planner_input->stop_point_info.has_stop_point) {
    stop_traj.clear();
  }

  const auto &has_stop_point =
      speed_planner_input->stop_point_info.has_stop_point;

  speed_planner_input->stop_point_info.from_closest_obs = false;
  if (has_stop_point && speed_planner_input->lon_obs.size()) {
    if (speed_planner_input->stop_point_info.stop_id ==
        speed_planner_input->lon_obs.front().id) {
      speed_planner_input->stop_point_info.from_closest_obs = true;
    }
  }

  double brake_distance =
      std::fmax(0.3, speed_planner_input->stop_point_info.stop_s -
                         speed_planner_input->ego_state.s);
  double ego_v_square = ego_state.ego_vel * ego_state.ego_vel;

  if (has_stop_point && planner_config.longitudinal_motion_planner_config
                            .enable_dynamic_stop_offset) {
    double dynamic_stop_offset = LinearInterpation::interpolation(
        ego_state.ego_vel, {0.0, 5.0}, {0.0, 2.0});
    // speed_planner_input->stop_point_info.stop_s -= dynamic_stop_offset;
  }

  auto &enable_modify_stop_point =
      speed_planner_input->stop_point_info.enable_modify_stop_point;
  if (has_stop_point &&
      planner_config.longitudinal_motion_planner_config
          .enable_modify_stop_point &&
      !enable_modify_stop_point) {
    // modify stop point
    const double max_v = 5.0;
    const double min_v = 1.0;
    if (ego_state.ego_vel > min_v && ego_state.ego_vel < max_v &&
        (ego_v_square / (2 * brake_distance) > 1.0) &&
        (ego_v_square / (2 * brake_distance) -
             ego_v_square / (2 * (brake_distance + 1.0)) >
         planner_config.longitudinal_motion_planner_config
             .modify_stop_point_accel_thr)) {
      enable_modify_stop_point = true;
    }
  }

  if (!has_stop_point) {
    enable_modify_stop_point = false;
  }

  if ((enable_modify_stop_point ||
       speed_planner_input->kick_start_time < K_kick_start_time_s) &&
      !speed_planner_input->enable_model_traj_modify) {
    // speed_planner_input->stop_point_info.stop_s += 1.0;
  }

  if (std::find(lidar_rb_id_vec.begin(), lidar_rb_id_vec.end(),
                stop_point_info.stop_id) != lidar_rb_id_vec.end()) {
    MSD_LOG(INFO, "DEBUG_LX_STOP_REASON: %d Lidar road edge causes stop point",
            stop_point_info.stop_id);
    speed_planner_input->lidar_road_edge_info.stop_for_road_edge = true;
    speed_planner_input->lidar_road_edge_info.road_edge_id =
        stop_point_info.stop_id;
    stop_point_info.stop_s -=
        speed_planner_input->lidar_road_edge_info.stop_offset;
  } else {
    MSD_LOG(INFO, "DEBUG_LX_STOP_REASON: no Lidar road edge causes stop point");
    speed_planner_input->lidar_road_edge_info.stop_for_road_edge = false;
    speed_planner_input->lidar_road_edge_info.road_edge_id = -1;
  }
}

bool LongitudinalPlannerPreprocessor::stop_signal_judge(
    const double &f_dist_m, const double &f_rel_v_mps,
    const double &f_ego_axdv_mps2) {
  const double K_stop_ax_comfort_jerk = -0.8;
  const double K_stop_ax_final = -0.75;

  double s = 0;
  double v = -f_rel_v_mps;
  double a = f_ego_axdv_mps2;
  double init_offset = 0.000001;
  a = 0.0;
  MSD_LOG(INFO, "NEWLP_STP,stop_signal:s:%f,v:%f,a:%f", f_dist_m, v, a);
  for (int i = 0; i < 100; ++i) {
    if (a > K_stop_ax_final) {
      a += K_stop_ax_comfort_jerk * K_cycle_s;
    } else {
      a -= K_stop_ax_comfort_jerk * K_cycle_s;
    }
    v += a * K_cycle_s;
    s += v * K_cycle_s;
    // in case s == f_dist_m == 0.0 , turn into  stop_mode
    if (s > f_dist_m - init_offset) {
      return true;
    }
  }
  return false;
}

void LongitudinalPlannerPreprocessor::stop_flag_decision() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  double f_ego_vx_mps = ego_state.ego_vel;
  bool &f_standstill = speed_planner_input->stop_point_info.standstill;
  int f_driving_direction = 1; // CJ:temp define:1 FRONT, -1 BACK
  bool &f_hold_flag = speed_planner_input->stop_point_info.hold_flag;
  bool &f_hold2kick = speed_planner_input->stop_point_info.hold2kick;
  double &f_stop_timer = speed_planner_input->stop_point_info.stop_timer;
  double &f_real_stop_timer =
      speed_planner_input->stop_point_info.real_stop_timer;
  const bool &f_stop_mode = speed_planner_input->stop_point_info.stop_mode;
  bool &low_spd = speed_planner_input->stop_point_info.low_speed_state;
  bool last_hold_flag = f_hold_flag;
  bool last_standstill = f_standstill;
  bool last_low_spd = low_spd;
  double f_dist_m = 0;
  bool auto_hold_status =
      world_model_->get_vehicle_status().epb_info.epb_data.auto_hold_status ==
      1;
  path_planner::ObsInfo cipv{};
  if (speed_planner_input->lon_obs.size() > 0) {
    for (auto obj : speed_planner_input->lon_obs) {
      if (obj.id == speed_planner_input->cipv_info.cipv_id) {
        cipv = obj;
        f_dist_m = cipv.polygon_init.rel_s -
                   speed_planner_input->vehicle_param.center_to_front;
      }
    }
  }

  const double K_stop_stopped_time_s = 0.5;
  const double K_stop_EPB_active_time_s = 600;
  if (f_ego_vx_mps < K_stop_ego_hold_spd_mps) {
    f_real_stop_timer =
        std::fmin(f_real_stop_timer + K_cycle_s, K_stop_EPB_active_time_s);
    if (f_real_stop_timer > K_stop_stopped_time_s) {
      f_hold2kick = true;
    }
  } else {
    f_real_stop_timer = 0;
  }
  // 1.not stop mode
  if (f_stop_mode == false) {
    f_hold_flag = false;
    f_stop_timer = 0;
    f_standstill = false;
    return;
  }
  // 2.hold flag judge
  low_spd = f_ego_vx_mps < K_stop_ego_hold_spd_mps;
  f_hold_flag = low_spd && (f_stop_timer >= K_stop_stopped_time_s ||
                            last_hold_flag || auto_hold_status);
  if (f_hold_flag == true) {
    f_hold2kick = true;
  }
  // 3.stop timer update
  f_stop_timer = std::fmin(f_stop_timer + K_cycle_s, K_stop_EPB_active_time_s);
  if (low_spd != last_low_spd ||
      f_hold_flag == true && last_hold_flag == false) {
    f_stop_timer = 0;
  }

  // 3.standstill judge
  f_standstill = (f_ego_vx_mps < K_stop_standstill_spd_mps &&
                  f_dist_m < K_stop_standstill_dist) ||
                 f_driving_direction == -1 || last_standstill == true ||
                 auto_hold_status == true;
  MSD_LOG(INFO, "NEWLP_STP:low_spd:%d,hold:%d,stop_timer:%f,still:%d", low_spd,
          f_hold_flag, f_stop_timer, f_standstill);
}

void LongitudinalPlannerPreprocessor::go_flag_decision() {
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const double &k_stop_offset =
      speed_planner_input->stop_point_info.stop_offset;
  bool &f_auto_go = speed_planner_input->stop_point_info.auto_go;
  bool &f_go_indicator = speed_planner_input->stop_point_info.go_indicator;
  const bool &f_hold_flag = speed_planner_input->stop_point_info.hold_flag;
  bool &ready_go = speed_planner_input->stop_point_info.ready_go;
  const bool &f_stop_for_road_edge =
      speed_planner_input->lidar_road_edge_info.stop_for_road_edge;
  const bool f_freespace = true;
  int f_CIPV_id = 0;
  double f_dist_m = 250;
  double f_rel_v_mps = 100;
  bool f_cipv_is_barrier = false;
  int &last_CIPV_id = speed_planner_input->stop_point_info.last_CIPV_id;
  bool &last_CIPV_lost = speed_planner_input->stop_point_info.last_CIPV_lost;
  const bool dbw_status = world_model_->get_vehicle_dbw_status();
  const bool f_override = speed_planner_input->throttle_override;
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  double f_ego_vx_mps = ego_state.ego_vel;
  bool auto_hold_status =
      world_model_->get_vehicle_status().epb_info.epb_data.auto_hold_status ==
          1 ||
      world_model_->is_force_stop();
  double f_stop_dist_thr;
  if (f_stop_for_road_edge) {
    f_stop_dist_thr = k_stop_offset + K_stop_go_CIPV_dist_offset +
                      speed_planner_input->lidar_road_edge_info.stop_offset;
  } else {
    f_stop_dist_thr = k_stop_offset + K_stop_go_CIPV_dist_offset;
  }
  MSD_LOG(INFO, "NEWLP_STP:Force_Stop:%d", world_model_->is_force_stop());
  path_planner::ObsInfo cipv{};
  if (speed_planner_input->lon_obs.size() > 0) {
    for (auto obj : speed_planner_input->lon_obs) {
      if (obj.id == speed_planner_input->cipv_info.cipv_id) {
        cipv = obj;
        f_dist_m = cipv.polygon_init.rel_s -
                   speed_planner_input->vehicle_param.center_to_front;
        f_CIPV_id = cipv.id;
        f_rel_v_mps = cipv.polygon_init.v_frenet - f_ego_vx_mps;
        f_cipv_is_barrier = speed_planner_input->cipv_info.is_barrier;
      }
    }
  }
  MSD_LOG(INFO, "NEWLP_STP:dist:%f,vx:%f,cipv_vx:%f", f_dist_m, f_ego_vx_mps,
          cipv.polygon_init.v_frenet);
  const double &f_stop_timer = speed_planner_input->stop_point_info.stop_timer;
  const bool &f_stop_mode = speed_planner_input->stop_point_info.stop_mode;

  bool CIPV_lost = false;
  // 1.not stop mode
  if (f_stop_mode == false) {
    f_auto_go = false;
    f_go_indicator = false;
    last_CIPV_id = -99;
    return;
  }
  // 2.CIPV lost judge
  if (f_hold_flag == true) {
    if (f_override == true) {
      CIPV_lost = false;
    } else {
      if ((obstacle_manager.find_obstacle(last_CIPV_id) == nullptr &&
           last_CIPV_id > 0) ||
          last_CIPV_lost == true) {
        CIPV_lost = true;
      } else {
        CIPV_lost = false;
      }
    }
    if (last_CIPV_lost == true &&
        !(f_ego_vx_mps + f_rel_v_mps > K_stop_go_CIPV_spd_mps ||
          f_dist_m > k_stop_offset + K_stop_go_CIPV_dist_offset)) {
      CIPV_lost = false;
    }
  }
  if (last_CIPV_lost == true && world_model_->is_force_stop() == true) {
    CIPV_lost = false;
  }

  // 3.go indicator update
  if (auto_hold_status == true) {
    f_auto_go = false;
    f_go_indicator = false;
  } else if (f_hold_flag == false) {
    f_auto_go = false;
    f_go_indicator = false;
  } else if (f_freespace == false) {
    f_auto_go = false;
    f_go_indicator = false;
  } else if (f_CIPV_id <= 0) {
    if (CIPV_lost == true) {
      f_auto_go = false;
      f_go_indicator = false;
    } else {
      f_auto_go = true;
      f_go_indicator = true;
    }
  } else if (f_ego_vx_mps + f_rel_v_mps > K_stop_go_CIPV_spd_mps ||
             f_dist_m > f_stop_dist_thr) {
    if (CIPV_lost == true) {
      f_auto_go = false;
      f_go_indicator = false;
    } else {
      if (f_stop_timer <= K_stop_autogo_time_s) {
        f_auto_go = true;
        f_go_indicator = false;
      } else {
        f_auto_go = false;
        f_go_indicator = true;
      }
    }
  } else {
    f_auto_go = false;
    f_go_indicator = false;
  }
  ready_go = false;
  if (f_CIPV_id <= 0) {
    ready_go = true;
  } else if (f_ego_vx_mps + f_rel_v_mps > K_stop_go_CIPV_spd_mps ||
             f_dist_m > f_stop_dist_thr) {
    if (f_stop_timer <= K_stop_autogo_time_s) {
      ready_go = true;
    }
  }
  last_CIPV_id = f_CIPV_id;
  last_CIPV_lost = CIPV_lost;
  MSD_LOG(INFO, "NEWLP_STP:auto_go:%d,go_indi:%d,", f_auto_go, f_go_indicator);
  MSD_LOG(INFO, "NEWLP_STP:override:%d", f_override);
  MSD_LOG(INFO, "NEWLP_STP:cipv_lost:%d", CIPV_lost);
  MSD_LOG(INFO, "NEWLP_STP:stop_for_road_edge %d", f_stop_for_road_edge);
}

void LongitudinalPlannerPreprocessor::update_stop_mode() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const double &k_stop_offset =
      speed_planner_input->stop_point_info.stop_offset;
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  path_planner::ObsInfo cipv{};
  double f_ego_vx_mps = ego_state.ego_vel;
  double f_ego_axdv_mps2 = ego_state.ego_acc;
  bool f_override = false;
  int f_acc_pedal_pos = 0;
  int f_driving_direction = 1; // CJ:temp define:1 FRONT, -1 BACK
  const bool f_driver_go = false;
  const bool &f_auto_go = speed_planner_input->stop_point_info.auto_go;
  const bool &f_standstill = speed_planner_input->stop_point_info.standstill;
  bool &f_stop_mode = speed_planner_input->stop_point_info.stop_mode;
  bool &f_last_stop_mode = speed_planner_input->stop_point_info.last_stop_mode;
  int &f_stop_in_code = speed_planner_input->stop_point_info.stop_in_code;
  int &f_stop_out_code = speed_planner_input->stop_point_info.stop_out_code;
  bool stop_mode = f_stop_mode;
  int f_CIPV_id = 0;
  double f_dist_m = 250;
  double f_rel_v_mps = 100;

  if (speed_planner_input->lon_obs.size() > 0) {
    for (auto obj : speed_planner_input->lon_obs) {
      if (obj.id == speed_planner_input->cipv_info.cipv_id) {
        cipv = obj;
        f_CIPV_id = cipv.id;
        f_dist_m = cipv.polygon_init.rel_s -
                   speed_planner_input->vehicle_param.center_to_front;
        f_rel_v_mps = cipv.polygon_init.v_frenet - f_ego_vx_mps;
      }
    }
  }

  double brake_dist = 0;
  if (speed_planner_input->stop_point_info.has_stop_point) {
    brake_dist = speed_planner_input->stop_point_info.stop_s -
                 ego_state.ego_frenet.x - K_stop_go_CIPV_dist_offset;
    MSD_LOG(INFO, "NEWLP_STP:brake_dist:%d", brake_dist);
  } else {
    brake_dist = DBL_MAX;
  }
  brake_dist = std::fmax(brake_dist, 0.0);

  bool &stop_signal = speed_planner_input->stop_point_info.stop_signal;
  stop_signal = stop_signal_judge(brake_dist, -f_ego_vx_mps, f_ego_axdv_mps2);
  bool enter_stop = false;
  bool exit_stop = false;
  bool stop_in_cond1 = false;
  bool stop_in_cond2 = false;
  bool stop_in_cond3 = false;
  bool stop_in_cond4 = false;
  bool stop_in_cond5 = false;
  bool stop_out_cond1 = false;
  bool stop_out_cond2 = false;
  bool stop_out_cond3 = false;
  bool stop_out_cond4 = false;
  bool stop_out_cond5 = false;
  bool stop_out_cond6 = false;
  MSD_LOG(INFO, "NEWLP_STP ======================================");
  MSD_LOG(INFO, "NEWLP_STP,Time:%.2f", MTIME()->timestamp().sec());
  MSD_LOG(INFO, "NEWLP_STP,CIPV_ID:%d", cipv.id);
  MSD_LOG(INFO, "NEWLP_STP,stop_signal:%d", stop_signal);
  MSD_LOG(INFO, "NEWLP_STP,brake_dist:%f,k_stop_offset:%f", brake_dist,
          k_stop_offset);
  bool auto_hold_status =
      world_model_->get_vehicle_status().epb_info.epb_data.auto_hold_status ==
      1;
  speed_planner_input->stop_point_info.auto_hold = auto_hold_status;
  speed_planner_input->stop_point_info.force_stop =
      world_model_->is_force_stop();
  MSD_LOG(INFO, "NEWLP_STP,auto_hold:%d, force_stop:%d",
          world_model_->get_vehicle_status().epb_info.epb_data.auto_hold_status,
          world_model_->is_force_stop());
  stop_in_cond1 = f_CIPV_id > 0 && stop_signal == true &&
                  f_ego_vx_mps + f_rel_v_mps <= K_stop_entry_CIPV_spd_mps &&
                  f_ego_vx_mps <= K_stop_entry_ego_spd_mps;

  stop_in_cond2 = f_CIPV_id > 0 &&
                  f_dist_m < k_stop_offset + K_stop_rolling_dist &&
                  f_ego_vx_mps + f_rel_v_mps < K_stop_entry_CIPV_spd_mps &&
                  f_ego_vx_mps < K_stop_entry_ego_spd_mps;

  stop_in_cond3 = f_driving_direction == -1 &&
                  std::fabs(f_ego_vx_mps) <= K_stop_slip_back_spd_mps;

  stop_in_cond4 = auto_hold_status;

  stop_in_cond5 = !f_override;
  enter_stop =
      (stop_in_cond1 || stop_in_cond2 || stop_in_cond3 || stop_in_cond4) &&
      stop_in_cond5;

  stop_out_cond1 = f_auto_go;
  stop_out_cond2 = f_acc_pedal_pos > K_stop_throttle_go;
  stop_out_cond3 = f_driver_go;
  // CIPV change during stop brake
  stop_out_cond4 = f_ego_vx_mps + f_rel_v_mps > K_stop_exit_CIPV_spd_mps &&
                   f_standstill == false && f_CIPV_id > 0;
  stop_out_cond5 = f_rel_v_mps > K_stop_exit_relv_mps &&
                   f_ego_vx_mps > K_stop_ego_hold_spd_mps &&
                   f_dist_m > k_stop_offset && f_CIPV_id > 0;
  stop_out_cond6 =
      (f_override == true || f_standstill == false) && f_CIPV_id <= 0;
  exit_stop = (stop_out_cond1 || stop_out_cond2 || stop_out_cond3 ||
               stop_out_cond4 || stop_out_cond5 || stop_out_cond6) &&
              (!auto_hold_status);

  if ((enter_stop == true || stop_mode == true) && exit_stop == false) {
    stop_mode = true;
  } else {
    stop_mode = false;
  }
  f_last_stop_mode = f_stop_mode;
  f_stop_mode = stop_mode;
  f_stop_in_code = (stop_in_cond5 << 4) + (stop_in_cond4 << 3) +
                   (stop_in_cond3 << 2) + (stop_in_cond2 << 1) + stop_in_cond1;
  f_stop_out_code = (stop_out_cond6 << 5) + (stop_out_cond5 << 4) +
                    (stop_out_cond4 << 3) + (stop_out_cond3 << 2) +
                    (stop_out_cond2 << 1) + stop_out_cond1;
  MSD_LOG(INFO, "NEWLP_STP,ci_code:", f_stop_in_code);
  MSD_LOG(INFO, "NEWLP_STP,co_code:", f_stop_out_code);
  MSD_LOG(INFO, "NEWLP_STP,ci1:%d,ci2:%d,ci3:%d,ci4:%d", stop_in_cond1,
          stop_in_cond2, stop_in_cond3, stop_in_cond4);
  MSD_LOG(INFO, "NEWLP_STP,co1:%d,co2:%d,co3:%d,co4:%d,co5:%d,co6:%d",
          stop_out_cond1, stop_out_cond2, stop_out_cond3, stop_out_cond4,
          stop_out_cond5, stop_out_cond6);
  MSD_LOG(INFO, "NEWLP_STP,Mode:%d,last_Mode:%d", stop_mode, f_last_stop_mode);
  MSD_LOG(INFO, "NEWLP_STP,vx:%.2f,ref_v:%.2f,CIPV:%d", f_ego_vx_mps,
          f_rel_v_mps, f_CIPV_id);
}

void LongitudinalPlannerPreprocessor::update_start_state() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const double &k_stop_offset =
      speed_planner_input->stop_point_info.stop_offset;
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  path_planner::ObsInfo cipv{};
  const double K_StandstillSpdThr_mps = 0.2;
  const int K_StandstillCycMax = 601;
  const double K_StartCIPVSpdThr_mps = 0.75;
  const double K_StartCIPVDistOffset_m = 1.5;
  const double K_QuitStandstillSpdThr_mps = 0.5;
  const double K_KickStart_v_mps = 0.2;
  const double K_KickStart_a_mps2 = 1;
  const double K_StandstillConfirmTime_s = 0.5;
  const double cycle_time_s = 0.1;
  bool &hold2kick = speed_planner_input->stop_point_info.hold2kick;

  if (speed_planner_input->lon_obs.size() > 0) {
    for (auto obj : speed_planner_input->lon_obs) {
      if (obj.id == speed_planner_input->cipv_info.cipv_id) {
        cipv = obj;
      }
    }
    MSD_LOG(INFO, "New CIPV_ID:%d", cipv.id);
  } else {
    cipv.polygon_init.rel_s = 100.0;
    cipv.polygon_init.v_frenet = 100.0;
    cipv.polygon_init.stop_offset = k_stop_offset;
    cipv.polygon_init.desired_headway = 1.8;
  }

  double ds_offseted =
      std::fmax(0.0, cipv.polygon_init.rel_s -
                         speed_planner_input->vehicle_param.length / 2);
  double ego_dv = ego_state.ego_vel - cipv.polygon_init.v_frenet;
  double rel_v = std::fmax(0.0, cipv.polygon_init.v_frenet - ego_state.ego_vel);

  double ds_follow_offseted, dv_curve, da_curve, dv_deadband;
  update_dv_curve(
      cipv.polygon_init.stop_offset, cipv.polygon_init.desired_headway,
      cipv.polygon_init.v_frenet, cipv.polygon_init.rel_s,
      speed_planner_input->vehicle_param.center_to_front,
      speed_planner_input->speed_tuning_params.deadband_region_ratio,
      ds_follow_offseted, dv_curve, da_curve, dv_deadband, ego_state.ego_vel);

  ACCRegionType region_type = ACCRegionType::NONE;
  if (ego_dv < dv_deadband) {
    region_type = ACCRegionType::ACCEL_REGION;
  }

  double kick_start_scale =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config.kick_start_scale;
  MSD_LOG(INFO, "kick_start_scale : %f", kick_start_scale);

  if (ego_state.ego_vel < K_KickStart_spd_mps &&
      !speed_planner_input->stop_point_info.stop_mode &&
      region_type == ACCRegionType::ACCEL_REGION &&
      !speed_planner_input->b_dagger_longitudinal) {
    auto kick_start_a = LinearInterpation::interpolation(
        rel_v, {0.0, 1 * K_stop_exit_relv_mps},
        {0.5, kick_start_scale * K_KickStart_a_mps2});

    if (hold2kick) {
      speed_planner_input->planning_init_state.v =
          std::fmax(ego_state.ego_vel, K_KickStart_v_mps);
      speed_planner_input->planning_init_state.a =
          std::fmax(ego_state.ego_acc, kick_start_a);
    } else {
      speed_planner_input->planning_init_state.jerk = 2.0;
      speed_planner_input->planning_init_state.da_ds =
          speed_planner_input->planning_init_state.jerk /
          std::fmax(0.5, speed_planner_input->planning_init_state.v);
    }
    speed_planner_input->kick_start_time = 0.0;
    speed_planner_input->planning_init_state.s = ego_state.ego_frenet.x;
  }

  if (!world_model_->get_vehicle_dbw_status()) {
    speed_planner_input->kick_start_time = 100.0;
  }
  speed_planner_input->kick_start_time += 1.0 / FLAGS_planning_loop_rate;

  if (speed_planner_input->kick_start_time < K_kick_start_time_s) {
    if (speed_planner_input->cipv_info.cipv_velocity_vector.size()) {
      if (speed_planner_input->cipv_info.cipv_average_velocity < 1.5) {
        speed_planner_input->planning_init_state.a =
            std::fmin(speed_planner_input->planning_init_state.a, 0.5);
      }
    }
  }

  if (ego_state.ego_vel > K_KickStart_spd_mps) {
    hold2kick = false;
  }
  // CJ: init ax should be between real ax and target ax
  MSD_LOG(INFO, "KickStartBug: Split Dash --------------------------");
  double K_StopFinalTargetAx = -1.0;
  double K_ResetInitVxThr_mps = 10 / 3.6;
  if ((speed_planner_input->stop_point_info.stop_mode == true ||
       ego_dv >= dv_curve && ego_state.ego_vel < K_ResetInitVxThr_mps) &&
      std::fabs(speed_planner_input->planning_init_state.a -
                K_StopFinalTargetAx) >
          std::fabs(ego_state.ego_acc - K_StopFinalTargetAx) + 0.5) {
    double ax_offset = 0;
    if (K_StopFinalTargetAx - ego_state.ego_acc > 0) {
      ax_offset = std::fmin(K_StopFinalTargetAx - ego_state.ego_acc, 0.5);
    } else {
      ax_offset = std::fmax(K_StopFinalTargetAx - ego_state.ego_acc, -0.5);
    }
    // speed_planner_input->planning_init_state.a = ego_state.ego_acc +
    // ax_offset; speed_planner_input->planning_init_state.v =
    // ego_state.ego_vel;
    MSD_LOG(INFO, "KickStartBug: Filter Reset!");
  }
  MSD_LOG(INFO, "KickStartBug: ego_dv:%f,dv_curve:%f", ego_dv, dv_curve);
  MSD_LOG(INFO, "KickStartBug: ego_ax:%f,init_ax:%f", ego_state.ego_acc,
          speed_planner_input->planning_init_state.a);
  // replan
  if (speed_planner_input->stop_point_info.hold_flag == true ||
      speed_planner_input->stop_point_info.last_stop_mode == false &&
          speed_planner_input->stop_point_info.stop_mode == true &&
          ego_state.ego_vel < K_stop_ego_hold_spd_mps) {
    speed_planner_input->planning_init_state.v = 0.0;
    speed_planner_input->planning_init_state.a = -1.0;
  }

  MSD_LOG(INFO,
          "Standstill_DEBUG:start:%d,stop:%d,still:%d,stop_timer:%f,hold:%d",
          speed_planner_input->stop_point_info.should_start,
          speed_planner_input->stop_point_info.stop_mode,
          speed_planner_input->stop_point_info.standstill,
          speed_planner_input->stop_point_info.stop_timer,
          speed_planner_input->stop_point_info.hold_flag);
  MSD_LOG(INFO, "Standstill_DEBUG:Vx:%f", ego_state.ego_vel);

  speed_planner_input->stop_point_info.should_start_last =
      speed_planner_input->stop_point_info.should_start;
  speed_planner_input->stop_point_info.has_stop_point_last =
      speed_planner_input->stop_point_info.has_stop_point;
  speed_planner_input->stop_point_info.standstill_last =
      speed_planner_input->stop_point_info.standstill;
}

void LongitudinalPlannerPreprocessor::generate_radar_info() {
  const auto &perception_radar = world_model_->get_perception_radar();
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  speed_planner_input->radar_points.clear();

  if (perception_radar.radar_perception_objects.available ==
      maf_perception_interface::RadarPerceptionObjects::
          BOSCH_RADAR_PERCEPTION_OBJECT_DATA) {
    const auto &radar_points = perception_radar.radar_perception_objects
                                   .bosch_radar_perception_object_data;
    const auto &cart_ego_state_manager =
        world_model_->get_cart_ego_state_manager();
    const auto &car2enu = cart_ego_state_manager.get_car2enu();
    const auto &cart_ego_state = cart_ego_state_manager.get_cart_ego_state();
    const auto &front_edge_to_center = ConfigurationContext::Instance()
                                           ->get_vehicle_param()
                                           .front_edge_to_center;
    const auto real_ax_to_head =
        (ConfigurationContext::Instance()->get_vehicle_param().length -
         ConfigurationContext::Instance()
             ->get_vehicle_param()
             .rear_bumper_to_rear_axle);

    Eigen::Vector3d car_point, car_point_ego, enu_point, enu_rel_velocity,
        enu_abs_velocity;
    speed_planner::RadarPoint radar_point;
    Point2D cart, fren;

    std::unordered_set<int> radar_points_set{};
    for (const auto &point : radar_points) {
      if (radar_points_set.find(point.object_id) != radar_points_set.end()) {
        continue;
      } else {
        radar_points_set.insert(point.object_id);
      }

      car_point.x() = point.relative_position.x -
                      std::cos(cart_ego_state.ego_pose.theta) * real_ax_to_head;
      car_point.y() = point.relative_position.y -
                      std::sin(cart_ego_state.ego_pose.theta) * real_ax_to_head;
      car_point.z() = point.relative_position.z;
      enu_point = car2enu * car_point;
      radar_point.id = point.object_id;
      radar_point.x = enu_point.x();
      radar_point.y = enu_point.y();

      cart.x = radar_point.x;
      cart.y = radar_point.y;
      (void)baseline_info_->get_frenet_coord()->CartCoord2FrenetCoord(cart,
                                                                      fren);
      radar_point.s = fren.x;
      radar_point.l = fren.y;

      car_point.x() = point.relative_velocity.x;
      car_point.y() = point.relative_velocity.y;
      car_point.z() = point.relative_velocity.z;
      car_point_ego.x() = 0.0;
      car_point_ego.y() = 0.0;
      car_point_ego.z() = 0.0;

      enu_rel_velocity = car2enu * car_point - car2enu * car_point_ego;
      enu_abs_velocity =
          enu_rel_velocity + Eigen::Vector3d{cart_ego_state.ego_vx_mps,
                                             cart_ego_state.ego_vy_mps, 0.0};
      radar_point.absolute_vel =
          std::hypot(enu_abs_velocity.x(), enu_abs_velocity.y());
      radar_point.relative_vel =
          radar_point.absolute_vel - cart_ego_state.ego_vel;

      static constexpr double RADAR_LAT_RANGE = 3.2;
      if (std::fabs(radar_point.l) < RADAR_LAT_RANGE) {
        speed_planner_input->radar_points.push_back(radar_point);
      }
    }
  }
}

void LongitudinalPlannerPreprocessor::set_vehicle_params() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  speed_planner_input->vehicle_param.length =
      msquare::ConfigurationContext::Instance()->get_vehicle_param().length;
  speed_planner_input->vehicle_param.width =
      msquare::ConfigurationContext::Instance()->get_vehicle_param().width;
  speed_planner_input->vehicle_param.center_to_front =
      speed_planner_input->vehicle_param.length / 2.0;
}

void LongitudinalPlannerPreprocessor::select_cipv() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const double &k_stop_offset =
      speed_planner_input->stop_point_info.stop_offset;
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  double half_length =
      msquare::ConfigurationContext::Instance()->get_vehicle_param().length /
      2.0;
  auto &cipv_info = speed_planner_input->cipv_info;
  auto &lon_obstacle_decision_info_map =
      context_->mutable_lon_decison_output()->obstacle_decision_info_map;
  const double K_SmoothBrakeOverlapThr = 0.5;
  const double K_SmoothBrakeCIPVSpdThr = 1.5;
  const double K_SmoothBrakeOverlapVRUThr = 0.25;
  const double K_SmoothBrakeCIPVSpdVRUThr = 0.5;
  std::vector<CIPVInfo> obj_sort;
  auto comp = [](const CIPVInfo &obj1, const CIPVInfo &obj2) {
    return obj1.v_frenet + obj1.dv_curve < obj2.v_frenet + obj2.dv_curve;
  };
  auto is_smalloverlap =
      [K_SmoothBrakeOverlapThr, K_SmoothBrakeCIPVSpdThr,
       K_SmoothBrakeOverlapVRUThr,
       K_SmoothBrakeCIPVSpdVRUThr](const CIPVInfo &obj) -> bool {
    MSD_LOG(INFO, "2ndCIPV:Lambda:ID:%d", obj.cipv_id);
    MSD_LOG(INFO, "2ndCIPV:Lambda:overlap:%.1f", obj.overlap);
    MSD_LOG(INFO, "2ndCIPV:Lambda:ol_thr:%.1f", K_SmoothBrakeOverlapThr);
    MSD_LOG(INFO, "2ndCIPV:Lambda:ol_VRU_thr:%.1f", K_SmoothBrakeOverlapVRUThr);
    MSD_LOG(INFO, "2ndCIPV:Lambda:v_thr:%.1f", K_SmoothBrakeCIPVSpdThr);
    MSD_LOG(INFO, "2ndCIPV:Lambda:v_VRU_thr:%.1f", K_SmoothBrakeCIPVSpdVRUThr);
    MSD_LOG(INFO, "2ndCIPV:Lambda:v_vector:%.1f", obj.v_vector);
    MSD_LOG(INFO, "2ndCIPV:Lambda:type:%d", obj.type);
    bool smalloverlap =
        obj.overlap < K_SmoothBrakeOverlapThr &&
            obj.v_vector < K_SmoothBrakeCIPVSpdThr &&
            (obj.type == path_planner::ObsInfo::COUPE ||
             obj.type == path_planner::ObsInfo::TRANSPORT_TRUNK) ||
        obj.overlap < K_SmoothBrakeOverlapVRUThr &&
            obj.v_vector < K_SmoothBrakeCIPVSpdVRUThr &&
            (obj.type == path_planner::ObsInfo::PEDESTRIAN ||
             obj.type == path_planner::ObsInfo::OFO);
    MSD_LOG(INFO, "2ndCIPV:Lambda:smalloverlap:%d", smalloverlap);
    return smalloverlap;
  };
  int cipv_id = -1;
  double cipv_v = 100;
  const double K_SmoothBrake_AxMin = -3.0;
  MSD_LOG(INFO, "2ndCIPV:Time:%.1f==========", MTIME()->timestamp().sec());
  if (speed_planner_input->lon_obs.size()) {
    for (const auto &obs : speed_planner_input->lon_obs) {
      double ds_offseted =
          std::fmax(0.0, obs.polygon_init.rel_s -
                             speed_planner_input->vehicle_param.length / 2);
      double ego_dv = ego_state.ego_vel - obs.polygon_init.v_frenet;

      double ds_follow_offseted, dv_curve, da_curve, dv_deadband;
      update_dv_curve(
          obs.polygon_init.stop_offset, obs.polygon_init.desired_headway,
          obs.polygon_init.v_frenet, obs.polygon_init.rel_s,
          speed_planner_input->vehicle_param.center_to_front,
          speed_planner_input->speed_tuning_params.deadband_region_ratio,
          ds_follow_offseted, dv_curve, da_curve, dv_deadband,
          ego_state.ego_vel);

      CIPVInfo obj_info;
      obj_info.cipv_id = obs.id;
      obj_info.a = obs.polygon_init.a;
      obj_info.dv = ego_dv;
      obj_info.rel_s = obs.polygon_init.rel_s - half_length;
      obj_info.set_dist =
          obs.polygon_init.desired_headway * obs.polygon_init.v_frenet +
          obs.polygon_init.stop_offset;
      obj_info.v_frenet = obs.polygon_init.v_frenet;
      obj_info.overlap = lon_obstacle_decision_info_map[obs.id].overlap;
      obj_info.dv_curve = dv_curve;
      obj_info.type = obs.type;
      obj_info.v_vector = obs.polygon_init.v;
      obj_info.vel = obs.polygon_init.v_frenet;
      obj_sort.push_back(obj_info);
      MSD_LOG(INFO, "2ndCIPV:Objs Considered1:");
      MSD_LOG(INFO, "2ndCIPV:ID:%d", obj_info.cipv_id);
      MSD_LOG(INFO, "2ndCIPV:v_frenet:%.1f", obj_info.v_frenet);
      MSD_LOG(INFO, "2ndCIPV:v_vector:%.1f", obj_info.v_vector);
      MSD_LOG(INFO, "2ndCIPV:rel_s:%.1f", obs.polygon_init.rel_s);
      MSD_LOG(INFO, "2ndCIPV:v_curve:%.1f", obj_info.v_frenet + dv_curve);
      MSD_LOG(INFO, "2ndCIPV:type:%d", obj_info.type);
      MSD_LOG(INFO, "2ndCIPV:overlap:%.1f", obj_info.overlap);
    }
    std::sort(obj_sort.begin(), obj_sort.end(), comp);
    // CJ:at hold state, don't change cipv until start off
    if (speed_planner_input->stop_point_info.hold_flag == true) {
      for (auto obs : obj_sort) {
        if (obs.cipv_id == cipv_info.cipv_id) {
          cipv_id = obs.cipv_id;
          cipv_v = obs.v_frenet;
          break;
        }
      }
      if (cipv_id < 0) {
        cipv_id = obj_sort.front().cipv_id;
        cipv_v = obj_sort.front().v_frenet;
      }
    } else {
      if (is_smalloverlap(obj_sort[0]) && obj_sort.size() > 1) {
        MSD_LOG(INFO, "2ndCIPV:CIPV_SMOL!ID:%d", obj_sort[0].cipv_id);
        bool find_second_cipv = false;
        int second_cipv_index = 0;
        for (int i = 1; i < obj_sort.size(); ++i) {
          if (is_smalloverlap(obj_sort[i])) {
            MSD_LOG(INFO, "2ndCIPV:CIPV_SMOL!ID:%d", obj_sort[i].cipv_id);
            continue;
          }
          double brake_dist = obj_sort[i].rel_s - obj_sort[i].set_dist;
          brake_dist = std::fmax(brake_dist, 1);
          double dv = obj_sort[i].dv;
          dv = std::fmax(dv, 0);
          double ref_a = dv * dv / 2 / brake_dist;
          if (ref_a > std::fabs(K_SmoothBrake_AxMin)) {
            find_second_cipv = true;
            second_cipv_index = i;
            MSD_LOG(INFO, "2ndCIPV:Find 2nd!ID:%d", obj_sort[i].cipv_id);
            break;
          } else {
            MSD_LOG(INFO, "2ndCIPV:Not Find 2nd!ID:%d", obj_sort[i].cipv_id);
            break;
          }
        }
        if (find_second_cipv == true) {
          for (int i = 0; i < second_cipv_index; ++i) {
            for (auto it = speed_planner_input->lon_obs.begin();
                 it < speed_planner_input->lon_obs.end();) {
              if (obj_sort[i].cipv_id == (*it).id) {
                it = speed_planner_input->lon_obs.erase(it);
              } else {
                ++it;
              }
            }
          }
          std::vector<CIPVInfo> temp_vec;
          for (int i = second_cipv_index; i < obj_sort.size(); ++i) {
            temp_vec.push_back(obj_sort[i]);
          }
          obj_sort.swap(temp_vec);
        }
      }
      for (auto obj : obj_sort) {
        MSD_LOG(INFO, "2ndCIPV:Objs Considered2:");
        MSD_LOG(INFO, "2ndCIPV:ID:%d", obj.cipv_id);
      }
      for (auto obj : speed_planner_input->lon_obs) {
        MSD_LOG(INFO, "2ndCIPV:Objs Considered3:");
        MSD_LOG(INFO, "2ndCIPV:ID:%d", obj.id);
      }
      cipv_id = obj_sort.front().cipv_id;
      cipv_v = obj_sort.front().v_frenet;
    }
  }
  MSD_LOG(INFO, "2ndCIPV:Final_CIPV:%d", cipv_id);
  if (cipv_id != -1) {
    CIPVInfo cipv = {};
    for (const auto obs : obj_sort) {
      if (cipv_id == obs.cipv_id) {
        cipv = obs;
        if (obs.vel >= K_stop_go_CIPV_spd_mps) {
          for (const auto obs1 : obj_sort) {
            if (obs1.rel_s < k_stop_offset + K_stop_go_CIPV_dist_offset &&
                obs1.vel < K_stop_go_CIPV_spd_mps) {
              cipv = obs1;
              break;
            }
          }
        }
        break;
      }
    }
    if (cipv_id != cipv_info.cipv_id) {
      cipv_info.cipv_time = 0.0;
      cipv_info.cipv_velocity_vector.clear();
      cipv_info.overlap = cipv.overlap;
    } else {
      cipv_info.cipv_time += 1.0 / FLAGS_planning_loop_rate;
      cipv_info.cipv_velocity_vector.insert(
          cipv_info.cipv_velocity_vector.begin(), cipv_v);
      if (cipv_info.cipv_velocity_vector.size() > 5) {
        cipv_info.cipv_velocity_vector.pop_back();
      }
      if (cipv_info.overlap < cipv.overlap) {
        cipv_info.overlap = cipv.overlap;
      }
    }
    cipv_info.cipv_id = cipv.cipv_id;
    cipv_info.vel = cipv.vel;
    cipv_info.ds_offseted = cipv.ds_offseted;
    cipv_info.dv_curve = cipv.dv_curve;
    cipv_info.dv_safe = cipv.dv_safe;
    cipv_info.ego_dv = cipv.ego_dv;
    cipv_info.v_frenet = cipv.v_frenet;
    cipv_info.rel_s = cipv.rel_s;
    cipv_info.a = cipv.a;
    cipv_info.set_dist = cipv.set_dist;
    cipv_info.dv = cipv.dv;
    cipv_info.type = cipv.type;
    cipv_info.v_vector = cipv.v_vector;

    if (cipv_info.cipv_velocity_vector.size()) {
      double sum{};
      for (const auto &vel : cipv_info.cipv_velocity_vector) {
        sum += vel;
      }
      cipv_info.cipv_average_velocity =
          sum / cipv_info.cipv_velocity_vector.size();
    }

    cipv_info.is_merge_flag =
        lon_obstacle_decision_info_map[cipv_id].is_merge_flag;

    if (is_smalloverlap(cipv_info)) {
      MSD_LOG(INFO, "2ndCIPV:CIPV_SMOL!ID:%d", cipv_info.cipv_id);
      cipv_info.need_smooth_brake = true;
    } else {
      cipv_info.need_smooth_brake = false;
    }
    const auto &obstacle_manager = baseline_info_->obstacle_manager();
    auto obs_ptr = obstacle_manager.find_obstacle(cipv_info.cipv_id);
    if (obs_ptr) {
      const auto &obstacle_sl_boundary = obs_ptr->PerceptionSLBoundary();
      bool ego_lane_overlap =
          std::fmin(std::fmin(std::fabs(obstacle_sl_boundary.start_l),
                              std::fabs(obstacle_sl_boundary.end_l)),
                    std::fabs(obs_ptr->R_frenet()) < 1.5);
      if (cipv_info.cipv_time > ENALE_ACC_DECEL_REGION_TIME_AT_EFTP_MODE ||
          ego_lane_overlap) {
        cipv_info.is_stable = true;
      }
    }
  } else {
    cipv_info = {};
  }

  // set pre_braking signal when cipv is lat far away
  cipv_info.is_need_pre_braking = false;
  if (cipv_id != -1) {
    auto cipv_obstacle =
        baseline_info_->mutable_obstacle_manager().find_obstacle(cipv_id);
    auto lane_status = context_->planning_status().lane_status;
    const double lane_width = 3.75;
    if (lane_status.status != LaneStatus::Status::LANE_CHANGE &&
        cipv_obstacle != nullptr &&
        std::fabs(cipv_obstacle->R_frenet()) > lane_width) {
      cipv_info.is_need_pre_braking = true;
    } else {
      cipv_info.is_need_pre_braking = false;
    }
  }

  // set is_need_accurate_control, adapt for control
  cipv_info.is_need_accurate_control = true;
  if (cipv_info.cipv_id != -1) {
    const auto &planner_config =
        ConfigurationContext::Instance()->planner_config();
    double set_distance =
        2.0 * cipv_info.vel +
        planner_config.longitudinal_motion_planner_config.eftp_stop_offset;
    double ego_dv = ego_state.ego_vel - cipv_info.vel;
    double accel =
        ego_dv > 0.0
            ? ego_dv * ego_dv / 2.0 /
                  std::fmax(
                      0.01,
                      cipv_info.ds_offseted -
                          planner_config.longitudinal_motion_planner_config
                              .eftp_stop_offset)
            : 0.0;
    const double velocity_thr = 5.0;
    const double accel_thr = 1.0;
    if (ego_state.ego_vel > velocity_thr && accel < accel_thr &&
        cipv_info.ds_offseted > set_distance) {
      cipv_info.is_need_accurate_control = false;
    }
  } else {
    cipv_info.is_need_accurate_control = false;
  }

  // set is_need_soft_control, adapt for control
  const double K_soft_ctrl_v_thr = 3.0;
  const double K_soft_ctrl_s_thr = 2.0;
  cipv_info.is_need_soft_control = false;
  if (cipv_info.cipv_id != -1) {
    if (ego_state.ego_vel < K_soft_ctrl_v_thr) {
      double j1 = -2, j2 = -4, a1 = 0, a2 = 0, v1 = ego_state.ego_vel,
             v2 = ego_state.ego_vel, s1 = 0, s2 = 0,
             rel_s0 = cipv_info.ds_offseted;
      double cyc = 0.1;
      for (int i = 0; i < 100; ++i) {
        if (v1 >= 0) {
          s1 += v1 * cyc;
          v1 += a1 * cyc;
          a1 += j1 * cyc;
          a1 = std::fmax(a1, -5);
        }
        if (v2 >= 0) {
          s2 += v2 * cyc;
          v2 += a2 * cyc;
          a2 += j2 * cyc;
          a2 = std::fmax(a2, -5);
        }
        if (v1 <= 0 && v2 <= 0) {
          break;
        }
      }
      if (v1 <= 0 && v2 <= 0 && s1 - s2 < K_soft_ctrl_s_thr && s1 < rel_s0) {
        cipv_info.is_need_soft_control = true;
      }
    }
  }

  // calculate cipv ttc
  const double K_DefaultTTC = 100.0;
  const double K_SpeedDiffThr = 0.1;
  cipv_info.ttc = K_DefaultTTC;
  double ds = cipv_info.rel_s;
  double dv = ego_state.ego_vel - cipv_info.v_frenet;
  if (cipv_info.cipv_id > 0 && dv > K_SpeedDiffThr) {
    cipv_info.ttc = ds / dv;
  }

  // set is_barrier, for cone_bucker / RB and so on
  cipv_info.is_barrier = false;
  bool cipv_is_cone_bucket = false;
  auto ptr_obstacle =
      baseline_info_->obstacle_manager().find_obstacle(cipv_info.cipv_id);
  // cone bucket
  MSD_LOG(INFO, "CB:------------------");
  if (ptr_obstacle != nullptr && cipv_info.cipv_id > 0 &&
      ptr_obstacle->Type() == ObjectType::CONE_BUCKET) {
    cipv_info.is_barrier = true;
    cipv_is_cone_bucket = true;
    MSD_LOG(INFO, "CB:TRUE");
  } else {
    cipv_info.is_barrier = false;
    cipv_is_cone_bucket = false;
    MSD_LOG(INFO, "CB:FALSE");
  }
  // lidar RB
  const auto &lidar_rb_id_vec = baseline_info_->get_lidar_rb_id_vec();
  cipv_info.is_road_boundary = false;
  if (cipv_info.cipv_id > 10000 &&
      std::find(lidar_rb_id_vec.begin(), lidar_rb_id_vec.end(),
                cipv_info.cipv_id) != lidar_rb_id_vec.end()) {
    cipv_info.is_barrier = true;
    cipv_info.is_road_boundary = true;
  }

  // calculate cone bucket warning level
  MSD_LOG(INFO, "DBCB_CB_cipv_id: %d", cipv_info.cipv_id);
  const double ego_spd = ego_state.ego_vel;
  const double cone_bucket_ttc =
      cipv_is_cone_bucket ? cipv_info.ttc : K_DefaultTTC;
  const bool in_lane_change = PlanningContext::Instance()
                                  ->lateral_behavior_planner_output()
                                  .lc_status != "none";
  const bool dbw_status = world_model_->get_vehicle_dbw_status();

  const double K_StopSpdThr = 0.25 / 3.6;
  const int K_TTCCntThr = 3;
  const int K_NonCipvCntThr = 5;
  MSD_LOG(INFO, "in_lane_change: %d", in_lane_change);
  const auto &longitudinal_motion_planner_config =
      ConfigurationContext::Instance()
          ->planner_config()
          .longitudinal_motion_planner_config;

  const double kConeBucketSoftBrakeDist = 50;
  if (cipv_is_cone_bucket && cipv_info.rel_s > kConeBucketSoftBrakeDist) {
    speed_planner_input->vaj_bound_info.a_min_value_pt = {-1, -1};
    speed_planner_input->speed_tuning_params.accel_limit_scale =
        ACCELERATION_LIMIT_SCALE * 10000;
  }

  const double warning_speed_thr_1 =
      longitudinal_motion_planner_config.cone_bucket_warning_speed_thr_1;
  const double warning_speed_thr_2 =
      longitudinal_motion_planner_config.cone_bucket_warning_speed_thr_2;
  const double warning_ttc_thr_1 =
      longitudinal_motion_planner_config.cone_bucket_warning_ttc_thr_1;
  const double warning_ttc_thr_2 =
      longitudinal_motion_planner_config.cone_bucket_warning_ttc_thr_2;
  const double warning_ttc_thr_3 =
      longitudinal_motion_planner_config.cone_bucket_warning_ttc_thr_3;

  const int K_BrakeCntThr = 3;
  const double K_BrakeDecThr = -0.25;

  auto &cone_bucket_info = speed_planner_input->cone_bucket_info;
  bool &has_cone_bucket = cone_bucket_info.has_cone_bucket;
  int &ttc_beyond_thr_cnt = cone_bucket_info.ttc_beyond_thr_cnt;
  int &warning_level = cone_bucket_info.warning_level;
  int &last_warning_level = cone_bucket_info.last_warning_level;
  int &non_cipv_cnt = cone_bucket_info.non_cipv_cnt;
  int &brake_cnt = cone_bucket_info.brake_cnt;
  bool &has_trigger_force_stop = cone_bucket_info.has_trigger_force_stop;

  const bool last_has_cone_bucket = has_cone_bucket;

  double cone_bucket_ttc_thr = warning_ttc_thr_1;
  if (ego_spd > warning_speed_thr_1) {
    cone_bucket_ttc_thr = warning_ttc_thr_1;
  } else if (ego_spd > warning_speed_thr_2) {
    cone_bucket_ttc_thr = warning_ttc_thr_2;
  } else {
    cone_bucket_ttc_thr = warning_ttc_thr_3;
  }

  cone_bucket_info.ttc_thr = cone_bucket_ttc_thr;
  cone_bucket_info.ttc = cone_bucket_ttc;

  // debounce design
  if (cipv_is_cone_bucket) {
    ++brake_cnt;
  } else {
    brake_cnt = 0;
  }

  if (dbw_status) {
    if (ego_spd >= K_StopSpdThr) {
      has_trigger_force_stop = false;
    }
    if (in_lane_change || brake_cnt <= K_BrakeCntThr) {
      warning_level = 0;
      ttc_beyond_thr_cnt = 0;
      non_cipv_cnt = 0;
    } else {
      if (cipv_is_cone_bucket) {
        if (ego_spd >= K_StopSpdThr) {
          if (last_warning_level == 2) {
            warning_level = 2;
            ttc_beyond_thr_cnt = 0;
          } else {
            ttc_beyond_thr_cnt = cone_bucket_ttc < cone_bucket_ttc_thr
                                     ? ttc_beyond_thr_cnt + 1
                                     : 0;

            warning_level = ttc_beyond_thr_cnt >= K_TTCCntThr
                                ? 2
                                : std::max(1, last_warning_level);
          }
        } else {
          if (has_trigger_force_stop) {
            warning_level = world_model_->is_force_stop() ? 0 : 3;
          } else {
            warning_level = 3;
            has_trigger_force_stop = true;
          }
          ttc_beyond_thr_cnt = 0;
        }
        non_cipv_cnt = 0;
      } else {
        if (last_has_cone_bucket || non_cipv_cnt > 0) {
          ++non_cipv_cnt;
        }
        if (non_cipv_cnt <= K_NonCipvCntThr) {
          warning_level = last_warning_level;
        } else {
          warning_level = 0;
          non_cipv_cnt = 0;
        }
        ttc_beyond_thr_cnt = 0;
      }
    }
  } else {
    warning_level = 0;
    ttc_beyond_thr_cnt = 0;
    non_cipv_cnt = 0;
    has_trigger_force_stop = false;
  }
  last_warning_level = warning_level;
  has_cone_bucket = cipv_is_cone_bucket;

  auto &cipv_lost_info = speed_planner_input->cipv_lost_info;
  int &pre_cipv_id = cipv_lost_info.pre_cipv_id;
  double &pre_cipv_rel_s = cipv_lost_info.pre_cipv_rel_s;
  double &pre_cipv_ttc = cipv_lost_info.pre_cipv_ttc;
  constexpr double kMinSpeedThr = 5.0;
  auto &history_cipv_ids = cipv_lost_info.history_cipv_ids;
  pre_cipv_id = cipv_info.cipv_id;
  pre_cipv_rel_s = cipv_info.rel_s;
  pre_cipv_ttc = pre_cipv_id > 0 ? pre_cipv_rel_s / std::fmax(kMinSpeedThr,
                                                              ego_state.ego_vel)
                                 : 100.0;

  if (history_cipv_ids.size() < 5) {
    history_cipv_ids.emplace_back(pre_cipv_id);
  } else {
    for (int i = 0; i <= 3; ++i) {
      history_cipv_ids[i] = history_cipv_ids[i + 1];
    }
    history_cipv_ids[4] = pre_cipv_id;
  }

  return;
}

void LongitudinalPlannerPreprocessor::compute_overlap_info() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();
  const auto &ego_vel = baseline_info_->get_ego_state().ego_vel;
  const double delta_t = 0.2;
  const double sv_limit_safety_buffer = 1.0 + 3.0 / 20.0 * ego_vel;
  auto frenet_coord = baseline_info_->get_frenet_coord();
  double length =
      msquare::ConfigurationContext::Instance()->get_vehicle_param().length;
  double width =
      msquare::ConfigurationContext::Instance()->get_vehicle_param().width;
  speed_planner_input->overlap_info_map.clear();

  // generate ego sl polygon seq
  SLPolygonSeq ego_sl_polygon_seq;
  static PolygonWithT ego_sl_polygon_tmp;
  ego_sl_polygon_tmp.second.clear();
  for (int i = 0; i < speed_planner_input->model_trajectory.size(); i++) {
    double ego_box_expansion_length = length;
    double ego_box_expansion_width = width;
    if (i <= 2) {
      ego_box_expansion_length +=
          planner_config.longitudinal_motion_planner_config
              .ego_box_expansion_length;
      ego_box_expansion_width +=
          planner_config.longitudinal_motion_planner_config
              .ego_box_expansion_width;
    }
    const auto &point = speed_planner_input->model_trajectory[i];
    auto ego_enu_box =
        planning_math::Box2d({point.x, point.y}, point.heading_angle,
                             ego_box_expansion_length, ego_box_expansion_width);
    auto ego_enu_polygon = planning_math::Polygon2d(ego_enu_box);
    static std::vector<planning_math::Vec2d> fren_vertexes;
    static planning_math::Polygon2d convex_polygon;
    convex_polygon.clear();
    fren_vertexes.clear();

    for (auto cart_vertex : ego_enu_polygon.GetAllVertices()) {
      Point2D cart_point, fren_point;
      cart_point.x = cart_vertex.x();
      cart_point.y = cart_vertex.y();
      if (frenet_coord->CartCoord2FrenetCoord(cart_point, fren_point) ==
              TRANSFORM_FAILED ||
          std::isnan(fren_point.x) || std::isnan(fren_point.y)) {
        continue;
      }
      planning_math::Vec2d fren_vertex(fren_point.x, fren_point.y);
      fren_vertexes.push_back(fren_vertex);
    }
    if (!planning_math::Polygon2d::ComputeConvexHull(fren_vertexes,
                                                     &convex_polygon)) {
    }

    ego_sl_polygon_tmp.first = i * delta_t;
    ego_sl_polygon_tmp.second = convex_polygon;
    ego_sl_polygon_seq.push_back(ego_sl_polygon_tmp);
  }

  for (const auto &obs : speed_planner_input->lon_obs) {
    auto ptr_obstacle = obstacle_manager.find_obstacle(obs.id);
    if (ptr_obstacle == nullptr) {
      continue;
    }

    auto obs_sl_polygon_seq = ptr_obstacle->sl_polygon_seq();
    for (int i = 0;
         i < std::min(obs_sl_polygon_seq.size(), ego_sl_polygon_seq.size());
         i++) {
      double time = i * delta_t;
      auto s_range = StGraphGenerator::sl_polygon_path_check(
          {ego_sl_polygon_seq[i].second}, obs_sl_polygon_seq[i].second);
      if (s_range.first < s_at_control_points_.back()) {
        const auto &traj_point = ptr_obstacle->GetPointAtTime(time);
        double theta_ref = frenet_coord->GetRefCurveHeading(s_range.first);
        double v_frenet = std::fmax(
            0.0,
            traj_point.v * std::cos(traj_point.velocity_direction - theta_ref));
        double s_need_to_add_limit = std::fmax(
            s_range.first - speed_planner_input->vehicle_param.center_to_front -
                sv_limit_safety_buffer,
            speed_planner_input->planning_init_state.s);
        speed_planner_input->overlap_info_map[obs.id] = {
            obs.id, time, s_need_to_add_limit, v_frenet};
        break;
      }
    }
  }

  speed_planner_input->enable_model_traj_modify = false;
  speed_planner_input->model_traj_modify_time_s +=
      1.0 / FLAGS_planning_loop_rate;
  if (speed_planner_input->overlap_info_map.size() && ego_vel < 5.0 &&
      speed_planner_input->use_eftp) {
    speed_planner_input->enable_model_traj_modify = true;
    speed_planner_input->model_traj_modify_time_s = 0.0;
  } else if (speed_planner_input->model_traj_modify_time_s <
             planner_config.longitudinal_motion_planner_config
                 .model_traj_modify_time_s) {
    speed_planner_input->enable_model_traj_modify = true;
  }

  if (speed_planner_input->enable_model_traj_modify) {
    const auto &planning_init_state = speed_planner_input->planning_init_state;
    BoundedConstantJerkTrajectory1d brake_trajectory(
        planning_init_state.s, planning_init_state.v, planning_init_state.a,
        -1.0, 0.2);
    brake_trajectory.set_bound(0.0, planning_init_state.v, -1.0, 0.0);
    for (int i = 0; i < speed_planner_input->model_trajectory.size(); i++) {
      double relative_time = i * 0.2;
      auto &point = speed_planner_input->model_trajectory[i];
      point.s = brake_trajectory.evaluate(0, relative_time);
      point.v = brake_trajectory.evaluate(1, relative_time);
    }
  }
}

void LongitudinalPlannerPreprocessor::calc_accel_limit() {
  // if cipv is safe, limit accel for comfort
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &cipv_info = speed_planner_input->cipv_info;
  const auto &obstacle_manager = baseline_info_->obstacle_manager();
  const auto &ego_state =
      baseline_info_->get_ego_state_manager().get_ego_state();
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();

  const double enable_ACC_comfot_cost_speed_thr_mps = 3.0;
  if (cipv_info.cipv_id != -1 &&
      planner_config.longitudinal_motion_planner_config
          .enable_ACC_comfot_cost &&
      ego_state.ego_vel > enable_ACC_comfot_cost_speed_thr_mps) {
    auto obs_ptr = obstacle_manager.find_obstacle(cipv_info.cipv_id);

    if (cipv_info.ego_dv > cipv_info.dv_curve) {
      const double v_normalizer = cipv_info.dv_safe - cipv_info.dv_curve;
      const double ratio = (cipv_info.ego_dv - cipv_info.dv_curve) /
                           std::fmax(0.1, v_normalizer);
      if (ratio < 0.5) {
        const double max_decel_accel = 3.0;
        const double min_decel_accel = 1.0;
        speed_planner_input->vaj_bound_info.a_min = -std::fmin(
            max_decel_accel,
            std::fmax(min_decel_accel,
                      (cipv_info.ego_dv - cipv_info.dv_curve) / 2.0));
        speed_planner_input->vaj_bound_info.a_min_reason =
            "cipv is safe, limit accel for comfort";
      }
    }
  }
}

void LongitudinalPlannerPreprocessor::calc_stop_ref() {
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  const auto &planner_config =
      ConfigurationContext::Instance()->planner_config();
  const auto &init_state = speed_planner_input->planning_init_state;
  const auto &cipv_info = speed_planner_input->cipv_info;
  auto &stop_point_info = speed_planner_input->stop_point_info;
  stop_point_info.ref_s_list.clear();
  stop_point_info.ref_v_list.clear();
  stop_point_info.ref_a_list.clear();

  if (stop_point_info.has_stop_point &&
      stop_point_info.stop_id == cipv_info.cipv_id) {
    const double brake_dist =
        std::fmax(stop_point_info.stop_s - init_state.s, 0.1);
    const double comfort_brake_jerk =
        planner_config.longitudinal_motion_planner_config.stop_ref_jerk;
    const double max_jerk = 2.0;
    const double max_brake_accel = 5.0;
    double min_brake_accel =
        LinearInterpation::interpolation(brake_dist, {0.0, 10.0}, {0.2, 0.7});

    SmoothBrakeTrajectory brake_traj_min(stop_point_info.stop_s,
                                         min_brake_accel, comfort_brake_jerk);
    SmoothBrakeTrajectory brake_traj_max(stop_point_info.stop_s,
                                         max_brake_accel, max_jerk);

    std::vector<double> dist_to_stop_list{};
    std::vector<double> dist_to_stop_list_seg1{};
    std::vector<double> dist_to_stop_list_seg2{};
    std::vector<double> dist_to_stop_list_seg3{};
    InterpolationData<double> s_v_max_list{};

    const double slide_a =
        init_state.v > K_stop_exit_relv_mps ? 0.7 : min_brake_accel;
    const std::vector<double> BRAKE_DIS_PT = {5, 10, 15};
    const std::vector<double> SLIDE_DIS_TABLE = {1, 2, 3};
    const double slide_dist = 1.0;
    double slide_v = std::sqrt(2 * slide_a * slide_dist);
    MSD_LOG(INFO,
            "STOP_REF:slide_dist:%.2f,slide_v:%.2f,slide_s:%.2f,slide_a:%.2f",
            slide_dist, slide_v, stop_point_info.stop_s - slide_dist, slide_a);
    Index<NUM_SPEED_CONTROL_POINTS> index;
    while (index.advance()) {
      auto &quad_point_info =
          speed_planner_input
              ->speed_segments[index.segment_index][index.quad_index]
              .quad_point_info;
      const auto &sample_s = quad_point_info.sample_s;
      if (sample_s < stop_point_info.stop_s) {
        dist_to_stop_list.emplace_back(stop_point_info.stop_s - sample_s);
      }
      s_v_max_list.emplace_back(sample_s, quad_point_info.v_max);
    }
    dist_to_stop_list.emplace_back(0.0);
    std::reverse(dist_to_stop_list.begin(), dist_to_stop_list.end());
    // build seg1
    double max_step = 1.0;
    for (const auto &dist_to_stop : dist_to_stop_list) {
      if (dist_to_stop < slide_dist) {
        max_step = 0.1;
        while (!dist_to_stop_list_seg1.empty() &&
               dist_to_stop > dist_to_stop_list_seg1.back() + max_step) {
          dist_to_stop_list_seg1.emplace_back(dist_to_stop_list_seg1.back() +
                                              max_step);
        }
        dist_to_stop_list_seg1.emplace_back(dist_to_stop);
      } else {
        max_step = 1.0;
      }
    }
    MSD_LOG(INFO, "STOP_REF:Time:%.2f============", MTIME()->timestamp().sec());
    for (const auto &distance_to_stop_point : dist_to_stop_list_seg1) {
      double s = stop_point_info.stop_s - distance_to_stop_point;
      auto svt_point = SmoothBrakeTrajectory::evaluateByConstAccel(
          s, stop_point_info.stop_s, slide_v, slide_dist, slide_a);
      stop_point_info.ref_s_list.emplace_back(s);
      stop_point_info.ref_v_list.emplace_back(svt_point.v);
      stop_point_info.ref_a_list.emplace_back(-svt_point.a);
      MSD_LOG(INFO, "STOP_REF:SEG1:s:%.2f,v:%.2f,a:%.2f", s, svt_point.v,
              -svt_point.a);
    }
    if (brake_dist > slide_dist) {
      // build seg2
      constexpr double kMaxTransitionJerk = 5.0;
      constexpr double kMinTransitionJerk = 1.0;
      constexpr double kMaxTransitionTime = 2.0;
      double seg2_ref_a = std::pow(std::max(init_state.v - slide_v, 0.0), 2) *
                          0.5 / (brake_dist - slide_dist);
      double seg3_time = std::fmax(init_state.v - slide_v, 0.0) / seg2_ref_a;
      // seg2_time_ratio is the time of const jerk / (const ax + const jerk)
      // inc seg2_time_ratio will decreas jerk and inc max ax
      double seg2_time_ratio = 0.3;
      double seg2_jerk =
          std::fmax(seg2_ref_a - slide_a, 0.0) / seg3_time / seg2_time_ratio;
      seg2_jerk = std::fmin(seg2_jerk, kMaxTransitionJerk);
      seg2_jerk = std::fmax(seg2_jerk, kMinTransitionJerk);
      dist_to_stop_list_seg2.emplace_back(slide_dist);
      constexpr double seg2_search_dt = 0.1;
      BoundedConstantJerkTrajectory1d seg2_traj(slide_dist, slide_v, slide_a,
                                                seg2_jerk, seg2_search_dt);
      seg2_traj.set_bound(slide_v, init_state.v, slide_a, max_brake_accel);
      double seg3_start_dist = 0.0;
      double seg3_a = 0.0;
      MSD_LOG(INFO,
              "STOP_REF:SEG2:seg2_ref_a:%.2f,seg3_time:%.2f,seg2_jerk:%.2f",
              seg2_ref_a, seg3_time, seg2_jerk);
      constexpr double time_seg2_last = 300; // max_last_time:30s
      for (int i = 0; i < time_seg2_last; ++i) {
        constexpr double kMathEpsilon = 1e-6;
        double time = i * seg2_search_dt;
        double search_seg2_dist = seg2_traj.evaluate(0, time);
        double search_seg2_v = seg2_traj.evaluate(1, time);
        double search_seg2_a = seg2_traj.evaluate(2, time);
        double search_seg3_ref_a = 0.1;
        double search_seg3_brake_dist =
            std::fmax(brake_dist - search_seg2_dist, 0.1);
        if (init_state.v > search_seg2_v) {
          search_seg3_ref_a =
              (std::pow(init_state.v, 2) - std::pow(search_seg2_v, 2)) / 2 /
              search_seg3_brake_dist;
        }
        MSD_LOG(INFO,
                "STOP_REF:SEG2:ref_a2:%.1f,ref_a3:%.1f,dist3:%.2f,time:%.1f",
                search_seg2_a, search_seg3_ref_a, search_seg3_brake_dist, time);
        if (search_seg3_ref_a <= search_seg2_a ||
            std::abs(search_seg3_brake_dist - 0.1) < kMathEpsilon ||
            time > kMaxTransitionTime) {
          // find split point of seg2 and seg3, seg2 building finish
          seg3_start_dist = brake_dist - search_seg3_brake_dist;
          seg3_a = search_seg3_ref_a;
          MSD_LOG(INFO, "STOP_REF:SEG2:stop const jerk time:%.1f", time);
          break;
        }
        dist_to_stop_list_seg2.emplace_back(search_seg2_dist);
        double s = stop_point_info.stop_s - search_seg2_dist;
        SmoothBrakeTrajectory::SVTPoint svt_point{s, search_seg2_v,
                                                  search_seg2_a};
        stop_point_info.ref_s_list.emplace_back(s);
        stop_point_info.ref_v_list.emplace_back(std::fmin(
            svt_point.v, LinearInterpation::interpolate(s_v_max_list, s)));
        stop_point_info.ref_a_list.emplace_back(-svt_point.a);
        MSD_LOG(INFO, "STOP_REF:SEG2:s:%.2f,v:%.2f,a:%.2f", s,
                stop_point_info.ref_v_list.back(), -svt_point.a);
      }
      // build seg3
      constexpr double nums_seg3_point = 200; // max_dis:200m;
      for (int i = 0; i < nums_seg3_point; ++i) {
        dist_to_stop_list_seg3.emplace_back(seg3_start_dist + i * max_step);
        double seg3_brake_dist = brake_dist - dist_to_stop_list_seg3.back();
        double s = stop_point_info.stop_s - dist_to_stop_list_seg3.back();
        double seg3_point_v = std::sqrt(std::fmax(
            std::pow(init_state.v, 2) - 2 * seg3_a * seg3_brake_dist, 0.0));
        if (dist_to_stop_list_seg3.back() > brake_dist) {
          break;
        }
        auto min_svt_point = brake_traj_min.evaluateByDistance(
            stop_point_info.stop_s + dist_to_stop_list_seg3.back());
        auto max_svt_point = brake_traj_max.evaluateByDistance(
            stop_point_info.stop_s + dist_to_stop_list_seg3.back());
        SmoothBrakeTrajectory::SVTPoint svt_point{s, seg3_point_v, seg3_a};
        svt_point = svt_point.v > max_svt_point.v ? max_svt_point : svt_point;
        svt_point = svt_point.v < min_svt_point.v ? min_svt_point : svt_point;
        stop_point_info.ref_s_list.emplace_back(s);
        stop_point_info.ref_v_list.emplace_back(std::fmin(
            svt_point.v, LinearInterpation::interpolate(s_v_max_list, s)));
        stop_point_info.ref_a_list.emplace_back(-svt_point.a);
        MSD_LOG(INFO, "STOP_REF:SEG3:s:%.2f,v:%.2f,a:%.2f", s,
                stop_point_info.ref_v_list.back(), -svt_point.a);
      }
    }

    std::reverse(stop_point_info.ref_s_list.begin(),
                 stop_point_info.ref_s_list.end());
    std::reverse(stop_point_info.ref_v_list.begin(),
                 stop_point_info.ref_v_list.end());
    std::reverse(stop_point_info.ref_a_list.begin(),
                 stop_point_info.ref_a_list.end());
  }
}

void LongitudinalPlannerPreprocessor::ProcessCipvFn() {
  const auto &dbw_status = world_model_->get_vehicle_dbw_status();
  const auto &speed_planner_input = context_->mutable_speed_planner_input();
  auto &cipv_lost_info = speed_planner_input->cipv_lost_info;
  bool &cipv_fn = cipv_lost_info.cipv_fn;
  bool &prohibit_acc = cipv_lost_info.prohibit_acc;
  bool &has_warned = cipv_lost_info.has_warned;
  int &counter = cipv_lost_info.counter;
  int &warning_level = cipv_lost_info.warning_level;
  double &speed_limit = cipv_lost_info.speed_limit;
  double &start_time = cipv_lost_info.start_time;
  double &end_time = cipv_lost_info.end_time;
  double &duration = cipv_lost_info.duration;
  const int &pre_cipv_id = cipv_lost_info.pre_cipv_id;
  const double &pre_cipv_rel_s = cipv_lost_info.pre_cipv_rel_s;
  const double &pre_cipv_ttc = cipv_lost_info.pre_cipv_ttc;
  int &pre_cipv_lost_id = cipv_lost_info.pre_cipv_lost_id;
  int &cipv_fn_tid = cipv_lost_info.cipv_fn_tid;
  const auto &history_cipv_ids = cipv_lost_info.history_cipv_ids;

  const auto &ego_vel =
      world_model_->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
  const auto &lon_config = ConfigurationContext::Instance()
                               ->planner_config()
                               .longitudinal_motion_planner_config;
  constexpr double kTimeEps = 0.001;
  constexpr double prohibit_acc_min_duration = 0.3;
  constexpr double kCipvLostTtcThr = 5.0;
  const double cipv_lost_min_cooldown_time =
      lon_config.cipv_lost_min_cooldown_time;
  const double cipv_lost_quit_cp_delay_time =
      lon_config.cipv_lost_quit_cp_delay_time;
  cipv_fn = world_model_->get_cipv_fn();
  cipv_fn_tid = world_model_->get_cipv_fn_tid();

  bool continous_flag = true;
  for (const auto &id : history_cipv_ids) {
    if (id != cipv_fn_tid) {
      continous_flag = false;
      break;
    }
  }

  if (cipv_fn && ego_vel > K_stop_ego_hold_spd_mps && dbw_status) {
    if (prohibit_acc && counter == 0 &&
        MTIME()->timestamp().sec() >
            start_time + prohibit_acc_min_duration + kTimeEps &&
        MTIME()->timestamp().sec() <
            start_time + cipv_lost_quit_cp_delay_time - kTimeEps) {
      counter = 0;
    } else if (end_time > 0 &&
               MTIME()->timestamp().sec() > end_time + kTimeEps &&
               MTIME()->timestamp().sec() <
                   end_time + cipv_lost_min_cooldown_time - kTimeEps) {
      counter = 0;
    } else {
      if (pre_cipv_lost_id > 0 && cipv_fn_tid == pre_cipv_lost_id) {
        ++counter;
      } else if (pre_cipv_id > 0 && cipv_fn_tid == pre_cipv_id &&
                 continous_flag && pre_cipv_ttc < kCipvLostTtcThr) {
        pre_cipv_lost_id = cipv_fn_tid;
        ++counter;
      } else {
        pre_cipv_lost_id = -1;
        counter = 0;
      }
    }
  } else {
    pre_cipv_lost_id = -1;
    counter = 0;
  }

  const bool finish_cooldown =
      start_time < 0.0 ||
      (end_time > 0.0 && MTIME()->timestamp().sec() - end_time >
                             lon_config.cipv_lost_min_cooldown_time - kTimeEps);

  if (counter == 1 && finish_cooldown && ego_vel > K_stop_ego_hold_spd_mps &&
      !has_warned) {
    start_time = MTIME()->timestamp().sec();
    end_time = start_time + lon_config.cipv_lost_prohibit_acc_duration;
    speed_limit = ego_vel;
  } else if (counter == 3 && !has_warned) {
    end_time += lon_config.cipv_lost_quit_cp_delay_time;
  }
  prohibit_acc = MTIME()->timestamp().sec() > start_time - kTimeEps &&
                 MTIME()->timestamp().sec() < end_time + kTimeEps;

  if (prohibit_acc) {
    duration = MTIME()->timestamp().sec() - start_time;
    if (counter >= lon_config.cipv_lost_quit_cp_counter_thr) {
      has_warned = true;
      warning_level = 2;
    } else {
      warning_level = 0;
    }
  } else {
    duration = 0.0;
    speed_limit = 0.0;
    warning_level = 0;
    has_warned = false;
  }
}

} // namespace msquare
