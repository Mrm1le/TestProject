#pragma once

#include "nlohmann/json.hpp"
#include "path_planner_interface.hpp"
#include "planner_constants.hpp"
#include "proto_idl/interface_speed_planner.hpp"
using json = nlohmann::json;

namespace cp_speed_planner {

template <typename T>
inline typename std::vector<T>::const_iterator vector_map_find(
    const std::vector<T> &vec, int32_t id) {
  auto itr = std::find_if(vec.begin(), vec.end(),
                          [&](const T &itm) { return itm.first == id; });

  return itr;
}

template <typename T>
inline T &vector_map_get(std::vector<T> &vec, int32_t id) {
  auto itr = vector_map_find(vec, id);
  if (itr != vec.end()) {
    return vec[itr - vec.begin()];
  } else {
    T res;
    res.first = id;
    res.second = {};
    vec.emplace_back(res);
    return vec.back();
  }
}

template <typename T>
inline bool vector_map_count(const std::vector<T> &vec, int32_t id) {
  auto itr = vector_map_find(vec, id);
  return itr != vec.end();
}

template <typename T>
inline void vector_map_erase(std::vector<T> &vec, int32_t id) {
  auto itr = vector_map_find(vec, id);

  if (itr != vec.end()) {
    vec.erase(itr);
  }
}

using SpeedPlannerOutput = cp_path_planner::PathPlannerOutput;

// debug info
struct SpeedPlannerDebug {
  std::array<double, TOTAL_NUM_RESIDUALS> residual_debug;
  std::array<std::array<double, TOTAL_NUM_RESIDUALS>, TOTAL_NUM_PARAMS>
      per_residual_per_param_gradient;
  std::array<double, TOTAL_NUM_PARAMS> total_gradients_per_param;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairInitVectorDouble, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RadarPoint, id, s, l, x, y, absolute_vel,
                                   relative_vel)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    StopPointInfo, has_stop_point, has_stop_point_last, should_start,
    should_start_last, standstill, standstill_last, stop_s, is_model_stop,
    enable_modify_stop_point, hold_cnt, stop_mode, last_stop_mode, stop_timer,
    real_stop_timer, hold_flag, auto_go, go_indicator, driver_go, ready_go,
    low_speed_state, last_cipv_id, stop_reason, stop_id, from_closest_obs,
    hold2kick, stop_in_code, stop_out_code, auto_hold, force_stop, stop_signal,
    last_cipv_lost, driver_go_timer, last_force_stop, stop_target_accel,
    stop_target_accel_entry_velocity, stop_target_accel_entry_distance,stop_offset,
    last_stop_for_barrier, stop_traj, ref_s_list, ref_v_list, ref_a_list)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedSampleInfo, sample_s, v_min, v_max,
                                   v_ref, v_min_reason, v_max_reason,
                                   v_ref_reason)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedSegmentInfo,
                                   end_segment_control_point_s, quad_point_info)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VelocityAccelJerkBoundInfo, a_min,
                                   a_min_reason, a_max, a_max_reason, j_min,
                                   j_min_reason, j_max, j_max_reason,
                                   a_max_value_pt, a_max_spd_pt, a_min_value_pt,
                                   a_min_spd_pt, j_min_value_pt, j_min_spd_pt,
                                   j_max_value_pt, j_max_spd_pt,
                                   is_ddmap_intersection)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ACCParams, safe_v_limit_scale,
                                   accel_ceiling_scale, cruise_scale,
                                   deadband_region_ratio, deadband_limit_scale,
                                   accel_ceiling_cost_scale,
                                   acc_decel_region_scale,
                                   acc_accel_region_scale, acc_v_max_scale)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    SpeedTuningParams, max_num_iterations, accel_jerk_ratio, accel_limit_scale,
    accel_scale, jerk_limit_scale, jerk_scale, stop_point_scale,
    model_ref_scale, model_safety_scale, gmp_ref_scale, safe_v_limit_scale,
    accel_ceiling_scale, cruise_scale, deadband_region_ratio,
    deadband_limit_scale, accel_ceiling_cost_scale, acc_decel_region_scale,
    acc_accel_region_scale, acc_v_max_scale)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleAxInfo, speed_memory, obstacle_alive,
                                   ax_filted)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedLimitSet, a_max, a_min, j_max, j_min,
                                   v_max_smooth, v_min_smooth, v_max, v_min)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ArraySpeedLimitSet, arr)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedLimitInfo, cruise_speed_set,
                                   curve_speed_set)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ArraySpeedSegmentInfo, arr)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleExtraInfo, type, is_alive, counter,
                                   duration_s, init_headway)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairIntObstacleExtraInfo, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CIPVInfo, cipv_id, vel, ds_offseted,
                                   dv_curve, dv_safe, ego_dv, cipv_time,
                                   is_need_pre_braking, is_merge_flag,
                                   need_smooth_brake, cipv_average_velocity,
                                   cipv_velocity_vector, is_stable,
                                   is_need_accurate_control,
                                   is_need_soft_control, v_frenet, rel_s,
                                   a, set_dist, dv, overlap, type, v_vector,
                                   is_barrier, ttc, is_road_boundary)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RearCarInfo, nearest_rear_car_id, 
                                   has_in_lane_rear_car, nearest_rear_car_v, 
                                   nearest_rear_car_dist)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RCCInfo, state_machine, is_rcc_in_cd, cd_cnt, 
                                   reset_cd_cnt, rcc_waiting_cnt, rcc_duration, rcc_acc, 
                                   last_rear_car_id, is_same_rear_car, rcc_max_v_gain,
                                   ref_s_list, ref_v_list)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ACCLCAssistInfo, is_acc_lane_change,
                                   acc_lane_change_cnt, left_acc, left_dec,
                                   right_acc, right_dec, adjust_cnt, lc_cipv_id, turn_signal)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IdDecisionLog, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LonDecisionInfo, lon_decision_info_map)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ModelPoint, t, v, a, s, x, y, heading_angle)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OverlapInfo, id, time, s, v)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairIntOverlapInfo, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VectorPairIntDouble, vec,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ConeBucketInfo, has_cone_bucket,
                                   has_trigger_force_stop, warning_level,
                                   last_warning_level, ttc_beyond_thr_cnt,
                                   non_cipv_cnt, brake_cnt, ttc, ttc_thr)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LidarRoadEdgeInfo, road_edge_id, has_road_edge, 
    stop_for_road_edge, stop_offset)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AccTakeoverInfo, enable_acc_takeover, need_acc_takeover, 
    init_flag, last_running, start_flag, stop_flag)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(StateMachineParam, real_run_time_s,
                                   real_cooldown_time_s, real_state,
                                   desired_state)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CIPVLostInfo, cipv_fn, prohibit_acc,
                                   has_warned, counter, warning_level,
                                   speed_limit, start_time, end_time, duration,
                                   pre_cipv_id, pre_cipv_lost_id, cipv_fn_tid,
                                   pre_cipv_rel_s, pre_cipv_ttc,
                                   history_cipv_ids)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    SpeedPlannerInput, cipv_info, rear_car_info, rcc_info, acc_lc_assist_info,
    is_mrc_inlane_brake, is_replan, is_lane_change, use_eftp, enable_model_ref,
    b_dagger_longitudinal, use_prediction, acc_overtake_assist_switch,
    is_acc_lane_change, acc_lane_change_cnt, throttle_override,
    enable_model_traj_modify, model_traj_modify_time_s, set_speed, set_hw_level,
    is_acc_mode, kick_start_time, v_curv, a_curv, planning_init_v, max_curv_s,
    max_curv, model_trajectory, ego_state, planning_init_state, vaj_bound_info,
    speed_segments, last_opt_params, s_at_control_points, speed_tuning_params,
    stop_point_info, radar_points, lon_obs, vehicle_param, obstacle_ax_info,
    speed_limit_info, obstacle_extra_info_map, cipv_map_mem, overlap_info_map,
    dbw_status, gmp_valid, cone_bucket_info, lidar_road_edge_info,
    acc_takeover_info, state_machine_param, enable_env_speed_limit,
    cipv_lost_info)
// debug info
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedPlannerDebug, residual_debug,
                                   per_residual_per_param_gradient,
                                   total_gradients_per_param)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleDecisionInfo, lon_decision,
                                   is_follow, exist, cut_in_flag, cut_in_count,
                                   crossing, on_heading, in_lane, collision,
                                   collision_time, lat_dist_head, obj_vy_dy,
                                   follow_time_count, ignore_time_count,
                                   rule_base_cutin, ftp_cutin, lat_cutin, is_merge_flag,
                                   overlap, type, lc_follow_count, lc_follow_id,
                                   lc_cipv_in_lane, virtual_overlap)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IntObstacleDecisionInfo, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LonDecisionOutput, lon_decision_info,
                                   obstacle_decision_info_map)

} // namespace speed_planner
