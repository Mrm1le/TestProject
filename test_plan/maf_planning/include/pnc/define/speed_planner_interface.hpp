#pragma once

#include "nlohmann/json.hpp"
#include "path_planner_interface.hpp"
#include "planner_constants.hpp"
#include <vector>
using json = nlohmann::json;

namespace speed_planner {

struct RadarPoint {
  int id;
  double s;
  double l;
  double x;
  double y;
  double absolute_vel;
  double relative_vel;
};

struct StopPointInfo {
  bool has_stop_point;
  bool has_stop_point_last;
  bool should_start;
  bool should_start_last;
  bool standstill;
  bool standstill_last;
  double stop_s;
  bool is_model_stop;
  bool enable_modify_stop_point;
  int hold_cnt;
  bool stop_mode;
  bool last_stop_mode;
  double stop_timer;
  double real_stop_timer;
  bool hold_flag;
  bool auto_go;
  bool go_indicator;
  bool driver_go;
  bool ready_go;
  bool low_speed_state;
  int last_CIPV_id;
  std::string stop_reason;
  int stop_id;
  bool from_closest_obs;
  bool hold2kick;
  int stop_in_code;
  int stop_out_code;
  bool auto_hold;
  bool force_stop;
  bool stop_signal;
  bool last_CIPV_lost;
  int driver_go_timer;
  bool last_force_stop;
  // Stop relevant params
  double stop_target_accel;
  double stop_target_accel_entry_velocity;
  double stop_target_accel_entry_distance;
  double stop_offset = 0.0;
  bool last_stop_for_barrier;
  std::vector<std::pair<double,double>> stop_traj;

  // Stop ref info
  std::vector<double> ref_s_list{};
  std::vector<double> ref_v_list{};
  std::vector<double> ref_a_list{};
};

struct SpeedSampleInfo {
  double sample_s;
  double v_min;
  double v_max;
  double v_ref;
  std::string v_min_reason;
  std::string v_max_reason;
  std::string v_ref_reason;
};

struct SpeedSegmentInfo {
  double end_segment_control_point_s;
  SpeedSampleInfo quad_point_info;
};

struct VelocityAccelJerkBoundInfo {
  double a_min;
  std::string a_min_reason;
  double a_max;
  std::string a_max_reason;
  double j_min;
  std::string j_min_reason;
  double j_max;
  std::string j_max_reason;
  std::vector<double> a_max_value_pt;
  std::vector<double> a_max_spd_pt;
  std::vector<double> a_min_value_pt;
  std::vector<double> a_min_spd_pt;
  std::vector<double> j_min_value_pt;
  std::vector<double> j_min_spd_pt;
  std::vector<double> j_max_value_pt;
  std::vector<double> j_max_spd_pt;
  bool is_ddmap_intersection;
};

// ACC tuning params
struct ACCParams {
  double safe_v_limit_scale;
  double accel_ceiling_scale;
  double cruise_scale;
  double deadband_region_ratio;
  double deadband_limit_scale;
  double accel_ceiling_cost_scale;
  double acc_decel_region_scale;
  double acc_accel_region_scale;
  double acc_v_max_scale;
};

struct SpeedTuningParams {
  int32_t max_num_iterations;
  double accel_limit_scale;
  double accel_jerk_ratio;
  double accel_scale;
  double jerk_limit_scale;
  double jerk_scale;
  double stop_point_scale;
  double model_ref_scale;
  double model_safety_scale;
  double gmp_ref_scale;

  // ACC relevant params
  double safe_v_limit_scale;
  double accel_ceiling_scale;
  double cruise_scale;
  double deadband_region_ratio;
  double deadband_limit_scale;
  double accel_ceiling_cost_scale;
  double acc_decel_region_scale;
  double acc_accel_region_scale;
  double acc_v_max_scale; // same as cruise speed scale

  const ACCParams get_acc_params() const {
    const ACCParams params = {
        .safe_v_limit_scale = safe_v_limit_scale,
        .accel_ceiling_scale = accel_ceiling_scale,
        .cruise_scale = cruise_scale,
        .deadband_region_ratio = deadband_region_ratio,
        .deadband_limit_scale = deadband_limit_scale,
        .accel_ceiling_cost_scale = accel_ceiling_cost_scale,
        .acc_decel_region_scale = acc_decel_region_scale,
        .acc_accel_region_scale = acc_accel_region_scale,
        .acc_v_max_scale = acc_v_max_scale,
    };
    return params;
  }
};

struct ObstacleAxInfo {
  std::map<int, std::vector<double>> speed_memory{};
  std::map<int, bool> obstacle_alive{};
  std::map<int, double> ax_filted{};
};

struct SpeedLimitSet {
  double a_max;
  double a_min;
  double j_max;
  double j_min;
  double v_max_smooth;
  double v_min_smooth;
  double v_max{120 / 3.6};
  double v_min;
};

struct SpeedLimitInfo {
  std::array<std::array<SpeedLimitSet, path_planner::QUADRATURE_ORDER>,
             NUM_SPEED_SEGMENTS>
      cruise_speed_set;
  std::array<std::array<SpeedLimitSet, path_planner::QUADRATURE_ORDER>,
             NUM_SPEED_SEGMENTS>
      curve_speed_set;
};

struct ObstacleExtraInfo {
  enum ObstacleType { IS_CIPV = 0, IN_ORIGIN_LANE = 1 };
  ObstacleType type;
  bool is_alive;
  int counter;
  double duration_s;
  double init_headway;
};

struct LonDecisionInfo {
  std::unordered_map<int, std::vector<std::string>> lon_decision_info_map;
};

struct CIPVInfo {
  int cipv_id{-1};
  double vel{100.0};
  double ds_offseted{100.0};
  double dv_curve{0.0};
  double dv_safe{100.0};
  double ego_dv{0.0};

  double cipv_time{0.0};
  bool is_need_pre_braking{false};
  bool is_merge_flag{false};
  bool need_smooth_brake{false};
  double cipv_average_velocity{100.0};
  std::vector<double> cipv_velocity_vector{};
  bool is_stable{false};
  bool is_need_accurate_control{false};
  bool is_need_soft_control{false};
  double v_frenet{100.0};
  double rel_s{100.0};
  double a{0.0};
  double set_dist{0.0};
  double dv{0.0};
  double overlap{0.0};
  int type{path_planner::ObsInfo::COUPE};
  double v_vector{100.0};
  bool is_barrier{false};
  double ttc{100.0};
  bool is_road_boundary{false};
};

struct ACCLCAssistInfo {
  bool is_ACC_lane_change;
  int ACC_lane_change_cnt;
  bool left_acc;
  bool left_dec;
  bool right_acc;
  bool right_dec;
  int adjust_cnt;
  int lc_cipv_id;
  int turn_signal;
};

struct ModelPoint {
  double t;
  double v;
  double a;
  double s;
  double x;
  double y;
  double heading_angle;
};

struct OverlapInfo {
  int id;
  double time;
  double s;
  double v;
};

struct ConeBucketInfo {
  bool has_cone_bucket{false};
  bool has_trigger_force_stop{false};
  int warning_level{0};
  int last_warning_level{0};
  int ttc_beyond_thr_cnt{0};
  int non_cipv_cnt{0};
  int brake_cnt{0};
  double ttc{100.0};
  double ttc_thr{3.0};
};

struct LidarRoadEdgeInfo {
  int road_edge_id{-1};
  bool has_road_edge{false};
  bool stop_for_road_edge{false};
  double stop_offset{1.0};
};

struct AccTakeoverInfo {
  bool enable_acc_takeover;
  bool need_acc_takeover;
  bool init_flag{false};
  bool last_running{false};
  bool start_flag{false};
  bool stop_flag{false};
};

struct StateMachineParam {
  double real_run_time_s{0.0};
  double real_cooldown_time_s{0.0};
  int real_state{0};
  int desired_state{0};
};

struct CIPVLostInfo {
  bool cipv_fn{false};
  bool prohibit_acc{false};
  bool has_warned{false};
  int counter{0};
  int warning_level{0};
  double speed_limit{0.0};
  double start_time{-1.0};
  double end_time{-1.0};
  double duration{0.0};
  int pre_cipv_id{-1};
  int pre_cipv_lost_id{-1};
  int cipv_fn_tid{-1};
  double pre_cipv_rel_s{200.0};
  double pre_cipv_ttc{100.0};
  std::vector<int> history_cipv_ids;
};

struct SpeedPlannerInput {
  CIPVInfo cipv_info;
  ACCLCAssistInfo ACC_lc_assist_info;
  bool is_replan;
  bool is_lane_change;
  bool use_eftp;
  bool enable_model_ref;
  bool b_dagger_longitudinal;
  bool use_prediction;
  bool acc_overtake_assist_switch;
  bool is_ACC_lane_change;
  int ACC_lane_change_cnt;
  bool throttle_override;
  bool enable_model_traj_modify;
  double model_traj_modify_time_s;
  int set_speed;
  int set_hw_level;
  bool is_ACC_mode;
  double kick_start_time;
  double v_curv;
  double a_curv;
  double planning_init_v;
  double max_curv_s;
  double max_curv;
  std::vector<ModelPoint> model_trajectory;
  path_planner::PathPlannerPoint ego_state;
  path_planner::PathPlannerPoint planning_init_state;
  VelocityAccelJerkBoundInfo vaj_bound_info;
  std::array<std::array<SpeedSegmentInfo, path_planner::QUADRATURE_ORDER>,
             NUM_SPEED_SEGMENTS>
      speed_segments;
  std::array<double, TOTAL_NUM_PARAMS> last_opt_params;
  std::array<double, NUM_SPEED_CONTROL_POINTS> s_at_control_points;
  SpeedTuningParams speed_tuning_params;
  StopPointInfo stop_point_info;
  std::vector<RadarPoint> radar_points;
  std::vector<path_planner::ObsInfo> lon_obs;
  path_planner::VehicleParam vehicle_param;
  ObstacleAxInfo obstacle_ax_info;
  SpeedLimitInfo speed_limit_info;
  std::unordered_map<int, ObstacleExtraInfo> obstacle_extra_info_map;
  std::vector<std::map<int, double>> CIPV_map_mem;
  std::unordered_map<int, OverlapInfo> overlap_info_map;
  bool dbw_status;
  bool gmp_valid;
  ConeBucketInfo cone_bucket_info;
  LidarRoadEdgeInfo lidar_road_edge_info;
  AccTakeoverInfo acc_takeover_info;
  StateMachineParam state_machine_param;
  CIPVLostInfo cipv_lost_info;
};

using SpeedPlannerOutput = path_planner::PathPlannerOutput;

// debug info
struct SpeedPlannerDebug {
  std::array<double, TOTAL_NUM_RESIDUALS> residual_debug;
  std::array<std::array<double, TOTAL_NUM_RESIDUALS>, TOTAL_NUM_PARAMS>
      per_residual_per_param_gradient;
  std::array<double, TOTAL_NUM_PARAMS> total_gradients_per_param;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RadarPoint, id, s, l, x, y, absolute_vel,
                                   relative_vel)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    StopPointInfo, has_stop_point, has_stop_point_last, should_start,
    should_start_last, standstill, standstill_last, stop_s, is_model_stop,
    enable_modify_stop_point, hold_cnt, stop_mode, last_stop_mode, stop_timer,
    real_stop_timer, hold_flag, auto_go, go_indicator, driver_go, ready_go,
    low_speed_state, last_CIPV_id, stop_reason, stop_id, from_closest_obs,
    hold2kick, stop_in_code, stop_out_code, auto_hold, force_stop, stop_signal,
    last_CIPV_lost, driver_go_timer, last_force_stop, stop_target_accel,
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
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedLimitInfo, cruise_speed_set,
                                   curve_speed_set)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleExtraInfo, type, is_alive, counter,
                                   duration_s, init_headway)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CIPVInfo, cipv_id, vel, ds_offseted,
                                   dv_curve, dv_safe, ego_dv, cipv_time,
                                   is_need_pre_braking, is_merge_flag,
                                   need_smooth_brake, cipv_average_velocity,
                                   cipv_velocity_vector, is_stable,
                                   is_need_accurate_control,
                                   is_need_soft_control, v_frenet, rel_s,
                                   a, set_dist, dv, overlap, type, v_vector,
                                   is_barrier, ttc, is_road_boundary)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ACCLCAssistInfo, is_ACC_lane_change,
                                   ACC_lane_change_cnt, left_acc, left_dec,
                                   right_acc, right_dec, adjust_cnt, lc_cipv_id, turn_signal)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LonDecisionInfo, lon_decision_info_map)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ModelPoint, t, v, a, s, x, y, heading_angle)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OverlapInfo, id, time, s, v)
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
    SpeedPlannerInput, cipv_info, ACC_lc_assist_info, is_replan, is_lane_change,
    use_eftp, enable_model_ref, b_dagger_longitudinal, use_prediction,
    acc_overtake_assist_switch, is_ACC_lane_change, ACC_lane_change_cnt,
    throttle_override, enable_model_traj_modify, model_traj_modify_time_s,
    set_speed, set_hw_level, is_ACC_mode, kick_start_time, v_curv, a_curv,
    planning_init_v, max_curv_s, max_curv, model_trajectory, ego_state,
    planning_init_state, vaj_bound_info, speed_segments, last_opt_params,
    s_at_control_points, speed_tuning_params, stop_point_info, radar_points,
    lon_obs, vehicle_param, obstacle_ax_info, speed_limit_info,
    obstacle_extra_info_map, CIPV_map_mem, overlap_info_map, dbw_status,
    gmp_valid, cone_bucket_info, lidar_road_edge_info, acc_takeover_info,
    state_machine_param, cipv_lost_info)
// debug info
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedPlannerDebug, residual_debug,
                                   per_residual_per_param_gradient,
                                   total_gradients_per_param)

// lon decision interface info
struct ObstacleDecisionInfo {
  path_planner::ObsInfo::LonDecision lon_decision;
  bool is_follow;
  bool exist;
  bool cut_in_flag;
  int cut_in_count;
  bool crossing;
  bool on_heading;
  bool in_lane;
  bool collision;
  double collision_time;
  double lat_dist_head;
  double obj_vy_dy;
  int follow_time_count;
  int ignore_time_count;
  bool rule_base_cutin;
  bool ftp_cutin;
  bool lat_cutin;
  bool is_merge_flag;
  double overlap;
  int type;
  int lc_follow_count;
  int lc_follow_id{-1};
  bool lc_cipv_in_lane{false};
  double virtual_overlap;
};

struct LonDecisionOutput {
  LonDecisionInfo lon_decision_info;
  std::unordered_map<int, ObstacleDecisionInfo> obstacle_decision_info_map;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleDecisionInfo, lon_decision,
                                   is_follow, exist, cut_in_flag, cut_in_count,
                                   crossing, on_heading, in_lane, collision,
                                   collision_time, lat_dist_head, obj_vy_dy,
                                   follow_time_count, ignore_time_count,
                                   rule_base_cutin, ftp_cutin, lat_cutin, is_merge_flag,
                                   overlap, type, lc_follow_count, lc_follow_id,
                                   lc_cipv_in_lane, virtual_overlap)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LonDecisionOutput, lon_decision_info,
                                   obstacle_decision_info_map)

} // namespace speed_planner
