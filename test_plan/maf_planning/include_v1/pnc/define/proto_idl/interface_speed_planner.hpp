#ifndef CP_INTERFACE_SPEED_PLANNER_H
#define CP_INTERFACE_SPEED_PLANNER_H

#include <stdint.h>
#include <limits>
#include <string>
#include <array>
#include <vector>
#include "interface_planner_common.hpp"
#include "interface_path_planner.hpp"

namespace cp_speed_planner {

struct RadarPoint {
  int32_t id;
  double s;
  double l;
  double x;
  double y;
  double absolute_vel;
  double relative_vel;
};
struct SlidePoint {
  double s;
  double v;
  double a;
};

struct EquivalentPoint {
  double s;
  double v;
  double a;
  bool is_first_equivalent{true};
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
  int32_t hold_cnt;
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
  int32_t last_cipv_id;
  std::string stop_reason;
  int32_t stop_id;
  bool from_closest_obs;
  bool hold2kick;
  int32_t stop_in_code;
  int32_t stop_out_code;
  bool auto_hold;
  bool force_stop;
  bool stop_signal;
  bool last_cipv_lost;
  int32_t driver_go_timer;
  bool last_force_stop;
  // Stop relevant params
  double stop_target_accel;
  double stop_target_accel_entry_velocity;
  double stop_target_accel_entry_distance;
  double stop_offset = 0.0;
  bool last_stop_for_barrier;
  std::vector<cp_path_planner::PairDoubleDouble> stop_traj;

  // Stop ref info
  std::vector<double> ref_s_list{};
  std::vector<double> ref_v_list{};
  std::vector<double> ref_a_list{};
  SlidePoint slide_point;
  EquivalentPoint equivalent_point;
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
};

struct PairInitVectorDouble {
  int32_t first;
  cp_path_planner::VectorDouble second;
  bool __convert_to_list___ = true;
};

struct ObstacleAxInfo {
  std::vector<PairInitVectorDouble> speed_memory{};
  std::vector<cp_path_planner::PairIntBool> obstacle_alive{};
  std::vector<cp_path_planner::PairIntDouble> ax_filted{};
};

struct ArraySpeedLimitSet {
  std::array<SpeedLimitSet, COMMON_QUADRATURE_ORDER> arr;
  bool __convert_to_list___ = true;
};

struct SpeedLimitInfo {
  std::array<ArraySpeedLimitSet, COMMON_NUM_SPEED_SEGMENTS> cruise_speed_set;
  std::array<ArraySpeedLimitSet, COMMON_NUM_SPEED_SEGMENTS> curve_speed_set;
};

struct ObstacleExtraInfo {
  enum ObstacleType { IS_CIPV = 0, IN_ORIGIN_LANE = 1 };
  uint8_t type;
  bool is_alive;
  int32_t counter;
  double duration_s;
  double init_headway;
};

// lon decision interface info
struct ObstacleDecisionInfo {
  uint8_t lon_decision;
  bool is_follow;
  bool exist;
  bool cut_in_flag;
  int32_t cut_in_count;
  bool crossing;
  bool on_heading;
  bool in_lane;
  bool collision;
  double collision_time;
  double lat_dist_head;
  double obj_vy_dy;
  int32_t follow_time_count;
  int32_t ignore_time_count;
  bool rule_base_cutin;
  bool ftp_cutin;
  bool lat_cutin;
  bool is_merge_flag;
  double overlap;
  int32_t type;
  int32_t lc_follow_count;
  int32_t lc_follow_id{-1};
  bool lc_cipv_in_lane{false};
  double virtual_overlap;
};

struct CIPVInfo {
  int32_t cipv_id{-1};
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
  int32_t type = cp_path_planner::ObsInfo::COUPE;
  double v_vector{100.0};
  bool is_barrier{false};
  double ttc{100.0};
  bool is_road_boundary{false};
};

struct RearCarInfo
{
  int32_t nearest_rear_car_id{-1};
  bool has_in_lane_rear_car{false};
  double nearest_rear_car_v{0.};
  double nearest_rear_car_dist{-999};
};

struct RCCInfo
{
  enum RCC_StateMachine {DISABLE = 0, WAITING = 1, ENABLE = 2};
  uint8_t state_machine = 0; // 0 disable, 1 waiting, 2 enable
  bool is_rcc_in_cd = false;
  int32_t cd_cnt = 50;
  int32_t reset_cd_cnt = 50;
  int32_t rcc_waiting_cnt = 0;
  int32_t rcc_duration = 0;
  double rcc_acc = 0.;
  int32_t last_rear_car_id = -1;
  bool is_same_rear_car = 0;
  double rcc_max_v_gain = 1.15;
  std::vector<double> ref_s_list{};
  std::vector<double> ref_v_list{};
};

struct ACCLCAssistInfo {
  bool is_acc_lane_change;
  int32_t acc_lane_change_cnt;
  bool left_acc;
  bool left_dec;
  bool right_acc;
  bool right_dec;
  int32_t adjust_cnt;
  int32_t lc_cipv_id;
  int32_t turn_signal;
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
  int32_t id;
  double time;
  double s;
  double v;
};

struct ConeBucketInfo {
  bool has_cone_bucket{false};
  bool has_trigger_force_stop{false};
  int32_t warning_level{0};
  int32_t last_warning_level{0};
  int32_t ttc_beyond_thr_cnt{0};
  int32_t non_cipv_cnt{0};
  int32_t brake_cnt{0};
  double ttc{100.0};
  double ttc_thr{3.0};
};

struct LidarRoadEdgeInfo {
  int32_t road_edge_id{-1};
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
  int32_t real_state{0};
  int32_t desired_state{0};
};

struct CIPVLostInfo {
  bool cipv_fn{false};
  bool prohibit_acc{false};
  bool has_warned{false};
  int32_t counter{0};
  int32_t warning_level{0};
  double speed_limit{0.0};
  double start_time{-1.0};
  double end_time{-1.0};
  double duration{0.0};
  int32_t pre_cipv_id{-1};
  int32_t pre_cipv_lost_id{-1};
  int32_t cipv_fn_tid{-1};
  double pre_cipv_rel_s{200.0};
  double pre_cipv_ttc{100.0};
  std::vector<int32_t> history_cipv_ids;
};

struct ArraySpeedSegmentInfo {
  std::array<SpeedSegmentInfo, COMMON_QUADRATURE_ORDER> arr;
  bool __convert_to_list___ = true;
};

struct PairIntObstacleExtraInfo {
  int32_t first;
  ObstacleExtraInfo second;
  bool __convert_to_list___ = true;
};

struct PairIntOverlapInfo {
  int32_t first;
  OverlapInfo second;
  bool __convert_to_list___ = true;
};

struct VectorPairIntDouble {
  std::vector<cp_path_planner::PairIntDouble> vec;
  bool __convert_to_list___ = true;
};

struct SpeedPlannerInput {
  CIPVInfo cipv_info;
  RearCarInfo rear_car_info;
  RCCInfo rcc_info;
  ACCLCAssistInfo acc_lc_assist_info;
  bool is_mrc_inlane_brake{false};
  bool is_replan;
  bool is_lane_change;
  bool use_eftp;
  bool enable_model_ref;
  bool b_dagger_longitudinal;
  bool use_prediction;
  bool acc_overtake_assist_switch;
  bool is_acc_lane_change;
  int32_t acc_lane_change_cnt;
  bool throttle_override;
  bool enable_model_traj_modify;
  double model_traj_modify_time_s;
  int32_t set_speed;
  int32_t set_hw_level;
  bool is_acc_mode;
  double kick_start_time;
  double v_curv;
  double a_curv;
  double max_curv_s;
  double max_curv;
  double planning_init_v;
  std::vector<ModelPoint> model_trajectory;
  cp_path_planner::PathPlannerPoint ego_state;
  cp_path_planner::PathPlannerPoint planning_init_state;
  VelocityAccelJerkBoundInfo vaj_bound_info;
  std::array<ArraySpeedSegmentInfo, COMMON_NUM_SPEED_SEGMENTS>
      speed_segments;
  std::array<double, COMMON_TOTAL_NUM_PARAMS> last_opt_params;
  std::array<double, COMMON_NUM_SPEED_CONTROL_POINTS> s_at_control_points;
  SpeedTuningParams speed_tuning_params;
  StopPointInfo stop_point_info;
  std::vector<RadarPoint> radar_points;
  std::vector<cp_path_planner::ObsInfo> lon_obs;
  cp_path_planner::VehicleParam vehicle_param;
  ObstacleAxInfo obstacle_ax_info;
  SpeedLimitInfo speed_limit_info;
  std::vector<PairIntObstacleExtraInfo> obstacle_extra_info_map;
  std::vector<VectorPairIntDouble>  cipv_map_mem;
  std::vector<PairIntOverlapInfo> overlap_info_map;
  bool dbw_status;
  bool gmp_valid;
  ConeBucketInfo cone_bucket_info;
  LidarRoadEdgeInfo lidar_road_edge_info;
  AccTakeoverInfo acc_takeover_info;
  StateMachineParam state_machine_param;
  bool enable_env_speed_limit{false};
  CIPVLostInfo cipv_lost_info;
};

struct IdDecisionLog {
  int32_t first;
  std::vector<std::string> second;
  bool __convert_to_list___ = true;
};

struct LonDecisionInfo {
  std::vector<IdDecisionLog> lon_decision_info_map;
};

struct IntObstacleDecisionInfo {
  int32_t first;
  ObstacleDecisionInfo second;
  bool __convert_to_list___ = true;
};

struct LonDecisionOutput {
  LonDecisionInfo lon_decision_info;
  std::vector<IntObstacleDecisionInfo> obstacle_decision_info_map;
};

}  // namespace speed_planner

#endif