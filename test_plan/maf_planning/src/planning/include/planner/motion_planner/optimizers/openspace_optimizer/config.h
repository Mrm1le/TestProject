#pragma once

#include "common/math/math_utils.h"
#include "nlohmann/json.hpp"
#include <mutex>
#include <string>
#include <yaml-cpp/yaml.h>

namespace msquare {
/**
 * @brief singleton class which contains configurations of hybridastar
 * example: float xy_grid_resolution =
 *            HybridAstarConfig::GetInstance()->x_bound;
 */
class HybridAstarConfig {
public:
  ~HybridAstarConfig();

  static HybridAstarConfig *GetInstance();

  bool loadFile(const std::string file_name);

  // TODO: replace with get() and set() functions
  YAML::Node yaml_node_;

private:
  HybridAstarConfig();
  static HybridAstarConfig *instance_;

public:
  // double x_bound;
  // double y_bound;
  bool use_t_line_;
  double inflation_for_points_ = 0.0;
  int max_zigzag_allowd;

  double xy_grid_resolution = 0.0;
  double phi_grid_resolution = 0.0;
  double grid_dijkstra_xy_resolution = 0.0;
  double pixel_resolution;

  // penalty
  double traj_forward_penalty = 0.0;
  double traj_back_penalty = 0.0;
  double traj_gear_switch_penalty = 0.0;
  double traj_steer_penalty = 0.0;
  double traj_steer_change_penalty = 0.0;
  double traj_steer_change_penalty_gear_switch = 0.05;
  double traj_sides_diff_penalty;
  double traj_obstacle_distance_1_penalty = 5.0;
  double traj_obstacle_distance_2_penalty = 10.0;
  double traj_obstacle_distance_3_penalty = 20.0;
  double traj_boundary_cost_penalty = 10.0;
  double traj_wrong_start_direction_penalty = 100.0;
  double traj_end_offset_penalty = 20.0;
  double traj_s_turn_penalty = 5.0;

  // state update
  double step_size = 0.0;
  int next_node_num;
  int step_direction = 0;

  double delta_t;

  // heuristic
  double holonomic_with_obs_heuristic = 0.0;
  double non_holonomic_without_obs_heuristic = 0.0;

  // strategy
  bool enable_delta_cost;
  bool enable_analytic_expansion;
  double max_analytic_expansion_length;
  double analytic_expansion_end_size_threshold;
  int force_analytic_expansion_end_direction = 0;

  // visualize
  int verbose;
  int display_points;

  // algorithm
  int planning_core = 0;

  int footprint_model_;
  int footprint_model_precise_;

  unsigned max_iter = 0;
  unsigned max_iter_base = 0;
  unsigned max_iter_max = 0;

  bool is_loaded;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
      HybridAstarConfig, use_t_line_, inflation_for_points_, max_zigzag_allowd,
      xy_grid_resolution, phi_grid_resolution, grid_dijkstra_xy_resolution,
      pixel_resolution, traj_forward_penalty, traj_back_penalty,
      traj_gear_switch_penalty, traj_steer_penalty, traj_steer_change_penalty,
      traj_sides_diff_penalty, step_size, next_node_num, step_direction,
      delta_t, holonomic_with_obs_heuristic,
      non_holonomic_without_obs_heuristic, enable_delta_cost,
      enable_analytic_expansion, max_analytic_expansion_length,
      analytic_expansion_end_size_threshold,
      force_analytic_expansion_end_direction, verbose, display_points,
      planning_core, footprint_model_, footprint_model_precise_, max_iter,
      max_iter_base, max_iter_max, traj_steer_change_penalty_gear_switch,
      traj_obstacle_distance_1_penalty, traj_obstacle_distance_2_penalty,
      traj_obstacle_distance_3_penalty, traj_boundary_cost_penalty,
      traj_wrong_start_direction_penalty, traj_end_offset_penalty,
      traj_s_turn_penalty);
};

// HybridAstarConfig *HybridAstarConfig::instance = nullptr;

/**
 * @brief singleton class which contains configuration of trajectory
 * optimizer
 * example: float wheel_base =
 *    TrajectoryOptimizerConfig::GetInstance()->param_cost_theta;
 */
class TrajectoryOptimizerConfig {
public:
  ~TrajectoryOptimizerConfig();

  static TrajectoryOptimizerConfig *GetInstance();

  bool loadFile(const std::string file_name);

  // TODO: replace with get() and set() functions
  YAML::Node yaml_node_;

private:
  TrajectoryOptimizerConfig();
  static TrajectoryOptimizerConfig *instance_;

public:
  // OBCA params
  bool OBCA_running;
  bool param_enable_check_parallel_trajectory;
  bool param_FLAGS_use_iterative_anchoring_smoother;
  bool param_FLAGS_use_dual_variable_warm_start;
  bool param_FLAGS_enable_smoother_failsafe;

  double param_is_near_destination_threshold;
  double param_delta_t;

  double param_front_edge_to_rear;
  double param_back_edge_to_rear;
  double param_left_edge_to_rear;
  double param_right_edge_to_rear;
  double param_wheel_base;
  double param_offset;

  double param_cost_end_state;          // 5
  double param_cost_xy;                 // 1
  double param_cost_theta;              // 1
  double param_cost_speed;              // 1
  double param_cost_steer;              // 1
  double param_cost_acc;                // 1
  double param_cost_steerrate;          // 1
  double param_cost_jerk;               // 2
  double param_cost_stitching_steer;    // 3
  double param_cost_first_order_time;   // 1
  double param_cost_second_order_time;  // 1
  double param_cost_stitching_a;        // 3
  double param_min_safe_dist;           // 0.05
  double param_max_steer_angle;         // 0.6
  double param_max_speed_forward = 5;   // 5
  double param_max_speed_reverse = 5;   // 5
  double param_max_acc_forward;         // 1.5
  double param_max_acc_reverse;         // 1.5
  double param_min_time_sample_scaling; // 0.5
  double param_max_time_sample_scaling; // 0.5
  double param_max_steer_rate;          // 2
  double param_warm_start_weight;
  bool param_if_use_fix_time;  // true
  bool param_constraint_check; // false
  bool param_jacobian_ad;      // false

  int param_info_level;                  // 0
  int param_max_itr;                     // 30
  double param_lin_torl;                 // 1
  int param_mumps_mem;                   // 10000
  double param_mu_init;                  // 1
  double param_conv_tol;                 // 1e6
  double param_constraints_tol;          // 3
  double param_accet_objchange_tol;      // 5
  double param_acceptable_compl_inf_tol; // 1e6
  double param_dual_inf_tol;             // 1e7
  double param_acceptable_tol;           // 1e7
  int param_acceptable_iter;             // 3
  int param_warmstart_acceptable_iter;   // 5
  std::string param_linear_solver;       // "ma86"

  bool is_loaded;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
      TrajectoryOptimizerConfig, OBCA_running,
      param_enable_check_parallel_trajectory,
      param_FLAGS_use_iterative_anchoring_smoother,
      param_FLAGS_use_dual_variable_warm_start,
      param_FLAGS_enable_smoother_failsafe, param_is_near_destination_threshold,
      param_delta_t, param_front_edge_to_rear, param_back_edge_to_rear,
      param_left_edge_to_rear, param_right_edge_to_rear, param_wheel_base,
      param_offset, param_cost_end_state, param_cost_xy, param_cost_theta,
      param_cost_speed, param_cost_steer, param_cost_acc, param_cost_steerrate,
      param_cost_jerk, param_cost_stitching_steer, param_cost_first_order_time,
      param_cost_second_order_time, param_cost_stitching_a, param_min_safe_dist,
      param_max_steer_angle, param_max_speed_forward, param_max_speed_reverse,
      param_max_acc_forward, param_max_acc_reverse,
      param_min_time_sample_scaling, param_max_time_sample_scaling,
      param_max_steer_rate, param_warm_start_weight, param_if_use_fix_time,
      param_constraint_check, param_jacobian_ad, param_info_level,
      param_max_itr, param_lin_torl, param_mumps_mem, param_mu_init,
      param_conv_tol, param_constraints_tol, param_accet_objchange_tol,
      param_acceptable_compl_inf_tol, param_dual_inf_tol, param_acceptable_tol,
      param_acceptable_iter, param_warmstart_acceptable_iter,
      param_linear_solver);
};

/**
 * @brief common config
 *
 */
struct CarCommonConfig {
  double test_config = 1.0;
  double min_block_len = 0.1; // min length for control to startup
  int debug_num;
  bool use_new_fs_block = false;
  bool use_sop_openspace_planner = false;
  bool use_sop_openspace_planner_parallel = false;
  bool sop_openspace_planner_no_steer_change_gear_switch = false;
  double min_finish_len = 0.1;
  int zigzag_limit = 11;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    CarCommonConfig, test_config, min_block_len, debug_num,
    use_sop_openspace_planner, use_sop_openspace_planner_parallel,
    min_finish_len, use_new_fs_block,
    sop_openspace_planner_no_steer_change_gear_switch, zigzag_limit)

/**
 * @brief config about car body, such as cutting angle
 *
 */
struct CarOnlyConfig {
  double min_radius = 5.0;
  double back_light_len = 0.27;
  double back_light_height = 0.32;
  double body_tire_offset = 0.03;
  double front_edge_to_mirror = 1.9;
  double mirror_length = 0.23;
  double body_tire_offset_space_slot = 0.03;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CarOnlyConfig, min_radius, back_light_len,
                                   back_light_height, body_tire_offset,
                                   front_edge_to_mirror, mirror_length,
                                   body_tire_offset_space_slot)

struct LongitudianlConfig {
  bool keep_people_dynamic = false;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LongitudianlConfig, keep_people_dynamic)

struct APOAConfig {
  std::string parkin_data_folder = "/home/ros/Downloads/";
  bool support_perpendicular = true;
  bool use_last_parkin_data = true;
  bool is_direct_limit = false;
  bool is_direct_dynamic_person_ignore = false;
  bool use_legacy_parkout = false;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(APOAConfig, parkin_data_folder,
                                   support_perpendicular, use_last_parkin_data,
                                   is_direct_limit,
                                   is_direct_dynamic_person_ignore,
                                   use_legacy_parkout)

struct ExpandParamConfig {
  std::string vehicle_type = "DEVCAR";
  double apa_lon_adjust = 0.0;
  double apa_lat_adjust = 0.0;
  double apoa_lon_adjust = 0.0;
  double apoa_lat_adjust = 0.0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ExpandParamConfig, vehicle_type,
                                   apa_lon_adjust, apa_lat_adjust,
                                   apoa_lon_adjust, apoa_lat_adjust)

struct KinoDynamicConfig {
  double vel_forward = 0.5;
  double vel_backward = 0.4;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(KinoDynamicConfig, vel_forward, vel_backward)

struct KinoDynamicSceneConfig {
  KinoDynamicConfig apa_parallel;
  KinoDynamicConfig apa_vertical;
  KinoDynamicConfig apa_oblique;

  KinoDynamicConfig apoa_parallel;
  KinoDynamicConfig apoa_vertical;
  KinoDynamicConfig apoa_oblique;

  KinoDynamicConfig rpa;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(KinoDynamicSceneConfig, apa_parallel,
                                   apa_vertical, apa_oblique, apoa_parallel,
                                   apoa_vertical, apoa_oblique, rpa)

struct LonConfig {
  bool keep_people_dynamic = false;
  double wheel_approaching_dis = 0.6;
  double max_v_approaching_wheel_stop = 0.3;
  double obstacle_consider_max = 0.35;
  double obstacle_consider_min = 0.2;
  double obstacle_consider_min_v = 0.4;
  double obstacle_consider_extend_length = 1.5;
  double window_s_size = 1.5;
  double acc = 1.0;
  double dec = 0.3;
  double limit_dec = 0.4;
  double rate = 10.0;
  double forbid_acc_max_s = 3.0;
  double min_duration_filter_time = 1.0;
  double curvature_change_threshold = 0.3;
  double curvature_limit_v = 0.2;

  double traj_end_speed_limit_s = 0.1;
  double control_take_over_remain_s = 0.8;
  double control_take_over_speed = 0.3;
  double control_take_over_acc = 0.2;
  double min_velocity = 0.2;
  double planning_deceleration = 0.3056;
  double planning_deceleration_last = 0.2; // dec at last segment
  double consider_obstacle_max_s = 1.5;
  double max_dis_for_adjust_target = 1.1;

  //  speed optimizer
  double s_w = 1.0;
  double v_w = 1.0;
  double j_lo = -1.0;
  double j_up = 1.0;
  double a_w = 1.0;
  double j_w = 1.0;
  double s_ref_w = 1.0;
  double v_ref_w = 1.0;

  // sop algorithm option
  bool use_sop_algorithm = false;
  // speed planner option
  bool use_margin_speed = false;
  bool use_osqp_speed = false;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    LonConfig, keep_people_dynamic, wheel_approaching_dis,
    max_v_approaching_wheel_stop, obstacle_consider_max, obstacle_consider_min,
    obstacle_consider_min_v, obstacle_consider_extend_length, window_s_size,
    curvature_change_threshold, curvature_limit_v, acc, dec, limit_dec, rate,
    forbid_acc_max_s, min_duration_filter_time, traj_end_speed_limit_s,
    control_take_over_remain_s, control_take_over_speed, control_take_over_acc,
    min_velocity, planning_deceleration, planning_deceleration_last,
    consider_obstacle_max_s, max_dis_for_adjust_target, s_w, v_w, j_lo, j_up,
    a_w, j_w, s_ref_w, v_ref_w, use_sop_algorithm, use_margin_speed,
    use_osqp_speed)

struct ParallelPlannerConfig {
  double check_empty_length = 3.5;
  bool is_min_r_priority = false;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ParallelPlannerConfig, check_empty_length,
                                   is_min_r_priority)

struct EndingCheckConfig {
  double lon_ending_thres = 0.7;
  double lat_ending_thres = 0.14;
  double angle_ending_thres = 0.03;
  double angle_ending_thres_after_adjust = 0.06;
  double lon_ending_thres_parallel = 0.7;
  double lat_ending_thres_parallel = 0.8;
  double angle_ending_thres_parallel = 0.03;
  double max_center_offset_parallel = 0.4;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EndingCheckConfig, lon_ending_thres,
                                   lat_ending_thres, angle_ending_thres,
                                   angle_ending_thres_after_adjust,
                                   lon_ending_thres_parallel,
                                   lat_ending_thres_parallel,
                                   angle_ending_thres_parallel,
                                   max_center_offset_parallel)

struct ParkinDeciderConfig {
  double curve_join_block_duration = 1.0;
  double mpc_traj_block_duration = 0.5;
  double parallel_in_channel_angle = 1.309;
  double planning_run_duration = 8.0;
  double narrow_channel_width = 5.0;
  double smaller_lon_inflation_narrow_perpendicular_slot = 0.25;
  double deviated_max_dist_allowd = 0.15;
  double has_moved_min_velocity = 0.07;
  double dist_to_stop_curvejoin = 3.0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ParkinDeciderConfig, curve_join_block_duration, mpc_traj_block_duration,
    parallel_in_channel_angle, planning_run_duration, narrow_channel_width,
    smaller_lon_inflation_narrow_perpendicular_slot, deviated_max_dist_allowd,
    has_moved_min_velocity, dist_to_stop_curvejoin)

struct SlotConfig {
  double outside_Tline_default_height = -0.5;
  double default_max_rule_slot_width = 3.1;
  double extra_max_rule_slot_width_parallel = 3.0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SlotConfig, outside_Tline_default_height,
                                   default_max_rule_slot_width,
                                   extra_max_rule_slot_width_parallel)

struct RpaConfig {
  double rpa_straight_distance = 5.0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RpaConfig, rpa_straight_distance)

struct SlotTypeDistanceConfig {
  double perpendicular = 0.3;
  double oblique = 0.3;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SlotTypeDistanceConfig, perpendicular,
                                   oblique)

struct ObsTypeDistanceConfig {
  SlotTypeDistanceConfig step;
  SlotTypeDistanceConfig wall;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsTypeDistanceConfig, step, wall)

struct SlotOriginDistanceConfig {
  ObsTypeDistanceConfig line;
  ObsTypeDistanceConfig space;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SlotOriginDistanceConfig, line, space)

struct TargetPoseConfig {
  SlotOriginDistanceConfig bottom_obs_distance_config;
  double oneside_dist_mirror_to_pillar = 0.3;
  double oneside_dist_mirror_to_obstacle_car = 0.275;
  double oneside_dist_wheel_to_step = 0.3;
  double twosides_narrowest_width_advanced_abandon = 2.52;
  double oneside_narrowest_width_advanced_abandon = 2.38;
  double rear_axis_to_stopper = 0.30;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TargetPoseConfig, bottom_obs_distance_config,
                                   oneside_dist_mirror_to_pillar,
                                   oneside_dist_mirror_to_obstacle_car,
                                   oneside_dist_wheel_to_step,
                                   twosides_narrowest_width_advanced_abandon,
                                   oneside_narrowest_width_advanced_abandon,
                                   rear_axis_to_stopper)
struct SVSpeedConfig {
  bool use_sv_speed_generator = false;
  double out_slot_coeff = 1.0;
  double large_curv_coeff = 0.625;
  double in_slot_coeff = 0.5;
  double min_radius_coeff = 0.9;
  bool no_acc_limit = false;
  double not_use_comfortable_min_s = 0.3;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SVSpeedConfig, use_sv_speed_generator,
                                   out_slot_coeff, large_curv_coeff,
                                   in_slot_coeff, min_radius_coeff,
                                   no_acc_limit, not_use_comfortable_min_s)

struct CarConfig {
  SVSpeedConfig sv_config;
  CarCommonConfig common_config;
  CarOnlyConfig car_only_config;
  KinoDynamicSceneConfig kino_config;
  LonConfig lon_config;
  APOAConfig apoa_config;
  ExpandParamConfig expand_param_config;
  ParallelPlannerConfig parallel_config;
  TargetPoseConfig target_pose_config;
  EndingCheckConfig ending_check_config;
  ParkinDeciderConfig parkin_decider_config;
  SlotConfig slot_config;
  RpaConfig rpa_config;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CarConfig, sv_config, common_config,
                                   car_only_config, kino_config, lon_config,
                                   apoa_config, expand_param_config,
                                   parallel_config, target_pose_config,
                                   ending_check_config, parkin_decider_config,
                                   slot_config, rpa_config)

/**
 * @brief singleton class which contains configuration of car
 * example: float wheel_base =
 *    CarParams::GetInstance()-> wheel_base;
 */
class CarParams {
public:
  ~CarParams();

  static CarParams *GetInstance();

  bool loadFile(const std::string file_name);
  bool loadFile4Plan(const std::string file_name);
  bool loadFile4Car(const std::string config_file_dir);

  bool setLonInflation(double inflation);
  void setLonInflationMin(double inflation);
  const double lon_inflation() const { return lon_inflation_; }
  const double get_max_steer_angle() const { return max_steer_angle; }
  bool setLatInflation(double inflation);
  const double lat_inflation() const { return lat_inflation_; }
  bool shrinkLatInflation(double inflation) {
    return setLatInflation(std::min(inflation, lat_inflation_));
  }
  void setLatInflationForce(double inflation, double deta);
  bool shrinkLonInflation(double inflation) {
    return setLonInflation(std::min(inflation, lon_inflation_));
  }
  void setLonInflationForce(double inflation, double deta);
  const bool isInflationAdjusted() const { return is_inflation_param_adjusted; }
  void resetInflationAdjustFlag();
  void setMaxSteer(double max_steer);
  void setMaxSteerRate(double max_steer_rate);
  void setMaxSteerRear(double max_steer_rear);
  void setMaxSteerRateRear(double max_steer_rate_rear);

  // TODO: replace with get() and set() functions
  YAML::Node yaml_node_;

  std::string type;

  double vehicle_length_real;
  double vehicle_width_real;
  double vehicle_width_wo_rearview_mirror;
  double vehicle_length;
  double vehicle_width;
  double vehicle_height;
  double max_acceleration;
  double min_acceleration;
  double max_steer_angle;
  double max_steer_angle_rate;
  double max_steer_angle_rear;
  double max_steer_angle_rate_rear;
  double steer_ratio;
  double wheel_base;
  double wheel_rolling_radius;
  double max_abs_speed_when_stopped;
  double brake_deadzone;
  double throttle_deadzone;
  double lon_inflation_min;
  double lon_inflation_max;
  double lat_inflation_min;
  double lat_inflation_max;
  double inflation_rearview_mirror;
  double shrink_ratio_for_lines_;
  double shrink_ratio_for_lines_min_;
  bool enable_multiple_steer_modes;

  double lat_inflation_low = 0.05; // inflation params for low-height obstacles

  // derivated params
  double max_delta_angle;
  double max_delta_angle_rate;
  double max_delta_angle_rear;
  double max_delta_angle_rate_rear;
  double min_turn_radius;

  // car params
  CarConfig car_config;

  bool is_loaded;
  bool is_inflation_param_adjusted = false;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
      CarParams, vehicle_length_real, vehicle_width_real,
      vehicle_width_wo_rearview_mirror, vehicle_length, vehicle_width,
      vehicle_height, max_acceleration, min_acceleration, max_steer_angle,
      max_steer_angle_rate, max_steer_angle_rear, max_steer_angle_rate_rear,
      steer_ratio, wheel_base, wheel_rolling_radius, max_abs_speed_when_stopped,
      brake_deadzone, throttle_deadzone, lon_inflation_min, lon_inflation_max,
      lat_inflation_min, lat_inflation_max, inflation_rearview_mirror,
      shrink_ratio_for_lines_, shrink_ratio_for_lines_min_,
      enable_multiple_steer_modes, lat_inflation_low, max_delta_angle,
      max_delta_angle_rate, max_delta_angle_rear, max_delta_angle_rate_rear,
      min_turn_radius, lon_inflation_, lat_inflation_, car_config, type,
      is_inflation_param_adjusted);

private:
  CarParams();
  void updateDerived();
  static CarParams *instance_;
  double lon_inflation_;
  double lat_inflation_;
};

/**
 * @brief singleton class which contains configuration of car
 * example: float wheel_base =
 *    StrategyParams::GetInstance()-> bcheckendsegment;
 */
class StrategyParams {
public:
  ~StrategyParams();

  static StrategyParams *GetInstance();

  bool loadFile(const std::string file_name);

  void setForceTerminate(bool f);
  bool getForceTerminate();

  /**
   * @brief reset to context from yaml
   */
  void reset();

  // TODO: replace with get() and set() functions
  YAML::Node yaml_node_;

  bool bcheckendsegment;
  double default_v_ = 0.0;
  double default_a_ = 0.0;
  double default_dt_ = 0.0;
  double check_endsegment_max_tan_tolerance_;
  double default_check_endsegent_len_ = 0.0;
  bool enable_endsegment_len_correction_ = 0.0;
  double time_out_;

  double low_speed_endsegment_len_;
  bool enable_revserse_search_ = false;
  bool enable_smooth_;

  bool is_loaded;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
      StrategyParams, bcheckendsegment, default_v_, default_a_, default_dt_,
      check_endsegment_max_tan_tolerance_, default_check_endsegent_len_,
      enable_endsegment_len_correction_, time_out_, low_speed_endsegment_len_,
      enable_revserse_search_, enable_smooth_, force_terminate);

private:
  StrategyParams();
  static StrategyParams *instance_;

  bool force_terminate;
  std::mutex mtx_;
};

} // namespace msquare
