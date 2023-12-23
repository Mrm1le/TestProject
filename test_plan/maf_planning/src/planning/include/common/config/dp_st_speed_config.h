#ifndef MODULES_PLANNING_OPTIMIZERS_DP_ST_SPEED_CONFIG_H_
#define MODULES_PLANNING_OPTIMIZERS_DP_ST_SPEED_CONFIG_H_

namespace msquare {
namespace dp_st_config {

const double FLAGS_speed_lon_decision_horizon = 100.0;
const double FLAGS_speed_lon_decision_time_horizon = 5.0;
const unsigned int matrix_dimension_s = 800;
const unsigned int matrix_dimension_t = 11;
const unsigned int revise_dim = 100;

const double FLAGS_virtual_stop_wall_length = 0.1;
const double FLAGS_follow_min_obs_lateral_distance = 1.5;
const double FLAGS_max_crusie_speed = 15;

const bool FLAGS_enable_dp_reference_speed = false;
const bool FLAGS_enable_multi_thread_in_dp_st_graph = false;
const bool FLAGS_enable_alwasy_stop_for_pedestrian = false;
const bool FLAGS_enable_revise_prediction_before_dp = true;
const bool FLAGS_enable_zoom_prediction_beore_dp = true;
const bool FLAGS_enable_s_update_mechanism = true;

const double revise_lower_speed = 5.0;
const double revise_start_time = 4.0;
const double zoom_start_time = 4.0;
const double zoom_shrink_ds = 0.2;
const double zoom_shrink_lower_speed = 3.0;

// cost weight
const double speed_weight = 0.0;
const double accel_weight = 10.0;
const double jerk_weight = 10.0;
const double obstacle_weight = 5.0;
const double reference_weight = 0.0;
const double go_down_buffer = 5.0;
const double go_up_buffer = 5.0;

// obstacle cost config
const double default_obstacle_cost = 1e+4;

// speed cost config
const double default_speed_cost = 1.0e+0;
const double exceed_speed_penalty = 1.0e+3;
const double low_speed_penalty = 1.0e+1;
const double exceed_soft_speed_penalty = 0.0;
const double low_soft_speed_penalty = 0.0;
const double reference_speed_penalty = 10.0;
const double keep_clear_low_speed_penalty = 10.0;

// accel cost config
const double accel_penalty = 10.0;
const double decel_penalty = 300.0;

// jerk cost config
const double positive_jerk_coeff = 1.0;
const double negative_jerk_coeff = 300.0;

// other cost config
const double max_acceleration = 2.5;
const double max_deceleration = -4.0;

// buffer
const double safe_time_buffer = 3.0;
const double safe_distance = 15.0;

/**
 * 0: piecewise speed interpolation (not correct)
 * 1: piecewise acceleration interpolation (default)
 * 2: piecewise jerk interpolation (bug)
 * 3: constant speed interpolation
 */
const int dp_interpolation_model = 3;

} // namespace dp_st_config
} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_DP_ST_SPEED_CONFIG_H_ */