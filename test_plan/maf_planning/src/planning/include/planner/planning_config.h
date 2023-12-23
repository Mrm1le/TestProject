#pragma once

const double FLAGS_planning_loop_rate = 10.0;
const double FLAGS_look_forward_extend_distance = 50.0;
const double FLAGS_reference_line_stitch_overlap_distance = 20.0;
const double FLAGS_reference_line_lateral_buffer = 0.5;
const double FLAGS_change_lane_fail_freeze_time = 3.0;
const double FLAGS_change_lane_success_freeze_time = 3.0;
const double FLAGS_change_lane_min_length = 30.0;
const double FLAGS_change_lane_speed_relax_percentage = 0.05;
const bool FLAGS_enable_s_update_mechanism = true;
const double FLAGS_replan_lateral_distance_threshold = 0.5;
const std::vector<double> FLAGS_replan_longitudinal_distance_threshold_speed{
    40 / 3.6, 80 / 3.6};
const std::vector<double> FLAGS_replan_longitudinal_distance_threshold_value{
    1.0, 2.0};
const double FLAGS_min_replan_longitudinal_distance_threshold = 0.5;
const double FLAGS_replan_longitudinal_time_threshold = 0.15;
const double FLAGS_replan_velocity_threshold = 3.0;
const double FLAGS_default_reference_line_width = 3.4;
const double FLAGS_smoothed_reference_line_max_diff = 5.0;
const double FLAGS_trajectory_time_length =
    8.0; // pt graph time,cruise sample time,
const double FLAGS_trajectory_time_length_stop = 10.0; // stop sample time
const double FLAGS_trajectory_path_length = 10.0;
const double FLAGS_planning_upper_speed_limit = 45.0 / 3.6;

const double FLAGS_lon_decision_time_steps = 25;
const double FLAGS_lon_decision_time_horizon = 5.0;
const double FLAGS_trajectory_time_min_interval = 0.02;
const double FLAGS_trajectory_time_max_interval = 0.1;
const double FLAGS_trajectory_time_high_density_period = 1.0;
const double FLAGS_speed_lower_bound = -0.1;
const double FLAGS_speed_upper_bound = 16.7;

const double FLAGS_longitudinal_acceleration_lower_bound =
    -5.0; // bound to generate reference velocity
const double FLAGS_longitudinal_acceleration_upper_bound =
    2.5; // bound to generate reference velocity
const double FLAGS_lateral_acceleration_bound = 10.0;
const double FLAGS_longitudinal_jerk_lower_bound = -10.0;
const double FLAGS_longitudinal_jerk_upper_bound = 10.0;
const double FLAGS_lateral_jerk_bound = 4.0;

const double FLAGS_dl_bound = 0.1;
const double FLAGS_kappa_bound = 0.5;
const double FLAGS_dkappa_bound = 0.02;

const double FLAGS_st_max_s = 100.0;
const double FLAGS_st_max_t = 5.0;

const double FLAGS_static_obstacle_speed_threshold = 2.0;
const double FLAGS_static_decision_nudge_l_buffer = 0.5;
const double FLAGS_max_stop_distance_obstacle = 10.0;
const double FLAGS_min_stop_distance_obstacle = 8.0;
const double FLAGS_nudge_distance_obstacle = 0.5;
const double FLAGS_follow_min_distance = 3.0;
const double FLAGS_yield_distance = 3.0;
const double FLAGS_yield_distance_pedestrian_bycicle = 5.0;
const double FLAGS_follow_time_buffer = 2.5;
const double FLAGS_follow_min_time_sec = 0.1;

const double FLAGS_stop_line_stop_distance = 1.0;
const double FLAGS_max_stop_speed = 0.2;
const double FLAGS_signal_light_min_pass_s_distance = 4.0;
const double FLAGS_destination_check_distance = 5.0;
const double FLAGS_virtual_stop_wall_length = 0.1;
const double FLAGS_virtual_stop_wall_height = 2.0;

const double FLAGS_prediction_total_time = 5.0;
const double FLAGS_turn_signal_distance = 100.0;

const double FLAGS_lattice_epsilon = 1e-6;
const double FLAGS_default_cruise_speed = 5.0;
const double FLAGS_trajectory_time_resolution = 0.1;
const double FLAGS_trajectory_density = 0.025;
const double FLAGS_trajectory_collision_density = 0.05;
const double FLAGS_trajectory_space_resolution = 1.0;
const double FLAGS_decision_horizon = 200.0;
const int FLAGS_num_velocity_sample =
    7; // to sample constant velocity ,this parameter must be odd number
const double FLAGS_min_velocity_sample_gap = 1.0;
const double FLAGS_default_dis_leader = 6.5; // safe distance cost distance
const double FLAGS_default_dis_redlight = 10.0;
const double FLAGS_default_dis_stop = 50.0;
const double FLAGS_min_dis_leader = 5.0;
const double FLAGS_default_tcc = 1.5;
const double FLAGS_lon_collision_buffer = 1.0;
const double FLAGS_lat_collision_buffer = 0.1;
const int FLAGS_num_sample_follow_per_timestamp =
    3; // 10;//3;  //to divide follow obstacle distance
constexpr const double FLAGS_num_cruise_sample = 7.0;

// const double FLAGS_weight_lon_object = 1.0;
// const double FLAGS_weight_lon_jerk = 10.0; //10
// const double FLAGS_weight_lon_collision = 40.0; //larger than
// lon_safedistance const double FLAGS_weight_lon_safedistance = 120.0; //200
// times of lon_object //120 const double FLAGS_weight_centripetal_acceleration
// = 1.5; const double FLAGS_weight_lat_offset = 10.0; const double
// FLAGS_weight_lat_comfort = 0.1;

const double FLAGS_cost_non_priority_reference_line = 5.0;
const double FLAGS_weight_same_side_offset = 1.0;
const double FLAGS_weight_opposite_side_offset = 1.0;

const double FLAGS_weight_target_speed = 10.0;
const double FLAGS_weight_dist_travelled = 1.0;

const double FLAGS_lat_offset_bound = 3.0;
const double FLAGS_lon_collision_yield_buffer =
    5.0; // 14,collision cost distance
const double FLAGS_lon_collision_overtake_buffer = 4.5;
const double FLAGS_lon_collision_cost_std = 0.5;
const double FLAGS_default_lon_buffer =
    5.0; // 6.0;//2.0; //safe distance behind obstacle
const double FLAGS_time_min_density =
    1.0; // 3.0;//2.0; //density get point around obstacle
const double FLAGS_comfort_acceleration_factor = 0.5;
const double FLAGS_comfort_acceleration_factor_acc = 0.35;
const double FLAGS_polynomial_minimal_param = 0.1;
const double FLAGS_polynomial_minimal_param_sub = 0.5;
const double FLAGS_lattice_stop_buffer = 0.02;
const double FLAGS_weight_lateral_offset = 1.0;
const double FLAGS_weight_lateral_derivative = 500.0;
const double FLAGS_weight_lateral_second_order_derivative = 1000.0;
const double FLAGS_weight_lateral_obstacle_distance = 0.0;
const double FLAGS_lateral_third_order_derivative_max = 0.1;
const double FLAGS_max_s_lateral_optimization = 50.0;
const double FLAGS_default_delta_s_lateral_optimization = 2.0;
const double FLAGS_bound_buffer = 0.1;
const double FLAGS_nudge_buffer = 0.3;

const double FLAGS_min_car_width = 1.0;
const double FLAGS_min_car_length = 1.0;
