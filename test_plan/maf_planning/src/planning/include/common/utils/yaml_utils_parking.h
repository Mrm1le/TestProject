#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/parking_planner_types.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <yaml-cpp/yaml.h>

// namespace msquare {
// namespace parking {

namespace YAML {
// template<>
// struct convert<msquare::planning_math:Box2d> {
//   static Node encode(const msquare::planning_math:Box2d& rhs) {
//     Node node;
//     node["center"]["x"] = rhs.center_x();
//     node["center"]["y"] = rhs.center_y();
//     node["heading"] = rhs.heading();
//     node["length"] = rhs.length();
//     node["width"] = rhs.width();
//     return node;
//   }

//   static bool decode(const Node& node, msquare::planning_math:Box2d& rhs) {
//     double center_x = node["center"]["x"].as<double>();
//     double center_y = node["center"]["y"].as<double>();
//     double heading = node["heading"].as<double>();
//     double length = node["length"].as<double>();
//     double width = node["width"].as<double>();
//     rhs = Box2d(Vec2d(center_x, center_y), heading, length, width);
//     return true;
//   }
// };

// template<>
// struct convert<msquare::planning_math:LineSegment2d> {
//   static Node encode(const msquare::planning_math:LineSegment2d& rhs) {
//     Node node;
//     node["center"]["x"] = rhs.center_x();
//     node["center"]["y"] = rhs.center_y();
//     node["heading"] = rhs.heading();
//     node["length"] = rhs.length();
//     node["width"] = rhs.width();
//     return node;
//   }

//   static bool decode(const Node& node, msquare::planning_math:LineSegment2d&
//   rhs) {
//     rhs.center_x() = node["center"]["x"].as<double>();
//     rhs.center_y() = node["center"]["y"].as<double>();
//     rhs.heading() = node["heading"].as<double>();
//     rhs.length() = node["length"].as<double>();
//     rhs.width() = node["width"].as<double>();
//     return true;
//   }
// };

// template<>
// struct convert<msquare::planning_math:Vec2d> {
//   static Node encode(const msquare::planning_math:Vec2d& rhs) {
//     Node node;
//     return node;
//   }

//   static bool decode(const Node& node, msquare::planning_math:Vec2d& rhs) {
//     return true;
//   }
// };

// TODO: implement in a better way as
// https://github.com/jbeder/yaml-cpp/wiki/Tutorial
inline void hand_convert(const YAML::Node &node,
                         msquare::planning_math::Box2d &box) {
  box = msquare::planning_math::Box2d(
      msquare::planning_math::Vec2d(node["center_x"].as<double>(),
                                    node["center_y"].as<double>()),
      node["heading"].as<double>(), node["length"].as<double>(),
      node["width"].as<double>());
}

inline void hand_convert(const msquare::planning_math::Box2d &box,
                         YAML::Node &node) {
  node["center_x"] = box.center_x();
  node["center_y"] = box.center_y();
  node["heading"] = box.heading();
  node["length"] = box.length();
  node["width"] = box.width();
}

inline void hand_convert(const YAML::Node &node,
                         msquare::planning_math::LineSegment2d &line) {
  line = msquare::planning_math::LineSegment2d(
      msquare::planning_math::Vec2d(node["start_x"].as<double>(),
                                    node["start_y"].as<double>()),
      msquare::planning_math::Vec2d(node["end_x"].as<double>(),
                                    node["end_y"].as<double>()));
}

inline void hand_convert(const msquare::planning_math::LineSegment2d &line,
                         YAML::Node &node) {
  node["start_x"] = line.start().x();
  node["start_y"] = line.start().y();
  node["end_x"] = line.end().x();
  node["end_y"] = line.end().y();
}

template <> struct convert<msquare::TrajectoryPoint> {
  static Node encode(const msquare::TrajectoryPoint &rhs) {
    Node node;
    node["path_point"].push_back(rhs.path_point.x);
    node["path_point"].push_back(rhs.path_point.y);
    node["path_point"].push_back(rhs.path_point.theta);
    node["path_point"].push_back(rhs.path_point.s);
    node["v"] = rhs.v;
    node["a"] = rhs.a;
    return node;
  }

  static bool decode(const Node &node, msquare::TrajectoryPoint &rhs) {
    if (!node["path_point"].IsSequence() || node["path_point"].size() != 4)
      return false;
    rhs.path_point.x = node["path_point"][0].as<double>();
    rhs.path_point.y = node["path_point"][1].as<double>();
    rhs.path_point.theta = node["path_point"][2].as<double>();
    rhs.path_point.s = node["path_point"][3].as<double>();
    rhs.v = node["v"].as<double>();
    rhs.a = node["a"].as<double>();
    return true;
  }
};

template <> struct convert<msquare::HybridAstarConfig> {
  static Node encode(const msquare::HybridAstarConfig &rhs) {
    Node node;
    node["use_t_line"] = rhs.use_t_line_;
    node["inflation_for_points"] = rhs.inflation_for_points_;
    node["max_zigzag_allowd"] = rhs.max_zigzag_allowd;
    node["xy_grid_resolution"] = rhs.xy_grid_resolution;
    node["phi_grid_resolution"] = rhs.phi_grid_resolution;
    node["grid_dijkstra_xy_resolution"] = rhs.grid_dijkstra_xy_resolution;
    node["pixel_resolution"] = rhs.pixel_resolution;

    node["traj_forward_penalty"] = rhs.traj_forward_penalty;
    node["traj_back_penalty"] = rhs.traj_back_penalty;
    node["traj_gear_switch_penalty"] = rhs.traj_gear_switch_penalty;
    node["traj_steer_penalty"] = rhs.traj_steer_penalty;
    node["traj_steer_change_penalty"] = rhs.traj_steer_change_penalty;
    node["traj_sides_diff_penalty"] = rhs.traj_sides_diff_penalty;

    node["step_size"] = rhs.step_size;
    node["next_node_num"] = static_cast<int>(rhs.next_node_num);
    node["step_direction"] = rhs.step_direction;
    node["delta_t"] = rhs.delta_t;

    node["holonomic_with_obs_heuristic"] = rhs.holonomic_with_obs_heuristic;
    node["non_holonomic_without_obs_heuristic"] =
        rhs.non_holonomic_without_obs_heuristic;

    node["enable_delta_cost"] = rhs.enable_delta_cost;
    node["enable_analytic_expansion"] = rhs.enable_analytic_expansion;
    node["max_analytic_expansion_length"] = rhs.max_analytic_expansion_length;
    node["analytic_expansion_end_size_threshold"] =
        rhs.analytic_expansion_end_size_threshold;
    node["force_analytic_expansion_end_direction"] =
        static_cast<int>(rhs.force_analytic_expansion_end_direction);

    node["verbose"] = static_cast<int>(rhs.verbose);
    node["display_points"] = rhs.display_points;
    node["planning_core"] = static_cast<int>(rhs.planning_core);
    node["max_iter_base"] = rhs.max_iter;
    node["max_iter_max"] = rhs.max_iter;
    node["footprint_model"] = static_cast<int>(rhs.footprint_model_);
    node["footprint_model_precise"] =
        static_cast<int>(rhs.footprint_model_precise_);

    return node;
  }

  static bool decode(const Node &node, msquare::HybridAstarConfig &rhs) {
    rhs.use_t_line_ = node["use_t_line"].as<bool>();
    rhs.inflation_for_points_ = node["inflation_for_points"].as<double>();
    rhs.max_zigzag_allowd = node["max_zigzag_allowd"].as<int>();
    rhs.xy_grid_resolution = node["xy_grid_resolution"].as<double>();
    rhs.phi_grid_resolution = node["phi_grid_resolution"].as<double>();
    rhs.grid_dijkstra_xy_resolution =
        node["grid_dijkstra_xy_resolution"].as<double>();
    rhs.pixel_resolution = node["pixel_resolution"].as<double>();

    // penalty
    rhs.traj_forward_penalty = node["traj_forward_penalty"].as<double>();
    rhs.traj_back_penalty = node["traj_back_penalty"].as<double>();
    rhs.traj_gear_switch_penalty =
        node["traj_gear_switch_penalty"].as<double>();
    rhs.traj_steer_penalty = node["traj_steer_penalty"].as<double>();
    rhs.traj_steer_change_penalty =
        node["traj_steer_change_penalty"].as<double>();
    rhs.traj_sides_diff_penalty = node["traj_sides_diff_penalty"].as<double>();

    // state update
    rhs.step_size = node["step_size"].as<double>();
    rhs.next_node_num = node["next_node_num"].as<int>();
    rhs.step_direction = node["step_direction"].as<int>();
    rhs.delta_t = node["delta_t"].as<double>();
    rhs.holonomic_with_obs_heuristic =
        node["holonomic_with_obs_heuristic"].as<double>();
    rhs.non_holonomic_without_obs_heuristic =
        node["non_holonomic_without_obs_heuristic"].as<double>();
    rhs.enable_delta_cost = node["enable_delta_cost"].as<bool>();
    rhs.enable_analytic_expansion =
        node["enable_analytic_expansion"].as<bool>();
    rhs.max_analytic_expansion_length =
        node["max_analytic_expansion_length"].as<double>();
    rhs.analytic_expansion_end_size_threshold =
        node["analytic_expansion_end_size_threshold"].as<double>();
    rhs.force_analytic_expansion_end_direction =
        node["force_analytic_expansion_end_direction"].as<int>();

    // visualize
    rhs.verbose = node["verbose"].as<int>();
    rhs.display_points = node["display_points"].as<int>();

    // algorithm
    rhs.planning_core = node["planning_core"].as<int>();

    rhs.max_iter = node["max_iter_base"].as<unsigned>();
    rhs.max_iter_base = node["max_iter_base"].as<unsigned>();
    rhs.max_iter_max = node["max_iter_max"].as<unsigned>();

    rhs.footprint_model_ = node["footprint_model"].as<int>();
    rhs.footprint_model_precise_ = node["footprint_model_precise"].as<int>();
    return true;
  }
};

template <> struct convert<msquare::TrajectoryOptimizerConfig> {
  static Node encode(const msquare::TrajectoryOptimizerConfig &rhs) {
    Node node;
    node["OBCA_running"] = rhs.OBCA_running;
    node["param_enable_check_parallel_trajectory"] =
        rhs.param_enable_check_parallel_trajectory;
    node["param_FLAGS_use_iterative_anchoring_smoother"] =
        rhs.param_FLAGS_use_iterative_anchoring_smoother;
    node["param_FLAGS_use_dual_variable_warm_start"] =
        rhs.param_FLAGS_use_dual_variable_warm_start;
    node["param_FLAGS_enable_smoother_failsafe"] =
        rhs.param_FLAGS_enable_smoother_failsafe;

    node["param_is_near_destination_threshold"] =
        rhs.param_is_near_destination_threshold;
    node["param_delta_t"] = rhs.param_delta_t;
    node["param_cost_end_state"] = rhs.param_cost_end_state;
    node["param_cost_xy"] = rhs.param_cost_xy;
    node["param_cost_theta"] = rhs.param_cost_theta;
    node["param_cost_speed"] = rhs.param_cost_speed;
    node["param_cost_steer"] = rhs.param_cost_steer;
    node["param_cost_acc"] = rhs.param_cost_acc;
    node["param_cost_steerrate"] = rhs.param_cost_steerrate;
    node["param_cost_jerk"] = rhs.param_cost_jerk;
    node["param_cost_stitching_steer"] = rhs.param_cost_stitching_steer;
    node["param_cost_first_order_time"] = rhs.param_cost_first_order_time;
    node["param_cost_second_order_time"] = rhs.param_cost_second_order_time;
    node["param_cost_stitching_a"] = rhs.param_cost_stitching_a;
    node["param_min_safe_dist"] = rhs.param_min_safe_dist;
    node["param_max_steer_angle"] = rhs.param_max_steer_angle;
    node["param_max_speed_forward"] = rhs.param_max_speed_forward;
    node["param_max_speed_reverse"] = rhs.param_max_speed_reverse;
    node["param_max_acc_forward"] = rhs.param_max_acc_forward;
    node["param_max_acc_reverse"] = rhs.param_max_acc_reverse;
    node["param_min_time_sample_scaling"] = rhs.param_min_time_sample_scaling;
    node["param_max_time_sample_scaling"] = rhs.param_max_time_sample_scaling;
    node["param_warm_start_weight"] = rhs.param_warm_start_weight;
    node["param_max_steer_rate"] = rhs.param_max_steer_rate;
    node["param_if_use_fix_time"] = rhs.param_if_use_fix_time;
    node["param_constraint_check"] = rhs.param_constraint_check;
    node["param_jacobian_ad"] = rhs.param_jacobian_ad;
    node["param_info_level"] = rhs.param_info_level;
    node["param_max_itr"] = rhs.param_max_itr;
    node["param_lin_torl"] = rhs.param_lin_torl;
    node["param_mumps_mem"] = rhs.param_mumps_mem;
    node["param_mu_init"] = rhs.param_mu_init;
    node["param_conv_tol"] = rhs.param_conv_tol;
    node["param_constraints_tol"] = rhs.param_constraints_tol;
    node["param_accet_objchange_tol"] = rhs.param_accet_objchange_tol;
    node["param_acceptable_compl_inf_tol"] = rhs.param_acceptable_compl_inf_tol;
    node["param_dual_inf_tol"] = rhs.param_dual_inf_tol;
    node["param_acceptable_tol"] = rhs.param_acceptable_tol;
    node["param_acceptable_iter"] = rhs.param_acceptable_iter;
    node["param_warmstart_acceptable_iter"] =
        rhs.param_warmstart_acceptable_iter;
    node["param_linear_solver"] = rhs.param_linear_solver;

    return node;
  }

  static bool decode(const Node &node,
                     msquare::TrajectoryOptimizerConfig &rhs) {
    rhs.OBCA_running = node["OBCA_running"].as<bool>();
    rhs.param_enable_check_parallel_trajectory =
        node["param_enable_check_parallel_trajectory"].as<bool>();
    rhs.param_FLAGS_use_iterative_anchoring_smoother =
        node["param_FLAGS_use_iterative_anchoring_smoother"].as<bool>();
    rhs.param_FLAGS_use_dual_variable_warm_start =
        node["param_FLAGS_use_dual_variable_warm_start"].as<bool>();
    rhs.param_FLAGS_enable_smoother_failsafe =
        node["param_FLAGS_enable_smoother_failsafe"].as<bool>();

    rhs.param_is_near_destination_threshold =
        node["param_is_near_destination_threshold"].as<double>();
    rhs.param_delta_t = node["param_delta_t"].as<double>();

    rhs.param_cost_end_state = node["param_cost_end_state"].as<double>(); // 5
    rhs.param_cost_xy = node["param_cost_xy"].as<double>();               // 1
    rhs.param_cost_theta = node["param_cost_theta"].as<double>();         // 1
    rhs.param_cost_speed = node["param_cost_speed"].as<double>();         // 1
    rhs.param_cost_steer = node["param_cost_steer"].as<double>();         // 1
    rhs.param_cost_acc = node["param_cost_acc"].as<double>();             // 1
    rhs.param_cost_steerrate = node["param_cost_steerrate"].as<double>(); // 1
    rhs.param_cost_jerk = node["param_cost_jerk"].as<double>();           // 2
    rhs.param_cost_stitching_steer =
        node["param_cost_stitching_steer"].as<double>(); // 3
    rhs.param_cost_first_order_time =
        node["param_cost_first_order_time"].as<double>(); // 1
    rhs.param_cost_second_order_time =
        node["param_cost_second_order_time"].as<double>(); // 1
    rhs.param_cost_stitching_a =
        node["param_cost_stitching_a"].as<double>();                    // 3
    rhs.param_min_safe_dist = node["param_min_safe_dist"].as<double>(); // 0.05
    rhs.param_max_steer_angle =
        node["param_max_steer_angle"].as<double>(); // 0.6
    rhs.param_max_speed_forward =
        node["param_max_speed_forward"].as<double>(); // 5
    rhs.param_max_speed_reverse =
        node["param_max_speed_reverse"].as<double>(); // 5
    rhs.param_max_acc_forward =
        node["param_max_acc_forward"].as<double>(); // 1.5
    rhs.param_max_acc_reverse =
        node["param_max_acc_reverse"].as<double>(); // 1.5
    rhs.param_min_time_sample_scaling =
        node["param_min_time_sample_scaling"].as<double>(); // 0.5
    rhs.param_max_time_sample_scaling =
        node["param_max_time_sample_scaling"].as<double>(); // 0.5
    rhs.param_warm_start_weight =
        node["param_warm_start_weight"].as<double>();                     // 2
    rhs.param_max_steer_rate = node["param_max_steer_rate"].as<double>(); // 2
    rhs.param_if_use_fix_time =
        node["param_if_use_fix_time"].as<bool>(); // true
    rhs.param_constraint_check =
        node["param_constraint_check"].as<bool>();                // false
    rhs.param_jacobian_ad = node["param_jacobian_ad"].as<bool>(); // false

    rhs.param_info_level = node["param_info_level"].as<int>(); // 0
    rhs.param_max_itr = node["param_max_itr"].as<int>();       // 30
    rhs.param_lin_torl = node["param_lin_torl"].as<double>();  // 1
    rhs.param_mumps_mem = node["param_mumps_mem"].as<int>();   // 10000
    rhs.param_mu_init = node["param_mu_init"].as<double>();    // 1
    rhs.param_conv_tol = node["param_conv_tol"].as<double>();  // 1e6
    rhs.param_constraints_tol = node["param_constraints_tol"].as<double>(); // 3
    rhs.param_accet_objchange_tol =
        node["param_accet_objchange_tol"].as<double>(); // 5
    rhs.param_acceptable_compl_inf_tol =
        node["param_acceptable_compl_inf_tol"].as<double>();              // 1e6
    rhs.param_dual_inf_tol = node["param_dual_inf_tol"].as<double>();     // 1e7
    rhs.param_acceptable_tol = node["param_acceptable_tol"].as<double>(); // 1e7
    rhs.param_acceptable_iter = node["param_acceptable_iter"].as<int>();  // 3
    rhs.param_warmstart_acceptable_iter =
        node["param_warmstart_acceptable_iter"].as<int>(); // 5
    rhs.param_linear_solver =
        node["param_linear_solver"].as<std::string>(); // "ma86"

    return true;
  }
};

template <> struct convert<msquare::CarParams> {
  static Node encode(const msquare::CarParams &rhs) {
    Node node;
    node["vehicle_length_real"] = rhs.vehicle_length_real;
    node["vehicle_width_real"] = rhs.vehicle_width_real;
    node["vehicle_width_wo_rearview_mirror"] =
        rhs.vehicle_width_wo_rearview_mirror;
    node["vehicle_length"] = rhs.vehicle_length;
    node["vehicle_width"] = rhs.vehicle_width;
    node["vehicle_height"] = rhs.vehicle_height;
    node["max_acceleration"] = rhs.max_acceleration;
    node["min_acceleration"] = rhs.min_acceleration;
    node["max_steer_angle"] = rhs.max_steer_angle;
    node["max_steer_angle_rate"] = rhs.max_steer_angle_rate;
    node["max_steer_angle_rear"] = rhs.max_steer_angle_rear;
    node["max_steer_angle_rate_rear"] = rhs.max_steer_angle_rate_rear;
    node["steer_ratio"] = rhs.steer_ratio;
    node["wheel_base"] = rhs.wheel_base;
    node["wheel_rolling_radius"] = rhs.wheel_rolling_radius;
    node["max_abs_speed_when_stopped"] = rhs.max_abs_speed_when_stopped;
    node["brake_deadzone"] = rhs.brake_deadzone;
    node["throttle_deadzone"] = rhs.throttle_deadzone;
    node["lon_inflation_min"] = rhs.lon_inflation_min;
    node["lon_inflation_max"] = rhs.lon_inflation_max;
    node["lat_inflation_min"] = rhs.lat_inflation_min;
    node["lat_inflation_max"] = rhs.lat_inflation_max;
    node["inflation_rearview_mirror"] = rhs.inflation_rearview_mirror;
    node["shrink_ratio_for_lines"] = rhs.shrink_ratio_for_lines_;
    node["enable_multiple_steer_modes"] = rhs.enable_multiple_steer_modes;

    return node;
  }

  static bool decode(const Node &node, msquare::CarParams &rhs) {
    rhs.vehicle_width_real = node["vehicle_width_real"].as<float>();
    rhs.vehicle_width_wo_rearview_mirror =
        node["vehicle_width_wo_rearview_mirror"].as<float>();
    rhs.vehicle_height = node["vehicle_height"].as<float>();
    rhs.steer_ratio = node["steer_ratio"].as<float>();
    rhs.wheel_base = node["wheel_base"].as<float>();
    rhs.wheel_rolling_radius = node["wheel_rolling_radius"].as<float>();
    rhs.max_abs_speed_when_stopped =
        node["max_abs_speed_when_stopped"].as<float>();
    rhs.brake_deadzone = node["brake_deadzone"].as<float>();
    rhs.throttle_deadzone = node["throttle_deadzone"].as<float>();

    rhs.lon_inflation_max = node["lon_inflation_max"].as<float>();
    rhs.lon_inflation_min = node["lon_inflation_min"].as<float>();
    rhs.lat_inflation_max = node["lat_inflation_max"].as<float>();
    rhs.lat_inflation_min = node["lat_inflation_min"].as<float>();
    rhs.inflation_rearview_mirror =
        node["inflation_rearview_mirror"].as<float>();
    rhs.shrink_ratio_for_lines_ = node["shrink_ratio_for_lines"].as<double>();
    rhs.max_acceleration = node["max_acceleration"].as<float>();
    rhs.min_acceleration = node["min_acceleration"].as<float>();
    rhs.max_steer_angle = node["max_steer_angle"].as<float>();
    rhs.max_steer_angle_rate = node["max_steer_angle_rate"].as<float>();
    rhs.max_steer_angle_rear = node["max_steer_angle_rear"].as<float>();
    rhs.max_steer_angle_rate_rear =
        node["max_steer_angle_rate_rear"].as<float>();
    rhs.enable_multiple_steer_modes =
        node["enable_multiple_steer_modes"].as<bool>();

    return true;
  }
};

template <> struct convert<msquare::StrategyParams> {
  static Node encode(const msquare::StrategyParams &rhs) {
    Node node;
    node["default_v"] = rhs.default_v_;
    node["default_a"] = rhs.default_a_;
    node["default_dt"] = rhs.default_dt_;
    node["bcheckendsegment"] = rhs.bcheckendsegment;
    node["check_endsegment_max_tan_tolerance"] =
        rhs.check_endsegment_max_tan_tolerance_;
    node["default_check_endsegent_len"] = rhs.default_check_endsegent_len_;
    node["enable_endsegment_len_correction"] =
        rhs.enable_endsegment_len_correction_;
    node["low_speed_endsegment_len"] = rhs.low_speed_endsegment_len_;
    node["enable_revserse_search"] = rhs.enable_revserse_search_;
    node["enable_smooth"] = rhs.enable_smooth_;

    return node;
  }

  static bool decode(const Node &node, msquare::StrategyParams &rhs) {
    rhs.default_v_ = node["default_v"].as<double>();
    rhs.default_a_ = node["default_a"].as<double>();
    rhs.default_dt_ = node["default_dt"].as<double>();
    rhs.bcheckendsegment = node["bcheckendsegment"].as<bool>();
    rhs.check_endsegment_max_tan_tolerance_ =
        node["check_endsegment_max_tan_tolerance"].as<double>();
    rhs.default_check_endsegent_len_ =
        node["default_check_endsegent_len"].as<double>();
    rhs.enable_endsegment_len_correction_ =
        node["enable_endsegment_len_correction"].as<bool>();
    rhs.low_speed_endsegment_len_ =
        node["low_speed_endsegment_len"].as<double>();
    rhs.enable_revserse_search_ = node["enable_revserse_search"].as<bool>();
    rhs.enable_smooth_ = node["enable_smooth"].as<bool>();

    return true;
  }
};

template <> struct convert<msquare::parking::TshapedAreaLines> {
  static Node encode(const msquare::parking::TshapedAreaLines &rhs) {
    Node node;
    using namespace msquare;

    node["is_inited"] = rhs.is_inited;
    YAML::Node line0;
    hand_convert(rhs.road_upper_bound, line0);
    node["lines"].push_back(line0);

    YAML::Node line1;
    hand_convert(rhs.road_lower_left_bound, line1);
    node["lines"].push_back(line1);

    YAML::Node line2;
    hand_convert(rhs.slot_left_bound, line2);
    node["lines"].push_back(line2);

    YAML::Node line3;
    hand_convert(rhs.slot_right_bound, line3);
    node["lines"].push_back(line3);

    YAML::Node line4;
    hand_convert(rhs.road_lower_right_bound, line4);
    node["lines"].push_back(line4);
    return node;
  }
  static bool decode(const Node &node,
                     msquare::parking::TshapedAreaLines &rhs) {
    msquare::planning_math::LineSegment2d line;
    if (node["is_inited"]) {
      rhs.is_inited = node["is_inited"].as<bool>();
    } else {
      rhs.is_inited = false;
    }
    hand_convert(node["lines"][0], line);
    rhs.road_upper_bound = line;
    hand_convert(node["lines"][1], line);
    rhs.road_lower_left_bound = line;
    hand_convert(node["lines"][2], line);
    rhs.slot_left_bound = line;
    hand_convert(node["lines"][3], line);
    rhs.slot_right_bound = line;
    hand_convert(node["lines"][4], line);
    rhs.road_lower_right_bound = line;

    return true;
  }
};

template <> struct convert<msquare::parking::OpenspaceDeciderOutput> {
  static Node encode(const msquare::parking::OpenspaceDeciderOutput &rhs) {
    Node node;
    using namespace msquare;
    node["is_request_square_map"] = rhs.is_request_square_map;

    YAML::Node tmp_node_box;
    hand_convert(rhs.map_boundary, tmp_node_box);
    node["map_boundary"] = tmp_node_box;

    node["init_state"] = YAML::Node(rhs.init_state);
    node["target_state"] = YAML::Node(rhs.target_state);

    node["obstacle_boxs"] = YAML::Node();
    for (const planning_math::Box2d &box : rhs.obstacle_boxs) {
      YAML::Node tmp_node_box;
      tmp_node_box["center_x"] = box.center_x();
      tmp_node_box["center_y"] = box.center_y();
      tmp_node_box["heading"] = box.heading();
      tmp_node_box["length"] = box.length();
      tmp_node_box["width"] = box.width();
      node["obstacle_boxs"].push_back(tmp_node_box);
    }

    node["obstacle_lines"] = YAML::Node();
    for (const planning_math::LineSegment2d &line : rhs.obstacle_lines) {
      YAML::Node tmp_node_line;
      tmp_node_line["start_x"] = line.start().x();
      tmp_node_line["start_y"] = line.start().y();
      tmp_node_line["end_x"] = line.end().x();
      tmp_node_line["end_y"] = line.end().y();
      node["obstacle_lines"].push_back(tmp_node_line);
    }
    for (const planning_math::LineSegment2d &line : rhs.lines) {
      YAML::Node tmp_node_line;
      tmp_node_line["start_x"] = line.start().x();
      tmp_node_line["start_y"] = line.start().y();
      tmp_node_line["end_x"] = line.end().x();
      tmp_node_line["end_y"] = line.end().y();
      node["lines"].push_back(tmp_node_line);
    }

    node["points"] = YAML::Node();
    for (const planning_math::Vec2d &point : rhs.points) {
      YAML::Node tmp_node_point;
      tmp_node_point["x"] = point.x();
      tmp_node_point["y"] = point.y();
      node["points"].push_back(tmp_node_point);
    }

    node["T_lines"] = YAML::Node(rhs.T_lines);
    return node;
  }

  static bool decode(const Node &node,
                     msquare::parking::OpenspaceDeciderOutput &rhs) {
    using namespace msquare;
    rhs.is_request_square_map = node["is_request_square_map"].as<bool>();
    hand_convert(node["map_boundary"], rhs.map_boundary);
    rhs.init_state = node["init_state"].as<msquare::TrajectoryPoint>();
    rhs.target_state = node["target_state"].as<msquare::TrajectoryPoint>();

    planning_math::Box2d box;
    for (size_t i = 0; i < node["obstacle_boxs"].size(); ++i) {
      hand_convert(node["obstacle_boxs"][i], box);
      rhs.obstacle_boxs.push_back(box);
    }

    planning_math::LineSegment2d line;
    for (size_t i = 0; i < node["obstacle_lines"].size(); ++i) {
      hand_convert(node["obstacle_lines"][i], line);
      rhs.obstacle_lines.push_back(line);
    }

    for (size_t i = 0; i < node["lines"].size(); ++i) {
      hand_convert(node["lines"][i], line);
      rhs.lines.push_back(line);
    }

    try {
      for (size_t i = 0; i < node["points"].size(); ++i) {
        rhs.points.push_back(
            planning_math::Vec2d(node["points"][i]["x"].as<double>(),
                                 node["points"][i]["y"].as<double>()));
      }
    } catch (std::exception &e) {
      // std::cout << e.what() << ": \"points\" doesn't exist in file" <<
      // std::endl;
    }

    rhs.T_lines = node["T_lines"].as<parking::TshapedAreaLines>();

    return true;
  }
};

} // namespace YAML

// } // namespace parking
// } // namespace msquare