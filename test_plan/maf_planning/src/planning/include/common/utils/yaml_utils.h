#pragma once

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/parking_planner_types.h"
#include "common/utils/yaml_utils_parking.h"
#include "maf_interface/maf_planning.h"
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

template <> struct convert<maf_planning::SBPRequest> {
  static Node encode(const maf_planning::SBPRequest &rhs) {

    Node node;
    using namespace msquare;
    node["x_bound"] = 28;
    node["y_bound"] = 28;

    // STRATEGY_PARAMS
    YAML::Node tmp_node_strategy_param;
    tmp_node_strategy_param["default_v"] = 0.5;
    tmp_node_strategy_param["default_a"] = 0.3;
    tmp_node_strategy_param["default_dt"] = 1.0;

    tmp_node_strategy_param["bcheckendsegment"] = true;
    tmp_node_strategy_param["check_endsegment_max_tan_tolerance"] = 0.02;
    tmp_node_strategy_param["default_check_endsegent_len"] = 0;
    tmp_node_strategy_param["enable_endsegment_len_correction"] = false;
    tmp_node_strategy_param["low_speed_endsegment_len"] = 2.0;
    tmp_node_strategy_param["enable_revserse_search"] = false;
    tmp_node_strategy_param["enable_smooth"] = false;

    node["STRATEGY_PARAM"] = tmp_node_strategy_param;

    // CAR_PARAMS
    YAML::Node tmp_node_car_params;
    tmp_node_car_params["lon_inflation_min"] =
        rhs.task_config.problem_config.params.lon_inflation;
    tmp_node_car_params["lon_inflation_max"] =
        rhs.task_config.problem_config.params.lon_inflation;
    tmp_node_car_params["lat_inflation_min"] =
        rhs.task_config.problem_config.params.lat_inflation;
    tmp_node_car_params["lat_inflation_max"] =
        rhs.task_config.problem_config.params.lat_inflation;
    tmp_node_car_params["inflation_rearview_mirror"] =
        rhs.task_config.problem_config.params.inflation_rearview_mirror;
    tmp_node_car_params["shrink_ratio_for_lines"] =
        rhs.task_config.problem_config.params.shrink_ratio_for_lines;
    tmp_node_car_params["max_acceleration"] = 2.0;
    tmp_node_car_params["min_acceleration"] = -3.0;
    tmp_node_car_params["max_steer_angle"] =
        rhs.task_config.problem_config.params.max_steer_angle;
    tmp_node_car_params["max_steer_angle_rate"] =
        rhs.task_config.problem_config.params.max_steer_angle_rate;
    tmp_node_car_params["max_steer_angle_rear"] =
        rhs.task_config.problem_config.params.max_steer_angle_rear;
    tmp_node_car_params["max_steer_angle_rate_rear"] =
        rhs.task_config.problem_config.params.max_steer_angle_rate_rear;
    tmp_node_car_params["enable_multiple_steer_modes"] =
        rhs.task_config.problem_config.params.enable_multiple_steer_modes;
    node["CAR_PARAMS"] = tmp_node_car_params;

    // HYBRID_ASTAR_PARAMS
    YAML::Node tmp_node_hybrid_astar_params;
    tmp_node_hybrid_astar_params["max_zigzag_allowd"] = 5;

    tmp_node_hybrid_astar_params["enable_delta_cost"] = true;
    tmp_node_hybrid_astar_params["enable_analytic_expansion"] =
        rhs.task_config.problem_config.params.enable_analytic_expansion;
    tmp_node_hybrid_astar_params["analytic_expansion_end_size_threshold"] =
        rhs.task_config.problem_config.params
            .analytic_expansion_end_size_threshold;
    tmp_node_hybrid_astar_params["force_analytic_expansion_end_direction"] =
        static_cast<int>(rhs.task_config.problem_config.params
                             .force_analytic_expansion_end_direction);

    tmp_node_hybrid_astar_params["xy_grid_resolution"] =
        rhs.task_config.problem_config.params.xy_grid_resolution;
    tmp_node_hybrid_astar_params["phi_grid_resolution"] =
        rhs.task_config.problem_config.params.phi_grid_resolution;
    tmp_node_hybrid_astar_params["grid_dijkstra_xy_resolution"] =
        rhs.task_config.problem_config.params.grid_dijkstra_xy_resolution;
    tmp_node_hybrid_astar_params["pixel_resolution"] = 0.3;

    tmp_node_hybrid_astar_params["traj_forward_penalty"] =
        rhs.task_config.problem_config.params.traj_forward_penalty;
    tmp_node_hybrid_astar_params["traj_back_penalty"] =
        rhs.task_config.problem_config.params.traj_back_penalty;
    tmp_node_hybrid_astar_params["traj_gear_switch_penalty"] =
        rhs.task_config.problem_config.params.traj_gear_switch_penalty;
    tmp_node_hybrid_astar_params["traj_steer_penalty"] =
        rhs.task_config.problem_config.params.traj_steer_penalty;
    tmp_node_hybrid_astar_params["traj_steer_change_penalty"] =
        rhs.task_config.problem_config.params.traj_steer_change_penalty;
    tmp_node_hybrid_astar_params["traj_sides_diff_penalty"] = 0;

    tmp_node_hybrid_astar_params["step_size"] =
        rhs.task_config.problem_config.params.step_size;
    tmp_node_hybrid_astar_params["next_node_num"] =
        static_cast<int>(rhs.task_config.problem_config.params.next_node_num);
    tmp_node_hybrid_astar_params["step_direction"] = 0;
    tmp_node_hybrid_astar_params["delta_t"] = 1;

    tmp_node_hybrid_astar_params["holonomic_with_obs_heuristic"] =
        rhs.task_config.problem_config.params.holonomic_with_obs_heuristic;
    tmp_node_hybrid_astar_params["non_holonomic_without_obs_heuristic"] =
        rhs.task_config.problem_config.params
            .non_holonomic_without_obs_heuristic;

    tmp_node_hybrid_astar_params["verbose"] =
        static_cast<int>(rhs.task_config.problem_config.params.verbose);
    tmp_node_hybrid_astar_params["display_points"] = 0;

    tmp_node_hybrid_astar_params["planning_core"] =
        static_cast<int>(rhs.task_config.problem_config.params.planning_core);
    tmp_node_hybrid_astar_params["max_iter_base"] =
        rhs.task_config.problem_config.params.max_iteration;
    tmp_node_hybrid_astar_params["max_iter_max"] =
        rhs.task_config.problem_config.params.max_iteration;

    tmp_node_hybrid_astar_params["footprint_model"] =
        static_cast<int>(rhs.task_config.problem_config.params.footprint_model);
    tmp_node_hybrid_astar_params["footprint_model_precise"] = static_cast<int>(
        rhs.task_config.problem_config.params.footprint_model_precise);
    tmp_node_hybrid_astar_params["max_analytic_expansion_length"] = -1;

    node["HYBRID_ASTAR_PARAMS"] = tmp_node_hybrid_astar_params;

    // OBCA_PARAMS
    YAML::Node tmp_node_obca_params;
    tmp_node_obca_params["OBCA_running"] = false;
    tmp_node_obca_params["param_enable_check_parallel_trajectory"] = true;
    tmp_node_obca_params["param_FLAGS_use_iterative_anchoring_smoother"] =
        false;
    tmp_node_obca_params["param_FLAGS_use_dual_variable_warm_start"] = true;
    tmp_node_obca_params["param_FLAGS_enable_smoother_failsafe"] = true;
    tmp_node_obca_params["param_is_near_destination_threshold"] = 0.1;
    tmp_node_obca_params["param_delta_t"] = 1;

    tmp_node_obca_params["param_cost_end_state"] = 100;
    tmp_node_obca_params["param_cost_xy"] = 1;
    tmp_node_obca_params["param_cost_theta"] = 1;
    tmp_node_obca_params["param_cost_speed"] = 100;
    tmp_node_obca_params["param_cost_steer"] = 10;
    tmp_node_obca_params["param_cost_acc"] = 10;
    tmp_node_obca_params["param_cost_steerrate"] = 10;
    tmp_node_obca_params["param_cost_jerk"] = 10;
    tmp_node_obca_params["param_cost_stitching_steer"] = 5;
    tmp_node_obca_params["param_cost_first_order_time"] = 1;
    tmp_node_obca_params["param_cost_second_order_time"] = 1;
    tmp_node_obca_params["param_cost_stitching_a"] = 5;
    tmp_node_obca_params["param_min_safe_dist"] = 0.00;
    tmp_node_obca_params["param_max_steer_angle"] = 0.51;
    tmp_node_obca_params["param_max_speed_forward"] = 1.00;
    tmp_node_obca_params["param_max_speed_reverse"] = 0.75;
    tmp_node_obca_params["param_max_acc_forward"] = 1.5;
    tmp_node_obca_params["param_max_acc_reverse"] = 1.5;
    tmp_node_obca_params["param_min_time_sample_scaling"] = 1;
    tmp_node_obca_params["param_max_time_sample_scaling"] = 1;
    tmp_node_obca_params["param_max_steer_rate"] = 0.3;
    tmp_node_obca_params["param_warm_start_weight"] = 1;
    tmp_node_obca_params["param_if_use_fix_time"] = false;
    tmp_node_obca_params["param_constraint_check"] = true;
    tmp_node_obca_params["param_jacobian_ad"] = true;

    tmp_node_obca_params["param_info_level"] = 0;
    tmp_node_obca_params["param_max_itr"] = 300;
    tmp_node_obca_params["param_lin_torl"] = 1e-2;
    tmp_node_obca_params["param_mumps_mem"] = 10000;
    tmp_node_obca_params["param_mu_init"] = 1;
    tmp_node_obca_params["param_conv_tol"] = 1;
    tmp_node_obca_params["param_constraints_tol"] = 0.05;
    tmp_node_obca_params["param_accet_objchange_tol"] = 1;
    tmp_node_obca_params["param_acceptable_compl_inf_tol"] = 1e6;
    tmp_node_obca_params["param_dual_inf_tol"] = 1e8;
    tmp_node_obca_params["param_acceptable_tol"] = 1e6;
    tmp_node_obca_params["param_acceptable_iter"] = 15;
    tmp_node_obca_params["param_warmstart_acceptable_iter"] = 15;
    tmp_node_obca_params["param_linear_solver"] = "ma57";

    node["OBCA_PARAMS"] = tmp_node_obca_params;

    return node;
  }

  static bool decode(const Node &node,
                     const msquare::parking::OpenspaceDeciderOutput &rhs) {
    return true;
  }
};

} // namespace YAML

// } // namespace parking
// } // namespace msquare