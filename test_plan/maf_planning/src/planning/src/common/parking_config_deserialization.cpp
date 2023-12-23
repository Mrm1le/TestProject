#include "common/parking_config_deserialization.h"
#include "nlohmann/json.hpp"

namespace msquare {
namespace parking {
using namespace msquare::planning_math;

void deserialization(const nlohmann::json &ha_cfg_json,
                     HybridAstarConfig *ha_cfg) {
  ha_cfg->use_t_line_ = ha_cfg_json["use_t_line_"].get<bool>();
  if (ha_cfg_json.find("inflation_for_points_") == ha_cfg_json.end()) {
    ha_cfg->inflation_for_points_ = 0.1;
  } else {
    ha_cfg->inflation_for_points_ =
        ha_cfg_json["inflation_for_points_"].get<double>();
  }
  ha_cfg->max_zigzag_allowd = ha_cfg_json["max_zigzag_allowd"].get<int>();

  ha_cfg->xy_grid_resolution = ha_cfg_json["xy_grid_resolution"].get<double>();
  ha_cfg->phi_grid_resolution =
      ha_cfg_json["phi_grid_resolution"].get<double>();
  ha_cfg->grid_dijkstra_xy_resolution =
      ha_cfg_json["grid_dijkstra_xy_resolution"].get<double>();
  ha_cfg->pixel_resolution = ha_cfg_json["pixel_resolution"].get<double>();

  ha_cfg->traj_forward_penalty =
      ha_cfg_json["traj_forward_penalty"].get<double>();
  ha_cfg->traj_back_penalty = ha_cfg_json["traj_back_penalty"].get<double>();
  ha_cfg->traj_gear_switch_penalty =
      ha_cfg_json["traj_gear_switch_penalty"].get<double>();
  ha_cfg->traj_steer_penalty = ha_cfg_json["traj_steer_penalty"].get<double>();
  ha_cfg->traj_steer_change_penalty =
      ha_cfg_json["traj_steer_change_penalty"].get<double>();
  ha_cfg->traj_sides_diff_penalty =
      ha_cfg_json["traj_sides_diff_penalty"].get<double>();

  ha_cfg->step_size = ha_cfg_json["step_size"].get<double>();
  ha_cfg->next_node_num = ha_cfg_json["next_node_num"].get<int>();
  ha_cfg->step_direction = ha_cfg_json["step_direction"].get<int>();

  ha_cfg->delta_t = ha_cfg_json["delta_t"].get<double>();

  ha_cfg->holonomic_with_obs_heuristic =
      ha_cfg_json["holonomic_with_obs_heuristic"].get<double>();
  ha_cfg->non_holonomic_without_obs_heuristic =
      ha_cfg_json["non_holonomic_without_obs_heuristic"].get<double>();

  ha_cfg->enable_delta_cost = ha_cfg_json["enable_delta_cost"].get<bool>();
  ha_cfg->enable_analytic_expansion =
      ha_cfg_json["enable_analytic_expansion"].get<bool>();
  ha_cfg->max_analytic_expansion_length =
      ha_cfg_json["max_analytic_expansion_length"].get<double>();
  ha_cfg->analytic_expansion_end_size_threshold =
      ha_cfg_json["analytic_expansion_end_size_threshold"].get<double>();
  ha_cfg->force_analytic_expansion_end_direction =
      ha_cfg_json["force_analytic_expansion_end_direction"].get<int>();

  ha_cfg->verbose = ha_cfg_json["verbose"].get<int>();
  ha_cfg->display_points = ha_cfg_json["display_points"].get<int>();

  ha_cfg->planning_core = ha_cfg_json["planning_core"].get<int>();
  ha_cfg->footprint_model_ = ha_cfg_json["footprint_model_"].get<int>();
  ha_cfg->footprint_model_precise_ =
      ha_cfg_json["footprint_model_precise_"].get<int>();

  ha_cfg->max_iter = ha_cfg_json["max_iter"].get<int>();
  ha_cfg->max_iter_base = ha_cfg_json["max_iter_base"].get<int>();
  ha_cfg->max_iter_max = ha_cfg_json["max_iter_max"].get<int>();
}

void deserialization(const nlohmann::json &veh_prm_json,
                     VehicleParam *veh_prm) {
  veh_prm->front_edge_to_center =
      veh_prm_json["front_edge_to_center"].get<double>();
  veh_prm->back_edge_to_center =
      veh_prm_json["back_edge_to_center"].get<double>();
  veh_prm->left_edge_to_center =
      veh_prm_json["left_edge_to_center"].get<double>();
  veh_prm->right_edge_to_center =
      veh_prm_json["right_edge_to_center"].get<double>();
  veh_prm->center_to_geometry_center =
      veh_prm_json["center_to_geometry_center"].get<double>();
  veh_prm->bumper_length = veh_prm_json["bumper_length"].get<double>();
  veh_prm->light_to_front_edge =
      veh_prm_json["light_to_front_edge"].get<double>();
  veh_prm->front_edge_to_mirror =
      veh_prm_json["front_edge_to_mirror"].get<double>();

  veh_prm->length = veh_prm_json["length"].get<double>();
  veh_prm->width = veh_prm_json["width"].get<double>();
  veh_prm->width_wo_rearview_mirror =
      veh_prm_json["width_wo_rearview_mirror"].get<double>();
  veh_prm->height = veh_prm_json["height"].get<double>();
  veh_prm->min_turn_radius = veh_prm_json["min_turn_radius"].get<double>();
  veh_prm->max_acceleration = veh_prm_json["max_acceleration"].get<double>();
  veh_prm->max_deceleration = veh_prm_json["max_deceleration"].get<double>();
  veh_prm->max_steer_angle = veh_prm_json["max_steer_angle"].get<double>();
  veh_prm->max_steer_angle_rate =
      veh_prm_json["max_steer_angle_rate"].get<double>();
  veh_prm->min_steer_angle_rate =
      veh_prm_json["min_steer_angle_rate"].get<double>();
  veh_prm->steer_ratio = veh_prm_json["steer_ratio"].get<double>();
  veh_prm->wheel_base = veh_prm_json["wheel_base"].get<double>();
  veh_prm->wheel_rolling_radius =
      veh_prm_json["wheel_rolling_radius"].get<double>();
  veh_prm->max_abs_speed_when_stopped =
      veh_prm_json["max_abs_speed_when_stopped"].get<double>();

  veh_prm->brake_deadzone = veh_prm_json["brake_deadzone"].get<double>();
  veh_prm->throttle_deadzone = veh_prm_json["throttle_deadzone"].get<double>();
  veh_prm->velocity_deadzone = veh_prm_json["velocity_deadzone"].get<double>();

  veh_prm->max_front_wheel_angle =
      veh_prm_json["max_front_wheel_angle"].get<double>();
  veh_prm->brake_distance_buffer =
      veh_prm_json["brake_distance_buffer"].get<double>();

  veh_prm->limiter_detect_linear_acceleration_z_delta =
      veh_prm_json["limiter_detect_linear_acceleration_z_delta"].get<double>();
  veh_prm->limiter_detect_linear_acceleration_x_positive =
      veh_prm_json["limiter_detect_linear_acceleration_x_positive"]
          .get<double>();
  veh_prm->limiter_detect_collision_duration_time =
      veh_prm_json["limiter_detect_collision_duration_time"].get<double>();
}

void deserialization(const nlohmann::json &car_prm_json, CarParams *car_prm) {
  car_prm->vehicle_length_real =
      car_prm_json["vehicle_length_real"].get<double>();
  car_prm->vehicle_width_real =
      car_prm_json["vehicle_width_real"].get<double>();
  car_prm->vehicle_width_wo_rearview_mirror =
      car_prm_json["vehicle_width_wo_rearview_mirror"].get<double>();
  car_prm->vehicle_length = car_prm_json["vehicle_length"].get<double>();
  car_prm->vehicle_width = car_prm_json["vehicle_width"].get<double>();
  car_prm->vehicle_height = car_prm_json["vehicle_height"].get<double>();
  car_prm->max_acceleration = car_prm_json["max_acceleration"].get<double>();
  car_prm->min_acceleration = car_prm_json["min_acceleration"].get<double>();
  car_prm->max_steer_angle = car_prm_json["max_steer_angle"].get<double>();
  car_prm->max_steer_angle_rate =
      car_prm_json["max_steer_angle_rate"].get<double>();
  car_prm->max_steer_angle_rear =
      car_prm_json["max_steer_angle_rear"].get<double>();
  car_prm->max_steer_angle_rate_rear =
      car_prm_json["max_steer_angle_rate_rear"].get<double>();
  car_prm->steer_ratio = car_prm_json["steer_ratio"].get<double>();
  car_prm->wheel_base = car_prm_json["wheel_base"].get<double>();
  car_prm->wheel_rolling_radius =
      car_prm_json["wheel_rolling_radius"].get<double>();
  car_prm->max_abs_speed_when_stopped =
      car_prm_json["max_abs_speed_when_stopped"].get<double>();
  car_prm->brake_deadzone = car_prm_json["brake_deadzone"].get<double>();
  car_prm->throttle_deadzone = car_prm_json["throttle_deadzone"].get<double>();
  car_prm->lon_inflation_min = car_prm_json["lon_inflation_min"].get<double>();
  car_prm->lon_inflation_max = car_prm_json["lon_inflation_max"].get<double>();
  car_prm->lat_inflation_min = car_prm_json["lat_inflation_min"].get<double>();
  car_prm->lat_inflation_max = car_prm_json["lat_inflation_max"].get<double>();
  car_prm->inflation_rearview_mirror =
      car_prm_json["inflation_rearview_mirror"].get<double>();
  car_prm->shrink_ratio_for_lines_ =
      car_prm_json["shrink_ratio_for_lines_"].get<double>();
  car_prm->shrink_ratio_for_lines_min_ =
      car_prm_json["shrink_ratio_for_lines_min_"].get<double>();
  car_prm->enable_multiple_steer_modes =
      car_prm_json["enable_multiple_steer_modes"].get<bool>();

  car_prm->max_delta_angle = car_prm_json["max_delta_angle"].get<double>();
  car_prm->max_delta_angle_rate =
      car_prm_json["max_delta_angle_rate"].get<double>();
  car_prm->max_delta_angle_rear =
      car_prm_json["max_delta_angle_rear"].get<double>();
  car_prm->max_delta_angle_rate_rear =
      car_prm_json["max_delta_angle_rate_rear"].get<double>();
  car_prm->min_turn_radius = car_prm_json["min_turn_radius"].get<double>();

  (void)car_prm->setLatInflation(car_prm_json["lat_inflation_"].get<double>());
  (void)car_prm->setLonInflation(car_prm_json["lon_inflation_"].get<double>());

  if (car_prm_json.contains("type")) {
    car_prm->type = car_prm_json["type"].get<std::string>();
  }
}

void deserialization(const nlohmann::json &stg_prm_json,
                     StrategyParams *stg_prm) {
  stg_prm->default_check_endsegent_len_ =
      stg_prm_json["default_check_endsegent_len_"].get<double>();
  stg_prm->enable_revserse_search_ =
      stg_prm_json["enable_revserse_search_"].get<bool>();
}

void deserialization(const nlohmann::json &param_json,
                     TrajectoryOptimizerConfig *param) {
  param->param_max_speed_forward =
      param_json["param_max_speed_forward"].get<double>();
  param->param_max_speed_reverse =
      param_json["param_max_speed_reverse"].get<double>();
  param->param_max_acc_forward =
      param_json["param_max_acc_forward"].get<double>();
  param->param_max_acc_reverse =
      param_json["param_max_acc_reverse"].get<double>();
}

} // namespace parking
} // namespace msquare