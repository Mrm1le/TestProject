#pragma once

#include "common/math/math_utils.h"
#include "nlohmann/json.hpp"
#include <mutex>
#include <string>
#include <yaml-cpp/yaml.h>

namespace msquare {
namespace grid {
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

  unsigned max_iter;
  unsigned max_iter_base;
  unsigned max_iter_max;

  bool is_loaded;

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(
      HybridAstarConfig, use_t_line_, inflation_for_points_, max_zigzag_allowd,

      xy_grid_resolution, phi_grid_resolution, grid_dijkstra_xy_resolution,
      pixel_resolution,

      traj_forward_penalty, traj_back_penalty, traj_gear_switch_penalty,
      traj_steer_penalty, traj_steer_change_penalty, traj_sides_diff_penalty,

      step_size, next_node_num, step_direction,

      delta_t, holonomic_with_obs_heuristic,
      non_holonomic_without_obs_heuristic,

      enable_delta_cost, enable_analytic_expansion,
      max_analytic_expansion_length, analytic_expansion_end_size_threshold,
      force_analytic_expansion_end_direction,

      verbose, display_points,

      planning_core, footprint_model_, footprint_model_precise_,

      max_iter, max_iter_base, max_iter_max)
};

// HybridAstarConfig *HybridAstarConfig::instance = nullptr;

} // namespace grid
} // namespace msquare
