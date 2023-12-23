#pragma once

#include "ceres/ceres.h"
#include "nlohmann/json.hpp"
#include "path_planner_constants.hpp"
#include "pnc/define/path_planner_interface.hpp"
#include "pnc/define/speed_planner_interface.hpp"
#include <unordered_map>

namespace path_planner {

template <typename T> struct CommonTerms {
  double sample_s = 0.0;
  double t = 0.0;
  T x;
  T dx_ds;
  T d2x_ds2;
  T d3x_ds3;
  T y;
  T dy_ds;
  T d2y_ds2;
  T d3y_ds3;
  T offset;
  double prev_plan_offset;
  T ds_dt;

  T curvature;
  T dc_ds;
  double v;
  double a;
  T lat_accel;
  T lat_jerk;
};

// Ceres solver will use double and ceres::Jet during optimization
template <typename T> struct Vector2T {
  T x;
  T y;
};

/*********************************************************************************************
 *                                  Debug struct
 ********************************************************************************************/

struct SamplePointDecisionInfo {
  double ref_offset;
  double left_activation_dist;
  double right_activation_dist;
  double left_map_hard_dist;
  double right_map_hard_dist;
  double left_map_hard_desire;
  double right_map_hard_desire;

  double l_min;
  double l_max;
  double left_obs_constrain;
  double right_obs_constrain;
  double left_obs_desire;
  double right_obs_desire;

  double left_obs_inflation;
  double right_obs_inflation;
};

using DecisionInfo =
    std::array<std::array<SamplePointDecisionInfo, QUADRATURE_ORDER>,
               NUM_PATH_SEGMENTS>;

using LaneLineActivationDist =
    std::array<std::array<std::pair<double, double>, QUADRATURE_ORDER>,
               NUM_PATH_SEGMENTS>;

struct PathPlannerDebug {
  std::array<double, TOTAL_NUM_RESIDUALS> residual_debug;
  std::array<std::array<double, TOTAL_NUM_RESIDUALS>, TOTAL_NUM_PARAMS>
      per_residual_per_param_gradient;
  std::array<double, TOTAL_NUM_PARAMS> total_gradients_per_param;
  LaneLineActivationDist lane_activation_dist;
  LaneLineActivationDist obj_activation_dist;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPlannerDebug, residual_debug,
                                   per_residual_per_param_gradient,
                                   total_gradients_per_param,
                                   lane_activation_dist, obj_activation_dist)

} // namespace path_planner