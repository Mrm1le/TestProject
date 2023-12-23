#pragma once
#include "ceres/ceres.h"
#include "speed_cost_functor.hpp"
#include "speed_spline_wrapper.hpp"

namespace speed_planner {

class SpeedPlanner {
public:
  SpeedPlanner();

  void update_input(const SpeedPlannerInput &input, const bool verbose,
                    path_planner::NotebookDebug *nb_debug = nullptr);

  void run_ceres_opt();

  void populate_output_info(
      const SpeedPlannerInput *input, const OptParamArray &opt_params,
      const PlannerCubicSpline<NUM_SPEED_CONTROL_POINTS> &acceleration_spline);

  SpeedPlannerDebug populate_debug_info(const OptParamArray &opt_params);

  void get_output(SpeedPlannerOutput &output) const { output = output_; }

  void get_debug(SpeedPlannerDebug &debug) const { debug = debug_; }

  void get_opt_params(OptParamArray &opt_params) const {
    opt_params = opt_params_;
  }

  void update_CIPV_info(SpeedPlannerInput *input);

  const path_planner::SolverReport &get_solver_report() const {
    return solver_report_;
  }

private:
  void select_init_seed(const SpeedPlannerInput &input);

private:
  // ceres solver variables
  ceres::Solver::Options solver_options_;
  ceres::Problem::Options ceres_options_;
  ceres::Problem ceres_problem_;
  ceres::Solver::Summary ceres_summary_;

  // cost function
  SpeedCostFunctor *speed_cost_functor_;
  ceres::AutoDiffCostFunction<SpeedCostFunctor, TOTAL_NUM_RESIDUALS,
                              TOTAL_NUM_PARAMS> *speed_cost_function_;

  // cost function variables
  bool verbose_{true};
  OptParamArray opt_params_;
  PlannerCubicSpline<NUM_SPEED_CONTROL_POINTS> acceleration_spline_;
  std::shared_ptr<AccelerationSpline<double>> result_acceleration_spline_;

  // output and debug data
  SpeedPlannerOutput output_;
  SpeedPlannerDebug debug_;
  path_planner::SolverReport solver_report_;
};
} // namespace speed_planner
