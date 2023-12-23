#pragma once
#include "ceres/ceres.h"
#include "path_cost_functor.hpp"
#include "path_planner_decider.hpp"
#include "path_spline_wrapper.hpp"
namespace path_planner {

class PathPlanner {
public:
  PathPlanner();

  void update_input(const PathPlannerInput &input, const bool verbose,
                    NotebookDebug *nb_debug = nullptr);

  void run_ceres_opt(const PathPlannerInput &input);

  void populate_output_info(const PathPlannerInput *input);

  void populate_debug_info();

  void populate_avd_aim_info();

  void check_truck_status(int truck_id, const PathPlannerInput *input);

  void get_output(PathPlannerOutput &output) const { output = output_; }

  const PathPlannerOutput &get_prev_output() const { return pre_output_; }

  PathPlannerOutput *mutable_prev_output() { return &pre_output_; }

  const Point2d &get_lc_end_point() const {
    return path_planner_decider_->get_lc_end_point();
  }

  void get_output_at_s(const PathPlannerInput *input,
                       path_planner::PathPlannerPoint &output,
                       double sample_s) const;

  void get_debug(PathPlannerDebug &debug) const { debug = debug_; }

  const SolverReport &get_solver_report() const { return solver_report_; }

  void add_other_info2nb_debug(NotebookDebug *nb_debug = nullptr);

  void reset() {
    count_active_avd_ = 0;
    last_active_avd_state_ = "none";
    last_active_avd_obs_id_ = -1;
    last_active_avd_obs_id_count_ = 0;
  }

  int get_count_active_avd() { return int(count_active_avd_); }
  std::string get_last_active_avd_state() { return last_active_avd_state_; }
  int get_last_active_avd_obs_id() { return last_active_avd_obs_id_; }
  int get_last_active_avd_obs_id_count() {
    return last_active_avd_obs_id_count_;
  }
  bool get_lc_end_clear_count() { return lc_end_clear_count_; }
  Intelligent_Dodge_Info get_dodge_info() {
    Intelligent_Dodge_Info dodge_info = path_planner_decider_->get_dodge_info();
    dodge_info.count_active_avd = count_active_avd_;
    dodge_info.last_active_avd_state = last_active_avd_state_;
    dodge_info.last_active_avd_obs_id = last_active_avd_obs_id_;
    dodge_info.last_active_avd_obs_id_count = last_active_avd_obs_id_count_;
    dodge_info.lc_end_clear_count = lc_end_clear_count_;
    return dodge_info;
  }

  DLPInfo get_dlp_info() {
    return current_dlp_info_;
  }

private:
  void init_opt_params(const PathPlannerInput &input);
  // ceres solver variables
  ceres::Solver::Options solver_options_;
  ceres::Problem::Options ceres_options_;
  ceres::Problem ceres_problem_;
  ceres::Solver::Summary ceres_summary_;

  // cost function
  PathCostFunctor *path_cost_functor_;
  ceres::AutoDiffCostFunction<PathCostFunctor, TOTAL_NUM_RESIDUALS,
                              TOTAL_NUM_PARAMS> *path_cost_function_;

  // cost function variables
  bool verbose_{false};
  OptParamArray opt_params_;
  PlannerCubicSpline<NUM_PATH_CONTROL_POINTS> path_offset_spline_;
  std::shared_ptr<PathSpline<double>> result_path_spline_;

  // path planner decison info
  std::shared_ptr<PathPlannerDecider> path_planner_decider_;

  // output and debug data
  PathPlannerOutput output_;
  PathPlannerOutput pre_output_;
  PathPlannerDebug debug_;
  SolverReport solver_report_;

  size_t count_active_avd_ = 0;
  std::string last_active_avd_state_{"none"};
  int last_active_avd_obs_id_{-1};
  int last_active_avd_obs_id_count_{0};
  bool lc_end_clear_count_{false};

  // DLP related
  DLPInfo current_dlp_info_;
};
} // namespace path_planner
