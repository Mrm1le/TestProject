#pragma once

#include "linear_interpolation_utility.hpp"
#include "planner/motion_planner/planner_preprocessor.h"
#include "planner/tasks/task.h"

namespace msquare {

class LongitudinalPlannerPreprocessor : public PlannerPreprocessor {
public:
  struct CacheData {
    planning_math::InterpolationData<path_planner::PathPlannerPoint>
        prev_speed_planner_output_pair_list;
  };

public:
  LongitudinalPlannerPreprocessor() : PlannerPreprocessor(){};

  virtual void process();

private:
  void update_last_speed_planner_result();

  void gerenate_s_at_control_quad_points();

  void compute_planning_init_state();

  void compute_speed_segments();

  void get_obs_overlap();

  void calc_obs_ax();

  void update_object_prediction(
      const std::array<std::array<double, path_planner::QUADRATURE_ORDER>,
                       speed_planner::NUM_SPEED_SEGMENTS> &time_list);

  // 每个request会修改v/a/j min/max/reason
  void set_vaj_bound_and_reason();

  void update_speed_limit_info(speed_planner::SpeedLimitSet &speed_limit_set,
                               const double &v_ego);

  void set_speed_limit_request();

  void set_curve_speed_request();

  void set_env_speed_limit_request();

  void set_model_speed_request();

  void set_select_gap_speed_request();

  void set_cipv_lost_speed_request();

  void set_speed_planner_init_params();

  void generate_lon_obstacles_info();

  void compute_stopping_point_info();

  void update_start_state();

  bool stop_signal_judge(const double &f_dist_m, const double &f_rel_v_mps,
                         const double &f_ego_axdv_mps2);

  void stop_flag_decision();

  void go_flag_decision();

  void update_stop_mode();

  void generate_radar_info();

  void set_vehicle_params();

  void select_cipv();

  void compute_overlap_info();

  void calc_accel_limit();

  void calc_stop_ref();

  void ProcessCipvFn();

private:
  std::array<double, speed_planner::NUM_SPEED_CONTROL_POINTS>
      s_at_control_points_;
  std::array<std::array<double, path_planner::QUADRATURE_ORDER>,
             speed_planner::NUM_SPEED_SEGMENTS>
      s_at_quad_points_;
  CacheData cache_data_;
};

} // namespace msquare
