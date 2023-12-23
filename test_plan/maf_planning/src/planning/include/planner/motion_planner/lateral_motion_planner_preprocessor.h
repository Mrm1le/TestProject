#pragma once

#include "common/trajectory/trajectory_stitcher.h"
#include "common/utils/gauss_quad_constants.h"
#include "data_driven_planner/common/ddp_context.h"
#include "linear_interpolation_utility.hpp"
#include "planner/message_type.h"
#include "planner/motion_planner/planner_preprocessor.h"
#include "planner/tasks/task.h"

namespace msquare {
class LateralMotionPlannerPreprocessor : public PlannerPreprocessor {
  struct CacheData {
    planning_math::InterpolationData<Point2D> last_path_frenet_pair_list;
    planning_math::InterpolationData<path_planner::PathPoint>
        last_path_cart_pair_list;
    planning_math::InterpolationData<RefPointFrenet>
        current_lane_map_info_pair_list;
    planning_math::InterpolationData<double> left_lane_width_pair_list;
    planning_math::InterpolationData<double> right_lane_width_pair_list;
    planning_math::InterpolationData<path_planner::SpeedPlan>
        speed_plan_pair_list;
    planning_math::InterpolationData<double> lon_s_pair_list;
    planning_math::InterpolationData<double> sample_s_pair_list;
    planning_math::InterpolationData<double> s_t_pair_list;
    planning_math::InterpolationData<path_planner::PathPlannerPoint>
        prev_path_planner_output_pair_list;
    planning_math::InterpolationData<double> ddp_path_s_l_list;
  };

public:
  LateralMotionPlannerPreprocessor() : PlannerPreprocessor(){};

  virtual void process();

private:
  void generate_output();
  void gerenate_s_at_control_quad_points();
  void compute_planning_init_state();
  void sample_ddp_path();
  void compute_path_segments();
  void set_init_params();
  void compute_lc_decider_info();
  void compute_intersection_info();
  void limit_jerk();
  void generate_obstacle_info();
  void generate_radar_info();
  void update_lc_status(const int lc_status);
  void clear_obstacle_histroy_info_without_nudge();
  void
  update_obstacle_history_info(std::vector<path_planner::ObsInfo> &obs_list);
  path_planner::PathSampleInfo generate_path_sample_info(double quad_point_s);
  void generate_map_info();
  path_planner::RefPointInfo generate_map_sample_info(const double s);
  void set_vehicle_params();
  void set_uniform_lon_motion_input(const TrajectoryPoint &planning_init_point);
  void set_pre_lon_result(const TrajectoryPoint &planning_init_point);
  size_t QueryNearestPoint(const std::vector<maf_planning::PathPoint> &pre_path,
                           const planning_math::Vec2d &position,
                           const double buffer);
  void set_intelligent_dodge_info();
  void set_dlp_info();

  CacheData cache_data_;

  std::array<double, path_planner::NUM_PATH_CONTROL_POINTS>
      s_at_control_points_;
  std::array<std::array<double, path_planner::QUADRATURE_ORDER>,
             path_planner::NUM_PATH_SEGMENTS>
      s_at_quad_points_;
};

} // namespace msquare
