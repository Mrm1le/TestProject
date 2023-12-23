#pragma once
#include "nlohmann/json.hpp"
#include "planner_constants.hpp"
#include "proto_idl/interface_planner_common.hpp"
#include "planner_common_interface.hpp"
#include "proto_idl/interface_path_planner.hpp"
#include "planner_common_interface.hpp"
using json = nlohmann::json;

namespace cp_path_planner {

// Note: this is only used for debugging in jupyter notebook
struct SampleDebugPoint {
  std::unordered_map<std::string, double> sample_data;
};

// Note: this is only used for debugging in jupyter notebook
struct NotebookDebug {
  std::unordered_map<double, SampleDebugPoint> s_to_samples{};
  std::string avd_direction;
  int object_id;
  int type;

  // PathPlannerDebug:
  std::array<double, TOTAL_NUM_RESIDUALS> residual_debug;
  std::array<std::array<double, TOTAL_NUM_RESIDUALS>, TOTAL_NUM_PARAMS>
      per_residual_per_param_gradient;
  std::array<double, TOTAL_NUM_PARAMS> total_gradients_per_param;
  std::array<std::array<std::pair<double, double>, QUADRATURE_ORDER>,
             NUM_PATH_SEGMENTS>
      lane_activation_dist;

  void clear() {
    s_to_samples = {};
    avd_direction = "";
    object_id = -1000;
    type = 1;
    residual_debug = {};
    per_residual_per_param_gradient = {};
    total_gradients_per_param = {};
    lane_activation_dist = {};
  }
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SpeedPlan, v, a, j)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AvdResultInfo, avd_direction, object_id,
                                   type, ego_faster_truck, overlap_lane)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AvdAimInfo, id, type, avd_direction, a,
                                   max_l, min_l, desire_buffer,
                                   lane_border_base_l)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    RefPointInfo, time, x, y, curvature, current_lane_width, left_lane_width,
    right_lane_width, left_lane_border, right_lane_border, left_road_border,
    right_road_border, left_lidar_road_border, right_lidar_road_border,
    theta_ref, cos_theta_ref, sin_theta_ref, left_road_border_type,
    right_road_border_type)
// NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point2d, x, y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPoint, position_enu, heading_yaw,
                                   curvature, path_follow_strength)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathSampleInfo, sample_s, last_cart_traj,
                                   refline_info, speed_plan)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathSegmentInfo, end_segment_control_point_s,
                                   end_segment_refline_info, quad_point_info)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PathTuningParams, max_num_iterations, soft_boundary_scale,
    hard_boundary_scale, init_curvature_scale, curvature_limit_scale,
    lat_accel_scale, lat_jerk_scale, heading_scale, ref_centering_scale,
    prev_plan_centering_scale, obstacle_constrain_scale, obstacle_desire_scale,
    obstacle_inflation_scale)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPlannerPoint, s, v, t, a, da_ds, x,
                                   dx_ds, d2x_ds2, y, dy_ds, d2y_ds2, l, dl,
                                   ddl, jerk, curvature, sample_info)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SampleDebugInfo, x, y, offset, curvature,
                                   dcurv_dt, lat_acc, lat_jerk, ref_offset,
                                   left_activation_dist, right_activation_dist,
                                   left_map_hard_dist, right_map_hard_dist,
                                   left_obs_desire, right_obs_desire,
                                   left_obs_constrain, right_obs_constrain)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPlannerOutput, path_planner_output,
                                   debug_info_output, avd_result_info,
                                   dyn_left_aim_info, dyn_right_aim_info,
                                   static_left_aim_info, static_right_aim_info)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LaneChangeInfo, lc_status, lc_status_time,
                                   is_lane_change, t_pred, lc_wait_dir,
                                   pre_lc_wait, lc_end,
                                   origin_lane_front_obs_id,
                                   prev_lc_end_point_fren)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IntersectionInfo, is_in_intersection,
                                   keep_in_intersection,
                                   keep_in_intersection_timer,
                                   dist_to_intersection,
                                   dist_to_nonintersection)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsPrediction, rel_s, s, v, v_frenet, a,
                                   desired_headway, stop_offset, heading,
                                   polygon_cart, polygon_fren)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VectorObsPrediction, vec,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairDoubleObsPrediction, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsInfo, id, type, lon_decision,
                                   lat_decision, nudge_side, polygon_init,
                                   polygon_list, object_with_t,
                                   distance_to_lane_line_history, overlap,
                                   overtake_decider_static,
                                   is_static_from_decider)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Header, seq, stamp, frame_id)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VehicleParam, length, width, center_to_front)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VectorDDPTrajectoryPoint, vec,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DDPInfo, valid, ddp_path)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Intelligent_Dodge_Info, state, elapsed_count,
                                   dodge_truck_id, dodge_l, avd_direction,
                                   lc_count, cp_count, end_debounce_count)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MinRefJerk, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DLPInfo, is_in_dlp)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPlannerInput, header, is_replan,
                                   lat_safety_improved, ego_theta, min_ref_jerk,
                                   planning_init_state, path_segments,
                                   path_tuning_params, lc_decider_info,
                                   intersec_info, obs_list, vehicle_param,
                                   ddp_info, lc_end, lc_wait_dir, pre_lc_wait,
                                   lat_use_ld3d, lane_border_avd_dist,
                                   enable_extend_rb_backward, dodge_info, pre_dlp_info)

// debug info
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SampleDebugPoint, sample_data)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NotebookDebug, s_to_samples, avd_direction,
                                   object_id, type, residual_debug,
                                   per_residual_per_param_gradient,
                                   total_gradients_per_param,
                                   lane_activation_dist)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairIntVectorDouble, first, second,
                                   __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LateralMotionPlannerOutput,
                                   lat_avoid_obstacle_history_info_map,
                                   path_planner_debug_info,
                                   prev_path_planner_output,
                                   path_planner_output, pre_dodge_info, pre_dlp_info)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SolverReport, type, steps, total_time,
                                   init_cost, final_cost, msg)

} // namespace path_planner