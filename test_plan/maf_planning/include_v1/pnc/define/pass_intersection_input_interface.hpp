#pragma once
#include "nlohmann/json.hpp"
// #include "common/refline_generator.h"
// #include "pnc/define/pass_intersection_input_interface.hpp"
#include "common/math/cartesian_quintic_poly_1d.h"
#include "proto_idl/interface_pass_intersection_input.hpp"

using json = nlohmann::json;

namespace cp_pass_intersection_planner {

struct SimpleLeaderInfoUtils {
  static void update_simple_leader_info_1(SimpleLeaderInfo& simple_leader_info,
                                      int id, double position_x, double position_y, double speed,
                                      double acc, double yaw, double yaw_diff, double angle_diff,
                                      double length, double width) {

    simple_leader_info.id_ = id;
    simple_leader_info.position_x_ = position_x;
    simple_leader_info.position_y_ = position_y;
    simple_leader_info.speed_ = speed;
    simple_leader_info.acc_ = acc;
    simple_leader_info.yaw_ = yaw;
    simple_leader_info.yaw_diff_ = yaw_diff;
    simple_leader_info.angle_diff_ = angle_diff;
    simple_leader_info.length_ = length;
    simple_leader_info.width_ = width;
  };

  static void update_simple_leader_info_2(SimpleLeaderInfo& simple_leader_info,
                                      int id, double info_time, double position_x, double position_y, double speed,
                                      double acc, double yaw, double yaw_diff, double angle_diff,
                                      double length, double width) {

    simple_leader_info.id_ = id;
    simple_leader_info.info_time_ = info_time;
    simple_leader_info.position_x_ = position_x;
    simple_leader_info.position_y_ = position_y;
    simple_leader_info.speed_ = speed;
    simple_leader_info.acc_ = acc;
    simple_leader_info.yaw_ = yaw;
    simple_leader_info.yaw_diff_ = yaw_diff;
    simple_leader_info.angle_diff_ = angle_diff;
    simple_leader_info.length_ = length;
    simple_leader_info.width_ = width;
  };
};

struct HistoricalLeaderInfoUtils {
  static void update_historical_leader_info(HistoricalLeaderInfo& historical_leader_info, 
                                              const int &leader_id) {
    historical_leader_info.leader_id_ = leader_id;
  };

  static std::vector<SimpleLeaderInfo> &mutable_infos(HistoricalLeaderInfo& historical_leader_info) {
    return historical_leader_info.infos_; 
  };
};

struct ObsHistPredTrajInfoUtils {
  static void update_obj_hist_pred_traj_info(ObsHistPredTrajInfo& obj_hist_pred_traj_info,
                                                const int &leader_id) {
    obj_hist_pred_traj_info.leader_id_ = leader_id;
  };

  static std::vector<SimpleLeaderInfo> &mutable_infos(ObsHistPredTrajInfo& obj_hist_pred_traj_info) {
    return obj_hist_pred_traj_info.infos_;
  };

  static std::vector<SimpleLeaderInfo> &mutable_pred_infos(ObsHistPredTrajInfo& obj_hist_pred_traj_info) {
    return obj_hist_pred_traj_info.pred_infos_;
  };
};

// Note: this is only used for debugging in jupyter notebook
struct PiNotebookDebug {
  std::vector<std::vector<double>> apf_refline{};
  int follow_car_id{-1};
  int refline_condition{-1};
  int attract_type{-1};
  double ego_theta{0.0};
  double road_theta{0.0};
  double lat_dis_in_intersection{0.0};
  double lat_dis_out_intersection{0.0};
  int quit_cp{-1};
  std::vector<double> ego_pos{};
  std::vector<double> refline_target_pt{};
  std::vector<double> quintic_end{};
  std::vector<std::vector<double>> quin_refline{};
  std::vector<std::vector<double>> origin_refline{};
  std::vector<std::vector<double>> extend_refline{};
  std::vector<CartesianQuintic> cartesian_quintic_poly_vector{};
  LaneCenterInfo target_lane;
  std::vector<CandidatePoint> candidate_sample_pts{};
  std::vector<Traj_Point> quintic_refline{};
  std::vector<Traj_Point> last_quintic_refline{};
  std::vector<std::vector<Traj_Point>> obs_hist_pred_traj{};
  std::vector<int> obs_hist_pred_traj_id{};
  std::vector<std::vector<std::vector<Traj_Point>>> obs_hist_pred_traj_box{};
  std::vector<double> lant_center_car_slope{};
  std::vector<double> lant_center_dis{};
  std::vector<double> origin_pt_to_lc{};
  std::vector<bool> is_parallel_out{};
  std::vector<Traj_Point> cartesian_quintic{};
  std::vector<double> lc_back_dis2other{};
  std::vector<LaneCenterInfo> origin_lane_extend_infos;
  std::vector<std::vector<Traj_Point>> last_plan_lines{};
  std::vector<CandidatePoint> intersection_pts{};
  std::vector<LaneCenterInfo> origin_lane_extend_seg_infos;
  std::vector<LaneBoundaryInfo> lb_lines;
  std::vector<LaneCenterInfo> ego2lcs;

  void clear() {
    apf_refline = {};
    follow_car_id = -1;
    refline_condition = 0;
    attract_type = 0;
    ego_theta = 0.0;
    road_theta = 0.0;
    lat_dis_in_intersection = 0.0;
    lat_dis_out_intersection = 0.0;
    quit_cp = 0;
    ego_pos = {};
    refline_target_pt = {};
    quintic_end = {};
    quin_refline = {};
    origin_refline = {};
    extend_refline = {};
    cartesian_quintic_poly_vector = {};
    target_lane = {};
    candidate_sample_pts = {};
    quintic_refline = {};
    last_quintic_refline = {};
    obs_hist_pred_traj = {};
    obs_hist_pred_traj_box = {};
    obs_hist_pred_traj_id = {};
    lant_center_car_slope = {};
    lant_center_dis = {};
    origin_pt_to_lc = {};
    is_parallel_out = {};
    cartesian_quintic = {};
    lc_back_dis2other = {};
    origin_lane_extend_infos = {};
    last_plan_lines = {};
    intersection_pts = {};
    origin_lane_extend_seg_infos = {};
    lb_lines = {};
    ego2lcs = {};
  }
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PathIntersectionTunedParams, max_length, min_length, step_size,
    stop_gradient_value, direction_field_intensity,
    direction_field_intensity_l, line_repulsion_weight,
    line_repulsion_max_distance, line_repulsion_2_weight,
    line_repulsion_2_attenuation, max_look_ahead_distance,
    min_look_ahead_distance, look_ahead_time, max_leader_distance,
    min_leader_distance, max_yaw_diff, max_angle_diff, max_l_tolerance,
    traj_select_s_tolerance, traj_select_l_tolerance, traj_select_yaw_tolerance, use_quintic, farthest_index)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SelectTargetLaneParams,
                                   lat_change_cost_weight, yaw_diff_cost_weight,
                                   max_jerk_abs_cost_weight,
                                   track_id_diff_cost_weight,
                                   frame_consistency_cost_weight,
                                   traffic_avoidance_cost_weight)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PathIntersectionParams, core_type, enable_follow_target_pt, shrink_ratio,
    step_size, min_turning_radius, vehicle_length, vehicle_width,
    center_to_geometry_center, line_repulsion_weight,
    line_repulsion_max_distance, line_repulsion_2_weight,
    line_repulsion_2_attenuation, direction_field_intensity,
    direction_field_intensity_l, back_extend_points_num,
    thin_out_angle_accuracy, thin_out_max_length, stop_gradient_value,
    max_length, min_length, lane_select_score, max_look_ahead_distance,
    min_look_ahead_distance, look_ahead_time, max_leader_distance,
    min_leader_distance, max_yaw_diff, max_angle_diff, use_agnle_diff_ratio,
    max_l_tolerance, discard_time, max_historical_info_length,
    traj_select_s_tolerance, traj_select_l_tolerance, traj_select_yaw_tolerance,
    refline_real_length, quit_cp_lat_dis_in_inter, quit_cp_lat_dis_refline,
    quit_cp_consider_refline_length, replan_threshold, zero_speed_threshold,
    coord_transform_precision, step_s, coarse_step_s, optimization_gamma,
    max_iter, use_quintic, farthest_index)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ApfAttractType, attract_type)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ReflineCondition, reflinecondition)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Traj_Point, x, y, theta, vel, curvature)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CandidatePoint, x, y, relative_id, index,
                                   dist, dis2car, car_x, car_y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SimpleLeaderInfo, id_, info_time_,
                                   position_x_, position_y_, speed_, acc_, yaw_,
                                   yaw_diff_, angle_diff_, length_, width_)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(HistoricalLeaderInfo, now_car_x_, now_car_y_,
                                   leader_id_, create_time_, update_time_,
                                   infos_)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsHistPredTrajInfo, now_car_x_, now_car_y_,
                                   leader_id_, create_time_, update_time_,
                                   infos_, pred_infos_)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LaneCenterInfo, relative_id, track_id,
                                   lane_center_enu_pts, is_same_lane,
                                   is_opposize_lane)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LaneBoundaryInfo, relative_id, lb_enu_pts)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ReflinePoints, enu_x, enu_y, curvature,
                                   distance_to_left_road_border,
                                   distance_to_right_road_border,
                                   distance_to_left_lane_border,
                                   distance_to_right_lane_border,
                                   distance_to_left_obstacle,
                                   distance_to_right_obstacle, lane_width)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    CartesianQuintic, relative_id, track_id, index, target_pt_index, sample_j,
    front_index, back_index, seg_buffer, max_jerk_abs, is_valid, is_reverse, is_valid_cost,
    track_id_diff_cost, total_cost, cartesian_quintic_poly1d_enu_pts,
    target_pt_dis2car, lat_jerk_avg, max_acc_abs, lat_acc_avg, overlane_cost,
    cumu_lat_dis_cost, cumu_lat_dis, candidate_pt)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VectorTrajPoint, vec, __convert_to_list___)
    
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PassIntersectionPlannerInput, force_replan_, is_in_intersection_cond_,
    lane_road_edges_avail_, in_intersection_ego_theta_, in_intersection_pos_,
    muti_historical_info_, obs_pred_info_, adc_historical_info_, hist_refline_,
    lanes_, road_edges_, cur_lane_left_, cur_lane_right_, last_refline_result_,
    refline_points_, refline_origin_, ego_pose_, target_pt_, target_pt_last_,
    pi_target_pt_exist_, target_pt_count_, current_lane_index_, lane_size_,
    current_last_car_point_x_, condition_, last_condition_, attract_type_,
    last_attract_type_, selected_hist_vehicle_id_, inter_intersection_count_,
    pi_params_, select_target_lane_params_, origin_lane_infos_,
    first_lane_infos_, last_plan_result_, lane_source_)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PiNotebookDebug, apf_refline, follow_car_id, refline_condition,
    attract_type, ego_theta, road_theta, lat_dis_in_intersection,
    lat_dis_out_intersection, quit_cp, ego_pos, refline_target_pt, quintic_end,
    quin_refline, origin_refline, extend_refline, cartesian_quintic_poly_vector,
    target_lane, candidate_sample_pts, quintic_refline, last_quintic_refline,
    obs_hist_pred_traj, obs_hist_pred_traj_box, obs_hist_pred_traj_id,
    lant_center_car_slope, lant_center_dis, origin_pt_to_lc, is_parallel_out,
    cartesian_quintic, lc_back_dis2other, origin_lane_extend_infos,
    last_plan_lines, intersection_pts, origin_lane_extend_seg_infos,
    lb_lines, ego2lcs)

} // namespace cp_pass_intersection_planner
