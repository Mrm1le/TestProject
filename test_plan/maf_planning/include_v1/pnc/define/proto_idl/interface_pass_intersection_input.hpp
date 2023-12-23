#ifndef CP_INTERFACE_PASS_INTERSECTION_INPUT_H
#define CP_INTERFACE_PASS_INTERSECTION_INPUT_H

#include <array>
#include <string>
#include <vector>
#include "interface_planner_common.hpp"

namespace cp_pass_intersection_planner {

struct ApfAttractType {
  enum : uint8_t { 
    Unknow = 0, 
    Traj = 1, 
    Theta = 2 
  };
  uint8_t attract_type;
};

struct ReflineCondition {
  enum : uint8_t {
    NO_INTERSECTION = 0,
    FIRST_HALF_INTERSECTION = 1,
    IN_INTERSECTION = 2,
    LAST_HALF_INTERSECTION = 3,
    UNKNOW = 4
  };
  uint8_t reflinecondition;
};

struct Traj_Point {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double vel{0.0};
  double curvature{0.0};
};

struct CandidatePoint {
  double x{0.0};
  double y{0.0};
  int32_t relative_id{-1};
  int32_t index{-1};
  double dist{-1.0};
  double dis2car{-1.0};
  double car_x{-1.0};
  double car_y{-1.0};
};
struct PathIntersectionTunedParams {
  // apf refline length
  double max_length;
  double min_length;
  double step_size;
  double stop_gradient_value;

  // attraction intensity
  double direction_field_intensity;
  double direction_field_intensity_l;
  
  // repulsion intensity
  double line_repulsion_weight;
  double line_repulsion_max_distance;
  double line_repulsion_2_weight;
  double line_repulsion_2_attenuation;

  // follow decider
  double max_look_ahead_distance;
  double min_look_ahead_distance;
  double look_ahead_time;
  double max_leader_distance;
  double min_leader_distance;
  double max_yaw_diff;
  double max_angle_diff;
  double max_l_tolerance;
  double traj_select_s_tolerance;
  double traj_select_l_tolerance;
  double traj_select_yaw_tolerance;

  // use quintic
  bool use_quintic;
  int32_t farthest_index;
};

struct PathIntersectionParams {
  int32_t core_type;
  bool enable_follow_target_pt;
  double shrink_ratio;
  double step_size; 
  double min_turning_radius;
  double vehicle_length;
  double vehicle_width;
  double center_to_geometry_center;
  double line_repulsion_weight;
  double line_repulsion_max_distance;
  double line_repulsion_2_weight;
  double line_repulsion_2_attenuation;
  double direction_field_intensity;
  double direction_field_intensity_l;
  double back_extend_points_num;
  double thin_out_angle_accuracy;
  double thin_out_max_length;
  double stop_gradient_value;
  double max_length;
  double min_length;
  double lane_select_score;
  double max_look_ahead_distance;
  double min_look_ahead_distance;
  double look_ahead_time;
  double max_leader_distance;
  double min_leader_distance;
  double max_yaw_diff;
  double max_angle_diff;
  double use_agnle_diff_ratio;
  double max_l_tolerance;
  double discard_time;
  double max_historical_info_length;
  double traj_select_s_tolerance;
  double traj_select_l_tolerance;
  double traj_select_yaw_tolerance;
  double refline_real_length;
  double quit_cp_lat_dis_in_inter;
  double quit_cp_lat_dis_refline;
  double quit_cp_consider_refline_length;
  double replan_threshold;
  double zero_speed_threshold;
  double coord_transform_precision;
  double step_s;
  double coarse_step_s;
  double optimization_gamma;
  double max_iter;
  bool use_quintic;
  int32_t farthest_index;
};

struct SelectTargetLaneParams {
  double lat_change_cost_weight;
  double yaw_diff_cost_weight;
  double max_jerk_abs_cost_weight;
  double track_id_diff_cost_weight;
  double frame_consistency_cost_weight;
  double traffic_avoidance_cost_weight;
};

struct LaneCenterInfo {
  int32_t relative_id;
  int32_t track_id;
  std::vector<Traj_Point> lane_center_enu_pts;
  bool is_same_lane {false};
  bool is_opposize_lane {false};
};

struct LaneBoundaryInfo {
  int32_t relative_id;
  std::vector<Traj_Point> lb_enu_pts;
};

struct ReflinePoints {
  double enu_x;
  double enu_y;
  double curvature{0.0};
  double distance_to_left_road_border{10.0};
  double distance_to_right_road_border{10.0};
  double distance_to_left_lane_border{10.0};
  double distance_to_right_lane_border{10.0};
  double distance_to_left_obstacle{1000.0};
  double distance_to_right_obstacle{1000.0};
  double lane_width{20.0};
};
    
struct SimpleLeaderInfo {
  int32_t id_;
  double info_time_{0.0};
  double position_x_;
  double position_y_;
  double speed_;
  double acc_;
  double yaw_;
  double yaw_diff_;
  double angle_diff_;
  double length_;
  double width_;
};

struct CartesianQuintic {
  int32_t relative_id{-1};
  int32_t track_id{-1};
  int32_t index{-1};
  int32_t target_pt_index{-1};
  int32_t sample_j{-1};
  int32_t front_index{-1};
  int32_t back_index{-1};
  double seg_buffer{0.0};
  double max_jerk_abs{-1};
  bool is_valid{true};
  bool is_reverse{false};
  double is_valid_cost{0.0};
  double track_id_diff_cost{0.0};
  double total_cost{-1};
  std::vector<ReflinePoints> cartesian_quintic_poly1d_enu_pts;
  double target_pt_dis2car{0.0};
  double lat_jerk_avg{0.0};
  double max_acc_abs{0.0};
  double lat_acc_avg{0.0};
  double overlane_cost{0.0};
  double cumu_lat_dis_cost{0.0};
  double cumu_lat_dis{0.0};
  CandidatePoint candidate_pt{};
};

struct HistoricalLeaderInfo {
  double now_car_x_;
  double now_car_y_;
  int32_t leader_id_;
  double create_time_;
  double update_time_;
  std::vector<SimpleLeaderInfo> infos_;
};

struct ObsHistPredTrajInfo {
  double now_car_x_;
  double now_car_y_;
  int32_t leader_id_;
  double create_time_;
  double update_time_;
  std::vector<SimpleLeaderInfo> infos_;
  std::vector<SimpleLeaderInfo> pred_infos_;
};

struct VectorTrajPoint{
  std::vector<Traj_Point> vec;
  bool __convert_to_list___ = true;
};

// PassIntersectionPlannerInput 每一帧update过程中，如果是值，必须要刷新，如果是vector，必须要clear!!
struct PassIntersectionPlannerInput {
  bool force_replan_ {false};
  bool is_in_intersection_cond_{false};
  bool lane_road_edges_avail_{true};
  double in_intersection_ego_theta_{0.0};
  std::vector<double> in_intersection_pos_;
  std::vector<HistoricalLeaderInfo> muti_historical_info_;
  std::vector<ObsHistPredTrajInfo> obs_pred_info_;
  std::vector<Traj_Point> adc_historical_info_;
  std::vector<Traj_Point> hist_refline_;
  std::vector<VectorTrajPoint> lanes_;
  std::vector<VectorTrajPoint> road_edges_;
  std::vector<Traj_Point> cur_lane_left_;
  std::vector<Traj_Point> cur_lane_right_;
  std::vector<Traj_Point> last_refline_result_;
  std::vector<Traj_Point> refline_points_;
  Traj_Point refline_origin_;
  Traj_Point ego_pose_;
  std::vector<double> target_pt_;
  std::vector<double> target_pt_last_;
  bool pi_target_pt_exist_;
  int32_t target_pt_count_ = 0;
  int32_t current_lane_index_;
  int32_t lane_size_;
  double current_last_car_point_x_;
  ReflineCondition condition_;
  ReflineCondition last_condition_;
  ApfAttractType attract_type_;
  ApfAttractType last_attract_type_;
  int32_t selected_hist_vehicle_id_ = -1;
  int32_t inter_intersection_count_ = 0;
  PathIntersectionParams pi_params_;
  SelectTargetLaneParams select_target_lane_params_;
  std::vector<LaneCenterInfo> origin_lane_infos_;
  std::vector<LaneCenterInfo> first_lane_infos_;
  std::vector<Traj_Point> last_plan_result_;
  std::string lane_source_;
};
} // namespace cp_pass_intersection_planner

#endif