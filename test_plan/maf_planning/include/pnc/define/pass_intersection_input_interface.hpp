#pragma once
#include "nlohmann/json.hpp"
// #include "common/refline_generator.h"
// #include "pnc/define/pass_intersection_input_interface.hpp"

using json = nlohmann::json;

namespace pass_intersection_planner {

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
  int farthest_index;
};

struct PathIntersectionParams {
  int core_type;
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
  int farthest_index;
};

struct SimpleLeaderInfo {
  int id_;
  double info_time_;
  double position_x_;
  double position_y_;
  double speed_;
  double acc_;
  double yaw_;
  double yaw_diff_;
  double angle_diff_;
  SimpleLeaderInfo(){}
  SimpleLeaderInfo(int id, double position_x, double position_y, double speed,
                   double acc, double yaw, double yaw_diff, double angle_diff)
      : id_(id), position_x_(position_x), position_y_(position_y),
        speed_(speed), acc_(acc), yaw_(yaw), yaw_diff_(yaw_diff),
        angle_diff_(angle_diff) {
    // info_time_ = MTIME()->timestamp().sec();
  };

  SimpleLeaderInfo(int id, double info_time, double position_x,
                   double position_y, double speed, double acc, double yaw,
                   double yaw_diff, double angle_diff)
      : id_(id), info_time_(info_time), position_x_(position_x),
        position_y_(position_y), speed_(speed), acc_(acc), yaw_(yaw),
        yaw_diff_(yaw_diff), angle_diff_(angle_diff){};
};


struct HistoricalLeaderInfo {

  HistoricalLeaderInfo(){}
  HistoricalLeaderInfo(const int &leader_id) {
    leader_id_ = leader_id;
    // create_time_ = MTIME()->timestamp().sec();
    // update_time_ = create_time_;
  };
  void add(const SimpleLeaderInfo &info) {
    infos_.push_back(info);
    // update_time_ = info.info_time_;
  };
  void reset() {
    infos_.clear();
    // create_time_ = MTIME()->timestamp().sec();
    // update_time_ = MTIME()->timestamp().sec();
  };
  std::vector<SimpleLeaderInfo> &infos() { return infos_; };
  std::vector<SimpleLeaderInfo> &mutable_infos() { return infos_; }

  const double &update_time() { return update_time_; };
  const double &create_time() { return create_time_; };
  const int leader_id() { return leader_id_; };

  HistoricalLeaderInfo &operator=(const HistoricalLeaderInfo &copy) {
    leader_id_ = copy.leader_id_;
    // create_time_ = copy.create_time_;
    // update_time_ = copy.update_time_;
    infos_ = copy.infos_;
    now_car_x_ = copy.now_car_x_;
    now_car_y_ = copy.now_car_y_;
    return *this;
  }
  double now_car_x_;
  double now_car_y_;
  int leader_id_;
  double create_time_;
  double update_time_;
  std::vector<SimpleLeaderInfo> infos_;
};

// PassIntersectionPlannerInput 每一帧update过程中，如果是值，必须要刷新，如果是vector，必须要clear!!
struct PassIntersectionPlannerInput {
  bool force_replan_ {false};
  bool is_in_intersection_cond_{false};
  bool lane_road_edges_avail_{true};
  double in_intersection_ego_theta_{0.0};
  std::vector<double> in_intersection_pos_;
  std::vector<HistoricalLeaderInfo> muti_historical_info_;
  std::vector<Traj_Point> adc_historical_info_;
  std::vector<Traj_Point> hist_refline_;
  std::vector<std::vector<Traj_Point>> lanes_;
  std::vector<std::vector<Traj_Point>> road_edges_;
  std::vector<Traj_Point> cur_lane_left_;
  std::vector<Traj_Point> cur_lane_right_;
  std::vector<Traj_Point> last_refline_result_;
  std::vector<Traj_Point> refline_points_;
  Traj_Point refline_origin_;
  Traj_Point ego_pose_;
  std::vector<double> target_pt_;
  std::vector<double> target_pt_last_;
  bool pi_target_pt_exist_;
  int target_pt_count_ = 0;
  int current_lane_index_;
  int lane_size_;
  double current_last_car_point_x_;
  ReflineCondition condition_;
  ReflineCondition last_condition_;
  ApfAttractType attract_type_;
  ApfAttractType last_attract_type_;
  int selected_hist_vehicle_id_ = -1;
  int inter_intersection_count_ = 0;
  PathIntersectionParams pi_params_;
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
  std::vector<std::vector<double>> quintic_refline{};
  std::vector<std::vector<double>> origin_refline{};
  std::vector<std::vector<double>> extend_refline{};

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
    quintic_refline = {};
    origin_refline = {};
    extend_refline = {};
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
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Traj_Point, x, y, theta, vel)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SimpleLeaderInfo, id_, info_time_,
                                   position_x_, position_y_, speed_, acc_, yaw_,
                                   yaw_diff_, angle_diff_)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(HistoricalLeaderInfo, now_car_x_, now_car_y_,
                                   leader_id_, create_time_, update_time_,
                                   infos_)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PassIntersectionPlannerInput, force_replan_, is_in_intersection_cond_,
    lane_road_edges_avail_, in_intersection_ego_theta_, in_intersection_pos_,
    muti_historical_info_, adc_historical_info_, hist_refline_, lanes_,
    road_edges_, cur_lane_left_, cur_lane_right_, last_refline_result_,
    refline_points_, refline_origin_, ego_pose_, target_pt_, target_pt_last_,
    pi_target_pt_exist_, target_pt_count_, current_lane_index_, lane_size_,
    current_last_car_point_x_, condition_, last_condition_, attract_type_,
    last_attract_type_, selected_hist_vehicle_id_, inter_intersection_count_,
    pi_params_)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PiNotebookDebug, apf_refline, follow_car_id,
                                   refline_condition, attract_type, ego_theta,
                                   road_theta, lat_dis_in_intersection,
                                   lat_dis_out_intersection, quit_cp, ego_pos,
                                   refline_target_pt, quintic_end,
                                   quintic_refline, origin_refline,
                                   extend_refline)

} // namespace path_planner
