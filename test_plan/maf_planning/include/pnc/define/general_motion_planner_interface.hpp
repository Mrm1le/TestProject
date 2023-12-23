#pragma once
#include "nlohmann/json.hpp"
#include "planner_constants.hpp"
using json = nlohmann::json;

namespace gmp_interface {

enum LC_Dir { NONE = 1, LEFT = 2, RIGHT = 3 };

struct Point2d {
  double x;
  double y;
};

struct Pose2d {
  double x;
  double y;
  double theta;
  double s = 0.0;
  double kappa_ = 0.0;
};

struct SVPoint {
  double s;
  double v_min;
  double v_max;
  double v_ref;
};

struct SafetyMargin {
  double longitu;
  double lateral;
};

struct ObjState {
  Point2d pos;
  double accel{-200.0};
  double vel{-200.0}; // linear speed
  double heading{-200.0};
  double s{-200.0};
  double v_frenet{-200.0};
  double rel_s{-200.0};
};

struct ObjPredSlice {
  SafetyMargin safety_margin;
  ObjState obj_state_local;
  ObjState obj_state_env;
  int lane_assignment;
};

struct ObjInfoRefined {
  int id{-1};
  int id_relative{-1};
  double headaway{2.0};
  gmp_interface::Pose2d pos_init;
};

struct ObjSize {
  double length;
  double width;
};

struct ObjMarginInfo {
  std::vector<SafetyMargin> pred_sm;
};

struct AllObjsMarginInfo {
  std::vector<ObjMarginInfo> all_objs_pred_sm;
};

struct ObsInfo {
  // enum Type { PEDESTRIAN = 0, OFO = 1, COUPE = 2, TRANSPORT_TRUNK = 4 };

  enum LonDecision {
    FOLLOW = 0,
    OVERTAKE = 1,
    LON_IGNORE = 2,
  };

  enum LatDecision {
    NUDGE = 0,
    LAT_IGNORE = 1,
  };

  enum NudgeType { LEFT_NUDGE = 0, RIGHT_NUDGE = 1 };

  int id;
  // Type type;
  LonDecision lon_decision;
  LatDecision lat_decision;
  NudgeType nudge_side;

  ObjSize obj_size;

  double noticed_timer{0.0};

  ObjPredSlice obj_state_init;
  std::vector<ObjPredSlice> pred_trajectory;
  std::vector<std::pair<double, ObjPredSlice>> pred_trajectory_with_t;

  // std::vector<ObsPrediction> polygon_list;
  // std::pair<double, ObsPrediction> object_with_t;
  // std::vector<double> distance_to_lane_line_history;
};

// struct Header {
//   uint32_t seq;
//   uint64_t stamp;
//   std::string frame_id;
// };

struct EgoState {
  double ego_vel;
  double ego_steer_angle;
  double ego_v_cruise;
  double ego_acc;
  gmp_interface::Pose2d ego_pose;
};

struct TrajectoryPointGMP {
  double x;
  double y;
  double s;
  double l;
  double t;
  double vel;

  void clear() {
    x = -1.0;
    y = -1.0;
    s = -1.0;
    l = -1.0;
    t = -1.0;
    vel = -1.0;
  }
};

struct TrajectoryGMP {
  std::vector<TrajectoryPointGMP> trajectory;
  int motion_type; // 1 LH 2 LC2R 3 LC2L; 5; LC_Wait; 6: LH_Emergency
  int lc_direc;    // 1: no lane change; 2: lc to right; 3: lc to left
};

struct DrivingTaskInfo {
  double lc_time_pass{0};
  double lc_time_remain{7};
  double lc_dist_remain{300};
  double lc_dist_pass{0};
  double lc_time_total{7};
  double lc_dist_total{300};
  double lc_latdist_abs_past{0.0};
  double lc_time_wait{0.0};
  bool lane_cross{false};
};

struct TargetPoint {
  double x{-100};
  double y{0};
  double heading{0.0};
  double vel{0.0};
  double dheading{0.0};
  int behavior_type{0};
};

struct BehaviorCandidate {
  std::vector<TargetPoint> target_points{};
  DrivingTaskInfo behavior_info{};
  LC_Dir behavior_type{LC_Dir::NONE};
  int behavior_index{-1};
  std::vector<double> c_poly{};
};

struct GeneralMotionPlannerOutput {
  std::vector<std::vector<TargetPoint>> target_points{};
  std::vector<double> speed_array{};
  std::vector<double> acce_array{};
  std::vector<double> t_array{};
  std::vector<TrajectoryGMP> trajectories_gmp{};
  bool gmp_valid{false};
  bool pp_state{false};
  bool sp_state{false};
  int lc_wait_speed_adjust_advice{0};
  int lc_wait_speed_adjust_count{0};
  double lc_wait_speed_advice{0.0};
  std::vector<std::vector<double>> speed_segments{};
  std::vector<gmp_interface::ObjInfoRefined> gap_info;
  std::vector<gmp_interface::ObjInfoRefined> cipv_info;
  std::vector<gmp_interface::ObjInfoRefined> lead_objs_info;
  std::vector<SVPoint> s_v_array;
  std::array<std::array<SVPoint, path_planner::QUADRATURE_ORDER>,
             speed_planner::NUM_SPEED_SEGMENTS>
      s_v_array_out;
  double lc_remain_dist{0.0};
  double lc_remain_time{0.0};
  int lc_action_state = 0;
  int lc_wait = 0;
  int motion_result = 0;
  int chosen_behavior_candidate_index = -1;
  double behavior_success_prob{0.0};
  double lc_in_proceed_time{0.0};
  double lc_to_static_obj_time{0.0};
};

struct GeneralMotionPlannerInput {
  double gmp_suggest_lc_time{7.0};
  double lc_duration{7.0};
  double veh_set_speed{25.0}; // m/s
  double headaway{2.0};
  double dheadaway{0.9};
  double headaway_smoothed{2.0};
  bool is_left_solid{false};
  bool is_right_solid{false};
  LC_Dir highlevel_guidance{LC_Dir::NONE};

  bool gmp_valid{false};
  bool pp_state{false};
  bool sp_state{false};

  std::vector<gmp_interface::ObsInfo> obj_infos{};
  gmp_interface::EgoState ego_state{};
  gmp_interface::EgoState ego_state_cart{};

  std::vector<gmp_interface::Point2d> refline_pos{};
  std::vector<double> refline_heading{};

  std::vector<Point2d> left_line;
  std::vector<Point2d> right_line;

  std::vector<BehaviorCandidate> behavior_candidates{};
  gmp_interface::DrivingTaskInfo lc_task_info{};
  bool valid_body{false};
  bool is_lane_stable{false};
  bool is_solid_line{false};
  bool has_olane{false};
  bool has_tlane{false};
  bool gmp_should_cancel{false};
  bool is_steer_over_limit{false};
  bool is_lca_state_activated{false};
  double target_speed_lc_wait_max{33.612};
  double total_time_offset{0.0};
  int driving_model_config{0};
  double one_lc_time{0.0};
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point2d, x, y)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose2d, x, y, theta)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SVPoint, s, v_min, v_max)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SafetyMargin, longitu, lateral)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjInfoRefined, id, id_relative, headaway,
                                   pos_init)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjState, pos, accel, vel, heading, s,
                                   v_frenet, rel_s)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TargetPoint, x, y, heading, vel, dheading)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DrivingTaskInfo, lc_time_pass,
                                   lc_time_remain, lc_dist_remain, lc_dist_pass,
                                   lc_time_total, lc_dist_total,
                                   lc_latdist_abs_past, lane_cross)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BehaviorCandidate, target_points,
                                   behavior_info, behavior_type, behavior_index)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPointGMP, x, y, s, l, t, vel)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryGMP, trajectory, motion_type,
                                   lc_direc)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    GeneralMotionPlannerOutput, target_points, speed_array, acce_array, t_array,
    trajectories_gmp, gmp_valid, pp_state, sp_state,
    lc_wait_speed_adjust_advice, lc_wait_speed_adjust_count,
    lc_wait_speed_advice, speed_segments, gap_info, cipv_info, lead_objs_info,
    s_v_array, s_v_array_out, lc_remain_dist, lc_remain_time, lc_action_state,
    lc_wait, motion_result, chosen_behavior_candidate_index,
    behavior_success_prob, lc_in_proceed_time, lc_to_static_obj_time)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjPredSlice, safety_margin, obj_state_local,
                                   obj_state_env, lane_assignment)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjSize, length, width)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsInfo, lon_decision, lat_decision,
                                   nudge_side, obj_size, noticed_timer,
                                   obj_state_init, pred_trajectory,
                                   pred_trajectory_with_t)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EgoState, ego_vel, ego_steer_angle,
                                   ego_v_cruise, ego_pose)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    GeneralMotionPlannerInput, lc_duration, gmp_suggest_lc_time, veh_set_speed,
    headaway, dheadaway, headaway_smoothed, highlevel_guidance, gmp_valid,
    pp_state, sp_state, obj_infos, ego_state, ego_state_cart, refline_pos,
    refline_heading, left_line, right_line, behavior_candidates, lc_task_info,
    valid_body, is_lane_stable, is_solid_line, has_olane, has_tlane,
    gmp_should_cancel, is_steer_over_limit, is_lca_state_activated,
    target_speed_lc_wait_max, total_time_offset, driving_model_config,
    one_lc_time)

} // namespace gmp_interface