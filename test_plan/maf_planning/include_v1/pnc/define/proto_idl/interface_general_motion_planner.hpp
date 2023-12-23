#ifndef CP_INTERFACE_GMP_H
#define CP_INTERFACE_GMP_H

#include <array>
#include <string>
#include <vector>
#include "interface_planner_common.hpp"

namespace cp_gmp_interface {

struct LCInfo {
  enum LC_Dir { NONE = 1, LEFT = 2, RIGHT = 3 };
};

struct SVPoint {
  double s = 0;
  double v_min = 0;
  double v_max = 0;
  double v_ref = 0;
};

struct ObjInfoRefined {
  int32_t id = -1;
  int32_t id_relative = -1;
  double headaway = 2.0;
  cp_path_planner::Pose2d pos_init;
};

struct ObjSize {
  double length = 0;
  double width = 0;
};

struct ObjMarginInfo {
  std::vector<cp_path_planner::SafetyMargin> pred_sm;
};

struct AllObjsMarginInfo {
  std::vector<ObjMarginInfo> all_objs_pred_sm;
};

struct ObsInfoGMP {
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

  int32_t id = 0;
  uint8_t lon_decision = 0;  // useless
  uint8_t lat_decision = 0;  // useless
  uint8_t nudge_side = 0;    // useless

  ObjSize obj_size;

  double noticed_timer = 0.0;

  cp_path_planner::ObjPredSlice obj_state_init;
  std::vector<cp_path_planner::ObjPredSlice> pred_trajectory;
};

struct TrajectoryPointGMP {
  double x = 0;
  double y = 0;
  double s = 0;
  double l = 0;
  double t = 0;
  double vel = 0;
};

struct TrajectoryGMP {
  std::vector<TrajectoryPointGMP> trajectory;
  int32_t motion_type = 1;  // 1 LH 2 LC 3 LC; 5; LC_Wait; 6: LH_Emergency
  int32_t lc_direc = 1;     // 1: no lane change; 2: lc to right; 3: lc to left
};

struct DrivingTaskInfo {
  double lc_time_pass = 0;
  double lc_time_remain = 7;
  double lc_dist_remain = 300;
  double lc_dist_pass = 0;
  double lc_time_total = 7;
  double lc_dist_total = 300;
  double lc_latdist_abs_past = 0.0;
  double lc_time_wait = 0.0;
  bool lane_cross = false;
};

struct TargetPoint {
  double x = -100;
  double y = 0;
  double heading = 0.0;
  double vel = 0.0;
  double dheading = 0.0;
  int32_t behavior_type = 0;
};

struct BehaviorCandidate {
  std::vector<TargetPoint> target_points;
  DrivingTaskInfo behavior_info;
  uint8_t behavior_type = 1;  // LCInfo::LC_Dir::NONE
  int32_t behavior_index = -1;
  std::vector<double> c_poly;
};

struct VectorTargetPoint {
  std::vector<TargetPoint> vec;
  bool __convert_to_list___ = true;
};

struct ArraySVPoint {
  std::array<SVPoint, COMMON_QUADRATURE_ORDER> arr;
  bool __convert_to_list___ = true;
};

struct GeneralMotionPlannerOutput {
  std::vector<VectorTargetPoint> target_points;
  std::vector<double> speed_array;
  std::vector<double> acce_array;
  std::vector<double> t_array;
  std::vector<TrajectoryGMP> trajectories_gmp;
  bool gmp_valid = false;
  bool pp_state = false;
  bool sp_state = false;
  int32_t lc_wait_speed_adjust_advice = 0;
  int32_t lc_wait_speed_adjust_count = 0;
  double lc_wait_speed_advice = 0.0;
  std::vector<cp_path_planner::VectorDouble> speed_segments;
  std::vector<cp_gmp_interface::ObjInfoRefined> gap_info;
  std::vector<cp_gmp_interface::ObjInfoRefined> cipv_info;
  std::vector<cp_gmp_interface::ObjInfoRefined> lead_objs_info;
  std::vector<SVPoint> s_v_array;
  std::array<ArraySVPoint,COMMON_NUM_SPEED_SEGMENTS> s_v_array_out;
  double lc_remain_dist = 0.0;
  double lc_remain_time = 0.0;
  int32_t lc_action_state = 0;
  int32_t lc_wait = 0;
  int32_t motion_result = 0;
  int32_t chosen_behavior_candidate_index = -1;
  double behavior_success_prob = 0.0;
  double lc_in_proceed_time = 0.0;
  double lc_to_static_obj_time = 0.0;
};

struct GeneralMotionPlannerInput {
  double lc_duration = 7.0;
  double veh_set_speed = 25.0;  // m/s
  double headaway = 2.0;
  double dheadaway = 0.9;
  double headaway_smoothed = 2.0;
  bool is_left_solid = false;
  bool is_right_solid = false;
  uint8_t highlevel_guidance = 1;  //{LCInfo::LC_Dir::NONE};
  double gmp_suggest_lc_time = 7.0;

  bool gmp_valid = false;
  bool pp_state = false;
  bool sp_state = false;

  std::vector<ObsInfoGMP> obj_infos;
  cp_path_planner::EgoState ego_state;
  cp_path_planner::EgoState ego_state_cart;

  std::vector<cp_path_planner::Point2d> refline_pos;
  std::vector<double> refline_heading;

  std::vector<cp_path_planner::Point2d> left_line;
  std::vector<cp_path_planner::Point2d> right_line;

  std::vector<BehaviorCandidate> behavior_candidates;
  cp_gmp_interface::DrivingTaskInfo lc_task_info;
  bool valid_body = false;
  bool is_lane_stable = false;
  bool is_solid_line = false;
  bool has_olane = false;
  bool has_tlane = false;
  bool gmp_should_cancel = false;
  bool is_steer_over_limit = false;
  bool is_lca_state_activated = false;
  double target_speed_lc_wait_max = 33.612;
  double total_time_offset = 0.0;
  int32_t driving_model_config = 0;
  double one_lc_time = 0.0;
};

} // namespace cp_gmp_interface

#endif