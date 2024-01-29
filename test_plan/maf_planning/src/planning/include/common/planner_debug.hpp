#pragma once

#include "nlohmann/json.hpp"
#include "pnc/define/general_motion_planner_interface.hpp"
#include "pnc/define/geometry.h"
#include "pnc/define/obstacle_decider_interface.hpp"
#include "pnc/define/pass_intersection_input_interface.hpp"
#include "pnc/define/path_planner_interface.hpp"
#include "pnc/define/request_manager_interface.hpp"
using json = nlohmann::json;

namespace msquare {

struct ReplanReson {
  float time_and_pos_matched_point_dist;
  int position_matched_index;
  int time_matched_index;
  std::string reason_str;
};

struct ApfDebugInfo {
  std::vector<std::vector<std::vector<double>>> lanes_info;
  std::vector<std::vector<std::vector<double>>> road_edge_info;
  std::vector<std::vector<double>> origin_refline_points;
  std::vector<std::vector<double>> apf_refline_points;
  std::vector<std::vector<double>> pi_target_points;
  int pi_condition{0};
  int pi_attract_type{0};
  int pi_follow_car_id{0};
  double ego_pose_theta{0.0};
  double refline_origin_theta{0.0};
  double road_ref_theta{0.0};
  int apf_quit_cp{0};
  string apf_quit_cp_reason;
  double max_ego2refline_lat_dis{0.0};
  double lat_dis_2_refline{0.0};
  double lat_dis_2_in_inter{0.0};
};

struct CostTime {
  std::string task;
  double value;
};

struct VirtualLaneDebugInfo {
  int id;
  float intercept_l;
  float intercept_r;
};

struct GapObstacleDebugInfo {
  int id;
  float drel;
  float speed;
  float accel;
  // 0 not target; 1 lead one ; 2 rear one;
  int gap_info;
};

struct LatDecisionDebugInfo {
  std::string lc_back_reason;
  std::string lc_invalid_reason;
  std::string state_change_reason;
  std::string lc_request;
  std::string lc_request_source;
  bool is_target_in_solid;
  // -1:left  1:right  0:None
  int alc_request;
  int int_request;
  int ddp_request;
  int tlane_id;
  int olane_id;
  int flane_id;
  int left_lane_stable_cnt;
  int right_lane_stable_cnt;
  int lc_condition_detail;
  std::vector<GapObstacleDebugInfo> obstacle_gap;
  std::vector<VirtualLaneDebugInfo> lane_list;
};

struct PointXY {
  double x;
  double y;
};

struct ObjectBox {
  std::vector<PointXY> box;
};

struct LonObject {
  int id;
  int CrossLine;
  std::vector<ObjectBox> BoxList;
  int type;
};

struct LonDecisionDebugInfo {
  std::vector<PointXY> left_line;
  std::vector<PointXY> right_line;
  std::vector<PointXY> left_virtual_line;
  std::vector<PointXY> right_virtual_line;
  std::map<int, LonObject> object_list;
};

struct DDPDebugInfo {
  std::vector<int> ddp_trajectory_point_num;
};

//------------------------ GMP Debug -----------------------//

struct GeneralMotionPlannerDebugInfo {
  std::vector<PointXY> refline_pos;
  std::vector<double> refline_heading;
  std::vector<gmp_interface::TrajectoryGMP> trajectories_gmp;
};

//------------------------ GMP Debug -----------------------//

struct ObjMemInfo {
  double lat_dist_head;
  double obj_vy_dy;
  bool exist;
  bool collision;
  double collision_time;
  bool in_lane;
  bool follow_k1;
  int follow_count;
  bool cut_in_k1;
  int cut_in_count;
  bool rule_base_cutin;
  bool ftp_cutin;
  bool lat_cutin;
  bool is_merge_flag;
  double overlap;
  int type;
  double virtual_overlap;
};

struct IntelligentAvdInfo {
  int count_active_avd = 0;
  std::string last_active_avd_state = "none";
  int last_active_avd_obs_id = -1;
  int last_active_avd_obs_id_count = 0;
  bool lc_end_clear_count = false;
  int ego_faster_truck = 0;
  int overlap_lane = 0;
};

struct LatDebugInfo {
  int sample_s_type{1};
};

// planner debug info
struct PlannerDebug {
  ReplanReson replan_reason;
  ApfDebugInfo apf_debug_info;
  std::vector<CostTime> cost_time;
  path_planner::SolverReport lat_solver_report;
  path_planner::SolverReport lon_solver_report;
  LatDecisionDebugInfo lat_dec_info;
  LonDecisionDebugInfo lon_debug_info;
  DDPDebugInfo ddp_debug_info;
  GeneralMotionPlannerDebugInfo gmp_debug_info;
  gmp_interface::GeneralMotionPlannerInput gmp_input;
  gmp_interface::GeneralMotionPlannerOutput gmp_output;
  std::map<int, ObjMemInfo> obj_mem_info_map;
  IntelligentAvdInfo intelligent_avd_info;
  odc_interface::ObstacleDeciderOutput odc_output;
  LatDebugInfo lat_debug_info;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(IntelligentAvdInfo, count_active_avd,
                                   last_active_avd_state,
                                   last_active_avd_obs_id,
                                   last_active_avd_obs_id_count,
                                   lc_end_clear_count, ego_faster_truck,
                                   overlap_lane)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LatDebugInfo, sample_s_type)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ReplanReson, time_and_pos_matched_point_dist,
                                   position_matched_index, time_matched_index,
                                   reason_str)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ApfDebugInfo, lanes_info, road_edge_info,
                                   origin_refline_points, apf_refline_points,
                                   pi_target_points, pi_condition,
                                   pi_attract_type, pi_follow_car_id,
                                   ego_pose_theta, refline_origin_theta,
                                   road_ref_theta, apf_quit_cp,
                                   apf_quit_cp_reason, max_ego2refline_lat_dis,
                                   lat_dis_2_refline, lat_dis_2_in_inter)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(CostTime, task, value)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GapObstacleDebugInfo, id, drel, speed, accel,
                                   gap_info)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VirtualLaneDebugInfo, id, intercept_l,
                                   intercept_r)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LatDecisionDebugInfo, lc_back_reason,
                                   state_change_reason, lc_invalid_reason,
                                   lc_request, lc_request_source,
                                   is_target_in_solid, alc_request, int_request,
                                   ddp_request, tlane_id, olane_id, flane_id,
                                   left_lane_stable_cnt, right_lane_stable_cnt,
                                   obstacle_gap, lane_list)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PointXY, x, y)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjectBox, box)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LonObject, id, CrossLine, BoxList, type)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LonDecisionDebugInfo, left_line, right_line,
                                   left_virtual_line, right_virtual_line,
                                   object_list)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DDPDebugInfo, ddp_trajectory_point_num)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjMemInfo, lat_dist_head, obj_vy_dy, exist,
                                   collision, collision_time, in_lane,
                                   follow_k1, follow_count, cut_in_k1,
                                   cut_in_count, rule_base_cutin, ftp_cutin,
                                   lat_cutin, is_merge_flag, overlap, type,
                                   virtual_overlap)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GeneralMotionPlannerDebugInfo, refline_pos,
                                   refline_heading, trajectories_gmp)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PlannerDebug, replan_reason, apf_debug_info,
                                   cost_time, lat_solver_report,
                                   lon_solver_report, lat_dec_info,
                                   lon_debug_info, ddp_debug_info,
                                   obj_mem_info_map, intelligent_avd_info,
                                   odc_output, lat_debug_info, gmp_debug_info,
                                   gmp_input, gmp_output)
} // namespace msquare