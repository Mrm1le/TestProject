#ifndef CP_INTERFACE_PLANNER_DEBUG_H
#define CP_INTERFACE_PLANNER_DEBUG_H

#include <stdint.h>
#include <limits>
#include <string>
#include <vector>

#include "interface_planner_common.hpp"
#include "interface_general_motion_planner.hpp"
#include "interface_obstacle_decider.hpp"
#include "interface_path_planner.hpp"
#include "interface_pass_intersection_input.hpp"

namespace cp {

struct PointXY {
  double x;
  double y;
};

struct ObjMemInfo {
  double lat_dist_head = 0.0;
  double obj_vy_dy= 0.0;
  bool exist =false;
  bool collision=false;
  double collision_time= 0.0;
  bool in_lane=false;
  bool follow_k1=false;
  int32_t follow_count = 0;
  bool cut_in_k1=false;
  int32_t cut_in_count= 0;
  bool rule_base_cutin=false;
  bool ftp_cutin = false;
  bool lat_cutin = false;
  bool is_merge_flag=false;
  double overlap= 0.0;
  int32_t type= 0;
  double virtual_overlap= 0.0;
};

struct IntelligentAvdInfo {
  int32_t count_active_avd = 0;
  std::string last_active_avd_state = "none";
  int32_t last_active_avd_obs_id = -1;
  int32_t last_active_avd_obs_id_count = 0;
  bool lc_end_clear_count = false;
  int32_t ego_faster_truck = 0;
  int32_t overlap_lane = 0;
};

struct LatDebugInfo {
  int32_t sample_s_type = 1;
};

struct LineInfo {
  int32_t relative_id;
  int32_t track_id;
  std::vector<PointXY> line_enu_pts;
};

struct CumuLatDisInfo {
  int32_t relative_id = -100;
  int32_t track_id = -100;
  double total_cost = 0.0;
  double cumu_lat_dis = 0.0;
  double cumu_lat_dis_front = 0.0;
  double cumu_lat_dis_back = 0.0;
  int32_t cumu_lat_dis_front_count = 0;
  int32_t cumu_lat_dis_back_count = 0;
  double crosslane_cost = 0;
  bool is_neighbor = true;
  bool cross_with_llb = false;
  bool cross_with_rlb = false;
};

struct EgoLaneDebugInfo {
  int32_t clane_relative_id = -100;
  int32_t clane_track_id = -100;
  int32_t left_lb_track_id = -1;
  int32_t right_lb_track_id = -1;
  double ego2lane_min = -1.;
  double clane_min_diff_total = -1.;
  std::vector<CumuLatDisInfo> cumu_lat_dis_info_list;
  std::vector<PointXY> ego_lane;
  std::string eld_type = "none";
  LineInfo last_plan_result;
  std::vector<LineInfo> lane_center_info_list;
  std::vector<LineInfo> lane_line_info_list;
};
struct EgoLaneDeciderInfo {
  EgoLaneDebugInfo ddlane_debug_info;
  EgoLaneDebugInfo ddld_debug_info;
  int32_t ddlane_num = -1;
  int32_t ddld_lane_num = -1;
};

struct PairIntObjMemInfo {
  int32_t first;
  ObjMemInfo second;
};

struct ReplanReson {
  float time_and_pos_matched_point_dist = 0.0;
  int32_t position_matched_index = 0;
  int32_t time_matched_index = 0;
  std::string reason_str;
  bool __add_to_json__ = true;
};


struct CostTime {
  std::string task;
  double value;
};

struct ApfDebugInfo {
  std::vector<cp_path_planner::VectorVectorDouble> lanes_info;
  std::vector<cp_path_planner::VectorVectorDouble> road_edge_info;
  std::vector<cp_path_planner::VectorDouble> origin_refline_points;
  std::vector<cp_path_planner::VectorDouble> apf_refline_points;
  std::vector<cp_path_planner::VectorDouble> pi_target_points;
  int32_t pi_condition = 0;
  int32_t pi_attract_type = 0;
  int32_t pi_follow_car_id = 0;
  double ego_pose_theta = 0.0;
  double refline_origin_theta = 0.0;
  double road_ref_theta = 0.0;
  int32_t apf_quit_cp = 0;
  std::string apf_quit_cp_reason;
  double max_ego2refline_lat_dis = 0.0;
  double lat_dis_2_refline = 0.0;
  double lat_dis_2_in_inter = 0.0;
  int32_t left_lb_index = -1;
  int32_t right_lb_index = -1;
  std::vector<cp_pass_intersection_planner::CartesianQuintic>
      cartesian_quintic_poly_vector;
};


struct VirtualLaneDebugInfo {
  int32_t id;
  float intercept_l;
  float intercept_r;
};

struct GapObstacleDebugInfo {
  int32_t id;
  float drel;
  float speed;
  float accel;
  // 0 not target; 1 lead one ; 2 rear one;
  int32_t gap_info;
};

struct LatDecisionDebugInfo {
  std::string lc_back_reason;
  std::string lc_invalid_reason;
  std::string state_change_reason;
  std::string lc_request;
  std::string lc_request_source;
  bool is_target_in_solid;
  // -1:left  1:right  0:None
  int32_t alc_request;
  int32_t int_request;
  int32_t ddp_request;
  int32_t tlane_id;
  int32_t olane_id;
  int32_t flane_id;
  int32_t left_lane_stable_cnt;
  int32_t right_lane_stable_cnt;
  int32_t lc_condition_detail;
  std::vector<GapObstacleDebugInfo> obstacle_gap;
  std::vector<VirtualLaneDebugInfo> lane_list;
};


struct ObjectBox {
  std::vector<PointXY> box;
};

struct LonObject {
  int32_t id;
  int32_t crossline;
  std::vector<ObjectBox> boxlist;
  int32_t type;
};

struct PairLonObject {
  int32_t first;
  LonObject second;
  bool __convert_to_list___ = true;
};
struct LonDecisionDebugInfo {
  std::vector<PointXY> left_line;
  std::vector<PointXY> right_line;
  std::vector<PointXY> left_virtual_line;
  std::vector<PointXY> right_virtual_line;
  std::vector<PairLonObject> object_list;
};

struct DDPDebugInfo {
  std::vector<int32_t> ddp_trajectory_point_num;
};


struct LaneFusionDeciderInfo {
  PointXY ddlane_splice_pt;
  PointXY ddld_splice_pt;
};

struct SparseDebugData{
  int32_t ddlane_cut_off{0};
  int32_t lc_splice{0};
  int32_t cross_boundary_risk{0};
  int32_t target_lc_track_id{0};
  int32_t cross_lb_num{0};
  int32_t cross_counter{0};
  int32_t cross_lb_most_track_id{-1};
  int32_t road_boundary_collision_risk{0};
  int32_t collision_cnt{0};
  float min_s{0.0};
  float min_dist{0.0};
  int32_t has_collision{0};
  int32_t linger_count{0};
  int32_t clear_invalid_lc{0};
  int32_t lc_linger{0};
  int32_t is_merge{0};
  int32_t is_merge_count{0};
  int32_t inter_status{0};
};

// planner debug info
struct PlannerDebug {
  ReplanReson replan_reason;
  ApfDebugInfo apf_debug_info;
  std::vector<CostTime> cost_time;
  cp_path_planner::SolverReport lat_solver_report;
  cp_path_planner::SolverReport lon_solver_report;
  LatDecisionDebugInfo lat_dec_info;
  LonDecisionDebugInfo lon_debug_info;
  DDPDebugInfo ddp_debug_info;
  cp_gmp_interface::GeneralMotionPlannerInput gmp_input;
  cp_gmp_interface::GeneralMotionPlannerOutput gmp_output;
  std::vector<PairIntObjMemInfo> obj_mem_info_map;
  IntelligentAvdInfo intelligent_avd_info;
  cp_odc_interface::ObstacleDeciderOutput odc_output;
  LatDebugInfo lat_debug_info;
  EgoLaneDeciderInfo ego_lane_decider_info;
  LaneFusionDeciderInfo lane_fusion_decider_info;
  SparseDebugData sparse_debug_data;
};

}  // namespace cp

#endif
