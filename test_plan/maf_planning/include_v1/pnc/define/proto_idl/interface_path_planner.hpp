#ifndef CP_INTERFACE_PATH_PLANNER_H
#define CP_INTERFACE_PATH_PLANNER_H

#include <array>
#include <string>
#include <vector>
#include "interface_planner_common.hpp"

namespace cp_path_planner {

struct SpeedPlan {
  double v;
  double a;
  double j;
};

struct RefPointInfo {
  double time = 0.0;
  double x;
  double y;
  double curvature;
  double current_lane_width;
  double left_lane_width;
  double right_lane_width;
  double left_lane_border;
  double right_lane_border;
  double left_road_border;
  double right_road_border;
  double left_lidar_road_border;
  double right_lidar_road_border;
  double theta_ref;
  double cos_theta_ref;
  double sin_theta_ref;
  // 1 : physical; 0 : virtual
  uint16_t left_road_border_type;
  uint16_t right_road_border_type;
};

struct PathPoint {
  Point2d position_enu;
  double heading_yaw;
  double curvature;
  double path_follow_strength;
};

struct PathSampleInfo {
  double sample_s;
  PathPoint last_cart_traj;
  RefPointInfo refline_info;
  SpeedPlan speed_plan;
};

struct PathSegmentInfo {
  double end_segment_control_point_s;
  RefPointInfo end_segment_refline_info;
  std::vector<PathSampleInfo> quad_point_info;
};

struct PathTuningParams {
  int32_t max_num_iterations;
  double soft_boundary_scale;
  double hard_boundary_scale;
  double init_curvature_scale;
  double curvature_limit_scale;
  double lat_accel_scale;
  double lat_jerk_scale;
  double heading_scale;
  double ref_centering_scale;
  double prev_plan_centering_scale;
  double obstacle_constrain_scale;
  double obstacle_desire_scale;
  double obstacle_inflation_scale;
};

struct PathPlannerPoint {
  double s;
  double v;
  double t;
  double a;
  double da_ds;
  double x;
  double dx_ds;
  double d2x_ds2;
  double y;
  double dy_ds;
  double d2y_ds2;
  double l;
  double dl;
  double ddl;
  double jerk;
  double curvature;
  PathSampleInfo sample_info;
};

struct SampleDebugInfo {
  double x;
  double y;
  double offset;
  double curvature;
  double dcurv_dt;
  double lat_acc;
  double lat_jerk;

  double ref_offset;
  double left_activation_dist;
  double right_activation_dist;
  double left_map_hard_dist;
  double right_map_hard_dist;
  double left_obs_desire;
  double right_obs_desire;
  double left_obs_constrain;
  double right_obs_constrain;
};

struct AvdResultInfo {
  std::string avd_direction;
  int32_t object_id;
  int32_t type; // 0: vru, 1: vehicle
  bool ego_faster_truck = false;
  bool overlap_lane = false;
};

struct AvdAimInfo {
  int32_t id;
  int32_t type;
  std::string avd_direction;
  double a;
  double max_l;
  double min_l;
  double desire_buffer;
  double lane_border_base_l;
};

struct PathPlannerOutput {
  std::vector<PathPlannerPoint> path_planner_output;
  std::vector<SampleDebugInfo> debug_info_output;
  AvdResultInfo avd_result_info;
  AvdAimInfo dyn_left_aim_info;
  AvdAimInfo dyn_right_aim_info;
  AvdAimInfo static_left_aim_info;
  AvdAimInfo static_right_aim_info;
};

struct LaneChangeInfo {
  enum : uint8_t {
    LaneKeep = 1,
    LeftLaneChange = 2,
    RightLaneChange = 4,
    LaneChangeLeftBack = 8,
    LaneChangeRightBack = 16
  };
  uint8_t lc_status;
  double lc_status_time;
  bool is_lane_change;
  double t_pred;

  std::string lc_wait_dir;
  bool pre_lc_wait = false;
  bool lc_end = false;
  int32_t origin_lane_front_obs_id = -1000;
  Point2d prev_lc_end_point_fren;
};

struct IntersectionInfo {
  bool is_in_intersection = false;
  bool keep_in_intersection = false;
  double keep_in_intersection_timer;
  double dist_to_intersection;
  double dist_to_nonintersection;
};

struct ObsPrediction {
  double rel_s = 0.0;
  double s = 0.0;
  double v = 0.0;
  double v_frenet = 0.0;
  double a = 0.0;
  double desired_headway = 0.0;
  double stop_offset = 0.0;
  double heading = 0.0;
  std::vector<Point2d> polygon_cart;
  std::vector<Point2d> polygon_fren;
};

struct PairDoubleObsPrediction {
  double first = 0;
  ObsPrediction second;
  bool __convert_to_list___ = true;
};

struct VectorObsPrediction {
  std::vector<ObsPrediction> vec;
  bool __convert_to_list___ = true;
};

struct ObsInfo {
  enum Type { 
    PEDESTRIAN = 0, 
    OFO = 1, 
    COUPE = 2, 
    TRANSPORT_TRUNK = 4,
    NOT_KNOW = 3,
    BUS = 5,
    ENGINEER_TRUCK = 6,
    TRICYCLE = 7,
    CONE_BUCKET = 8,
    STOP_LINE = 9,
    GATE = 10,
    FREESPACE = 11 };

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

  int32_t id = -1000;
  uint8_t type;
  uint8_t lon_decision;
  uint8_t lat_decision;
  uint8_t nudge_side;
  ObsPrediction polygon_init;
  std::vector<VectorObsPrediction> polygon_list;//no?yes
  std::vector<PairDoubleObsPrediction> object_with_t;//no?yes
  std::vector<double> distance_to_lane_line_history;
  double overlap;
  bool overtake_decider_static;
  bool is_static_from_decider;
};

struct Header {
  uint32_t seq;
  uint64_t stamp;
  std::string frame_id;
};

struct VehicleParam {
  double length;
  double width;
  double center_to_front;
};

struct VectorDDPTrajectoryPoint {
  std::vector<Point2d> vec;
  bool __convert_to_list___ = true;
};

struct DDPInfo {
  bool valid;
  std::vector<VectorDDPTrajectoryPoint> ddp_path;//no?yes
};

struct Intelligent_Dodge_Info {
  enum State {
    Idle = 0,    // 空闲状态
    CoolingDown = 1, // 冷却状态
    Control = 2     // 控制开启状态
  };
  uint8_t state = 0;
  uint32_t elapsed_count = 0;
  int32_t dodge_truck_id = -1;
  double dodge_l = 0.0;
  std::string avd_direction = "none";
  uint32_t lc_count = 0;
  uint32_t cp_count = 0;
  uint32_t end_debounce_count = 0;
  // for hmi
  uint32_t count_active_avd = 0;
  std::string last_active_avd_state = "none";
  int32_t last_active_avd_obs_id = -1;
  int32_t last_active_avd_obs_id_count = 0;
  bool lc_end_clear_count = false;
};

struct MinRefJerk {
  double first = 0;
  double second = 0;
  bool __convert_to_list___ = true;
};

struct DLPInfo {
  bool is_in_dlp{false};
};

struct PathPlannerInput {
  Header header;
  bool is_replan;
  bool lat_safety_improved;
  double ego_theta;
  MinRefJerk min_ref_jerk; // [min_ref_jerk_v, min_ref_jerk]ok
  PathPlannerPoint planning_init_state;
  std::vector<PathSegmentInfo> path_segments;
  PathTuningParams path_tuning_params;
  LaneChangeInfo lc_decider_info;
  IntersectionInfo intersec_info;
  std::vector<ObsInfo> obs_list;
  VehicleParam vehicle_param;
  DDPInfo ddp_info;
  bool lc_end = false ;
  std::string lc_wait_dir;
  bool pre_lc_wait = false;
  bool lat_use_ld3d = false;
  double lane_border_avd_dist = 0.3;
  bool enable_extend_rb_backward = false;
  Intelligent_Dodge_Info dodge_info;
  DLPInfo pre_dlp_info;
  bool __add_to_json___ = true;
};

struct PairIntVectorDouble{
  int32_t first = 0;
  cp_path_planner::VectorDouble second;
  bool __convert_to_list___ = true;
};

struct LateralMotionPlannerOutput {
  Point2d prev_lc_end_point;
  std::vector<PairIntVectorDouble> lat_avoid_obstacle_history_info_map;//no
  // std::unordered_map<int, std::vector<double>>
  //     lat_avoid_obstacle_history_info_map;
  std::vector<cp_path_planner::SampleDebugInfo> path_planner_debug_info;
  cp_path_planner::PathPlannerOutput prev_path_planner_output;
  cp_path_planner::PathPlannerOutput path_planner_output;
  cp_path_planner::Intelligent_Dodge_Info pre_dodge_info;
  cp_path_planner::DLPInfo pre_dlp_info;
};

struct SolverReport {
  int32_t type;
  int32_t steps;
  double total_time;
  double init_cost;
  double final_cost;
  std::string msg;
};


} // namespace path_planner

#endif