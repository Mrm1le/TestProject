#pragma once
#include "nlohmann/json.hpp"
#include "planner_constants.hpp"
using json = nlohmann::json;

namespace path_planner {

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
  double theta_ref;
  double cos_theta_ref;
  double sin_theta_ref;
  // 1 : physical; 0 : virtual
  uint16_t left_road_border_type;
  uint16_t right_road_border_type;
  // 1 : solid; 0 : dashed
  uint16_t left_lane_border_type;
  uint16_t right_lane_border_type;
};

struct Point2d {
  double x;
  double y;
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
  int object_id;
  size_t type; // 0: vru, 1: vehicle
  bool ego_faster_truck;
  bool overlap_lane;
};

struct AvdAimInfo {
  int id;
  int type;
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
  AvdAimInfo left_aim_info;
  AvdAimInfo right_aim_info;
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
  bool pre_lc_wait{false};
  bool lc_end{false};
  int origin_lane_front_obs_id{-1000};
  Point2d prev_lc_end_point_fren;
};

struct IntersectionInfo {
  bool is_in_intersection{false};
  bool keep_in_intersection{false};
  double keep_in_intersection_timer;
  double dist_to_intersection;
  double dist_to_nonintersection;
};

struct ObsPrediction {
  ObsPrediction() = default;
  double rel_s = 0.0;
  double s = 0.0;
  double v = 0.0;
  double v_frenet = 0.0;
  double a = 0.0;
  double desired_headway;
  double stop_offset;
  double heading;
  std::vector<Point2d> polygon_cart;
  std::vector<Point2d> polygon_fren;
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

  int id{-1000};
  Type type;
  LonDecision lon_decision;
  LatDecision lat_decision;
  NudgeType nudge_side;
  ObsPrediction polygon_init;
  std::vector<std::vector<ObsPrediction>> polygon_list;
  std::vector<std::pair<double, ObsPrediction>> object_with_t;
  std::vector<double> distance_to_lane_line_history;
  double overlap;
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

struct DDPInfo {
  bool valid;
  std::vector<std::vector<Point2d>> ddp_path;
};

struct Intelligent_Dodge_Info {
  enum class State {
    Idle = 0,    // 空闲状态
    CoolingDown, // 冷却状态
    Control,     // 控制开启状态
  };
  State state{State::Idle};
  size_t elapsed_count{0};
  int dodge_truck_id{-1};
  double dodge_l{0.0};
  std::string avd_direction{"none"};
  size_t lc_count{0};
  size_t cp_count{0};
  size_t end_debounce_count{0};
  // for hmi
  size_t count_active_avd{0};
  std::string last_active_avd_state{"none"};
  int last_active_avd_obs_id{-1};
  int last_active_avd_obs_id_count{0};
  bool lc_end_clear_count{false};
  void reset() {
    state = State::Idle;
    elapsed_count = 0;
    dodge_truck_id = -1;
    dodge_l = 0.0;
    avd_direction = "none";
    end_debounce_count = 0;
    // for hmi
    count_active_avd = 0;
    last_active_avd_state = "none";
    last_active_avd_obs_id = -1;
    last_active_avd_obs_id_count = 0;
    lc_end_clear_count = false;
  }
};

struct DLPInfo {
  bool is_in_dlp{false};
};

struct PathPlannerInput {
  Header header;
  bool is_replan;
  bool lat_safety_improved;
  double ego_theta;
  std::pair<double, double> min_ref_jerk; // [min_ref_jerk_v, min_ref_jerk]
  PathPlannerPoint planning_init_state;
  std::vector<PathSegmentInfo> path_segments;
  PathTuningParams path_tuning_params;
  LaneChangeInfo lc_decider_info;
  IntersectionInfo intersec_info;
  std::vector<ObsInfo> obs_list;
  VehicleParam vehicle_param;
  DDPInfo ddp_info;
  bool lat_use_ld3d{false};
  Intelligent_Dodge_Info dodge_info;
  DLPInfo pre_dlp_info;
};

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

struct SolverReport {
  int type;
  int steps;
  double total_time;
  double init_cost;
  double final_cost;
  std::string msg;
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
    right_road_border, theta_ref, cos_theta_ref, sin_theta_ref,
    left_road_border_type, right_road_border_type, left_lane_border_type,
    right_lane_border_type)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point2d, x, y)
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
                                   left_aim_info, right_aim_info)
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
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsInfo, id, type, lon_decision,
                                   lat_decision, nudge_side, polygon_init,
                                   polygon_list, object_with_t,
                                   distance_to_lane_line_history, overlap)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Header, seq, stamp, frame_id)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VehicleParam, length, width, center_to_front)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DDPInfo, valid, ddp_path)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Intelligent_Dodge_Info, state, elapsed_count,
                                   dodge_truck_id, dodge_l, avd_direction,
                                   lc_count, cp_count, end_debounce_count)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DLPInfo, is_in_dlp)                                   
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPlannerInput, header, is_replan,
                                   lat_safety_improved, ego_theta,
                                   min_ref_jerk, planning_init_state,
                                   path_segments, path_tuning_params,
                                   lc_decider_info, intersec_info, obs_list,
                                   vehicle_param, ddp_info, lat_use_ld3d, 
                                   dodge_info, pre_dlp_info)
// debug info
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SampleDebugPoint, sample_data)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(NotebookDebug, s_to_samples, avd_direction,
                                   object_id, type, residual_debug,
                                   per_residual_per_param_gradient,
                                   total_gradients_per_param,
                                   lane_activation_dist)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SolverReport, type, steps, total_time,
                                   init_cost, final_cost, msg)

} // namespace path_planner
