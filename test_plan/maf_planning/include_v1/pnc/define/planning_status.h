#ifndef CP_COMMON_PLANNING_STATUS_
#define CP_COMMON_PLANNING_STATUS_

#include "maf_interface/maf_planning.h"
#include "msd/cp_planning.h"
#include "pnc/define/geometry.h"
#include "nlohmann/json.hpp"
#include "mtime_core/mtime.h"
#include "mjson/mjson.hpp"
#include <chrono>
#include <string>
#include <unordered_map>
#include <set>
#include <vector>

namespace cp {

typedef struct {
  std::vector<int> target_lane_cars;
  double most_front_car_vel{100.0};
  double min_gap_length{0.0};
  double max_gap_length{0.0};
} TrafficFlowInfo;

typedef struct {
  enum Status {
    CHANGE_LANE_PREPARATION = 0, // before change lane state
    IN_CHANGE_LANE = 1,          // during change lane state
    CHANGE_LANE_BACK = 2,        // lane change back
    CHANGE_LANE_FAILED = 3,      // change lane failed
    CHANGE_LANE_FINISHED = 4,    // change lane finished
  };
  Status status{CHANGE_LANE_PREPARATION};
  int path_id;
  bool is_active_lane_change{false};
  std::string direction{"none"};
  double start_timestamp;
  bool exist_lane_change_start_position{false};
  Point2D lane_change_start_position;
  double last_succeed_timestamp;
  bool is_current_opt_succeed{false};
  // obs to follow and obs to overtake in target lane
  // which together determine where the gap is
  double gap_length_before_change{0.0};
  std::pair<int, int> target_gap_obs{-10, -10};
  TrafficFlowInfo target_lane_traffic_flow;
  bool enable_gap_protection{false};
  int gap_protection_counter{0};
  bool set_virtual_obstacle{false};
  double virtual_obstacle_distance{0.0};
  int origin_lane_leader_id{0};
  double left_dash_line_length;
  double lane_change_wait_time{0.0};
  bool enable_interactive_mode{false};
  bool enable_lane_change_traj_checker{false};
  int rival_obstacle_id{0};
} ChangeLaneStatus;

typedef struct {
  enum Status {
    LANE_KEEP = 1,   // during change lane state
    LANE_CHANGE = 2, // change lane failed
    LANE_BORROW = 3, // change lane finished
  };
  Status status{LANE_KEEP};
  ChangeLaneStatus change_lane;
  int target_lane_id = 0;
  int target_lane_map_id;
  double target_lane_lat_offset;
} LaneStatus;

typedef struct {
  int id;
  double start_stopstamp;
} StopTime;

typedef struct {
  std::string initial_direction{"none"};
  std::string current_direction{"none"};
  double start_stopstamp;
} StopInfo;

typedef struct {
  int crosswalk_id;
  std::unordered_map<int, double> stop_times; // stop before pedestrain
  // std::vector<StopTime> stop_times;
  std::unordered_map<int, StopInfo> stopline_times; // stop before crosswalk
} CrosswalkStatus;

typedef struct {
  double last_rerouting_time;
  bool need_rerouting;
  std::string routing_request;
} ReroutingStatus;

typedef struct {
  std::unordered_map<int, bool> junction;
} RightOfWayStatus;

typedef struct {
  std::string scenario_type;
  std::string stage_type;
} ScenarioStatus;

typedef struct {
  std::string traffic_light_status;
} TrafficLightStatus;

typedef struct {
  double prebrake_acc;
  double prebrake_duration;
  double preacc_duration;
  bool enable_prebrake;
  bool enable_preacc;
  int n_prebrake_slip;
  int n_prebrake_slow;
  int n_preacc;
  double v_lim_curv;
  double a_lim_curv;
  int n_prebrake_curv;
} PreActionResults;

struct AvdInfo {
  int priority;
  int ignore_loop;
  bool lon_ignore;
  double time_buffer;
};

struct ObstacleDisInfo {
  int obs_id{0};
  double time{0.0};
  double distance{std::numeric_limits<double>::max()};
};

struct TrajCollisionCheckInfo {
  ObstacleDisInfo min_obs_dis;
  ObstacleDisInfo min_obs_lat_dis;
  ObstacleDisInfo min_obs_lon_dis;
};

struct PlanningResult{
  PlanningResult(){
    traj_pose_array.reserve(210);
    traj_vel_array.reserve(210);
    traj_acceleration.reserve(210);
  };

  double timestamp_sec;
  double next_timestamp_sec;

  float lon_error = 0.0F;
  float lat_error = 0.0F;

  float v_target = 0.0F;
  float a_target = 0.0F;
  std::vector<float> v_array;
  std::vector<float> a_array;

  // traffic light
  int traffic_light_state = 0;
  bool stop_flag = false;
  bool is_passed_stop_line = false;
  double dist_to_stop = 0.0;
  
  // lateral static object overtake decision
  std::set<int> static_obstacles_from_decider;
  std::vector<int> static_obstacle_overtake_list;
  std::map<int, double> overtake_obstacles_map;
  int block_to_follow = -100;
  std::map<int, double> lat_block_obs;
  
  // decision
  std::vector<uint32_t> lon_follow_obstacles;
  std::vector<uint32_t> lon_overtake_obstacles;
  std::vector<uint32_t> lat_nudge_obstacles;
  std::unordered_map<int, AvdInfo> avd_info;
  std::unordered_map<int, int> yield_history;
  TrajCollisionCheckInfo traj_collision_info;

  // TODO: discriminate diffenent type of trajectory to judge if
  // the corresponding result is approvable amd select the best solution
  std::string matched_scenario_type;

  // longitudinal debug
  int flag_closelead;
  int flag_invalid;
  int flag_type0;
  int flag_softbrake;
  int flag_fastcutin;
  double v_set;
  double v_set_obs;
  double a_set;
  double ds_set;
  double t_set;
  double v_limit;
  double v_limit_map;
  double v_limit_map_current;
  int num_yield;
  int N_hard;
  double vl_yield;
  double dsl_yield;
  int type_yield;
  int id_yield;
  int tag_yield;
  double v_overtake;
  double dsl_overtake;
  int type_overtake;
  int id_overtake;
  int type_merge;
  double dis2merge;
  double dis2cross;
  double cutin_score;
  double probability;
  double lon_weights[4]{};
  bool NeedAEB;
  bool pnc_start{false};
  bool pnc_stop{false};
  double steer_angle;
  double steer_rate;
  double dis_close;
  double ds_lon_front;
  double ttc_lon_front;
  double dt_lon_front;
  double ds_lon_rear;
  double ttc_lon_rear;
  double dt_lon_rear;
  std::vector<int> id_nudge;
  std::vector<double> ds_yield_seq;
  std::vector<double> d_safe_seq;
  std::vector<double> ds_flag_seq;
  std::vector<double> v_ref_seq;

  // lateral motion debug
  int lat_plan_status;
  int static_steer_loops{0};
  double angle_steers_static;
  double lat_behavior_offset_premoving; // add by xhy
  std::vector<int> id_clear;
  std::vector<double> curv_rate;
  
  int prev_lc_status{0};
  int lc_status_loop{0};

  std::vector<double> acc_output;
  std::vector<double> jerk_output;
  double min_acc;
  double max_acc;
  double min_jerk;
  double max_jerk;

  // traj
  std::vector<maf_planning::PathPoint> traj_pose_array;
  std::vector<maf_planning::VelocityPoint> traj_vel_array;
  std::vector<double> traj_acceleration;
  std::vector<Point2D> traj_pose_frenet;

  // avd in lane info
  int lane_avd_in_lane_first_id = -1;
  int lane_avd_in_lane_first_dir = -1;
  int lane_avd_in_lane_first_type = 0;
  int ego_faster_truck = 0;
  int overlap_lane = 0;
  int avd_in_lane = -1;
  std::vector<std::tuple<int, std::string>> avoid_in_lane_info;

  // last frenet_enu_points for ddmap
  std::vector<maf_planning::PathPoint> last_frenet_enu_points;
  std::vector<maf_planning::PathPoint> last_offsets_filter;
  double last_ego_l = 0.;
  int ego_off_refline_n = 0;
  int enter_auto_driving_n = 100;

  // refline generator
  int condition_attract_type_n = 0;

  maf_planning::TurnSignal turn_signal_cmd{
      .value = maf_planning::TurnSignal::NONE};
  maf_planning::Gear gear_cmd{
      .value = maf_planning::Gear::DRIVE};

  // for control
  std::string extra_json;
  nlohmann::json extra_json_raw;
  mjson::Json debug_json;
  // traffic light control result, will send them to HMI and Mviz
};

typedef struct {
  bool enable_cp_tfl_from_hmi{true}; // switch ON or OFF: HMI -> MFF -> Planning
  bool driver_override_from_mff{false};
  bool has_lead_one{false}; 
  bool make_virtual_obs{false};
  bool is_yellow_blinking{false}; // slow down to 80% of v_cuise at yellow blinking intersection
  
  // below variables exposed to HMI and Mviz
  bool has_traffic_light{false};   // 是否收到红绿灯信号: TFL Perception -> Planning -> HMI
  double distance_to_stopline{9999.0};  // DDLD给Planning的停止线距离，如果上一条“has_traffic_light”为ture，HMI需要提示司机前方多少米有红绿灯。
  bool is_need_driver_confirm_pass{false};  //Planning根据Planning一开始的决策结果和实际红绿灯检测结果，通过HMI提示司机需要踩油门进行pass的确认。（如，Planning一开始的决策为停止，实际红绿灯检测为绿灯，需要司机踩油门确认通过）。
  bool is_planning_decision_pass{true};  //Planning结合红绿灯信息和司机交互信息的最终决策结果：true为通过，对应HMI显示为绿色停止线。false为不通过，对应HMI为红色停止线。
  bool is_passed_stopline;  //ego car是否通过停止线，true的时候HMI的停止线可以不再显示
  int stopline_id; // stopline_id, which makes ego car stops
}TrafficLightControlInfo;

typedef struct {
  std::vector<maf_planning::PathPoint> traj_pose_array;
  std::vector<maf_planning::VelocityPoint> traj_vel_array;
  std::vector<double> traj_acceleration;
} PublishedTrajectory;

typedef enum {
  PRIMARY,
  SECONDARY,
  BACKUP,
} SchemeStage;

typedef struct {
  int64_t planning_loop = 0;
  bool last_planning_success = false;
  bool planning_success = false;
  bool use_stitching_trajectory{true};
  double v_limit{40.0};
  double a_limit{1.0};
  PreActionResults pre_action;
  LaneStatus lane_status;
  CrosswalkStatus crosswalk;
  ReroutingStatus rerouting;
  RightOfWayStatus right_of_way;
  ScenarioStatus scenario;
  TrafficLightStatus traffic_light;
  PlanningResult planning_result;
  PlanningResult pre_planning_result;
  SchemeStage scheme_stage;
  int backup_consecutive_loops = 0;
  std::string backup_reason = "none";
  double time_consumption = 0.0;
  std::string trigger_msg_id;
  TrafficLightControlInfo traffic_light_control_info;
} PlanningStatus;

struct PathPose {
  Point3D pos;
  Quaternion orient;
};

struct OpenSpacePath {
  std::vector<PathPose> path_poses;
};

struct SquareMap {
  Point2D box_corners[4];
  double box_height = 0.0;
};

struct AimedPoi {
  int id = 0;
  std::string type;
};

} // namespace cp
#endif // COMMON_PLANNING_STATUS_
