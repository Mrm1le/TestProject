#ifndef COMMON_PLANNING_CONTEXT_
#define COMMON_PLANNING_CONTEXT_

#include "common/ldp_result.h"
#include "common/math/math_utils.h"
#include "common/obstacle_decision_manager.h"
#include "common/parking_lot.h"
#include "common/path/path_boundary.h"
#include "common/planner_debug.hpp"
#include "common/speed/speed_data.h"
#include "common/speed/speed_limit.h"
#include "common/utils/macro.h"
#include "mlog_core/mlog_data_stream.h"
#include "nlohmann/json.hpp"
#include "parking_planner_types.h"
#include "planner/message_type.h"
#include "pnc.h"
#include "common/grid_map/grid_map.h"


using json = nlohmann::json;
// #include "pnc/define/parking_vehicle_report.h"
namespace msquare {

constexpr int kBackUpFreezeLoop = 3;

enum class BehaviorType {
  flagCloseSlowLeader,
  flagCloseFastLeader,
  flagCloseSlowPotentialLeader,
  flagNoLeader,
  flagSlowPotentialLeader,
  flagSlowLeader,
  flagFastLeader
};

struct LongitudinalBehaviorPlannerOutput {
  BehaviorType lane_keeping_behavior_type;
  double cost_weight_lane_keeping[4];
  int cipv_id;
};

struct POMDPPlannerOutput {
  // bool enable_interactive_mode{false};
  // bool enable_lane_change_traj_checker{false};
  int last_result_time_counter{100};
  bool prediction_revised{false};
  bool valid_lane_change_solution{false};
  int rival_obstacle_id{-300};
  SpeedData ego_speed_data;
  SpeedData rival_speed_data;
  std::vector<std::pair<double, FrenetState>> ego_traj;
  std::vector<std::pair<double, FrenetState>> rival_traj;
};

class PlanningContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext);

public:
  struct FallBackInfo {
    std::string last_successful_path_label;
  };

public:
  const FallBackInfo &fallback_info() const { return fallback_info_; }
  FallBackInfo *mutable_fallback_info() { return &fallback_info_; }

  const PlanningStatus &planning_status() const { return planning_status_; }
  PlanningStatus *mutable_planning_status() { return &planning_status_; }

  const PlanningStatus &prev_planning_status() const {
    return prev_planning_status_;
  }
  PlanningStatus *mutable_prev_planning_status() {
    return &prev_planning_status_;
  }

  path_planner::PathPlannerInput *mutable_path_planner_input() {
    return &path_planner_input_;
  }
  const path_planner::PathPlannerInput &path_planner_input() const {
    return path_planner_input_;
  }

  PlannerDebug *mutable_planner_debug() { return &planner_debug_; }
  const PlannerDebug &planner_debug() const { return planner_debug_; }

  std::string *mutable_ddp_model_input() { return &ddp_model_input_; }
  const std::string &ddp_model_input() const { return ddp_model_input_; }

  pass_intersection_planner::PassIntersectionPlannerInput *
  mutable_pass_intersection_planner_input() {
    return &pass_intersection_planner_input_;
  }
  const pass_intersection_planner::PassIntersectionPlannerInput &
  pass_intersection_planner_input() const {
    return pass_intersection_planner_input_;
  }

  speed_planner::SpeedPlannerInput *mutable_speed_planner_input() {
    return &speed_planner_input_;
  }
  const speed_planner::SpeedPlannerInput &speed_planner_input() const {
    return speed_planner_input_;
  }

  speed_planner::LonDecisionOutput *mutable_lon_decison_output() {
    return &lon_decison_output_;
  }
  const speed_planner::LonDecisionOutput &lon_decison_output() const {
    return lon_decison_output_;
  }

  speed_planner::SpeedPlannerOutput *mutable_speed_planner_output() {
    return &speed_planner_output_;
  }
  const speed_planner::SpeedPlannerOutput &speed_planner_output() const {
    return speed_planner_output_;
  }

  RequestManagerInput *mutable_request_manager_input() {
    return &request_manager_input_;
  }

  const RequestManagerInput &request_manager_input() const {
    return request_manager_input_;
  }

  RequestManagerOutput *mutable_request_manager_outut() {
    return &request_manager_output_;
  }

  const RequestManagerOutput &request_manager_output() const {
    return request_manager_output_;
  }

  const LongitudinalBehaviorPlannerOutput &
  longitudinal_behavior_planner_output() const {
    return longitudinal_behavior_planner_output_;
  }
  LongitudinalBehaviorPlannerOutput *
  mutable_longitudinal_behavior_planner_output() {
    return &longitudinal_behavior_planner_output_;
  }

  const SpeedData &longitudinal_motion_planner_output() const {
    return speed_data_;
  }
  SpeedData *mutable_longitudinal_motion_planner_output() {
    return &speed_data_;
  }

  const gmp_interface::GeneralMotionPlannerOutput &
  general_motion_planner_output() const {
    return general_motion_planner_output_;
  }
  gmp_interface::GeneralMotionPlannerOutput *
  mutable_general_motion_planner_output() {
    return &general_motion_planner_output_;
  }

  const ObstacleDecisionManager &obstacle_decision_manager() const {
    return obstacle_decision_manager_;
  }

  ObstacleDecisionManager &mutable_obstacle_decision_manager() {
    return obstacle_decision_manager_;
  }

  const SpeedLimit &speed_limit() const { return speed_limit_; }
  SpeedLimit *mutable_speed_limit() { return &speed_limit_; }

  // const OpenSpacePath &open_space_path() const { return open_space_path_; }
  // OpenSpacePath *mutable_open_space_path() { return &open_space_path_; }

  // const SquareMap &square_map() const { return square_map_; }
  // SquareMap *mutable_square_map() { return &square_map_; }

  // const AimedPoi &aimed_poi() const { return aimed_poi_; }
  // AimedPoi *mutable_aimed_poi() { return &aimed_poi_; }

  // const AimedPoi &parking_out() const { return parking_out_; }
  // AimedPoi *mutable_parking_out() { return &parking_out_; }

  // const ParkingUiResponse &parking_ui_response() const {
  //   return parking_ui_response_;
  // }
  // ParkingUiResponse *mutable_parking_ui_response() {
  //   return &parking_ui_response_;
  // }

  const LateralBehaviorPlannerOutput &lateral_behavior_planner_output() const {
    return lateral_behavior_planner_output_;
  }

  LateralBehaviorPlannerOutput &mutable_lateral_behavior_planner_output() {
    return lateral_behavior_planner_output_;
  }

  const LateralMotionPlannerOutput &lateral_motion_planner_output() const {
    return lateral_motion_planner_output_;
  }

  LateralMotionPlannerOutput &mutable_lateral_motion_planner_output() {
    return lateral_motion_planner_output_;
  }

  const MSDStateMachineOutput &state_machine_output() const {
    return state_machine_output_;
  }

  MSDStateMachineOutput &mutable_state_machine_output() {
    return state_machine_output_;
  }

  const LdpResult ldp_result() { return ldp_result_; }

  LdpResult &mutable_ldp_result() { return ldp_result_; }

  const ElkResult elk_result() { return elk_result_; }

  ElkResult &mutable_elk_result() { return elk_result_; }

  const std::string &get_config_file_dir() const { return config_file_dir_; }

  void set_config_file_dir(const std::string &config_file_dir) {
    config_file_dir_ = config_file_dir;
  }

  const std::string &get_version() const { return version_; }

  void set_version(const std::string &version) { version_ = version; }

private:
  FallBackInfo fallback_info_;
  PlanningStatus planning_status_;
  PlanningStatus prev_planning_status_;
  path_planner::PathPlannerInput path_planner_input_;
  speed_planner::LonDecisionOutput lon_decison_output_;
  speed_planner::SpeedPlannerInput speed_planner_input_;
  speed_planner::SpeedPlannerOutput speed_planner_output_;
  RequestManagerInput request_manager_input_;
  RequestManagerOutput request_manager_output_;
  gmp_interface::GeneralMotionPlannerOutput general_motion_planner_output_;
  LongitudinalBehaviorPlannerOutput longitudinal_behavior_planner_output_;
  LateralBehaviorPlannerOutput lateral_behavior_planner_output_;
  LateralMotionPlannerOutput lateral_motion_planner_output_;
  SpeedData speed_data_;
  SpeedLimit speed_limit_;
  ObstacleDecisionManager obstacle_decision_manager_;
  MSDStateMachineOutput state_machine_output_;
  LdpResult ldp_result_;
  ElkResult elk_result_;

  OpenSpacePath open_space_path_;
  SquareMap square_map_;
  AimedPoi aimed_poi_;
  AimedPoi parking_out_;
  // ParkingUiResponse parking_ui_response_;

  std::string config_file_dir_;
  std::string version_;
  PlannerDebug planner_debug_;
  std::string ddp_model_input_;
  pass_intersection_planner::PassIntersectionPlannerInput
      pass_intersection_planner_input_;
};

namespace parking {

enum GateStatus : int {
  HANGUP = 0,
  PREPARE = 1,
  BEFORE = 2,
  BESIDE = 3,
  FINISH = 4,
  RESET = 5,
};

struct GateInfo {
  std::vector<int> id_;
  std::vector<std::string> sidepass_dir_;
  std::vector<TrajectoryPoint> gate_via_point_;
  double theta;
  bool is_entrance_;
  bool is_entrance_last_;
  planning_math::Box2d box;
  SLBoundary sl_boundary;
  GateStatus status;
  TrajectoryPoint start;
  TrajectoryPoint end;
  bool using_end_state;

  GateInfo() {
    is_entrance_ = false;
    is_entrance_last_ = false;
    using_end_state = false;
  };
  void reset() {
    using_end_state = false;
    is_entrance_ = false;
    is_entrance_last_ = false;
  }
  mjson::Json encoder() const {
    auto encoderTrajectoryPoint = [](const TrajectoryPoint &in) {
      mjson::Json result = mjson::Json(mjson::Json::object());
      result["v"] = mjson::Json(in.v);
      result["a"] = mjson::Json(in.a);
      result["relative_time"] = mjson::Json(in.relative_time);
      result["steer"] = mjson::Json(in.steer);
      result["prediction_prob"] = mjson::Json(in.prediction_prob);
      result["velocity_direction"] = mjson::Json(in.velocity_direction);
      result["sigma_x"] = mjson::Json(in.sigma_x);
      result["sigma_y"] = mjson::Json(in.sigma_y);
      result["relative_ego_yaw"] = mjson::Json(in.relative_ego_yaw);
      auto p = in.path_point;
      mjson::Json::array p_array{
          mjson::Json(p.x),           mjson::Json(p.y),
          mjson::Json(p.z),           mjson::Json(p.theta),
          mjson::Json(p.kappa),       mjson::Json(p.rho),
          mjson::Json(p.s),           mjson::Json(p.l),
          mjson::Json(p.dkappa),      mjson::Json(p.ddkappa),
          mjson::Json(p.lane_id),     mjson::Json(p.x_derivative),
          mjson::Json(p.y_derivative)};
      result["path_point"] = mjson::Json(p_array);
      return result;
    };

    mjson::Json result = mjson::Json(mjson::Json::object());
    mjson::Json::array id_array{};
    for (const auto &i : id_) {
      id_array.emplace_back(i);
    }
    result["id_"] = mjson::Json(id_array);
    mjson::Json::array sidepass_dir_array{};
    for (const auto &s : sidepass_dir_) {
      sidepass_dir_array.emplace_back(s);
    }
    result["sidepass_dir_"] = mjson::Json(sidepass_dir_array);
    mjson::Json::array gate_via_point_array{};
    for (const auto &g : gate_via_point_) {
      gate_via_point_array.emplace_back(encoderTrajectoryPoint(g));
    }
    result["gate_via_point_"] = mjson::Json(gate_via_point_array);
    result["theta"] = mjson::Json(theta);
    result["is_entrance_"] = mjson::Json(is_entrance_);
    result["is_entrance_last_"] = mjson::Json(is_entrance_last_);
    result["box"] = mjson::Json(mjson::Json::array{
        mjson::Json(box.center_x()), mjson::Json(box.center_y()),
        mjson::Json(box.length()), mjson::Json(box.width()),
        mjson::Json(box.heading())});
    // result["sl_boundary"] = mjson::Json();
    result["status"] = mjson::Json(static_cast<int>(status));
    result["start"] = encoderTrajectoryPoint(start);
    result["end"] = encoderTrajectoryPoint(end);
    result["using_end_state"] = mjson::Json(using_end_state);
    return result;
  }
};

enum class LateralPlanStatus : int {
  FAIL = 0,
  SUCCESS = 1,
  WAITING = 2,
  DEFAULT = 3
};

typedef struct LateralPlanningInfo_ {
  // for feedforward
  bool is_decider_succeed = false;
  std::vector<int> blocked_obstacle_id;
  std::vector<int> sidepass_static_obj;
  std::vector<int> sidepass_dynamic_obj;
  std::vector<std::pair<int, std::vector<planning_math::LineSegment2d>>>
      sidepass_obj;
  std::vector<int> sidepass_left_ignore;
  std::vector<int> sidepass_right_ignore;
  std::vector<std::pair<double, double>> pwj_bounds_info;
  std::vector<double> theta_ref;
  std::vector<double> lref;
  std::vector<double> interp_lref;
  std::vector<TrajectoryPoint> init_plan;
  std::vector<BoundaryPointInfo> boundary_decision_info;
  std::vector<BoundaryPointInfo> boundary_decision_info_expend;
  std::vector<int> obs_interf_via;
  std::vector<std::pair<Point2D, double>> via_points;
  bool teb_replan;
  // for feedback
  bool is_failsafe = false;
  bool is_replan = false;
  LateralPlanStatus lat_plan_status;
  double traj_length;
  double dis2closestobst = 1e19;
  double dis2closestfixobst = 1e19;
  double dis2closestfreeobst = 1e19;
  double dis2closestvirtualobst = 1e19;
  std::vector<std::vector<int>> obs_id_map;
  std::vector<TrajectoryPoint> traj_points;
  std::vector<double> traj_radius;
  bool is_obstacle_feedback;
} LateralPlanningInfo;

typedef struct ParkingLateralBehaviorPlannerOutput_ {
  bool is_pwj_init_state_success = false;
  double delta_s;
  double adc_vel;
  double init_state[4];
  std::string traj_tag;
  std::string last_traj_tag;
  TrajectoryPoint init_state_cart;
  TrajectoryPoint init_state_frenet;
  bool using_frenet_init_state;
  TrajectoryPoint target_state_cart;
  GateInfo gate_info;
  planning_math::Box2d map_boundary;
  bool using_fix_end;
  std::unordered_map<std::string, LateralPlanningInfo> lateral_planning_info;
  bool enable_arc_extention_before_apa;
  mjson::Json encoder() const {
    auto encoderTrajectoryPoint = [](const TrajectoryPoint &in) {
      mjson::Json result = mjson::Json(mjson::Json::object());
      result["v"] = mjson::Json(in.v);
      result["a"] = mjson::Json(in.a);
      result["relative_time"] = mjson::Json(in.relative_time);
      result["steer"] = mjson::Json(in.steer);
      result["prediction_prob"] = mjson::Json(in.prediction_prob);
      result["velocity_direction"] = mjson::Json(in.velocity_direction);
      result["sigma_x"] = mjson::Json(in.sigma_x);
      result["sigma_y"] = mjson::Json(in.sigma_y);
      result["relative_ego_yaw"] = mjson::Json(in.relative_ego_yaw);
      auto p = in.path_point;
      mjson::Json::array p_array{mjson::Json(p.x), mjson::Json(p.y), mjson::Json(p.z), 
          mjson::Json(p.theta), mjson::Json(p.kappa), mjson::Json(p.rho),
          mjson::Json(p.s), mjson::Json(p.l), mjson::Json(p.dkappa), mjson::Json(p.ddkappa),
          mjson::Json(p.lane_id), mjson::Json(p.x_derivative), mjson::Json(p.y_derivative)};
      result["path_point"] = mjson::Json(p_array);
      return result;
    };

    mjson::Json result = mjson::Json(mjson::Json::object());
    result["is_pwj_init_state_success"] = mjson::Json(is_pwj_init_state_success);
    result["delta_s"] = mjson::Json(delta_s);
    result["adc_vel"] = mjson::Json(adc_vel);
    result["init_state"] = mjson::Json(mjson::Json::array{
      mjson::Json(init_state[0]), mjson::Json(init_state[1]), mjson::Json(init_state[2]), mjson::Json(init_state[3])});
    result["traj_tag"] = mjson::Json(traj_tag);
    result["last_traj_tag"] = mjson::Json(last_traj_tag);
    result["init_state_cart"] = encoderTrajectoryPoint(init_state_cart);
    result["init_state_frenet"] = encoderTrajectoryPoint(init_state_frenet);
    result["using_frenet_init_state"] = mjson::Json(using_frenet_init_state);
    result["target_state_cart"] = encoderTrajectoryPoint(target_state_cart);
    result["gate_info"] = gate_info.encoder();
    result["map_boundary"] = mjson::Json(mjson::Json::array{
      mjson::Json(map_boundary.center_x()), mjson::Json(map_boundary.center_y()), 
      mjson::Json(map_boundary.length()), mjson::Json(map_boundary.width()), mjson::Json(map_boundary.heading())});
    result["using_fix_end"] = mjson::Json(using_fix_end);
    // result["lateral_planning_info"] = mjson::Json();
    result["enable_arc_extention_before_apa"] = mjson::Json(enable_arc_extention_before_apa);
    return result;
  }
} ParkingLateralBehaviorPlannerOutput;

enum StatusType {
  WAIT = 0,
  SEARCH,
  AVP,
  APA,
  APOA,
  RPA_STRAIGHT_STANDBY,
  RPA_STRAIGHT,
  DONE,
};

enum TaskStatusType {
  RUNNING = 0,
  PAUSED,
  SUCCEEDED,
  FAILED,
};

enum FailureReason {
  UNKNOW_FAILED = 0x1001,           //!< 未定义错误
  PLANNING_FAILED = 0x1002,         //!< 规划失败
  BARRIER_IN_PARKING_SLOT = 0x1003, //!< 车位内有障碍物
  ADJUEST_TOO_MANY = 0x1004,        //!< 调整次数过多
  START_INFEASIBLE = 0x1005,        //!< 起始位置不可用
  RPA_COMPLETE_FAILED = 0x1006,
};

typedef struct TaskStatus_ {
  StatusType task;
  TaskStatusType status;
  FailureReason failure_reason;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["task"] = mjson::Json(static_cast<int>(task));
    result["status"] = mjson::Json(static_cast<int>(status));
    result["failure_reason"] = mjson::Json(static_cast<int>(failure_reason));
    return result;
  }
} TaskStatus;

typedef struct ScenarioStatus_ {
  std::string scenario_type;
  std::string stage_type;
  StatusType status_type;
  StatusType next_status_type;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["scenario_type"] = mjson::Json(scenario_type);
    result["stage_type"] = mjson::Json(stage_type);
    result["status_type"] = mjson::Json(static_cast<int>(status_type));
    result["next_status_type"] = mjson::Json(static_cast<int>(next_status_type));
    return result;
  }
} ScenarioStatus;

enum ParkingSidePassType : int {
  SIDEPASS_NOTHING = 0,
  SIDEPASS_BOTH,
  SIDEPASS_PED,
  SIDEPASS_COUPLE,
};

enum ParkingSceneType : int {
  SCENE_NONE = 0x00,
  SCENE_ENTRANCE = 0x01,
  SCENE_AVP = 0x01 << 1,
  SCENE_SIDEPASS = 0x01 << 2,
  SCENE_TURN = 0x01 << 3,
  SCENE_RAMP = 0x01 << 4,
  SCENE_HACK = 0x01 << 5,
};

struct BagRecorderFilterScenario {
  bool abrupt_brake = false;
  bool side_pass = false;
  bool openspace = false;
  bool apa = false;
  bool reparkin = false;
};

typedef struct PlanningResult_ {
  struct PiecewiseJerkPoint {
    double s;
    double l;
    double dl;
    double ddl;
  };

  double time;

  float lon_error = 0.0F;
  float lat_error = 0.0F;

  float v_target = 0.0F;
  float a_target = 0.0F;
  std::vector<float> v_array;
  std::vector<float> a_array;

  float a_target_min = 0.0F;
  float a_target_max = 0.0F;

  // traffic light
  int traffic_light_state = 0;
  bool stop_flag = false;
  bool is_passed_stop_line = false;
  double dist_to_stop = 0.0;

  // task manager
  bool state_changed = false;
  bool is_loaded = false;
  // decision
  std::vector<uint32_t> lon_follow_obstacles;
  std::vector<uint32_t> lon_overtake_obstacles;
  std::vector<uint32_t> lat_nudge_obstacles;

  int flag_closelead;
  int flag_invalid;
  int flag_type0;
  int flag_softbrake;
  int flag_fastcutin;
  double v_set;
  double ds_set;
  double t_set;
  double v_limit;
  int num_yield;
  double vl_yield;
  double dsl_yield;
  int type_yield;
  int id_yield;
  int tag_yield;
  int type_merge;
  double dis2cross;
  double traj_length;
  double dis2closestobst;
  double dis2closestfixobst;
  double dis2closestfreeobst;
  double dis2closestvirtualobst;

  // traj
  std::vector<Pose2D> traj_pose_array;
  std::vector<Pose2D> second_traj_pose_array;
  std::vector<double> second_traj_curvature;
  // std::vector<PathPose> mpc_path;
  std::vector<PathPose> ref_trajectory;
  std::vector<float> traj_vel_array;
  std::vector<float> second_traj_vel_array;
  std::vector<float> traj_curvature;
  std::vector<float> traj_curvature_radius;
  std::vector<float> traj_acceleration;
  std::vector<float> traj_relative_time;
  std::vector<float> traj_relative_s;

  std::vector<TrajectoryPoint> pwj_trajectory;
  uint64_t pre_planning_time = 0;
  // pwj output
  std::vector<PiecewiseJerkPoint> frenet_pwj_traj;
  std::vector<double> frenet_pwj_theta;
  std::vector<PathPoint> cart_pwj_traj;
  std::vector<PathPoint> path_points_real;

  LateralPlanStatus lat_plan_status;

  bool pwj_status;
  bool is_failsafe = false;
  bool is_apa;
  bool is_entrance;

  int scene_avp;
  ParkingSidePassType sidepass_type;
  BagRecorderFilterScenario bag_recorder_filter_scenario;

  // for teb output
  std::vector<std::vector<int>> obs_id_map;
  std::vector<TrajectoryPoint> traj_points;
  std::vector<double> traj_radius;
  bool is_obstacle_feedback;

  bool is_use_reach_the_end_flag = false;
  int pre_segment_index = 0;
  bool real_gear_change_flag = false;
  int pre_real_gear = 0;
  double pre_wheel_velocity = 0.0;
  std::pair<double, double>
      second_traj_not_replan_scope = {0.0, 0.0};
  bool is_finish_switch_next_traj_flag = false;
  bool is_finished_flag = false;

  GearState gear;
  bool gear_changing;
  void clear_traj() {
    traj_pose_array.clear();
    traj_vel_array.clear();
    traj_curvature.clear();
    traj_curvature_radius.clear();
    traj_acceleration.clear();
    traj_relative_time.clear();
    traj_relative_s.clear();
    // frenet_pwj_traj.clear();
    frenet_pwj_theta.clear();
    cart_pwj_traj.clear();
    path_points_real.clear();
  }

  mjson::Json encoder() const {
    auto f = [](const std::vector<float> &in) {
      mjson::Json::array in_array{};
      for (const auto &i : in) {
        if (std::isinf(i)) {
          in_array.emplace_back(mjson::Json("Infinity"));
        } else {
          in_array.emplace_back(mjson::Json(i));
        }
      }
      return mjson::Json(in_array);
    };

    mjson::Json result = mjson::Json(mjson::Json::object());
    result["time"] = mjson::Json(time);
    result["state_changed"] = mjson::Json(state_changed);
    result["is_loaded"] = mjson::Json(is_loaded);
    mjson::Json::array traj_pose{};
    for (const auto &p : traj_pose_array) {
      traj_pose.emplace_back(mjson::Json(mjson::Json::array{mjson::Json(p.x), mjson::Json(p.y), mjson::Json(p.theta)}));
    }
    result["traj_pose_array"] = mjson::Json(traj_pose);
    mjson::Json::array ref_trajectory_array{};
    for (const auto &p : ref_trajectory) {
      ref_trajectory_array.emplace_back(p.encoder());
    }
    result["ref_trajectory"] = mjson::Json(ref_trajectory_array);
    result["traj_vel_array"] = f(traj_vel_array);
    result["traj_curvature"] = f(traj_curvature);
    result["traj_curvature_radius"] = f(traj_curvature_radius);
    result["traj_acceleration"] = f(traj_acceleration);
    result["traj_relative_time"] = f(traj_relative_time);
    result["traj_relative_s"] = f(traj_relative_s);
    result["pwj_status"] = mjson::Json(pwj_status);
    result["is_failsafe"] = mjson::Json(is_failsafe);
    result["is_apa"] = mjson::Json(is_apa);
    result["is_entrance"] = mjson::Json(is_entrance);
    result["gear"] = mjson::Json(static_cast<int>(gear));
    result["gear_changing"] = mjson::Json(gear_changing);
    return result;
  }
} PlanningResult;

struct WlcInfo {
  bool x_offset_valid = false;
  bool y_offset_valid = false;
  double x_offset;
  double y_offset;
  bool is_fusion_wlc_property_valid = false;
  bool is_valid = false;
  bool is_approached = false;
  bool is_approaching =false;
};

struct PlanControlInterface{
    bool is_extended = true;  // true: 可延长；false: 不可延长
    unsigned int extend_type; // 0: default; 1: straight; 2: circular curve; 可延长状态下为0。
                              // 在不可延长状态下，水平窄车位场景为2， 其他为1
    double remain_traj;    //  规划路径的剩余距离
    double second_remain_traj; //第二段轨迹剩余距离
    double second_traj_target_v; //第二段轨迹的目标速度
    bool is_use_traj_s_and_v = false; // 是否使用第二段轨迹目标速度和剩余距离
    unsigned int first_gear;   // 第一段轨迹档位
    unsigned int second_gear;    // 第二段轨迹档位
    unsigned int gear_change_index;    // 第二段轨迹的起始点索引值
    bool is_static = false;
};

typedef struct PlanningStatus_{
  int64_t planning_loop = 0;
  bool planning_success = false;
  bool is_reached_end = false;
  double v_limit;
  double a_limit;
  bool control_block_feedback;
  bool blocked;
  bool blocked_timeout;
  bool collide_to_sth;
  bool stopping;
  bool fs_block;
  bool advanced_abandon = false;
  bool is_pullover;
  bool pullover_enable;
  double block_timeout_duration = 5.0;
  bool has_running_enough_long = false;
  double block_time;
  int zigzag_num;
  RpaStraightDirectionType rpa_straight_direction;
  LaneStatus lane_status;
  CrosswalkStatus crosswalk;
  ReroutingStatus rerouting;
  RightOfWayStatus right_of_way;
  ScenarioStatus scenario;
  TaskStatus task_status;
  TrafficLightStatus traffic_light;
  PlanningResult planning_result;
  mlog::MLogDataStream statemachine_stringstream;
  std::string trigger_msg_id;
  WlcInfo wlc_info;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["planning_loop"] = mjson::Json(planning_loop);
    result["planning_success"] = mjson::Json(planning_success);
    result["v_limit"] = mjson::Json(v_limit);
    result["a_limit"] = mjson::Json(a_limit);
    result["control_block_feedback"] = mjson::Json(control_block_feedback);
    result["blocked"] = mjson::Json(blocked);
    result["blocked_timeout"] = mjson::Json(blocked_timeout);
    result["collide_to_sth"] = mjson::Json(collide_to_sth);
    result["stopping"] = mjson::Json(stopping);
    result["fs_block"] = mjson::Json(fs_block);
    result["advanced_abandon"] = mjson::Json(advanced_abandon);
    result["is_pullover"] = mjson::Json(is_pullover);
    result["pullover_enable"] = mjson::Json(pullover_enable);
    result["block_timeout_duration"] = mjson::Json(block_timeout_duration);
    result["has_running_enough_long"] = mjson::Json(has_running_enough_long);
    result["block_time"] = mjson::Json(block_time);
    result["zigzag_num"] = mjson::Json(zigzag_num);
    result["lane_status"] = lane_status.encoder();
    result["crosswalk"] = crosswalk.encoder();
    result["rerouting"] = rerouting.encoder();
    result["right_of_way"] = right_of_way.encoder();
    result["scenario"] = scenario.encoder();
    result["task_status"] = task_status.encoder();
    result["traffic_light"] = traffic_light.encoder();
    result["planning_result"] = planning_result.encoder();
    // result["statemachine_stringstream"] = mjson::Json();
    result["trigger_msg_id"] = mjson::Json(trigger_msg_id);
    // result["wlc_info"] = mjson::Json();
    return result;
  }
  PlanControlInterface plan_control_interface;
} PlanningStatus;

struct Leader {
  int id = -1;
  ObjectType type = ObjectType::NOT_KNOW;
  int direction = 0;
  double d_rel = 200; // min s rel
  double d_path = 0.0;
  double v_lat = 0.0; // lateral velocity
  double v_lon = 0.0; // longitudinal velocity
  double a_lat = 0.0; // lateral acceleration
  double a_lon = 0.0; // longitudinal acceleration
  double yaw_relative_frenet = 0.0;
  bool is_static = false;
  bool is_sf_static = false;
  bool is_sidepass_obj = false;
  bool is_apa_status = false;
  bool is_approaching_gate = false;
  bool is_need_fillet_cutting = false;
};

typedef std::pair<Leader, Leader> LeaderPair;

enum CollisionType : int {
  COLLISIONFREE = 0,
  PLANPATH = 1,
  MPC = 2,
  BOTH = 3,
};

struct FreespacePoint {
  int id = -1;
  bool is_collision = false;
  ObjectType type = ObjectType::NOT_KNOW;
  double d_rel = 200.0;
  double d_path = 100.0;
  double x = 0.0;
  double y = 0.0;
  double s = 0.0;
  double l = 0.0;
  Pose2D ego_danger_location;
  CollisionType collision_type = CollisionType::COLLISIONFREE;  // 0-free, 1-pp, 2-mpc, 3-both
};

struct FreespaceLine {
  int id = -1;
  double d_rel = 200.0;
  double x_start = 0.0;
  double x_end = 0.0;
  double y_start = 0.0;
  double y_end = 0.0;
  double s_start = 0.0;
  double s_end = 0.0;
  double l_start = 0.0;
  double l_end = 0.0;
  ObjectType type = ObjectType::NOT_KNOW;
};

struct MultiDirectionalCars {
  int id = -1;
  double TTC_pwj = 100.0;
  double TTC_ego = 100.0;
  double s_prediction = 100.0;
  double decay_rate = 1.0;
  int decel_level = 0;
  int coordinate = 0; // 1: Frenet 2: Ego_Cartesian
  ObjectType type = ObjectType::NOT_KNOW;
};

struct MultiDirectionalHuman {
  int id = -1;
  int decel_level = 0;
  ObjectType type = ObjectType::NOT_KNOW;
  int frame_type = 0; // 1: frenet 2: ego
  double TTC_obs = 100.0;
  double TTC_ego = 100.0;
  double s_prediction = 100.0;
};

struct IntentionStatusObstacles {
  int id = -1;
  IntentionObsType type = IntentionObsType::NONE;
};

struct LongitudinalBehaviorPlannerOutput {
  struct RemainDistInfo {
    double d_rel_ = 10.0;
    int id_ = -1;
    ObjectType type_ = ObjectType::NOT_KNOW;
    bool is_static_ = true;
    double lon_safe_dis_ = 0.0;
    bool is_traj_have_dynamic_obs_ = false;
    double remaining_distance_ = 1e9;
    // debug
    int obstacle_type_ = 9;
    int slot_type_ = 9;
    std::string now_time_seq_str_ = "";
    int is_need_pause_ = 0;

    void set_value(double d_rel, int id, ObjectType type, bool is_static,
                   double lon_safe_dis, bool is_traj_have_dynamic_obs) {
      d_rel_ = d_rel;
      id_ = id;
      type_ = type;
      is_static_ = is_static;
      lon_safe_dis_ = lon_safe_dis;
      is_traj_have_dynamic_obs_ = is_traj_have_dynamic_obs;
      if (id >= 0)  // temp
        remaining_distance_ = d_rel_ - lon_safe_dis_;
      else
        remaining_distance_ = d_rel;
    }
  };
  BehaviorType lane_keeping_behavior_type;
  double cost_weight_lane_keeping[4];
  std::vector<Pose2D> trajectory;
  std::vector<Pose2D> mpc_trajectory;
  std::vector<Pose2D> refactored_mpc_trajectory;
  std::vector<double> planning_mpc_diff;
  std::vector<Pose2D> plan_path;
  std::vector<double> curvatures;
  std::vector<double> relative_s;
  double ego_lat_diff = 0.0;
  bool deviated;
  double traj_length;
  bool path_segment_updated = false;
  bool is_at_last_segment = false;
  bool update_wheel_stop = false;
  bool is_dynamic_planning = false;
  std::string speed_margin_debug = "";
  planning_math::Box2d exclusive_box;
  LeaderPair lead_cars;
  FreespacePoint free_space;
  Point2D last_block_fs;
  FreespaceLine fs_line;
  std::vector<MultiDirectionalCars> multidirectional_cars;
  std::vector<MultiDirectionalHuman> multidirectional_human;
  std::vector<IntentionStatusObstacles> intention_status_obstacles;
  std::vector<int> prediction_obstacles;
  bool is_blocked_by_obstacle_behind_in_slot;

  // remaining distance
  RemainDistInfo remain_dist_info_;
  int is_need_pause_;

public:
  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["is_need_pause_"] = mjson::Json(is_need_pause_);
    result["remain_dist_info_"] = mjson::Json(mjson::Json::array{
        mjson::Json(remain_dist_info_.d_rel_),
        mjson::Json(remain_dist_info_.id_),
        mjson::Json(static_cast<int>(remain_dist_info_.type_)),
        mjson::Json(remain_dist_info_.is_static_),
        mjson::Json(remain_dist_info_.lon_safe_dis_),
        mjson::Json(remain_dist_info_.is_traj_have_dynamic_obs_),
        mjson::Json(remain_dist_info_.remaining_distance_),
        mjson::Json(remain_dist_info_.obstacle_type_),
        mjson::Json(remain_dist_info_.slot_type_), 
        mjson::Json(remain_dist_info_.now_time_seq_str_),
        mjson::Json(remain_dist_info_.is_need_pause_)});
    return result;
  }
};

enum class PlannerType { PARKING_LATERAL, OPENSPACE };

struct KeyPoint {
  bool available;
  Pose2D point;
  double distance;
  KeyPoint() : available(false), point({0.0, 0.0, 0.0}), distance(1e19) {}
};

struct ParkingSlotType {
  uint8_t value;

  enum : uint8_t { PERPENDICULAR = 0, PARALLEL, OBLIQUE };
  ParkingSlotType() : value(0) {}
};

struct ParkOutType {
  uint32_t value;

  enum : uint32_t {
    INVALID = 0,                   //!< invalid park out direction
    PERPENDICULAR_OBLIQUE_FRONT = 1,       //!< perpendicular or oblique slot front side
    PERPENDICULAR_OBLIQUE_FRONT_LEFT = 2,  //!< perpendicular or oblique slot front-left side
    PERPENDICULAR_OBLIQUE_FRONT_RIGHT = 4, //!< perpendicular or oblique slot front-right side
    PERPENDICULAR_OBLIQUE_REAR = 8,        //!< perpendicular or oblique slot rear side
    PERPENDICULAR_OBLIQUE_REAR_LEFT = 16,  //!< perpendicular or oblique slot rear-left side
    PERPENDICULAR_OBLIQUE_REAR_RIGHT = 32, //!< perpendicular or oblique slot rear-right side
    PARALLEL_LEFT = 64,            //!< parallel slot left side
    PARALLEL_LEFT_FRONT = 128,     //!< parallel slot left-front side
    PARALLEL_RIGHT = 256,          //!< parallel slot right side
    PARALLEL_RIGHT_FRONT = 512,    //!< parallel slot right-front side
    CALC_INVALID = 1024,          //!< calc park out direction invalid
  };
  ParkOutType() : value(0) {}
};

struct WheelStopInfo {
  bool available;
  bool vision_wheel_stop_available;
  Point3D vision_point1;
  Point3D vision_point2;
  bool collision_wheel_stop_available;
  Point3D collision_point1;
  Point3D collision_point2;
  double wheel_stop_depth;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["available"] = mjson::Json(available);
    result["vision_wheel_stop_available"] = mjson::Json(vision_wheel_stop_available);
    result["vision_point1"] = mjson::Json(mjson::Json::array{mjson::Json(vision_point1.x), mjson::Json(vision_point1.y), mjson::Json(vision_point1.z)});
    result["vision_point2"] = mjson::Json(mjson::Json::array{mjson::Json(vision_point2.x), mjson::Json(vision_point2.y), mjson::Json(vision_point2.z)});
    result["collision_wheel_stop_available"] = mjson::Json(collision_wheel_stop_available);
    result["collision_point1"] = mjson::Json(mjson::Json::array{mjson::Json(collision_point1.x), mjson::Json(collision_point1.y), mjson::Json(collision_point1.z)});;
    result["collision_point2"] = mjson::Json(mjson::Json::array{mjson::Json(collision_point2.x), mjson::Json(collision_point2.y), mjson::Json(collision_point2.z)});
    result["wheel_stop_depth"] = mjson::Json(wheel_stop_depth);
    return result;
  }
};

struct ParkingSlotInfo {
  int id;
  ParkingSlotType type;
  ParkOutType park_out_type;
  ParkOutType available_park_out_type;
  ParkOutType recommend_park_out_type;
  std::vector<Point3D> corners;
  std::vector<Point3D> original_corners;
  bool is_space_slot;
  bool left_empty;
  bool right_empty;
  std::vector<int> nearby_obstacle_id;
  KeyPoint park_in_point;
  KeyPoint park_out_point;
  Pose2D park_out_init_pose;
  Pose2D park_out_error_tolerence;
  planning_math::Vec2d front_direction;
  planning_math::Vec2d left_direction;
  double distance_to_parking_slot;
  WheelStopInfo wheel_stop_info = {};
  /*
    slot_length_type: only used in parallel slot
    0: normal
    1: slot length < vehicle length + 2m
    2: slot length < vehicle length + 1.1m
    3: slot length < vehicle length + 0.9m
  */
  int slot_length_type = 0;
  /*
    special_slot_type:
    0: normal
    1: parallel narrow slot(slot length < vehicle length + 2m)
  */
  int special_slot_type = 0;
  /*
    bottom_line_type:
    1:perpendicular bottom wall, 2:perpendicular bottom step
    3:parallel bottom wall, 4:parallel bottom step
  */
  int bottom_line_type = 0;
  /*
    is_direction_limit: 代表泊出方向是否受限的标志位
    0: 代表正常，不受限制
    1: 代表水平车位，自车前方无障碍物，不返回泊出方向
  */
  int is_direction_limit = 0;

  planning_math::LineSegment2d center_line;
  struct VirtualCorner {
    /**
    0：UNKOWN；
    1：垂直；
    2：平行；
    3：普通斜列；
    4：矩形斜列；
     */
    int slot_type = 0;
    Point3D p;
    int index;

    mjson::Json encoder() const {
      mjson::Json result = mjson::Json(mjson::Json::object());
      result["slot_type"] = mjson::Json(slot_type);
      result["p"] = mjson::Json(mjson::Json::array{mjson::Json(p.x), mjson::Json(p.y), mjson::Json(p.z)});
      result["index"] = mjson::Json(index);
      return result;
    }
  };
  VirtualCorner virtual_corner;
  ParkingSlotInfo()
      : id(-1), type(ParkingSlotType()), park_out_type(ParkOutType()),
        available_park_out_type(ParkOutType()), corners(std::vector<Point3D>()),
        original_corners(std::vector<Point3D>()), is_space_slot(false),
        left_empty(false), right_empty(false), is_direction_limit(0),
        nearby_obstacle_id(std::vector<int>()), park_in_point(KeyPoint()),
        park_out_point(KeyPoint()), front_direction(planning_math::Vec2d()),
        left_direction(planning_math::Vec2d()), distance_to_parking_slot(1e19) {
  }

  double slot_width() {
    return std::sqrt((corners[3].x - corners[0].x) * (corners[3].x - corners[0].x) + 
        (corners[3].y - corners[0].y) * (corners[3].y - corners[0].y));
  }

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["id"] = mjson::Json(id);
    result["type"] = mjson::Json(static_cast<int>(type.value));
    result["park_out_type"] = mjson::Json(static_cast<int>(park_out_type.value));
    result["available_park_out_type"] = mjson::Json(static_cast<int>(available_park_out_type.value));
    mjson::Json::array corners_array{};
    for (const auto &c : corners) {
      corners_array.emplace_back(mjson::Json(mjson::Json::array{mjson::Json(c.x), mjson::Json(c.y), mjson::Json(c.z)}));
    }
    result["corners"] = mjson::Json(corners_array);
    mjson::Json::array original_corners_array{};
    for (const auto &c : original_corners) {
      original_corners_array.emplace_back(mjson::Json(mjson::Json::array{mjson::Json(c.x), mjson::Json(c.y), mjson::Json(c.z)}));
    }
    result["original_corners"] = mjson::Json(original_corners_array);
    result["is_space_slot"] = mjson::Json(is_space_slot);
    result["left_empty"] = mjson::Json(left_empty);
    result["right_empty"] = mjson::Json(right_empty);
    result["special_slot_type"] = mjson::Json(special_slot_type);
    mjson::Json::array nearby_obstacle_id_array{};
    for (const auto &id : nearby_obstacle_id) {
      nearby_obstacle_id_array.emplace_back(mjson::Json(id));
    }
    result["nearby_obstacle_id"] = mjson::Json(nearby_obstacle_id_array);
    result["park_in_point"] = mjson::Json(mjson::Json::array{mjson::Json(park_in_point.available), 
        mjson::Json(park_in_point.point.x), mjson::Json(park_in_point.point.y), mjson::Json(park_in_point.point.theta),
        mjson::Json(park_in_point.distance)});
    result["park_out_point"] = mjson::Json(mjson::Json::array{mjson::Json(park_out_point.available), 
        mjson::Json(park_out_point.point.x), mjson::Json(park_out_point.point.y), mjson::Json(park_out_point.point.theta),
        mjson::Json(park_out_point.distance)});
    result["front_direction"] = mjson::Json(mjson::Json::array{mjson::Json(front_direction.x()), mjson::Json(front_direction.y()), mjson::Json(front_direction.Id())});
    result["left_direction"] = mjson::Json(mjson::Json::array{mjson::Json(left_direction.x()), mjson::Json(left_direction.y()), mjson::Json(left_direction.Id())});
    result["distance_to_parking_slot"] = mjson::Json(distance_to_parking_slot);
    result["wheel_stop_info"] = wheel_stop_info.encoder();
    result["virtual_corner"] = virtual_corner.encoder();
    return result;
  }
};

// struct ParkingLotInfo {
//   int id;
//   std::string type;
//   std::vector<Point3D> corners;
//   Pose2D key_point;
//   double distance;
//   double distance_to_poi;
//   bool is_key_point_available;
//   ParkingLotInfo()
//     : id(-1),
//     distance(1e19),
//     is_key_point_available(false)
//     {}
// };

// struct PerceptionSnapshot {
//   std::vector<std::pair<bool, planning_math::Box2d>> boxes;
//   bool is_available;
//   PerceptionSnapshot()
//     : is_available(false)
//     {}
// };

// parkout with memory
struct LastParkinData{
  bool is_loaded = false;
  bool is_valid = false;
  std::vector<planning_math::Vec2d> corner_pts;
  double last_theta = 0.0;
};

struct ParkingBehaviorPlannerOutput {
  PlannerType planner_type;
  TrajectoryPoint init_traj_point;   // for openspace only
  TrajectoryPoint target_traj_point; // for openspace only
  ParkingSlotInfo parking_slot_info;
  LastParkinData last_parkin_data;  // for parkout with memory
  APAMetaState apa_meta_state;
  bool has_ever_been_inside_slot = false;
  // ParkingLotInfo park_in_info;
  // ParkingLotInfo park_out_info;
  // PerceptionSnapshot park_in_snapshot;
  int current_parking_lot_id;

  bool is_move_ready;
  bool has_moved;
  bool has_planned;
  bool is_finish;
  bool approaching_wheel_stop;
  double dist_to_wheel_stop;
  bool is_request_to_ego_slot;
  double min_dist_to_opening;
  double min_dist_to_bottom;
  double rpa_forward_dis = -1.0;
  double rpa_backward_dis = -1.0;
  std::string behavior;
  bool is_last_path;
  bool is_narrow_channel;
  bool has_paused = false;
  double min_remain_distance = 999.0;
  bool reached_parkout_takeover_point = false;

  std::shared_ptr<ParkingSlotInterface> parking_lot;

  ParkingBehaviorPlannerOutput()
      : planner_type(PlannerType::PARKING_LATERAL), current_parking_lot_id(0),
        is_move_ready(false), is_finish(false), dist_to_wheel_stop(2.0),
        min_dist_to_opening(-1.0), min_dist_to_bottom(10.0) {}

  mjson::Json encoder() const {
    auto encoderTrajectoryPoint = [](const TrajectoryPoint &in) {
      mjson::Json result = mjson::Json(mjson::Json::object());
      result["v"] = mjson::Json(in.v);
      result["a"] = mjson::Json(in.a);
      result["relative_time"] = mjson::Json(in.relative_time);
      result["steer"] = mjson::Json(in.steer);
      result["prediction_prob"] = mjson::Json(in.prediction_prob);
      result["velocity_direction"] = mjson::Json(in.velocity_direction);
      result["sigma_x"] = mjson::Json(in.sigma_x);
      result["sigma_y"] = mjson::Json(in.sigma_y);
      result["relative_ego_yaw"] = mjson::Json(in.relative_ego_yaw);
      auto p = in.path_point;
      mjson::Json::array p_array{mjson::Json(p.x), mjson::Json(p.y), mjson::Json(p.z), 
          mjson::Json(p.theta), mjson::Json(p.kappa), mjson::Json(p.rho),
          mjson::Json(p.s), mjson::Json(p.l), mjson::Json(p.dkappa), mjson::Json(p.ddkappa),
          mjson::Json(p.lane_id), mjson::Json(p.x_derivative), mjson::Json(p.y_derivative)};
      result["path_point"] = mjson::Json(p_array);
      return result;
    };

    mjson::Json result = mjson::Json(mjson::Json::object());
    result["planner_type"] = mjson::Json(static_cast<int>(planner_type));
    result["init_traj_point"] = encoderTrajectoryPoint(init_traj_point);
    result["target_traj_point"] = encoderTrajectoryPoint(target_traj_point);
    result["parking_slot_info"] = parking_slot_info.encoder();
    result["current_parking_lot_id"] = mjson::Json(current_parking_lot_id);
    result["is_move_ready"] = mjson::Json(is_move_ready);
    result["has_moved"] = mjson::Json(has_moved);
    result["has_planned"] = mjson::Json(has_planned);
    result["is_finish"] = mjson::Json(is_finish);
    result["approaching_wheel_stop"] = mjson::Json(approaching_wheel_stop);
    result["dist_to_wheel_stop"] = mjson::Json(dist_to_wheel_stop);
    result["is_request_to_ego_slot"] = mjson::Json(is_request_to_ego_slot);
    // result["behavior"] = mjson::Json(behavior);
    result["is_last_path"] = mjson::Json(is_last_path);
    result["is_narrow_channel"] = mjson::Json(is_narrow_channel);
    // result["parking_lot"] = mjson::Json();
    return result;
  }

};

// struct FreespaceDeciderOutput {
//   bool is_send_map_square = false;
//   std::vector<std::vector<std::pair<int, Point3D>>> fs_pts;
// };

class OpenSpacePath {
public:
  OpenSpacePath() : traj_points(), is_read(true) {}
  std::vector<PathPose> path_poses;
  void stash(std::vector<TrajectoryPoint> path) {
    traj_points = path;
    path_poses.clear();
    path_poses.reserve(path.size());
    for (auto iter = path.begin(); iter != path.end(); ++iter) {
      PathPose path_pose;
      path_pose.pos.x = iter->path_point.x;
      path_pose.pos.y = iter->path_point.y;
      path_pose.pos.z = iter->path_point.z;
      path_poses.push_back(path_pose);
    }
    is_read = false;
  }
  std::vector<TrajectoryPoint> unstash() {
    is_read = true;
    return traj_points;
  }
  bool isNew() const { return !is_read; }
  std::vector<TrajectoryPoint> trajPoints() { return traj_points; }

private:
  std::vector<TrajectoryPoint> traj_points;
  bool is_read;
};

class ReferenceLine {
public:
  ReferenceLine() : refline_points_(), available_(false) {}
  void SetReflinePoints(const std::vector<TrajectoryPoint> &points) {
    refline_points_ = points;
    available_ = true;
  }
  void ClearReflinePoints() {
    refline_points_.clear();
    available_ = false;
  }
  const std::vector<TrajectoryPoint> &refline_points() const {
    return refline_points_;
  }
  const bool &available() const { return available_; }

private:
  std::vector<TrajectoryPoint> refline_points_;
  bool available_;
};

struct OpenspaceMotionPlannerOutput {
  enum PlannerStatus : int {
    SUCCESS = 0,
    INFEASIBLE = 1, //!< Infeasible
    TIMEOUT = 2,    //!< TimeOut
    EXCEPTION = 4,  //!< Exeception
    START_INFEASIBLE = 8,
    END_INFEASIBLE = 16,
  };

  bool is_fail; // only valid when not planning
  bool is_planning;
  bool is_plan_ready;
  double planner_calc_duration;
  PlannerStatus status;
  size_t openspace_fallback_cnt;
  size_t times_try_parking_in;
  size_t times_try_parking_in_about_hit_sth;
  size_t times_tiny_slot_overlap;
  std::vector<TrajectoryPoint> traj;
  OpenspaceMotionPlannerOutput()
      : is_fail(false), is_planning(false), is_plan_ready(false),
        openspace_fallback_cnt(0) {}
};

struct ParkingUiNotificationMsg {
  std::vector<std::string> msg;
};

struct FlagFilter {
  bool output_;
  size_t filter_length_;
  std::vector<int> stack_;
  size_t flag_count_;
  FlagFilter() { filter_length_ = 0; }
  explicit FlagFilter(const size_t filter_length) {
    output_ = false;
    filter_length_ = filter_length;
    stack_.resize(filter_length_, 0);
    flag_count_ = 0;
  }
  void reset() {
    output_ = false;
    stack_.resize(filter_length_, 0);
    flag_count_ = 0;
  }
  void set_flag(bool flag) {
    flag_count_++;
    for (int i = 0; i < (int)stack_.size() - 1; i++) {
      stack_[i] = stack_[i + 1];
    }
    stack_.back() = int(flag);
    if (flag_count_ < filter_length_)
      output_ = false;
    else {
      output_ = true;
      for (size_t i = 0; i < filter_length_; i++)
        output_ = output_ && stack_[i];
    }
  }
  bool get_flag() { return output_; }
};

class PlanningContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(PlanningContext);

public:
  struct FallBackInfo {
    std::string last_successful_path_label;
  };
  using LeaderPair = std::pair<Leader, Leader>;

public:
  const FallBackInfo &fallback_info() const { return fallback_info_; }
  FallBackInfo *mutable_fallback_info() { return &fallback_info_; }

  const PlanningStatus &planning_status() const { return planning_status_; }
  PlanningStatus *mutable_planning_status() { return &planning_status_; }

  const LongitudinalBehaviorPlannerOutput &
  longitudinal_behavior_planner_output() const {
    return longitudinal_behavior_planner_output_;
  }
  LongitudinalBehaviorPlannerOutput *
  mutable_longitudinal_behavior_planner_output() {
    return &longitudinal_behavior_planner_output_;
  }

  const ReferenceLine &reference_line() const { return reference_line_; }
  ReferenceLine *mutable_reference_line() { return &reference_line_; }

  const OpenSpacePath &open_space_path() const { return open_space_path_; }
  OpenSpacePath *mutable_open_space_path() { return &open_space_path_; }

  const SquareMap &square_map() const { return square_map_; }
  SquareMap *mutable_square_map() { return &square_map_; }
  const SquareMap &square_map2() const { return square_map2_; }
  SquareMap *mutable_square_map2() { return &square_map2_; }

  const AimedPoi &aimed_poi() const { return aimed_poi_; }
  AimedPoi *mutable_aimed_poi() { return &aimed_poi_; }

  const AimedPoi &parking_out() const { return parking_out_; }
  AimedPoi *mutable_parking_out() { return &parking_out_; }

  // const ParkingUiResponse &parking_ui_response() const {
  //   return parking_ui_response_;
  // }
  // ParkingUiResponse *mutable_parking_ui_response() {
  //   return &parking_ui_response_;
  // }

  // const ParkingUiReport &parking_ui_report() const {
  //   return parking_ui_report_;
  // }
  // ParkingUiReport *mutable_parking_ui_report() { return &parking_ui_report_;
  // }

  // const ParkingUiNotificationMsg &parking_ui_notification_msg() const {
  //   return parking_ui_notification_msg_;
  // }
  // ParkingUiNotificationMsg *mutable_parking_ui_notification_msg() {
  //   return &parking_ui_notification_msg_;
  // }

  const ParkingBehaviorPlannerOutput &parking_behavior_planner_output() const {
    return parking_behavior_planner_output_;
  }
  ParkingBehaviorPlannerOutput *mutable_parking_behavior_planner_output() {
    return &parking_behavior_planner_output_;
  }

  // const FreespaceDeciderOutput &freespace_decider_output() const {
  //   return freespace_decider_output_;
  // }
  // FreespaceDeciderOutput *mutable_freespace_decider_output() {
  //   return &freespace_decider_output_;
  // }
  const maf_vehicle_status::TurnSignalType &turn_signal_cmd() const {
    return turn_signal_cmd_;
  }
  maf_vehicle_status::TurnSignalType *mutable_turn_signal_cmd() {
    return &turn_signal_cmd_;
  }

  const OpenspaceDeciderOutput &openspace_decider_output() const {
    return openspace_decider_output_;
  }
  OpenspaceDeciderOutput *mutable_openspace_decider_output() {
    return &openspace_decider_output_;
  }

  const OpenspaceMotionPlannerOutput &openspace_motion_planner_output() const {
    return openspace_motion_planner_output_;
  }
  OpenspaceMotionPlannerOutput *mutable_openspace_motion_planner_output() {
    return &openspace_motion_planner_output_;
  }

  const std::string& planning_debug_info() const {
    return planning_debug_info_;
  }

  std::string *mutable_planning_debug_info() {
    return &planning_debug_info_;
  }

  const int sbp_request_count() const {
    return sbp_request_count_;
  }

  int *mutable_sbp_request_count() {
    return &sbp_request_count_;
  }

  std::string *mutable_now_time_seq_str() {
    return &now_time_seq_str_;;
  }

  const std::vector<std::vector<std::pair<double, double>>>& vec_sl_points() const {
    return vec_sl_points_;
  }

  std::vector<std::vector<std::pair<double, double>>>* const mutable_vec_sl_points() {
    return &vec_sl_points_;
  }

  const double& planning_start_time_ms() const {
    return planning_start_time_ms_;
  }

  double* mutable_planning_start_time_ms() {
    return &planning_start_time_ms_;
  }

  bool* mutable_is_planner_update_plan_path() {
    return &is_planner_update_plan_path_;
  }

  std::shared_ptr<grid::MultiCircleFootprintModel>& lon_mc_footprint_model() {
    return lon_mc_footprint_model_;
  }

  std::vector<planning_math::Vec2d>* const mutable_full_body_model_contour() {
    return &full_body_model_contour_;
  }

  std::vector<std::vector<planning_math::Vec2d>>* const mutable_vec_extra_obstacle_points() {
    return &vec_extra_obstacle_points_;
  }
  
  const ParkingLateralBehaviorPlannerOutput &
  parking_lateral_behavoir_planner_output() const {
    return parking_lateral_behavoir_planner_output_;
  }
  ParkingLateralBehaviorPlannerOutput *
  mutable_parking_lateral_behavoir_planner_output() {
    return &parking_lateral_behavoir_planner_output_;
  }

  bool has_scene(int scene, int value) { return (scene & value) != 0; }

  int add_scene(int scene, int value) {
    if (has_scene(scene, value)) {
      return scene;
    }
    return (scene | value);
  }

  int remove_scene(int scene, int value) {
    if (!has_scene(scene, value)) {
      return scene;
    }
    return (scene ^ value);
  }

  const std::string &get_config_file_dir() const { return config_file_dir_; }

  void set_config_file_dir(const std::string &config_file_dir) {
    config_file_dir_ = config_file_dir;
  }

  const std::string &get_version() const { return version_; }

  void set_version(const std::string &version) { version_ = version; }

  mjson::Json encoder_planning_context() const {
    // 反馈planning_status_, parking_behavior_planner_output_, parking_lateral_behavoir_planner_output_
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["planning_status_"] = planning_status_.encoder();
    result["parking_behavior_planner_output_"] = parking_behavior_planner_output_.encoder();
    result["parking_lateral_behavoir_planner_output_"] = parking_lateral_behavoir_planner_output_.encoder();
    result["longitudinal_behavior_planner_output_"] = longitudinal_behavior_planner_output_.encoder();
    return result;
    // return planning_status_.encoder();
  }

private:
  FallBackInfo fallback_info_;
  PlanningStatus planning_status_;
  LongitudinalBehaviorPlannerOutput longitudinal_behavior_planner_output_;

  OpenspaceDeciderOutput openspace_decider_output_;
  OpenspaceMotionPlannerOutput openspace_motion_planner_output_;
  ReferenceLine reference_line_;
  OpenSpacePath open_space_path_;
  SquareMap square_map_;
  SquareMap square_map2_;
  AimedPoi aimed_poi_;
  AimedPoi parking_out_;
  // ParkingUiNotificationMsg parking_ui_notification_msg_;
  // ParkingUiResponse parking_ui_response_;
  // ParkingUiReport parking_ui_report_;

  ParkingBehaviorPlannerOutput parking_behavior_planner_output_;
  ParkingLateralBehaviorPlannerOutput parking_lateral_behavoir_planner_output_;
  // FreespaceDeciderOutput freespace_decider_output_;
  maf_vehicle_status::TurnSignalType turn_signal_cmd_;

  std::string config_file_dir_;
  std::string version_;

  // planning_debug_info for apa debug
  std::string planning_debug_info_;
  std::vector<std::vector<std::pair<double, double>>> vec_sl_points_;
  double planning_start_time_ms_;
  bool is_planner_update_plan_path_;
  int sbp_request_count_;
  std::string now_time_seq_str_;

  // ego model
  std::shared_ptr<grid::MultiCircleFootprintModel> lon_mc_footprint_model_;
  std::vector<planning_math::Vec2d> full_body_model_contour_;
  
  std::vector<std::vector<planning_math::Vec2d>> vec_extra_obstacle_points_;


  // zjt debug
  // public:
  // const OpenSpacePath &last_apf_path() const { return last_apf_path_; }
  // OpenSpacePath *mutable_last_apf_path() { return &last_apf_path_; }
  // const ApfStatus &apf_status() const { return apf_status_; }
  // ApfStatus *mutable_apf_status() { return &apf_status_; }
  // const Pose2D &park_out_target_pose() {return park_out_target_pose_; };
  // Pose2D *mutable_park_out_target_pose(){return &park_out_target_pose_; };
  // private:
  //   OpenSpacePath last_apf_path_;
  //   ApfStatus apf_status_;
  //   Pose2D park_out_target_pose_;
};

} // namespace parking

} // namespace msquare

#endif // COMMON_PLANNING_CONTEXT_
