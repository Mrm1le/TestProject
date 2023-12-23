#ifndef COMMON_PLANNING_STATUS_
#define COMMON_PLANNING_STATUS_

#include "maf_interface/maf_planning.h"
#include "msd/planning.h"
#include "pnc/define/geometry.h"
#include "nlohmann/json.hpp"
#include "mtime_core/mtime.h"
#include "mjson/mjson.hpp"
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

namespace msquare {

typedef struct {
  std::vector<int> target_lane_cars;
  double most_front_car_vel{100.0};
  double min_gap_length{0.0};
  double max_gap_length{0.0};

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    mjson::Json::array target_lane_cars_array{};
    for (const auto &car : target_lane_cars) {
      target_lane_cars_array.emplace_back(mjson::Json(car));
    }
    result["target_lane_cars"] = mjson::Json(target_lane_cars_array);
    result["most_front_car_vel"] = mjson::Json(most_front_car_vel);
    result["min_gap_length"] = mjson::Json(min_gap_length);
    result["max_gap_length"] = mjson::Json(max_gap_length);
    return result;
  }
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

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["status"] = mjson::Json(static_cast<int>(status));
    result["path_id"] = mjson::Json(path_id);
    result["is_active_lane_change"] = mjson::Json(is_active_lane_change);
    result["direction"] = mjson::Json(direction);
    result["start_timestamp"] = mjson::Json(start_timestamp);
    result["exist_lane_change_start_position"] = mjson::Json(exist_lane_change_start_position);
    result["lane_change_start_position"] = 
        mjson::Json(mjson::Json::array{mjson::Json(lane_change_start_position.x), mjson::Json(lane_change_start_position.y)});
    result["last_succeed_timestamp"] = mjson::Json(last_succeed_timestamp);
    result["is_current_opt_succeed"] = mjson::Json(is_current_opt_succeed);
    result["gap_length_before_change"] = mjson::Json(gap_length_before_change);
    result["target_gap_obs"] = 
        mjson::Json(mjson::Json::array{mjson::Json(target_gap_obs.first), mjson::Json(target_gap_obs.second)});
    result["target_lane_traffic_flow"] = target_lane_traffic_flow.encoder();
    result["enable_gap_protection"] = mjson::Json(enable_gap_protection);
    result["gap_protection_counter"] = mjson::Json(gap_protection_counter);
    result["set_virtual_obstacle"] = mjson::Json(set_virtual_obstacle);
    result["virtual_obstacle_distance"] = mjson::Json(virtual_obstacle_distance);
    result["origin_lane_leader_id"] = mjson::Json(origin_lane_leader_id);
    result["left_dash_line_length"] = mjson::Json(left_dash_line_length);
    result["lane_change_wait_time"] = mjson::Json(lane_change_wait_time);
    result["enable_interactive_mode"] = mjson::Json(enable_interactive_mode);
    result["enable_lane_change_traj_checker"] = mjson::Json(enable_lane_change_traj_checker);
    result["rival_obstacle_id"] = mjson::Json(rival_obstacle_id);
    return result;
  }
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

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["status"] = mjson::Json(static_cast<int>(status));
    result["change_lane"] = change_lane.encoder();
    result["target_lane_id"] = mjson::Json(target_lane_id);
    result["target_lane_map_id"] = mjson::Json(target_lane_map_id);
    result["target_lane_lat_offset"] = mjson::Json(target_lane_lat_offset);
    return result;
  }
} LaneStatus;

typedef struct {
  int id;
  double start_stopstamp;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["id"] = mjson::Json(id);
    result["start_stopstamp"] = mjson::Json(start_stopstamp);
    return result;
  }
} StopTime;

typedef struct {
  std::string initial_direction{"none"};
  std::string current_direction{"none"};
  double start_stopstamp;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["initial_direction"] = mjson::Json(initial_direction);
    result["current_direction"] = mjson::Json(current_direction);
    result["start_stopstamp"] = mjson::Json(start_stopstamp);
    return result;
  }
} StopInfo;

typedef struct {
  int crosswalk_id;
  std::unordered_map<int, double> stop_times; // stop before pedestrain
  // std::vector<StopTime> stop_times;
  std::unordered_map<int, StopInfo> stopline_times; // stop before crosswalk

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["crosswalk_id"] = mjson::Json(crosswalk_id);
    mjson::Json stop_times_map = mjson::Json(mjson::Json::object());
    for(auto it = stop_times.begin(); it != stop_times.end(); ++it)
    {
      stop_times_map[std::to_string(it->first)] = mjson::Json(it->second);
    }
    result["stop_times"] = stop_times_map;
    mjson::Json stopline_times_map = mjson::Json(mjson::Json::object());
    for(auto it = stop_times.begin(); it != stop_times.end(); ++it)
    {
      stopline_times_map[std::to_string(it->first)] = mjson::Json(it->second);
    }
    result["stopline_times_map"] = stopline_times_map;
    return result;
  }
} CrosswalkStatus;

typedef struct {
  double last_rerouting_time;
  bool need_rerouting;
  std::string routing_request;

   mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["last_rerouting_time"] = mjson::Json(last_rerouting_time);
    result["need_rerouting"] = mjson::Json(need_rerouting);
    result["routing_request"] = mjson::Json(routing_request);
    return result;
   }
} ReroutingStatus;

typedef struct {
  std::unordered_map<int, bool> junction;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    mjson::Json junction_map = mjson::Json(mjson::Json::object());
    for(auto it = junction.begin(); it != junction.end(); ++it)
    {
      junction_map[std::to_string(it->first)] = mjson::Json(it->second);
    }
    result["junction"] = junction_map;
    return result;
  }
} RightOfWayStatus;

typedef struct {
  std::string scenario_type;
  std::string stage_type;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["scenario_type"] = mjson::Json(scenario_type);
    result["stage_type"] = mjson::Json(stage_type);
    return result;
  }
} ScenarioStatus;

typedef struct {
  std::string traffic_light_status;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["traffic_light_status"] = mjson::Json(traffic_light_status);
    return result;
  }
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

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["prebrake_acc"] = mjson::Json(prebrake_acc);
    result["prebrake_duration"] = mjson::Json(prebrake_duration);
    result["preacc_duration"] = mjson::Json(preacc_duration);
    result["enable_prebrake"] = mjson::Json(enable_prebrake);
    result["enable_preacc"] = mjson::Json(enable_preacc);
    result["n_prebrake_slip"] = mjson::Json(n_prebrake_slip);
    result["n_prebrake_slow"] = mjson::Json(n_prebrake_slow);
    result["n_preacc"] = mjson::Json(n_preacc);
    result["v_lim_curv"] = mjson::Json(v_lim_curv);
    result["a_lim_curv"] = mjson::Json(a_lim_curv);
    result["n_prebrake_curv"] = mjson::Json(n_prebrake_curv);
    return result;
  }
} PreActionResults;

struct AvdInfo {
  int priority;
  int ignore_loop;
  bool lon_ignore;
  double time_buffer;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["priority"] = mjson::Json(priority);
    result["ignore_loop"] = mjson::Json(ignore_loop);
    result["lon_ignore"] = mjson::Json(lon_ignore);
    result["time_buffer"] = mjson::Json(time_buffer);
    return result;
  }
};

struct ObstacleDisInfo {
  int obs_id{0};
  double time{0.0};
  double distance{std::numeric_limits<double>::max()};

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["obs_id"] = mjson::Json(obs_id);
    result["time"] = mjson::Json(time);
    result["distance"] = mjson::Json(distance);
    return result;
  }
};

struct TrajCollisionCheckInfo {
  ObstacleDisInfo min_obs_dis;
  ObstacleDisInfo min_obs_lat_dis;
  ObstacleDisInfo min_obs_lon_dis;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["min_obs_dis"] = min_obs_dis.encoder();
    result["min_obs_lat_dis"] = min_obs_lat_dis.encoder();
    result["min_obs_lon_dis"] = min_obs_lon_dis.encoder();
    return result;
  }
};

typedef struct {
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

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["timestamp_sec"] = mjson::Json(timestamp_sec);
    result["next_timestamp_sec"] = mjson::Json(next_timestamp_sec);
    result["matched_scenario_type"] = mjson::Json(matched_scenario_type);
    mjson::Json::array traj_pose{};
    for(const auto &p : traj_pose_array) {
      traj_pose.emplace_back(mjson::Json(mjson::Json::array{mjson::Json(p.position_enu.x), mjson::Json(p.position_enu.y),
                             mjson::Json(p.heading_yaw), mjson::Json(p.curvature), mjson::Json(p.path_follow_strength)}));
    }
    result["traj_pose_array"] = mjson::Json(traj_pose);
    mjson::Json::array traj_vel{};
    for(const auto &p : traj_vel_array) {
      traj_vel.emplace_back(mjson::Json(mjson::Json::array{mjson::Json(p.target_velocity), mjson::Json(p.relative_time),
                            mjson::Json(p.distance)}));
    }
    result["traj_vel_array"] = mjson::Json(traj_vel);
    mjson::Json::array traj_acc{};
    for(const auto &p : traj_acceleration) {
      traj_acc.emplace_back(mjson::Json(p));
    }
    result["traj_acceleration"] = mjson::Json(traj_acc);
    mjson::Json::array traj_pose_f{};
    for(const auto &p : traj_pose_frenet) {
      traj_pose_f.emplace_back(mjson::Json(mjson::Json::array{mjson::Json(p.x), mjson::Json(p.y)}));
    }
    result["traj_pose_frenet"] = mjson::Json(traj_pose_f);
    return result;
  }
} PlanningResult;

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

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["planning_loop"] = mjson::Json(planning_loop);
    result["last_planning_success"] = mjson::Json(last_planning_success);
    result["planning_success"] = mjson::Json(planning_success);
    result["use_stitching_trajectory"] = mjson::Json(use_stitching_trajectory);
    result["v_limit"] = mjson::Json(v_limit);
    result["a_limit"] = mjson::Json(a_limit);
    result["pre_action"] = pre_action.encoder();
    result["lane_status"] = lane_status.encoder();
    result["crosswalk"] = crosswalk.encoder();
    result["rerouting"] = rerouting.encoder();
    result["right_of_way"] = right_of_way.encoder();
    result["scenario"] = scenario.encoder();
    result["traffic_light"] = traffic_light.encoder();
    result["planning_result"] = planning_result.encoder();
    result["pre_planning_result"] = pre_planning_result.encoder();
    result["scheme_stage"] = mjson::Json(static_cast<int>(scheme_stage));
    result["backup_consecutive_loops"] = mjson::Json(backup_consecutive_loops);
    result["backup_reason"] = mjson::Json(backup_reason);
    result["time_consumption"] = mjson::Json(time_consumption);
    result["trigger_msg_id"] = mjson::Json(trigger_msg_id);
    return result;
  }
} PlanningStatus;

struct PathPose {
  Point3D pos;
  Quaternion orient;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["pos"] = 
      mjson::Json(mjson::Json::array{mjson::Json(pos.x), mjson::Json(pos.y), mjson::Json(pos.z)});
    result["orient"] = 
      mjson::Json(mjson::Json::array{mjson::Json(orient.x), mjson::Json(orient.y), mjson::Json(orient.z), mjson::Json(orient.w)});
    return result;
  }
};

struct OpenSpacePath {
  std::vector<PathPose> path_poses;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    mjson::Json::array path_poses_array{};
    for (const auto &path_pose : path_poses) {
      path_poses_array.emplace_back(path_pose.encoder());
    }
    result["path_poses"] = mjson::Json(path_poses_array);
    return result;
  }
};

struct SquareMap {
  Point2D box_corners[4];
  double box_height = 0.0;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    mjson::Json::array box_corners_array{};
    for (int i = 0; i < 4; ++i) {
      box_corners_array.emplace_back(mjson::Json(mjson::Json::array{mjson::Json(box_corners[i].x), mjson::Json(box_corners[i].y)}));
    }
    result["box_corners"] = mjson::Json(box_corners_array);
    result["box_height"] = mjson::Json(box_height);
    return result;
  }
};

struct AimedPoi {
  int id = 0;
  std::string type;

  mjson::Json encoder() const {
    mjson::Json result = mjson::Json(mjson::Json::object());
    result["id"] = mjson::Json(id);
    result["type"] = mjson::Json(type);
    return result;
  }
};

} // namespace msquare
#endif // COMMON_PLANNING_STATUS_
