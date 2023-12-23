#pragma once

#include "common/tracked_object.h"

namespace msquare {

struct RequestManagerParams {
  double ego_length;
  double min_dist_to_intsect;
  double min_dist_to_ramp;
  double min_alc_speed;
  double efficiency_cost_filter_time;
  double blocked_unit_cost;
  double conservative_unit_cost;
  double min_safe_dist;
  double none_to_trigger_thresh;
  double balance_factor;
  double min_saft_back_one_ttc;
  double car_following_factor;
  double base_ttc_factor;
  double min_wait_time_after_change;
};

struct MapInfo {
  // solid line
  bool is_left_solid_lane;
  bool is_right_solid_lane;
  // curvature limit
  double v_curvature_limit;
  // cruise limit
  double v_cruise;
  double v_cruise_current;
  double v_cruise_change_dis;
  // distance info
  bool is_in_vision_intsect;
  double dist_to_intsect;
  double dist_to_ramp;
  double dist_to_tollgate;
  // lanes info
  int lanes_num;
  int current_lane_index;
  uint8_t current_lane_marks;
  uint8_t left_lane_marks;
  uint8_t right_lane_marks;
};

enum LaneChangeDirection { NONE = 0, LEFT, RIGHT };

struct SimpilfiedTrackedObject {
  int track_id = 0; // 目标id
  int type = 0;     // 目标类型, 定义为FusionObjectType

  double length = 0; // 长
  double width = 0;  // 宽

  double center_x = 0; // 中心点的相对x距离(相对坐标原点在自车头, x为车头方向)
  double center_y =
      0; // 中心点的相对y距离(相对坐标原点在自车头, y为垂直车头方向向左)
  double s = 0; // 中心点的s(TrackletMaintainer中维护的相对坐标系下的frenet)
  double l = 0; // 中心点的l(TrackletMaintainer中维护的相对坐标系下的frenet)
  double theta = 0;     // 他车heading的相对朝向
  double speed_yaw = 0; // 他车速度的相对朝向

  double a = 0;      // 纵向加速度
  double v = 0;      // 绝对速度
  double v_lead = 0; // 纵向绝对速度
  double v_rel = 0;  // 纵向相对速度
  double vy_rel = 0; // 横向相对速度(l方向)

  double d_rel =
      0; // 他车相对自车的纵向距离
         // 1.他车车尾在自车车头前方(+)，d_rel = s_obj_tail - s_ego_head
         // 2.他车车头在自车车头后方(-)，d_rel = s_obj_head - s_ego_head
         // 3.他车车头超过自车车头但他车车尾未超过自车车头，d_rel = 0.0

  double y_rel = 0; // d_path基础上加了y_rel_ori - l
};

struct RequestManagerInput {
  // params
  RequestManagerParams params;

  // ego related
  double ego_length;
  bool enable_ilc;
  bool enable_alc;
  bool enable_recommend_alc;
  bool enable_merge_alc;
  double ego_vel;
  double t_headway;
  int lc_status;
  int last_lc_status;
  LaneChangeDirection lc_direction;

  // map_info
  MapInfo map_info;

  // ilc related
  uint8_t ego_blinker;
  double ilc_limit_velocity;

  // virtual lane related
  bool has_origin_lane;
  bool on_origin_lane;
  bool has_target_lane;
  bool on_target_lane;

  // obstacles
  std::vector<SimpilfiedTrackedObject> front_tracks_l;
  std::vector<SimpilfiedTrackedObject> front_tracks_c;
  std::vector<SimpilfiedTrackedObject> front_tracks_r;
  std::vector<SimpilfiedTrackedObject> side_tracks_l;
  std::vector<SimpilfiedTrackedObject> side_tracks_r;

  // lane efficiency cost history
  std::deque<double> ego_lane_efficiency_cost_history;
  std::deque<double> left_lane_efficiency_cost_history;
  std::deque<double> right_lane_efficiency_cost_history;
  double last_lane_change_finish_time;
};

enum NotLaneChangeReason {
  DEFAULT_VAL = 0,
  INPUT_INVALID = 1,
  NOT_ENABLE_ALC = 2,
  NOT_IN_ODD = 3,
  NO_LANE = 4,
  EFFICIENCY_NOT_BETTER = 5,
  SOLID_LINE = 6,
  DANGEROUS_OBJECT = 7,
  STATE_MACHINE_BACK = 8,
  IS_REVERSE_CHANGING = 9,
  WAIT_AFTER_LAST_CHANGE = 10
};

struct RequestManagerOutput {
  std::deque<double> ego_lane_efficiency_cost_history;
  std::deque<double> left_lane_efficiency_cost_history;
  std::deque<double> right_lane_efficiency_cost_history;
  NotLaneChangeReason not_left_change_reason;
  NotLaneChangeReason not_right_change_reason;
  double last_lane_change_finish_time;
  SimpilfiedTrackedObject left_back_one;
  SimpilfiedTrackedObject right_back_one;
  vector<SimpilfiedTrackedObject> ego_lane_front_objects;
  vector<SimpilfiedTrackedObject> left_lane_front_objects;
  vector<SimpilfiedTrackedObject> right_lane_front_objects;
  double ego_lane_cost_now;
  double left_lane_cost_now;
  double right_lane_cost_now;
  double ego_lane_cost_filtered;
  double left_lane_cost_filtered;
  double right_lane_cost_filtered;
  LaneChangeDirection alc_recommend_dir;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RequestManagerParams, min_dist_to_intsect,
                                   min_dist_to_ramp, min_alc_speed,
                                   efficiency_cost_filter_time,
                                   blocked_unit_cost, conservative_unit_cost,
                                   min_safe_dist, none_to_trigger_thresh,
                                   balance_factor, min_saft_back_one_ttc,
                                   car_following_factor, base_ttc_factor,
                                   min_wait_time_after_change)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    MapInfo, is_left_solid_lane, is_right_solid_lane, v_curvature_limit,
    v_cruise, v_cruise_current, v_cruise_change_dis, is_in_vision_intsect,
    dist_to_intsect, dist_to_ramp, dist_to_tollgate, lanes_num,
    current_lane_index, current_lane_marks, left_lane_marks, right_lane_marks)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SimpilfiedTrackedObject, track_id, type,
                                   length, width, center_x, center_y, s, l,
                                   theta, speed_yaw, a, v, v_lead, v_rel,
                                   vy_rel, d_rel, y_rel)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RequestManagerInput, params, ego_length,
                                   enable_ilc, enable_alc, enable_recommend_alc,
                                   enable_merge_alc, ego_vel,
    t_headway, lc_status, last_lc_status, lc_direction, map_info, ego_blinker,
    ilc_limit_velocity, has_origin_lane, on_origin_lane, has_target_lane,
    on_target_lane, front_tracks_l, front_tracks_c, front_tracks_r,
    side_tracks_l, side_tracks_r, ego_lane_efficiency_cost_history,
    left_lane_efficiency_cost_history, right_lane_efficiency_cost_history,
    last_lane_change_finish_time)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    RequestManagerOutput, ego_lane_efficiency_cost_history,
    left_lane_efficiency_cost_history, right_lane_efficiency_cost_history,
    not_left_change_reason, not_right_change_reason,
    last_lane_change_finish_time, left_back_one, right_back_one,
    ego_lane_front_objects, left_lane_front_objects, right_lane_front_objects,
    ego_lane_cost_now, left_lane_cost_now, right_lane_cost_now,
    ego_lane_cost_filtered, left_lane_cost_filtered, right_lane_cost_filtered,
    alc_recommend_dir)

} // namespace msquare