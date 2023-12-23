#ifndef CP_INTERFACE_REQUEST_MANAGER_H
#define CP_INTERFACE_REQUEST_MANAGER_H

#include <stdint.h>
#include <limits>
#include <string>
#include <vector>

#include "interface_planner_common.hpp"

namespace cp {

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
  double min_wait_time_after_light;
  double backone_safe_wait_process_scale;
  double min_wait_time_enter_dbw;
};

struct MapInfo {
  // solid line
  bool is_left_solid_lane;
  bool is_right_solid_lane;
  double left_dash_lane_length;
  double right_dash_lane_length;
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
  int32_t lanes_num;
  int32_t current_lane_index;
  uint8_t current_lane_marks;
  uint8_t left_lane_marks;
  uint8_t right_lane_marks;
};

struct LCDir{
    enum LaneChangeDirection { NONE = 0, LEFT = 1, RIGHT = 2};
}; 

struct SimpilfiedTrackedObject {
  int32_t track_id = 0; // 目标id
  int32_t type = 0;     // 目标类型, 定义为FusionObjectType

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
  bool enable_right_change;
  bool disable_auto_lane_change;
  bool enable_recommend_alc;
  bool enable_merge_alc;
  bool is_merge;
  bool is_alc_merge_request;
  int32_t merge_direction;
  double ego_vel;
  double t_headway;
  int32_t lc_status;
  int32_t last_lc_status;
  uint8_t lc_direction;
  int32_t last_lc_request;

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
  std::vector<double> ego_lane_efficiency_cost_history;
  std::vector<double> left_lane_efficiency_cost_history;
  std::vector<double> right_lane_efficiency_cost_history;
  double last_lane_change_finish_time;
  bool is_now_dbw_status;
  uint8_t country;
};

struct NotALCReason{
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
    WAIT_AFTER_LAST_CHANGE = 10,
    NOT_IN_DBW = 11,
    FRONT_INTERSECTION = 12,
    FRONT_RAMP = 13,
    SINGLE_LANE = 14,
    EGO_SPEED_LOW = 15,
    WAIT_AFTER_ENTERING_DBW = 16,
    CURVATURE_LIMIT = 17,
    IN_INTERSECTION = 18,
    AT_EUROPE = 19,
    WAIT_AFTER_LAST_LIGHT = 20,
    DISABLE_RIGHT_CHANGE = 21,
    DISABLE_AUTO_CHANGE_BY_MANUAL = 22
    };
};

struct RequestManagerOutput {
  std::vector<double> ego_lane_efficiency_cost_history;
  std::vector<double> left_lane_efficiency_cost_history;
  std::vector<double> right_lane_efficiency_cost_history;
  uint8_t not_left_change_reason;
  uint8_t not_right_change_reason;
  double last_lane_change_finish_time;
  SimpilfiedTrackedObject left_back_one;
  SimpilfiedTrackedObject right_back_one;
  std::vector<SimpilfiedTrackedObject> ego_lane_front_objects;
  std::vector<SimpilfiedTrackedObject> left_lane_front_objects;
  std::vector<SimpilfiedTrackedObject> right_lane_front_objects;
  double ego_lane_cost_now;
  double left_lane_cost_now;
  double right_lane_cost_now;
  double ego_lane_cost_filtered;
  double left_lane_cost_filtered;
  double right_lane_cost_filtered;
  uint8_t alc_recommend_dir;
  bool alc_merge_request_source;
};

}  // namespace path_planner

#endif