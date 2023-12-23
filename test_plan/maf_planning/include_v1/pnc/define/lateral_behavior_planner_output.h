#ifndef CP_COMMON_LATERAL_BEHAVIOR_PLANNER_OUTPUT_
#define CP_COMMON_LATERAL_BEHAVIOR_PLANNER_OUTPUT_

#include "geometry.h"
#include "optimal_planner_trigger.h"
#include "path_point.h"
#include <array>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <float.h>

namespace cp {

struct TrackInfo {
  TrackInfo() {}

  TrackInfo(int id, double drel, double vrel)
      : track_id(id), d_rel(drel), v_rel(vrel) {}

  TrackInfo(const TrackInfo &track_info) {
    track_id = track_info.track_id;
    d_rel = track_info.d_rel;
    v_rel = track_info.v_rel;
  }

  TrackInfo &operator=(const TrackInfo &track_info) {
    track_id = track_info.track_id;
    d_rel = track_info.d_rel;
    v_rel = track_info.v_rel;
    return *this;
  }

  void set_value(int id, double drel, double vrel) {
    track_id = id;
    d_rel = drel;
    v_rel = vrel;
  }

  void reset() {
    track_id = -10000;
    d_rel = 0.0;
    v_rel = 0.0;
  }

  int track_id = -10000;
  double d_rel = 0.0;
  double v_rel = 0.0;
};

typedef enum IntCancelReasonTypeEnum {
  NO_CANCEL,
  SOLID_LC,
  MANUAL_CANCEL,
  TIMEOUT_LC,
  UNSUITABLE_VEL,
  ENV_ERROR
} IntCancelReasonType;

typedef enum AlcCancelReasonTypeEnum {
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
} AlcCancelReasonType;

typedef enum LaneChangeConditionENUM {
  DEFAULT_LCC = 0,
  SUITABLE_LCC,
  SOLID_LINE_LCC,
  UNINSERTABLE_LCC,
  ENV_UNSUITABLE_LCC,
  VEHICLE_UNSUITABLE_LCC
} LaneChangeCondition;

typedef enum FailReasonEnum {
  UNSUITABLE = 0,
  FAIL_SOLID_LINE,
  TIME_OUT
} FailReason;

typedef enum ConDitionDetailEnum {
  DEFAULT = 0,
  LEFT_CHANGE_SUITABLE = 1,
  RIGHT_CHANGE_SUITABLE = 2,
  LEFT_CHANGE_SOLID_LINE = 3,
  RIGHT_CHANGE_SOLID_LINE = 4,
  LEFT_CHANGE_UNINSERTABLE = 5,
  RIGHT_CHANGE_UNINSERTABLE = 6,
  LEFT_CHANGE_LANE_UNSUITABLE = 7,
  RIGHT_CHANGE_LANE_UNSUITABLE = 8,
  LEFT_CHANGE_VEHICLE_UNSUITABLE = 9,
  RIGHT_LANE_VEHICLE_UNSUITABLE = 10,
  DISABLE_BY_MANUAL = 11,
  FRONT_RAMP_LCC = 12,
  FRONT_INTERSECTION_LCC = 13,
  EGO_SPEED_LOW_LCC = 14,
  SINGLE_LANE_LCC = 15,
  CURVATURE_LIMIT_LCC = 16,
  NO_LANE_LCC = 17
} ConDitionDetail;

struct CarCount {
  int pos;
  int neg;

  CarCount() {
    pos = 0;
    neg = 0;
  }

  CarCount(int p, int n) {
    pos = p;
    neg = n;
  }

  CarCount(const CarCount &car_cnt) {
    pos = car_cnt.pos;
    neg = car_cnt.neg;
  }

  CarCount &operator=(const CarCount &car_cnt) {
    pos = car_cnt.pos;
    neg = car_cnt.neg;
    return *this;
  }

  bool operator==(const CarCount &car_cnt) const {
    return (pos == car_cnt.pos && neg == car_cnt.neg);
  }
};

struct PathPlannerContext {
  bool premoving = false;
  double lane_width = 3.8;
  double lat_offset = 0;
  double curr_time = 0;
  std::array<double, 4> c_poly;
  std::array<double, 4> d_poly;
  std::array<std::vector<double>, 2> avd_car_past;

  void clear() {
    premoving = false;
    lane_width = 3.8;
    lat_offset = 0;
    curr_time = 0;
    c_poly = {};
    d_poly = {};
    avd_car_past = {};
  }
};

struct IntRequestContext {
  int request = 0;
  int turn_light = 0;
  int counter_left = 0;
  int counter_right = 0;
  double tstart = 0.0;
  double tfinish = 0.0;
};

struct MapRequestContext {
  int request = 0;
  int turn_light = 0;
  double tstart = 0.0;
  double tfinish = 0.0;
  int order = 0;
};

struct ActRequestContext {
  int request = 0;
  int turn_light = 0;
  double tstart = 0.0;
  double tfinish = 0.0;
  std::string act_request_source = "none";
};

struct LCRequestManagerContext {
  int request = 0;
  int request_source = 0;
  int turn_light = 0;
  IntRequestContext int_request;
  MapRequestContext map_request;
  ActRequestContext act_request;
};

struct RawRefLineContext {
  bool exist = false;
  int position = -100;
  std::vector<Point2D> waypoints;
  std::map<int32_t, bool> point_ids;
};

struct VirtualLaneContext {
  bool exist = false;
  int32_t track_id = -1;
  int type = 0;
  int status = 3;
  double max_refline_x = DBL_MAX;
  double min_refline_x = DBL_MAX;
  int source = 0;
  int position = -100;
  double width = 3.8;
  double front_width = 3.8;

  std::array<int, 2> rids;
  std::array<unsigned long, 2> ids;
  std::array<double, 2> intercepts;
  double dist_to_center_line = DBL_MAX;
  double curvature = 0.0;
  double relative_theta = 0.0;
  std::array<std::vector<double>, 2> polys;
  std::vector<double> c_poly;
  std::array<double, 2> heads;
  std::array<double, 2> tails;
  RawRefLineContext raw_refline;
  bool has_raw_refline = false;
  int master_position = -100;
};

struct FixRefLineContext {
  int position = -100;
  std::vector<cp::PathPoint> path_points;
};

struct VirtualLaneManagerContext {
  bool flane_update = false;
  VirtualLaneContext flane;
  VirtualLaneContext olane;
  VirtualLaneContext tlane;
  FixRefLineContext f_refline;
};

struct LaneTracksManagerContext {};

struct LateralBehaviorPlannerContext {
  bool no_sp_car = true;

  double t_avd_sp_car = 3.0;
  double final_y_rel = 10;

  int ncar_change = 0;
  int flag_avd = 0;
  int avd_back_cnt = 0;
  int avd_leadone = 0;
  int pre_leadone_id = 0;

  int ignore_track_id = -10000;

  PathPlannerContext path_planner;

  std::array<std::vector<double>, 2> avd_car_past;
  std::array<std::vector<double>, 2> avd_sp_car_past;

  std::vector<double> vel_sequence;
  std::set<int> ignore_change_false;
  std::set<int> ignore_change_true;

  void clear() {
     no_sp_car = true;

    t_avd_sp_car = 3.0;
    final_y_rel = 10;

    ncar_change = 0;
    flag_avd = 0;
    avd_back_cnt = 0;
    avd_leadone = 0;
    pre_leadone_id = 0;

    ignore_track_id = -10000;

    path_planner.clear();

    avd_car_past = {};
    avd_sp_car_past = {};

    vel_sequence.clear();
    ignore_change_false.clear();
    ignore_change_true.clear();
  }
};

struct MSDStateMachineOutput {
  int scenario;
  int curr_state;
  std::string state_name;
  std::string lc_back_reason = "none";
  std::string lc_invalid_reason = "none";
  std::string state_change_reason = "none";

  int turn_light;

  bool should_premove;
  bool should_suspend;
  bool must_change_lane;

  bool lc_pause;
  int lc_pause_id;
  double tr_pause_l;
  double tr_pause_s;

  int lc_request;
  int lc_request_source;
  int lc_turn_light;
  std::string act_request_source;

  double l_dash_length;
  double r_dash_length;

  cp::IntCancelReasonType int_cancel_reason_fsm =
      cp::IntCancelReasonType::NO_CANCEL;
  cp::AlcCancelReasonType alc_cancel_left_reason_output =
      cp::AlcCancelReasonType::DEFAULT_VAL;
  cp::AlcCancelReasonType alc_cancel_right_reason_output =
      cp::AlcCancelReasonType::DEFAULT_VAL;
  cp::LaneChangeCondition lane_change_condition =
      cp::LaneChangeCondition::DEFAULT_LCC;

  VirtualLaneManagerContext virtual_lane_mgr_context;
};

struct LateralBehaviorPlannerOutput {
  bool enable = false;
  int track_id = -10000;
  double v_limit = 40.0;
  double lat_offset = 0.0;
  std::string which_lane = "current_line";
  std::string lc_request = "none";
  std::string lc_status = "none";
  std::vector<cp::AvdMsg> avd_info;

  int scenario = 1;
  int fix_refline_index = 0;
  int origin_refline_index = -99;
  int target_refline_index = -99;
  double flane_width = 3.8;
  std::vector<cp::PathPoint> path_points;
  bool cur_wide_lane = false;
  bool tleft_lane = false;
  bool rightest_lane = false;
  bool isOnHighway = false;
  bool isRedLightStop = false;
  double dist_intersect = 1000;
  double intersect_length = 1000;
  double lc_end_dis = 10000;
  double dis_to_ramp = 10000;
  double l_dash_length = 0.;
  double r_dash_length = 0.;

  bool premoving = false;
  int lc_pause_id = -1000;
  bool lc_pause = false;
  double tr_pause_l = 0.0;
  double tr_pause_s = -100.0;
  bool must_change_lane = false;
  std::vector<double> c_poly;
  std::vector<double> d_poly;
  uint64_t planner_scene = 0;
  uint64_t planner_action = 0;
  uint64_t planner_status = 0;

  double lead_one_drel = 0.0;
  double lead_one_vrel = 0.0;

  std::string state_name = "none";
  std::string scenario_name = "none";
  std::string turn_light = "none";
  std::string lc_request_source = "none";
  std::string lc_better_request_source = "none";
  std::string act_request_source = "none";
  std::string turn_light_source = "none";

  std::vector<std::tuple<double, double>> s_v_limit;
  std::vector<std::tuple<double, double>> s_a_limit;
  std::vector<std::tuple<double, double>> s_r_offset;

  std::array<std::vector<double>, 2> avd_car_past;
  std::array<std::vector<double>, 2> avd_sp_car_past;
  std::vector<double> vel_sequence;

  std::set<int> ignore_change_false;
  std::set<int> ignore_change_true;

  std::array<double, 4> l_poly;
  std::array<double, 4> r_poly;

  bool avd_in_lane = false;

  cp::IntCancelReasonType int_cancel_reason_output =
      cp::IntCancelReasonType::NO_CANCEL;
  cp::AlcCancelReasonType alc_cancel_reason_output =
      cp::AlcCancelReasonType::DEFAULT_VAL;
  cp::LaneChangeCondition lane_change_condition =
      cp::LaneChangeCondition::DEFAULT_LCC;
  LateralBehaviorPlannerContext planner_context;

  void clear() {
    enable = false;
    track_id = -10000;
    v_limit = 40.0;
    lat_offset = 0.0;
    which_lane = "current_line";
    lc_request = "none";
    lc_status = "none";
    avd_info.clear();

    scenario = 1;
    fix_refline_index = 0;
    origin_refline_index = -99;
    target_refline_index = -99;
    flane_width = 3.8;
    path_points.clear();
    cur_wide_lane = false;
    tleft_lane = false;
    rightest_lane = false;
    isOnHighway = false;
    isRedLightStop = false;
    dist_intersect = 1000;
    intersect_length = 1000;
    lc_end_dis = 10000;
    dis_to_ramp = 10000;
    l_dash_length = 0.;
    r_dash_length = 0.;

    premoving = false;
    lc_pause_id = -1000;
    lc_pause = false;
    tr_pause_l = 0.0;
    tr_pause_s = -100.0;
    must_change_lane = false;
    c_poly.clear();
    d_poly.clear();
    planner_scene = 0;
    planner_action = 0;
    planner_status = 0;

    lead_one_drel = 0.0;
    lead_one_vrel = 0.0;

    state_name = "none";
    scenario_name = "none";
    turn_light = "none";
    lc_request_source = "none";
    lc_better_request_source = "none";
    act_request_source = "none";
    turn_light_source = "none";

    s_v_limit.clear();
    s_a_limit.clear();
    s_r_offset.clear();

    avd_car_past = {};
    avd_sp_car_past = {};
    vel_sequence.clear();

    ignore_change_false.clear();
    ignore_change_true.clear();

    l_poly = {};
    r_poly = {};

    avd_in_lane = false;

    int_cancel_reason_output = cp::IntCancelReasonType::NO_CANCEL;
    lane_change_condition = cp::LaneChangeCondition::DEFAULT_LCC;
    planner_context.clear();
  }
};

} // namespace cp

#endif
