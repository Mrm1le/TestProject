#ifndef COMMON_LATERAL_BEHAVIOR_PLANNER_OUTPUT_
#define COMMON_LATERAL_BEHAVIOR_PLANNER_OUTPUT_

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

namespace msquare {

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

typedef enum {
  NO_CANCEL,
  SOLID_LC,
  MANUAL_CANCEL,
  TIMEOUT_LC,
  UNSUITABLE_VEL,
  ENV_ERROR
} IntCancelReasonType;

typedef enum {
  DEFAULT_LCC = 0,
  SUITABLE_LCC,
  SOLID_LINE_LCC,
  UNINSERTABLE_LCC,
  ENV_UNSUITABLE_LCC,
  VEHICLE_UNSUITABLE_LCC
} LaneChangeCondition;

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
  int type = 0;
  int status = 3;
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
  std::vector<PathPoint> path_points;
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

  IntCancelReasonType int_cancel_reason_fsm = NO_CANCEL;
  LaneChangeCondition lane_change_condition = DEFAULT_LCC;

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
  std::vector<AvdMsg> avd_info;

  int scenario = 1;
  int fix_refline_index = 0;
  int origin_refline_index = -99;
  int target_refline_index = -99;
  double flane_width = 3.8;
  std::vector<PathPoint> path_points;
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
  
  IntCancelReasonType int_cancel_reason_output = NO_CANCEL;
  LaneChangeCondition lane_change_condition = DEFAULT_LCC;
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
  
    int_cancel_reason_output = NO_CANCEL;
    lane_change_condition = LaneChangeCondition::DEFAULT_LCC;
    planner_context.clear();
  }
};

} // namespace msquare

#endif
