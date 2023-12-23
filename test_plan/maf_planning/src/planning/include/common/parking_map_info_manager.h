#pragma once

#include <memory>
#include <string>

#include "common/math/transform.h"
#include "common/planning_context.h"
#include "common/refline.h"
#include "common/trajectory_loader.h"
#include "planner/message_type.h"
#include "pnc/define/parking_map_info.h"

namespace msquare {

namespace parking {

class WorldModel;

enum class ScenarioType {
  STATE_INIT,
  STATE_LANEFOLLOW,
  STATE_STOPSIGN,
  STATE_INTERSECTION,
  STATE_ABORT,
  STATE_FINISH
};

struct MapTaskRange {
  double begin_pos;
  double end_pos;
};

enum class Pattern {
  UNKNOWN = 0,
  SOLIDIRCLE,
  LEFT_ARROW,
  STRAIGHT_ARROW,
  RIGHT_ARROW,
  OTHER
};

struct TrafficLightInfo {
  std::vector<Pattern> patterns;
  RoadType direction;
};

struct Segment {
  // add environment function
  int begin_index;
  int end_index;
  int direction;
  int lane_type;
  bool is_in_intersection;
};

struct BoundaryInfo {
  double length;
  int type;
};

struct MapInfo {

  MapInfo();
  double DEFAULT_MAX_STOP_DIS = 200.0;
  double distance_to_stopline{std::numeric_limits<double>::max()};
  bool is_in_intersection{false};
  bool is_on_ramp{false};
  double distance_to_crossing{std::numeric_limits<double>::max()};
  RoadType road_type;
  double v_cruise;
  ScenarioType scenario_type;
  PlanningType lane_change_type;
  PlanningType last_lane_change_type;
  LaneStatus prev_lane_status;
  double distance_to_lane_change_point{std::numeric_limits<double>::max()};
  std::vector<std::pair<double, double>> curr_lane_width;
  std::vector<std::pair<double, double>> target_lane_width;

  std::vector<RefLinePoint> ref_trajectory;
  // DiscretizedPath reference_line;

  std::vector<MapTaskRange> first_task_ranges;

  std::vector<RefLinePoint> current_refline_points;
  std::vector<RefLinePoint> right_refline_points;
  std::vector<RefLinePoint> left_refline_points;

  std::vector<BoundaryInfo> left_boundary_info;
  std::vector<BoundaryInfo> right_boundary_info;

  int current_lane_index;
  std::vector<int8_t> current_tasks;

  std::vector<RoadType> lanes_info_marks;

  TrafficLightInfo traffic_light;

  uint8_t next_merge_type;
  double distance_to_stop_line;
  // add environment members
  bool has_task;
  bool mmp_update;
  bool is_in_map_area;
  bool is_next_rightmost;

  int current_tasks_id;
  int left_lane_tasks_id;
  int right_lane_tasks_id;

  int current_parking_lot_id;
  bool flane_update;
  // std::vector<LaneBoundary> lane_boundaries;

  std::vector<BoundaryInfo> left_boundary;
  std::vector<BoundaryInfo> right_boundary;

  std::vector<Segment> curr_refline_segment;
  std::vector<Segment> left_refline_segment;
  std::vector<Segment> right_refline_segment;

  std::vector<int> lane_status;
  std::vector<int> lane_types;
  int curr_lane_index;
  int curr_lanes_num;
  int lc_map_decision;
  double lc_start_dis;
  double lc_end_dis;
  double global_end_dis;
  double dist_to_intsect;
  double dist_to_last_intsect;
  double intsect_length;
  double max_speed;
  uint8_t traffic_light_direction;
  std::vector<uint8_t> left_lane_direct;
  std::vector<uint8_t> right_lane_direct;

  double distance_to_aimed_poi;
  SquareMapResponse square_mapping_result;

  // RawRefLine c_raw_refline;
  // RawRefLine l_raw_refline;
  // RawRefLine r_raw_refline;

  // Lane clane;
  // Lane llane;
  // Lane rlane;
  // Lane rrlane;
  // Lane lllane;

  // VirtualLane flane;
  // VirtualLane olane;
  // VirtualLane tlane;

  RefLine f_refline;
};

class MapInfoManager {
public:
  bool update(const std::shared_ptr<WorldModel> &world_model);
  const MapInfo &get_map_info() const { return map_info_; }
  MapInfo &get_mutable_map_info() { return map_info_; }
  void set_map_info(const MapInfo &map_info);
  void set_map_info(const std::shared_ptr<WorldModel> &world_model);
  void set_map_info_from_apf();

private:
  void get_ref_line(const std::shared_ptr<WorldModel> &world_model,
                    std::vector<RefLinePoint> &ref_trajectory);
  void linear_interpolation_parking();
  void set_range_of_trajectory();
  bool get_ref_trajectory(const std::shared_ptr<WorldModel> &world_model);
  void frame_trans_from_car2enu(float lat_offset, const Transform &car2enu);
  void cut_loop(std::vector<RefLinePoint> &ref_trajectory);

  MapInfo map_info_;
  TrajectoryLoader trajectory_loader_ = TrajectoryLoader();
};

} // namespace parking

} // namespace msquare