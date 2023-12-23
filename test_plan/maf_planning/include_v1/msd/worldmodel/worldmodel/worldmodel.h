#pragma once

#include "maf_interface.h"
#include "monitor_define/node_status_define.h"
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace cp_worldmodel {
struct LaneId {
  int64_t tile_id = -1000;
  int road_count = 0;
  bool dir_reversed = false;
  int lane_count = 0;
};

struct LaneData {
  int32_t relative_i = -1000;
  int32_t track_id = -1000;
  LaneId lane_id;
  maf_worldmodel::Direction lane_marks;
  maf_worldmodel::LaneType lane_type;
  maf_worldmodel::LaneBoundary left_lane_boundary;
  maf_worldmodel::LaneBoundary right_lane_boundary;
  maf_worldmodel::ReferenceLine reference_line;
  double entrance_width = 0.;
  maf_worldmodel::MergePoint merge_point;
  maf_worldmodel::MergePoint y_point;
  int8_t bias = 0;
};
struct InnerIntersectionData {
  maf_worldmodel::IntersectionData intersectionData;
  int intersection_id;
  int v2x_intersection_id;
  double enter_yaw;
  double exit_yaw;
};
struct InnerProcessedMapData {
  uint8_t available;
  maf_worldmodel::TargetPosition target_position;
  maf_worldmodel::SelfPositionData self_position;
  std::vector<LaneData> lanes;
  std::vector<maf_worldmodel::MapPOIInfoData> map_poi_info;
  std::vector<InnerIntersectionData> intersections;
  std::vector<maf_worldmodel::LaneMergingSplittingPointData>
      lane_merging_splitting_points;
  maf_worldmodel::LaneStrategy lane_stratery;
  int intersection_id;
  int v2x_intersection_id;
  double enter_yaw;
  double exit_yaw;

  enum : uint8_t {
    TARGET_POTISION = 1,
    SELF_POSITION = 2,
    LANE = 4,
    MAP_POI_INFO = 8,
    INTERSECTION = 16,
    LANE_MERGING_SPLITTING_POINT = 32,
    LANE_STRATERY = 64,
  };
};

struct HealthStatus {
  node_status::Status status;
  std::string message;
};

struct TopicStatus {
  std::string topic_name;
  std::string topic_type;
  double normal_hz;
  double warning_hz;
  double error_hz;
  bool hdmap;
  bool ddmap;
};

} // namespace cp_worldmodel
