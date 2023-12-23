#pragma once

#include<stdint.h>
#include <vector>

namespace maf_lane {

struct Pose3D {
  double x;
  double y;
  double z;
};

struct CameraSource {
  uint8_t value{0};
};

struct LaneHorizontalTypeEnum {
  uint8_t value{0};
};

struct Lane {
  int index;
  uint32_t track_id;
  std::vector<double> points_3d_x;
  std::vector<double> points_3d_y;
  std::vector<double> enu_points_3d_x;
  std::vector<double> enu_points_3d_y;
  std::vector<double> ego_points_3d_x;
  std::vector<double> ego_points_3d_y;

  CameraSource camera_source;
  bool is_failed_3d{false};
  uint8_t lane_type{0};
  bool is_centerline{false};
  bool is_horizontal_line;
  LaneHorizontalTypeEnum lane_horizontal_type;
  double stopline_depth;
};

struct RoadEdge {
  int index;
  uint32_t track_id;
  std::vector<double> points_3d_x;
  std::vector<double> points_3d_y;
  std::vector<double> enu_points_3d_x;
  std::vector<double> enu_points_3d_y;

  CameraSource camera_source;
  bool is_failed_3d{false};
};

struct RoadLinePerception{
  std::vector<Lane> lanes;
  std::vector<RoadEdge> road_edges;
};

}