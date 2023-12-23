#ifndef CP_MSQUARE_DECISION_PLANNING_COMMON_PARKING_MAP_INFO_H
#define CP_MSQUARE_DECISION_PLANNING_COMMON_PARKING_MAP_INFO_H

#include <vector>

#include "geometry.h"
// #include "momenta_utils/cartesian_coordinate_system.h"

namespace cp {

struct AimedPoiInfo {
  uint32_t id = 0;
  uint32_t rerouting_status = 0;
  std::string type;
  double distance = 100;
  std::vector<Point3D> corners;
};

struct ParkingOutInfo {
  uint32_t id = 0;
  std::string type;
  std::vector<Point3D> corners;

  double apoa_yaw = 0.0;
  Point3D apoa_enu;
};

struct FuzzySearchResult {
  int id = 0;
  int prev_id = 0;
  int next_id = 0;
  int lane_id = 0;
  double time_stamp = 0.0;

  Point3D cpos;
  Point3D epos;
  std::vector<Point3D> corners;
};

struct AccurateSearchResult : public FuzzySearchResult {
  double inner_width = 0.0;
  double outer_width = 0.0;
  double left_dist = 0.0;
  double right_dist = 0.0;
  double score = 0.0;
};

enum class RoadBorderType {
  UNKNOWN,
  SOLID_LINE,
  DASH_LINE,
  VIRTUAL_LINE,
  ROAD_BORDER_PHYSICAL,
  ROAD_BORDER_VIRTUAL
  // 车道中心线
  // 车道线
  // 道路边缘线
  // 路沿
  // 隔离带
  // 减速带 ？
  // 闸道口
  // 出入口
};

struct ParkingLotDetectionInfo {
  int id = 0;
  bool is_good = false;
  bool is_empty = false;
  bool is_car_in = false;
  bool is_on_map_list = false;
  struct CornerPoint {
    Point3D position;
    bool is_visible;
    double confidence;
  };
  std::vector<CornerPoint> corners;
  struct WheelStop {
    bool available;
    Point3D point1;
    Point3D point2;
  };
  WheelStop wheel_stop;
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
  };
  VirtualCorner virtual_corner;
};

enum class LaneLineType {
  UNKNOWN,
  SOLID_LINE,
  DASH_LINE,
  // ...
  PHYSICAL,
};

struct SquareRoadBorder {
  int id = 0;
  std::vector<Point3D> pts;
  LaneLineType type = LaneLineType::UNKNOWN;
};

struct ObstacleItem {
  int id = 0;
  std::vector<Point3D> pts;
};

struct SquareObstacles {
  std::vector<ObstacleItem> pillar;
  std::vector<ObstacleItem> fire_exit_door;
  std::vector<ObstacleItem> unknown;
};

struct SquareMapResponse {
  std::vector<SquareRoadBorder> road_borders;
  SquareObstacles obstacles;
};

struct ParkingMapInfo {
  AimedPoiInfo aimed_poi_info;
  ParkingOutInfo parking_out_info;
  std::vector<FuzzySearchResult> fuzzy_search_results;
  std::vector<AccurateSearchResult> accurate_search_results;
  std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results;
  SquareMapResponse square_map_response;
};

} // namespace cp

#endif
