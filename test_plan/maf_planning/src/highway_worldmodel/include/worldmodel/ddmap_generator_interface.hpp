#pragma once
#include "maf_interface/maf_perception_interface.h"
#include "msd/worldmodel/worldmodel/worldmodel.h"
#include "msd/worldmodel/worldmodel_generator.h"
#include "worldmodel/utility.hpp"

namespace msd_worldmodel {

namespace worldmodel_v1 {

struct InterCenterLine {
  int center_line_index; // from right to left
  int left_lane_index;
  int right_lane_index;
  int relative_id;
  int index;
};

struct InterRoadEdge {
  bool left_road_edge_exist = false;
  int left_road_edge_index{};
  bool right_road_edge_exist = false;
  int right_road_edge_index{};
};

struct IntersectionPoint {
  bool is_valid = false;
  maf_perception_interface::Point3f intersection_point_car;
};

static constexpr auto INVALID_INDEX = -1;
static constexpr float INVALID_DISTANCE = std::numeric_limits<float>::max();
static constexpr auto KEY_LEFT_LANE = "left_lane";
static constexpr auto KEY_RIGHT_LANE = "right_lane";
static constexpr auto KEY_IS_INTERSECTION_POINT = "is_intersection_point";

bool update_center_lane_and_find_current_index(
    const maf_perception_interface::LanePerception &lane_perception,
    std::vector<InterCenterLine> &center_lines,
    IntersectionPoint &intersection_point);

class DdmapGenerator {
public:
  static std::shared_ptr<DdmapGenerator> make();
  virtual void updateLocation(MLALocalizationPtr localization_ptr) = 0;
  virtual std::shared_ptr<CoordinateTransformer> getCoordTransformer() = 0;

  virtual bool processLanePtr(const PerceptionLanePtr lane_ptr,
                              ProcessedMapPtr &processed_map) = 0;
  virtual void convertLane(const PerceptionLanePtr perception_lane_ptr,
                           ProcessedMapPtr &processed_map) = 0;

  virtual void convertTimeStamp(ProcessedMapPtr &processed_map) = 0;
  virtual void convertOther(ProcessedMapPtr &processed_map) = 0;
  virtual void convertSelfPosition(ProcessedMapPtr &processed_map) = 0;
  virtual void convertExtraInfo(ProcessedMapPtr &processed_map) = 0;
  virtual void reset() = 0;
  virtual ~DdmapGenerator() = default;

private:
  virtual bool getNearestLocalization(const uint64_t &query_timestamp,
                                      MLALocalizationPtr &localization_ptr) = 0;

  virtual void
  preProcessLanePtr(const PerceptionLanePtr perception_lane_ptr) = 0;

  virtual void
  calculateLaneBoundary(const maf_perception_interface::Lane &lane,
                        const int &lane_points_size,
                        maf_worldmodel::LaneBoundary &lane_boundary) = 0;
  virtual void
  calculateRoadEdge(const maf_perception_interface::RoadEdge &road_edge,
                    const int &road_edge_points_size,
                    maf_worldmodel::RoadEdge &output_road_edge) = 0;
  virtual void calculateNearestRotatedDistance(
      const std::vector<float> &lane_points_3d_x,
      const std::vector<float> &lane_points_3d_y, const int &lane_points_size,
      const int direction,
      const std::function<void(int index, float distance)> &post_processor) = 0;
};

} // namespace worldmodel_v1

} // namespace msd_worldmodel
