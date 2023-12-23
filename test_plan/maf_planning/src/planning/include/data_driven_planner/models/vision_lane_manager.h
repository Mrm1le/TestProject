#pragma once
#include <data_driven_planner/common/basic_types.h>
#include <deque>

namespace msquare {
namespace ddp {

struct LaneBoundary {
  std::vector<Point> points;
  double length = 0.0; // no use current!!!
  uint8_t type;
  int32_t id = -100;
};

struct LaneBoundaryGroup {
  std::vector<LaneBoundary> lane_boundaries;
  double timestamp;
};

class VisionLaneManager {
public:
  VisionLaneManager() = default;
  ~VisionLaneManager() = default;

  // feed
  void feed_vision_lane(
      double timestamp,
      const maf_perception_interface::RoadLinePerception &vision_lane);

  // get
  bool get_vision_lane(double timestamp, const Point &position,
                       std::vector<NodeLane> &lanes, double &lanes_ts);

  bool get_lane_proposal(double timestamp, const Point &position,
                         std::vector<NodeLane> &lane_proposals,
                         double &lane_proposals_ts);

  void get_lane_boundary_group(std::vector<LaneBoundary> &lane_boundaries,
                               double latest_fusion_timestamp,
                               double &lanes_ts);

  void get_lane_proposal_group(std::vector<LaneBoundary> &lane_boundaries,
                               double latest_fusion_timestamp,
                               double &lane_proposals_ts);

  LineType
  get_vectornet_lane_boundary_type(const LaneBoundary &lane_boundary_seg) const;

private:
  std::deque<LaneBoundaryGroup> lane_boundary_groups_;
  std::deque<LaneBoundaryGroup> lane_proposal_groups_;
};

} // namespace ddp
} // namespace msquare