#pragma once

#include <string>
#include <utility>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/vec2d.h"

namespace msquare {

struct LaneWaypoint {
  LaneWaypoint() = default;
  double s = 0.0;
  double l = 0.0;
};

class MapPathPoint : public planning_math::Vec2d {
public:
  MapPathPoint() = default;
  MapPathPoint(const planning_math::Vec2d &point, double heading)
      : Vec2d(point.x(), point.y()), heading_(heading) {}
  MapPathPoint(const planning_math::Vec2d &point, double heading,
               LaneWaypoint lane_waypoint)
      : Vec2d(point.x(), point.y()), heading_(heading) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }
  MapPathPoint(const planning_math::Vec2d &point, double heading,
               std::vector<LaneWaypoint> lane_waypoints)
      : Vec2d(point.x(), point.y()), heading_(heading),
        lane_waypoints_(std::move(lane_waypoints)) {}

  double heading() const { return heading_; }
  void set_heading(const double heading) { heading_ = heading; }

  const std::vector<LaneWaypoint> &lane_waypoints() const {
    return lane_waypoints_;
  }

  void add_lane_waypoint(LaneWaypoint lane_waypoint) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }
  void add_lane_waypoints(const std::vector<LaneWaypoint> &lane_waypoints) {
    lane_waypoints_.insert(lane_waypoints_.end(), lane_waypoints.begin(),
                           lane_waypoints.end());
  }

  void clear_lane_waypoints() { lane_waypoints_.clear(); }

protected:
  double heading_ = 0.0;
  std::vector<LaneWaypoint> lane_waypoints_;
};

class InterpolatedIndex {
public:
  InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}
  int id = 0;
  double offset = 0.0;
};

} // namespace msquare
