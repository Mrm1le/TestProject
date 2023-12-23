#pragma once
#include "common/math/line_segment2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"
#include "sbp_obstacle_util.h"

#include <limits>

namespace msquare {

class SbpMapLine : public SbpObstacleInterface {
private:
  std::vector<planning_math::LineSegment2d> lines_;

public:
  SbpMapLine(const std::vector<planning_math::LineSegment2d> &lines);
  ~SbpMapLine();
  virtual double getCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footpint_model) {
    return 0;
  }
  virtual bool checkCollision(const SearchNodePtr &node,
                              const FootprintModelPtr &footpint_model) {
    return footpint_model->checkOverlap(Pose2D(node->x, node->y, node->theta),
                                        lines_, true);
  }
  virtual double getDistance(const planning_math::Vec2d &point) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const planning_math::LineSegment2d &line : lines_) {
      min_dist = std::min(min_dist, line.DistanceTo(point));
    }
    return min_dist;
  }
  virtual std::vector<planning_math::Vec2d>
  getNearestPoints(const planning_math::LineSegment2d &ego_centerline) {
    std::vector<planning_math::Vec2d> nearest_pts{};
    for (auto line : lines_) {
      nearest_pts.push_back(getSingleNearestPoint(line, ego_centerline));
    }
    return nearest_pts;
  }
};

inline SbpMapLine::SbpMapLine(
    const std::vector<planning_math::LineSegment2d> &lines)
    : lines_(lines) {}

inline SbpMapLine::~SbpMapLine() {}

} // namespace msquare