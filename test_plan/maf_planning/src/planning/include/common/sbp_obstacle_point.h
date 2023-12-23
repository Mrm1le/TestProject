#pragma once
#include "common/math/vec2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"

namespace msquare {

class SbpObstaclePoint : public SbpObstacleInterface {
private:
  std::vector<planning_math::Vec2d> points_;

public:
  SbpObstaclePoint(const std::vector<planning_math::Vec2d> &points);
  ~SbpObstaclePoint();
  const std::vector<planning_math::Vec2d> &getPoints() const { return points_; }
  virtual double getCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footpint_model) {
    return 0;
  }
  virtual bool checkCollision(const SearchNodePtr &node,
                              const FootprintModelPtr &footpint_model) {
    return footpint_model->checkOverlap(Pose2D(node->x, node->y, node->theta),
                                        points_);
  }
  virtual double getDistance(const planning_math::Vec2d &point) {
    double min_dist = std::numeric_limits<double>::infinity();
    for (const planning_math::Vec2d &pt : points_) {
      min_dist = std::min(min_dist, pt.DistanceTo(point));
    }
    return min_dist;
  }
  virtual std::vector<planning_math::Vec2d>
  getNearestPoints(const planning_math::LineSegment2d &ego_centerline) {
    return points_;
  }
  virtual std::vector<planning_math::Vec2d>
  getDiscretePoints(double step) const {
    return points_;
  };
};

inline SbpObstaclePoint::SbpObstaclePoint(
    const std::vector<planning_math::Vec2d> &points)
    : points_(points) {}

inline SbpObstaclePoint::~SbpObstaclePoint() {}

} // namespace msquare