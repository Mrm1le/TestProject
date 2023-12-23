#pragma once
#include "common/math/line_segment2d.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/sbp_obstacle_interface.h"

namespace msquare {

class SbpComfortGearZone : public SbpObstacleInterface {
private:
  planning_math::Box2d box_;

public:
  SbpComfortGearZone(planning_math::Box2d box);
  ~SbpComfortGearZone();
  virtual double getCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footpint_model) {
    if (node->previous == nullptr) {
      return 0;
    }
    bool flag = false;
    if (box_.DistanceTo({node->previous->x, node->previous->y}) > 0) {
      if (node->previous->vel * node->vel <= 0) {
        return 2 * HybridAstarConfig::GetInstance()->traj_gear_switch_penalty;
      }
    }
    return 0;
  }
  virtual bool checkCollision(const SearchNodePtr &node,
                              const FootprintModelPtr &footpint_model) {
    return false;
  }
  virtual double getDistance(const planning_math::Vec2d &point) {
    return INT_MAX;
  }
  virtual std::vector<planning_math::Vec2d>
  getNearestPoints(const planning_math::LineSegment2d &ego_centerline) {
    std::vector<planning_math::Vec2d> nearest_pts{};
    nearest_pts.emplace_back(0, 0, -1);
    return nearest_pts;
  }
};

inline SbpComfortGearZone::SbpComfortGearZone(planning_math::Box2d box)
    : box_(box) {}

inline SbpComfortGearZone::~SbpComfortGearZone() {}

} // namespace msquare