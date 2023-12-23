#pragma once
#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include <algorithm>

namespace msquare {
inline planning_math::Vec2d
getSingleNearestPoint(const planning_math::LineSegment2d &obs_line,
                      const planning_math::LineSegment2d &ego_centerline) {
  if (obs_line.HasIntersect(ego_centerline)) {
    return planning_math::Vec2d(0, 0, -1);
  }

  // project obs_line onto ego_centerline
  std::vector<double> dis(4, std::numeric_limits<double>::infinity());
  dis[0] = ego_centerline.DistanceTo(obs_line.start());
  dis[1] = ego_centerline.DistanceTo(obs_line.end());

  // project ego_centerline onto obs_line
  planning_math::Vec2d *nearest_pt_2 = new planning_math::Vec2d();
  planning_math::Vec2d *nearest_pt_3 = new planning_math::Vec2d();
  dis[2] = obs_line.DistanceTo(ego_centerline.start(), nearest_pt_2);
  dis[3] = obs_line.DistanceTo(ego_centerline.end(), nearest_pt_3);

  int min_index = min_element(dis.begin(), dis.end()) - dis.begin();

  switch (min_index) {
  case 0:
    return obs_line.start();
  case 1:
    return obs_line.end();
  case 2:
    return *nearest_pt_2;
  case 3:
    return *nearest_pt_3;
  default:
    return planning_math::Vec2d(0, 0, -1);
  }

  // TODO(@lzq):
  // estimate if parallel situations lead to negative influence or corner cases
}
} // namespace msquare