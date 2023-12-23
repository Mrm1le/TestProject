#pragma once

#include "multi_circle_footprint_model.h"
#include "obstacle_grid.h"

namespace msquare {

namespace hybrid_a_star_2 {

inline bool
checkCollisionIfGearSwitch(bool forward, float x, float y, float cos_theta,
                           float sin_theta, float lat_inflation,
                           float lat_inflation_low, float lon_inflation,
                           MultiCircleFootprintModel &footprint_model,
                           const ObstacleGrid &obstacle_grid) {
  if (lon_inflation > lat_inflation) {
    float diff = (lon_inflation - lat_inflation) * (forward ? 1.0f : -1.0f);
    float check_x = x + diff * cos_theta;
    float check_y = y + diff * sin_theta;
    footprint_model.updatePose(check_x, check_y, cos_theta, sin_theta);
    MultiModelMinDistances min_dis =
        obstacle_grid.calcMultiModelMinDistance(footprint_model);
    if (min_dis.distances[MultiModelMinDistances::FULL_BODY] < lat_inflation ||
        min_dis.distances[MultiModelMinDistances::LOW_WHEELS] <
            lat_inflation_low) {
      return true;
    }
  }
  footprint_model.updatePose(x, y, cos_theta, sin_theta);
  MultiModelMinDistances min_dis =
      obstacle_grid.calcMultiModelMinDistance(footprint_model);
  if (min_dis.distances[MultiModelMinDistances::FULL_BODY] < lat_inflation ||
      min_dis.distances[MultiModelMinDistances::LOW_WHEELS] <
          lat_inflation_low) {
    return true;
  }
  return false;
}

inline float calcSpiralScale(float speed, float curvature_rate) {
  return std::sqrt(2.0f * speed / curvature_rate);
}

} // namespace hybrid_a_star_2

} // namespace msquare
