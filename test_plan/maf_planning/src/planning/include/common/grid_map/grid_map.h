#pragma once

#include "common/config/vehicle_param.h"
#include "common/grid_map/multi_circle_footprint_model.h"
#include "common/grid_map/obstacle_grid.h"

#include <iostream>
#include <memory>

namespace msquare {
namespace grid {

class GridMap {
public:
  GridMap();

  ~GridMap() = default;

  bool Process();

private:
  void updateConfig();

private:
  // std::shared_ptr<MultiCircleFootprintModel> mc_footprint_model_;
};

} // namespace grid
} // namespace msquare
