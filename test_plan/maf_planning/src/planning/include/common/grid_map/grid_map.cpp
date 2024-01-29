#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "common/grid_map/grid_map.h"

#include "planning/common/timer.h"

namespace msquare {
namespace grid {

/*
调用方法
  hybrid_a_star_solver_ = std::make_shared<HybridAstar2>(
      PlanningContext::Instance()->get_config_file_dir() +
      "/scenario_configs_json/parking/");
*/
GridMap::GridMap() {}

bool GridMap::Process() { return true; }

} // namespace grid
} // namespace msquare
