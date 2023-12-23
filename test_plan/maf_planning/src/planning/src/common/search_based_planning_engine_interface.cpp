#include "pnc/search_based_planning_engine_interface.h"

#include "common/search_based_planning_engine.h"

namespace msquare {

std::shared_ptr<SearchBasedPlanningEngineInterface>
SearchBasedPlanningEngineInterface::make(
    const std::string &vehicle_calib_file,
    const std::string &mtaskflow_config_file,
    const std::string &config_file_dir) {
  return std::make_shared<parking::SearchBasedPlanningEngine>(
      vehicle_calib_file, mtaskflow_config_file, config_file_dir);
}

} // namespace msquare
