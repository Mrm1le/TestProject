#include "pnc/planning_engine_interface.h"

#include "common/planning_engine.h"

namespace msquare {

MSD_API std::shared_ptr<PlanningEngineInterface>
PlanningEngineInterface::make(const MSDPlanningConfig &planning_config) {
  return std::make_shared<PlanningEngine>(planning_config);
}

} // namespace msquare
