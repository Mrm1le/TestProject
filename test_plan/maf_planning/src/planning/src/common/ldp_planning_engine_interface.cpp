#include "pnc/ldp_planning_engine_interface.h"

#include "common/ldp_planning_engine.h"

namespace msquare {

MSD_API std::shared_ptr<LdpPlanningEngineInterface>
LdpPlanningEngineInterface::make(const MSDLdpPlanningConfig &planning_config) {
  return std::make_shared<LdpPlanningEngine>(planning_config);
}

} // namespace msquare
