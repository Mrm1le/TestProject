#pragma once
#include "common/config/vehicle_param.h"
#include "common/parking_config_deserialization.h"
#include "common/parking_planner_types.h"
#include "maf_interface/maf_planning.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/search_based_planner.h"

namespace msquare {
namespace parking {

void convert_to_maf(const msquare::SbpResult &sbp_result,
                    maf_planning::Result &msg_result);
void convert_from_maf(const maf_planning::Result &msg_result,
                      SbpResult &sbp_result);

} // namespace parking
} // namespace msquare