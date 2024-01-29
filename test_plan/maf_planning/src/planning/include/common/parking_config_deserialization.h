#pragma once
#include "common/config/vehicle_param.h"
#include "common/parking_planner_types.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/search_based_planner.h"

namespace msquare {
namespace parking {

void deserialization(const nlohmann::json &ha_cfg_json,
                     HybridAstarConfig *ha_cfg);
void deserialization(const nlohmann::json &veh_prm_json, VehicleParam *veh_prm);
void deserialization(const nlohmann::json &car_prm_json, CarParams *car_prm);
void deserialization(const nlohmann::json &stg_prm_json,
                     StrategyParams *stg_prm);
void deserialization(const nlohmann::json &param_json,
                     TrajectoryOptimizerConfig *param);

} // namespace parking
} // namespace msquare