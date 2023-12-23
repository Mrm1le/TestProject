#pragma once
#include "nlohmann/json.hpp"
#include "planner_constants.hpp"
#include "path_planner_interface.hpp"
#include "proto_idl/interface_obstacle_decider.hpp"
#include "planner_common_interface.hpp"

// #include "general_motion_planner_interface.hpp"
using json = nlohmann::json;

namespace cp_odc_interface {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsInfoOBD, lon_decision,lat_decision,
                                    nudge_side, 
                                    obj_state_init)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairIntObsInfo, first,
                                    second)
                              
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPoint, x, y, heading_angle, curvature, t,
                                    v, a, s, l, frenet_valid,x_local,y_local)                              

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DdpTrajectoryODC, trajectory,
                                    logit)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObstacleDeciderOutput,odc_decision_valid,
                                    obs_infos,obs_infos_map,current_lane_ddp_trajectory,ddp_multipath)

} // namespace cp_odc_interface