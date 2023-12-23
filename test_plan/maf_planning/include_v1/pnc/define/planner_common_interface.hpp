#pragma once
#include "proto_idl/interface_planner_common.hpp"
#include "nlohmann/json.hpp"
using json = nlohmann::json;

namespace cp_path_planner {
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Pose2d, x, y, theta)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point2d, x, y)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SafetyMargin, longitu, lateral)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjState, pos, accel, vel, heading, s,
                                   v_frenet, rel_s)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjPredSlice, safety_margin, obj_state_local,
                                   obj_state_env, lane_assignment)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EgoState, ego_vel, ego_steer_angle,
                                   ego_v_cruise, ego_pose)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VectorDouble, vec, __convert_to_list___)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VectorVectorDouble, vec_vec, __convert_to_list___)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairDoubleDouble, first, second,
                                   __convert_to_list___)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairIntDouble, first, second,
                                   __convert_to_list___)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PairIntBool, first, second,
                                   __convert_to_list___)

} // namespace path_planner
