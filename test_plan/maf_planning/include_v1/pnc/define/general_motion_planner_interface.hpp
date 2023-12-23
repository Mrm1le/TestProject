#pragma once
#include "nlohmann/json.hpp"
#include "planner_constants.hpp"
#include "proto_idl/interface_general_motion_planner.hpp"
#include "planner_common_interface.hpp"
using json = nlohmann::json;

namespace cp_gmp_interface {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SVPoint, s, v_min, v_max)



NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjInfoRefined, id, id_relative, headaway,
                                   pos_init)



NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TargetPoint, x, y, heading, vel, dheading)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DrivingTaskInfo, lc_time_pass,
                                   lc_time_remain, lc_dist_remain, lc_dist_pass,
                                   lc_time_total, lc_dist_total,
                                   lc_latdist_abs_past, lane_cross)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BehaviorCandidate, target_points,
                                   behavior_info, behavior_type, behavior_index)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPointGMP, x, y, s, l, t, vel)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryGMP, trajectory, motion_type,
                                   lc_direc)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(VectorTargetPoint, vec, __convert_to_list___)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ArraySVPoint, arr, __convert_to_list___)


NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    GeneralMotionPlannerOutput, target_points, speed_array, acce_array, t_array,
    trajectories_gmp, gmp_valid, pp_state, sp_state,
    lc_wait_speed_adjust_advice, lc_wait_speed_adjust_count,
    lc_wait_speed_advice, speed_segments, gap_info, cipv_info, lead_objs_info,
    s_v_array, s_v_array_out, lc_remain_dist, lc_remain_time, lc_action_state,
    lc_wait, motion_result, chosen_behavior_candidate_index,
    behavior_success_prob, lc_in_proceed_time, lc_to_static_obj_time)



NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObjSize, length, width)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ObsInfoGMP, lon_decision, lat_decision,
                                   nudge_side, obj_size, noticed_timer,
                                   obj_state_init, pred_trajectory
                                   )

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    GeneralMotionPlannerInput, lc_duration, gmp_suggest_lc_time, veh_set_speed,
    headaway, dheadaway, headaway_smoothed, highlevel_guidance, gmp_valid,
    pp_state, sp_state, obj_infos, ego_state, ego_state_cart, refline_pos,
    refline_heading, left_line, right_line, behavior_candidates, lc_task_info,
    valid_body, is_lane_stable, is_solid_line, has_olane, has_tlane,
    gmp_should_cancel, is_steer_over_limit, is_lca_state_activated,
    target_speed_lc_wait_max, total_time_offset, driving_model_config,
    one_lc_time)

} // namespace cp_gmp_interface