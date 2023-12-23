#pragma once

#include "common/config_context.h"
#include "common/tracked_object.h"
#include "proto_idl/interface_request_manager.hpp"

namespace cp {

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    RequestManagerParams, min_dist_to_intsect, min_dist_to_ramp, min_alc_speed,
    efficiency_cost_filter_time, blocked_unit_cost, conservative_unit_cost,
    min_safe_dist, none_to_trigger_thresh, balance_factor,
    min_saft_back_one_ttc, car_following_factor, base_ttc_factor,
    min_wait_time_after_change, min_wait_time_after_light, backone_safe_wait_process_scale,
    min_wait_time_enter_dbw)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    MapInfo, is_left_solid_lane, is_right_solid_lane, left_dash_lane_length,
    right_dash_lane_length, v_curvature_limit, v_cruise, v_cruise_current,
    v_cruise_change_dis, is_in_vision_intsect, dist_to_intsect, dist_to_ramp,
    dist_to_tollgate, lanes_num, current_lane_index, current_lane_marks,
    left_lane_marks, right_lane_marks)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SimpilfiedTrackedObject, track_id, type,
                                   length, width, center_x, center_y, s, l,
                                   theta, speed_yaw, a, v, v_lead, v_rel,
                                   vy_rel, d_rel, y_rel)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(RequestManagerInput, params, ego_length,
                                   enable_ilc, enable_alc, enable_recommend_alc,
                                   enable_merge_alc,
                                   enable_right_change, disable_auto_lane_change, ego_vel,
    t_headway, lc_status, last_lc_status, lc_direction, map_info, ego_blinker,
    ilc_limit_velocity, has_origin_lane, on_origin_lane, has_target_lane,
    on_target_lane, front_tracks_l, front_tracks_c, front_tracks_r,
    side_tracks_l, side_tracks_r, ego_lane_efficiency_cost_history,
    left_lane_efficiency_cost_history, right_lane_efficiency_cost_history,
    last_lane_change_finish_time)

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    RequestManagerOutput, ego_lane_efficiency_cost_history,
    left_lane_efficiency_cost_history, right_lane_efficiency_cost_history,
    not_left_change_reason, not_right_change_reason,
    last_lane_change_finish_time, left_back_one, right_back_one,
    ego_lane_front_objects, left_lane_front_objects, right_lane_front_objects,
    ego_lane_cost_now, left_lane_cost_now, right_lane_cost_now,
    ego_lane_cost_filtered, left_lane_cost_filtered, right_lane_cost_filtered,
    alc_recommend_dir)

} // namespace cp