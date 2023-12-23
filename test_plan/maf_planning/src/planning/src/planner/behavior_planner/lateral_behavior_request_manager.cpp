#include "planner/behavior_planner/lateral_behavior_request_manager.h"
#include "common/config_context.h"
#include "planner/behavior_planner/lateral_behavior_state.h"
#include "planner/motion_planner/speed_planner_ceres/speed_planner_constants.hpp"

namespace msquare {

LaneChangeRequest::LaneChangeRequest() {}

void LaneChangeRequest::gen_request(RequestType direction) {
  if (direction != LEFT_CHANGE && direction != RIGHT_CHANGE) {
    MSD_LOG(ERROR, "[LaneChangeRequest::gen_request] Illgeal direction[%d]",
            direction);
  }

  if (request_ == direction) {
    MSD_LOG(
        INFO,
        "[LaneChangeRequest::gen_request] duplicated request, direction[%d]",
        direction);
    return;
  }

  request_ = direction;
  turn_light_ = direction;
  tstart_ = get_system_time();
}

void LaneChangeRequest::finish() {
  if (request_ == NO_CHANGE) {
    MSD_LOG(INFO, "[LaneChangeRequest::finish] No request to finish");
    turn_light_ = NO_CHANGE;
    return;
  }

  request_ = NO_CHANGE;
  turn_light_ = NO_CHANGE;
  tfinish_ = get_system_time();
}

MapRequest::MapRequest(
    std::shared_ptr<RequestManagerOutput> request_manager_output) {
  request_manager_output_ = request_manager_output;
}

ActRequest::ActRequest(
    std::shared_ptr<RequestManagerOutput> request_manager_output) {
  request_manager_output_ = request_manager_output;
  left_back_one_.track_id = -1;
  right_back_one_.track_id = -1;
  not_left_change_reason_ = DEFAULT_VAL;
  not_right_change_reason_ = DEFAULT_VAL;
}

void ActRequest::update(const RequestManagerInput &request_manager_input) {
  if (!is_request_manager_input_valid(request_manager_input)) {
    clear_act_request_output();
    return;
  }
  // 1.update efficiency cost history
  auto ego_history = request_manager_input.ego_lane_efficiency_cost_history;
  auto left_history = request_manager_input.left_lane_efficiency_cost_history;
  auto right_history = request_manager_input.right_lane_efficiency_cost_history;
  update_efficiency_cost_history(request_manager_input, ego_history,
                                 left_history, right_history);
  MSD_LOG(INFO, "[AutoLaneChange] 1.update efficiency cost history");

  // 2.Calculate efficiency cost of lanes
  double ego_lane_cost =
      calculate_ego_lane_efficiency_cost(request_manager_input);
  double left_lane_cost =
      calculate_left_lane_efficiency_cost(request_manager_input);
  double right_lane_cost =
      calculate_right_lane_efficiency_cost(request_manager_input);
  MSD_LOG(INFO, "[AutoLaneChange] 2.Calculate current efficiency cost");
  MSD_LOG(INFO, "[AutoLaneChange] ego_lane_cost = %.2f", ego_lane_cost);
  MSD_LOG(INFO, "[AutoLaneChange] left_lane_cost = %.2f", left_lane_cost);
  MSD_LOG(INFO, "[AutoLaneChange] right_lane_cost = %.2f", right_lane_cost);
  MSD_LOG(INFO, "[AutoLaneChange] left_back_one_id = %d",
          left_back_one_.track_id);
  MSD_LOG(INFO, "[AutoLaneChange] right_back_one_id = %d",
          right_back_one_.track_id);
  request_manager_output_->ego_lane_cost_now = ego_lane_cost;
  request_manager_output_->left_lane_cost_now = left_lane_cost;
  request_manager_output_->right_lane_cost_now = right_lane_cost;

  // 3.Calculate final cost result
  const double planning_rate = 10.0;
  const double temp =
      request_manager_input.params.efficiency_cost_filter_time * planning_rate;
  const size_t min_history_size = static_cast<size_t>(temp);
  update_filtered_efficiency_cost(min_history_size, ego_lane_cost, ego_history);
  update_filtered_efficiency_cost(min_history_size, left_lane_cost,
                                  left_history);
  update_filtered_efficiency_cost(min_history_size, right_lane_cost,
                                  right_history);
  MSD_LOG(INFO, "[AutoLaneChange] 3.Calculate filtered efficiency cost");
  MSD_LOG(INFO, "[AutoLaneChange] ego_lane_cost = %.2f", ego_lane_cost);
  MSD_LOG(INFO, "[AutoLaneChange] left_lane_cost = %.2f", left_lane_cost);
  MSD_LOG(INFO, "[AutoLaneChange] right_lane_cost = %.2f", right_lane_cost);
  request_manager_output_->ego_lane_cost_filtered = ego_lane_cost;
  request_manager_output_->left_lane_cost_filtered = left_lane_cost;
  request_manager_output_->right_lane_cost_filtered = right_lane_cost;

  // 4.Decide change intention
  decide_lane_change_intention(request_manager_input, ego_lane_cost,
                               left_lane_cost, right_lane_cost);
  MSD_LOG(INFO, "[AutoLaneChange] 4.Decide change intention dir = %d",
          request());

  // 5.Fill act request output
  request_manager_output_->ego_lane_efficiency_cost_history = ego_history;
  request_manager_output_->left_lane_efficiency_cost_history = left_history;
  request_manager_output_->right_lane_efficiency_cost_history = right_history;
  request_manager_output_->not_left_change_reason = not_left_change_reason_;
  request_manager_output_->not_right_change_reason = not_right_change_reason_;
  request_manager_output_->left_back_one = left_back_one_;
  request_manager_output_->right_back_one = right_back_one_;
  MSD_LOG(INFO, "[AutoLaneChange] 5.Fill act request output");
  MSD_LOG(INFO, "[AutoLaneChange] not_left_change_reason = %d",
          not_left_change_reason_);
  MSD_LOG(INFO, "[AutoLaneChange] not_right_change_reason = %d",
          not_right_change_reason_);
  not_left_change_reason_ = DEFAULT_VAL;
  not_right_change_reason_ = DEFAULT_VAL;
}

bool ActRequest::is_request_manager_input_valid(
    const RequestManagerInput &request_manager_input) const {
  if (!is_map_info_valid(request_manager_input.map_info)) {
    return false;
  }
  return true;
}

bool ActRequest::is_map_info_valid(const MapInfo &map_info) const {
  if ((map_info.current_lane_index < 0) || (map_info.lanes_num <= 0) ||
      (map_info.current_lane_index >= map_info.lanes_num)) {
    return false;
  }
  return true;
}

void ActRequest::clear_act_request_output() {
  request_manager_output_->ego_lane_efficiency_cost_history.clear();
  request_manager_output_->left_lane_efficiency_cost_history.clear();
  request_manager_output_->right_lane_efficiency_cost_history.clear();
  finish();
  not_left_change_reason_ = INPUT_INVALID;
  not_right_change_reason_ = INPUT_INVALID;
}

void ActRequest::update_efficiency_cost_history(
    const RequestManagerInput &request_manager_input,
    std::deque<double> &ego_history, std::deque<double> &left_history,
    std::deque<double> &right_history) const {
  switch (request_manager_input.lc_direction) {
  case LaneChangeDirection::LEFT:
    right_history = ego_history;
    ego_history = left_history;
    left_history.clear();
    break;
  case LaneChangeDirection::RIGHT:
    left_history = ego_history;
    ego_history = right_history;
    right_history.clear();
    break;
  default:
    break;
  }
}

double ActRequest::calculate_ego_lane_efficiency_cost(
    const RequestManagerInput &request_manager_input) {
  request_manager_output_->ego_lane_front_objects.clear();
  double cost = 0.0;
  auto front_objects = request_manager_input.front_tracks_c;
  if (front_objects.empty()) {
    return cost;
  }
  std::sort(front_objects.begin(), front_objects.end(),
            [](const auto &a, const auto &b) { return a.d_rel < b.d_rel; });
  MSD_LOG(INFO, "[AutoLaneChange] ego_v = %.2f", request_manager_input.ego_vel);
  for (const auto &object : front_objects) {
    double obj_cost = calculate_object_cost(request_manager_input, object);
    MSD_LOG(
        INFO,
        "[AutoLaneChange] ego lane object-[%d] d_rel: %.2f, v: %.2f cost: %.2f",
        object.track_id, object.d_rel, object.v, obj_cost);
    cost = std::max(cost, obj_cost);
  }

  // for plot script
  request_manager_output_->ego_lane_front_objects = front_objects;
  return cost;
}

double ActRequest::calculate_left_lane_efficiency_cost(
    const RequestManagerInput &request_manager_input) {
  request_manager_output_->left_lane_front_objects.clear();
  left_back_one_.track_id = -1;
  const bool no_left_lane =
      (request_manager_input.map_info.current_lane_index == 0);
  if (no_left_lane) {
    return std::numeric_limits<double>::infinity();
  }

  std::vector<SimpilfiedTrackedObject> lane_objects;
  lane_objects.insert(lane_objects.end(),
                      request_manager_input.front_tracks_l.begin(),
                      request_manager_input.front_tracks_l.end());
  lane_objects.insert(lane_objects.end(),
                      request_manager_input.side_tracks_l.begin(),
                      request_manager_input.side_tracks_l.end());

  if (lane_objects.empty()) {
    return 0.0;
  }

  std::vector<SimpilfiedTrackedObject> front_objects;
  std::sort(lane_objects.begin(), lane_objects.end(),
            [](const auto &a, const auto &b) { return a.s < b.s; });
  for (auto it = lane_objects.begin(); it != lane_objects.end(); ++it) {
    if (it->d_rel < request_manager_input.params.min_safe_dist) {
      left_back_one_ = *it;
      continue;
    }
    front_objects.insert(front_objects.end(), it, lane_objects.end());
    break;
  }
  if ((left_back_one_.track_id != -1) && (left_back_one_.d_rel >= 0.0)) {
    front_objects.insert(front_objects.begin(), left_back_one_);
  }

  if (front_objects.empty()) {
    return 0.0;
  }

  double cost = 0.0;
  for (const auto &object : front_objects) {
    double obj_cost = calculate_object_cost(request_manager_input, object);
    MSD_LOG(INFO,
            "[AutoLaneChange] left lane object-[%d] d_rel: %.2f, v: %.2f cost: "
            "%.2f",
            object.track_id, object.d_rel, object.v, obj_cost);
    cost = std::max(cost, obj_cost);
  }

  // for plot script
  request_manager_output_->left_lane_front_objects = front_objects;
  return cost;
}

double ActRequest::calculate_right_lane_efficiency_cost(
    const RequestManagerInput &request_manager_input) {
  request_manager_output_->right_lane_front_objects.clear();
  right_back_one_.track_id = -1;
  const bool no_right_lane =
      (request_manager_input.map_info.current_lane_index ==
       (request_manager_input.map_info.lanes_num - 1));
  if (no_right_lane) {
    return std::numeric_limits<double>::infinity();
  }

  std::vector<SimpilfiedTrackedObject> lane_objects;
  lane_objects.insert(lane_objects.end(),
                      request_manager_input.front_tracks_r.begin(),
                      request_manager_input.front_tracks_r.end());
  lane_objects.insert(lane_objects.end(),
                      request_manager_input.side_tracks_r.begin(),
                      request_manager_input.side_tracks_r.end());

  if (lane_objects.empty()) {
    return 0.0;
  }

  std::vector<SimpilfiedTrackedObject> front_objects;
  std::sort(lane_objects.begin(), lane_objects.end(),
            [](const auto &a, const auto &b) { return a.s < b.s; });
  for (auto it = lane_objects.begin(); it != lane_objects.end(); ++it) {
    if (it->d_rel < request_manager_input.params.min_safe_dist) {
      right_back_one_ = *it;
      continue;
    }
    front_objects.insert(front_objects.end(), it, lane_objects.end());
    break;
  }
  if ((right_back_one_.track_id != -1) && (right_back_one_.d_rel >= 0.0)) {
    front_objects.insert(front_objects.begin(), right_back_one_);
  }

  if (front_objects.empty()) {
    return 0.0;
  }

  double cost = 0.0;
  for (const auto &object : front_objects) {
    double obj_cost = calculate_object_cost(request_manager_input, object);
    MSD_LOG(
        INFO,
        "[AutoLaneChange] right lane object-[%d] d_rel: %.2f, v: %.2f cost: "
        "%.2f",
        object.track_id, object.d_rel, object.v, obj_cost);
    cost = std::max(cost, obj_cost);
  }

  // for plot script
  request_manager_output_->right_lane_front_objects = front_objects;
  return cost;
}

double ActRequest::calculate_object_cost(
    const RequestManagerInput &request_manager_input,
    const SimpilfiedTrackedObject &object) {
  const double weight =
      calculate_weight(request_manager_input, object.d_rel, object.v);
  const double ego_prefered_v =
      std::min(request_manager_input.map_info.v_cruise_current,
               request_manager_input.map_info.v_curvature_limit);
  const double obj_cost = request_manager_input.params.conservative_unit_cost *
                          std::max(ego_prefered_v - object.v, 0.0) * weight;
  MSD_LOG(INFO,
          "[AutoLaneChange] object-[%d], v = %.2f, ego_prefered_v = %.2f, "
          "d_rel = %.2f, weight = %.2f, obj_cost = %.2f",
          object.track_id, object.v, ego_prefered_v, object.d_rel, weight,
          obj_cost);
  return obj_cost;
}

double
ActRequest::calculate_weight(const RequestManagerInput &request_manager_input,
                             const double d_rel, const double object_v) const {
  if (d_rel < 0.0) {
    return 0.0;
  }
  const double ego_prefered_v =
      std::min(request_manager_input.map_info.v_cruise_current,
               request_manager_input.map_info.v_curvature_limit);
  if (object_v >= ego_prefered_v) {
    return 0.0;
  }

  const double base_ttc =
      request_manager_input.params.car_following_factor /
      (1.0 - request_manager_input.params.car_following_factor) *
      request_manager_input.t_headway *
      request_manager_input.params.base_ttc_factor;

  const double ttc = d_rel / (ego_prefered_v - object_v);
  if (ttc > base_ttc) {
    return 0.0;
  } else {
    return 1.0 - ttc / base_ttc;
  }
}

void ActRequest::update_filtered_efficiency_cost(
    const size_t min_history_size, double &current_cost,
    std::deque<double> &cost_history) const {
  if (cost_history.size() < min_history_size) {
    cost_history.push_back(current_cost);
  } else {
    cost_history.pop_front();
    cost_history.push_back(current_cost);
    current_cost =
        std::accumulate(cost_history.begin(), cost_history.end(), 0.0) /
        static_cast<double>(min_history_size);
  }
}

void ActRequest::decide_lane_change_intention(
    const RequestManagerInput &request_manager_input,
    const double ego_lane_cost, const double left_lane_cost,
    const double right_lane_cost) {
  if (!request_manager_input.enable_alc) {
    not_left_change_reason_ = NOT_ENABLE_ALC;
  }
  if (!is_in_operational_design_domain(request_manager_input)) {
    finish();
    not_left_change_reason_ = NOT_IN_ODD;
    not_right_change_reason_ = NOT_IN_ODD;
    return;
  }
  double last_lane_change_finish_time =
      request_manager_input.last_lane_change_finish_time;
  if (is_changing_state(request_manager_input.last_lc_status) &&
      (request_manager_input.lc_status == ROAD_NONE)) {
    request_manager_output_->last_lane_change_finish_time = get_system_time();
    last_lane_change_finish_time = get_system_time();
  }
  if (get_system_time() - last_lane_change_finish_time <
      request_manager_input.params.min_wait_time_after_change) {
    finish();
    not_left_change_reason_ = WAIT_AFTER_LAST_CHANGE;
    not_right_change_reason_ = WAIT_AFTER_LAST_CHANGE;
    return;
  }

  MSD_LOG(INFO, "[AutoLaneChange] lc_status = %d",
          request_manager_input.lc_status);
  switch (request_manager_input.lc_status) {
  case ROAD_NONE:
    process_state_none(request_manager_input, ego_lane_cost, left_lane_cost,
                       right_lane_cost);
    break;
  case ROAD_LC_LWAIT:
    process_state_left_change_wait(request_manager_input, ego_lane_cost,
                                   left_lane_cost);
    break;
  case ROAD_LC_RWAIT:
    process_state_right_change_wait(request_manager_input, ego_lane_cost,
                                    right_lane_cost);
    break;
  case ROAD_LC_LCHANGE:
    process_state_left_change();
    break;
  case ROAD_LC_RCHANGE:
    process_state_right_change();
    break;
  case ROAD_LC_LBACK:
  case ROAD_LC_RBACK:
    process_state_change_back();
    break;
  default:
    break;
  }
}

bool ActRequest::is_in_operational_design_domain(
    const RequestManagerInput &request_manager_input) const {
  const auto &params = request_manager_input.params;
  const auto &map_info = request_manager_input.map_info;

  if (map_info.dist_to_intsect < params.min_dist_to_intsect) {
    return false;
  }

  if (map_info.dist_to_ramp < params.min_dist_to_ramp) {
    return false;
  }

  if (map_info.lanes_num == 1) {
    return false;
  }

  if (request_manager_input.ego_vel < params.min_alc_speed / 3.6) {
    return false;
  }

  const double max_curvature_limit_v = 65.0 / 3.6;
  if (request_manager_input.map_info.v_curvature_limit <
      max_curvature_limit_v) {
    return false;
  }

  if (request_manager_input.map_info.is_in_vision_intsect) {
    return false;
  }

  return true;
}

bool ActRequest::is_changing_state(int state) const {
  return (state == ROAD_LC_LWAIT) || (state == ROAD_LC_RWAIT) ||
         (state == ROAD_LC_LCHANGE) || (state == ROAD_LC_RCHANGE) ||
         (state == ROAD_LC_LBACK) || (state == ROAD_LC_RBACK);
}

void ActRequest::process_state_none(
    const RequestManagerInput &request_manager_input,
    const double ego_lane_cost, const double left_lane_cost,
    const double right_lane_cost) {
  const double delta_l = ego_lane_cost - left_lane_cost;
  const double delta_r = ego_lane_cost - right_lane_cost;
  const bool is_left_optional =
      is_left_lane_optional(request_manager_input, delta_l);
  const bool is_right_optional =
      is_right_lane_optional(request_manager_input, delta_r);
  if (is_left_optional) {
    gen_request(LEFT_CHANGE);
  } else if (is_right_optional) {
    gen_request(RIGHT_CHANGE);
  } else {
    finish();
  }
}

bool ActRequest::is_left_lane_optional(
    const RequestManagerInput &request_manager_input, const double delta_l) {
  const bool is_left_back_one_safe_enough =
      is_back_one_safe_enough(request_manager_input, left_back_one_);
  const bool is_optional =
      (delta_l > request_manager_input.params.none_to_trigger_thresh) &&
      is_left_back_one_safe_enough &&
      (!request_manager_input.map_info.is_left_solid_lane);
  if (!is_optional) {
    if (!std::isfinite(delta_l)) {
      not_left_change_reason_ = NO_LANE;
    } else if (request_manager_input.map_info.is_left_solid_lane) {
      not_left_change_reason_ = SOLID_LINE;
    } else if (!is_left_back_one_safe_enough) {
      not_left_change_reason_ = DANGEROUS_OBJECT;
    } else {
      not_left_change_reason_ = EFFICIENCY_NOT_BETTER;
    }
  }
  return is_optional;
}

bool ActRequest::is_right_lane_optional(
    const RequestManagerInput &request_manager_input, const double delta_r) {
  const bool is_right_back_one_safe_enough =
      is_back_one_safe_enough(request_manager_input, right_back_one_);
  const bool is_optional =
      (delta_r * request_manager_input.params.balance_factor >
       request_manager_input.params.none_to_trigger_thresh) &&
      is_right_back_one_safe_enough &&
      (!request_manager_input.map_info.is_right_solid_lane);
  if (!is_optional) {
    if (!std::isfinite(delta_r)) {
      not_right_change_reason_ = NO_LANE;
    } else if (request_manager_input.map_info.is_right_solid_lane) {
      not_right_change_reason_ = SOLID_LINE;
    } else if (!is_right_back_one_safe_enough) {
      not_right_change_reason_ = DANGEROUS_OBJECT;
    } else {
      not_right_change_reason_ = EFFICIENCY_NOT_BETTER;
    }
  }
  return is_optional;
}

bool ActRequest::is_back_one_safe_enough(
    const RequestManagerInput &request_manager_input,
    const SimpilfiedTrackedObject &object) const {
  if (object.track_id == -1) {
    return true;
  }

  const bool is_parrallel_with_ego =
      object.d_rel > -(request_manager_input.ego_length +
                       request_manager_input.params.min_safe_dist);

  if (is_parrallel_with_ego) {
    return false;
  }

  if (object.v > request_manager_input.ego_vel) {
    const double ttc =
        (std::abs(object.d_rel) - request_manager_input.ego_length) /
        (object.v - request_manager_input.ego_vel);
    if (ttc < request_manager_input.params.min_saft_back_one_ttc) {
      return false;
    }
  }

  return true;
}

void ActRequest::process_state_left_change_wait(
    const RequestManagerInput &request_manager_input,
    const double ego_lane_cost, const double left_lane_cost) {
  if (ego_lane_cost <= left_lane_cost) {
    finish();
    not_left_change_reason_ = EFFICIENCY_NOT_BETTER;
  } else if (request_manager_input.map_info.is_left_solid_lane) {
    finish();
    not_left_change_reason_ = SOLID_LINE;
  } else if (!is_back_one_safe_enough(request_manager_input, left_back_one_)) {
    finish();
    not_left_change_reason_ = DANGEROUS_OBJECT;
  } else {
    not_right_change_reason_ = IS_REVERSE_CHANGING;
  }
}

void ActRequest::process_state_right_change_wait(
    const RequestManagerInput &request_manager_input,
    const double ego_lane_cost, const double right_lane_cost) {
  if (ego_lane_cost <= right_lane_cost) {
    finish();
    not_right_change_reason_ = EFFICIENCY_NOT_BETTER;
  } else if (request_manager_input.map_info.is_right_solid_lane) {
    finish();
    not_right_change_reason_ = SOLID_LINE;
  } else if (!is_back_one_safe_enough(request_manager_input, right_back_one_)) {
    finish();
    not_right_change_reason_ = DANGEROUS_OBJECT;
  } else {
    not_left_change_reason_ = IS_REVERSE_CHANGING;
  }
}

void ActRequest::process_state_left_change() {
  not_right_change_reason_ = IS_REVERSE_CHANGING;
}

void ActRequest::process_state_right_change() {
  not_left_change_reason_ = IS_REVERSE_CHANGING;
}

void ActRequest::process_state_change_back() {
  finish();
  not_left_change_reason_ = STATE_MACHINE_BACK;
  not_right_change_reason_ = STATE_MACHINE_BACK;
}

IntRequest::IntRequest(
    std::shared_ptr<RequestManagerOutput> request_manager_output) {
  request_manager_output_ = request_manager_output;
}

void IntRequest::update(const RequestManagerInput &request_manager_input) {
  MSD_LOG(INFO, "WRDEBUG: ego_blinker %d", request_manager_input.ego_blinker);
  if (!request_manager_input.enable_ilc) {
    finish_and_clear();
    reset();
    return;
  }
  if (request_ == NO_CHANGE)
    request_cancel_reason_ = NO_CANCEL;
  double v_ego = request_manager_input.ego_vel;
  double int_limit_v = request_manager_input.ilc_limit_velocity;
  constexpr double Kkph2m_s = 3.6;
  // generate request
  if (request_manager_input.ego_blinker ==
          maf_endpoint::LeverStatus::LEVER_STATE_LEFT &&
      request_ != LEFT_CHANGE) {
    counter_right_ = 0;
    counter_left_++;
    if (v_ego * Kkph2m_s < int_limit_v) {
      request_cancel_reason_ = UNSUITABLE_VEL;
      finish_and_clear();
      counter_left_ = 0;
    }
    if (counter_left_ > 2) {
      // judge if exist dash line
      if (!request_manager_input.map_info.is_left_solid_lane)
        exist_process_dash_line_ = true;
      else
        exist_process_dash_line_ = false;
      gen_request(LEFT_CHANGE);
      request_cancel_reason_ = NO_CANCEL;
      MSD_LOG(INFO, "[IntRequest::update] Ask for Int changing lane to left");
    }
    MSD_LOG(INFO, "[IntRequest::update] counter_left_: %d", counter_left_);
  } else if (request_manager_input.ego_blinker ==
                 maf_endpoint::LeverStatus::LEVER_STATE_RIGHT &&
             request_ != RIGHT_CHANGE) {
    counter_left_ = 0;
    counter_right_++;
    if (v_ego * Kkph2m_s < int_limit_v) {
      request_cancel_reason_ = UNSUITABLE_VEL;
      finish_and_clear();
      counter_right_ = 0;
    }
    if (counter_right_ > 2) {
      if (!request_manager_input.map_info.is_right_solid_lane)
        exist_process_dash_line_ = true;
      else
        exist_process_dash_line_ = false;
      gen_request(RIGHT_CHANGE);
      request_cancel_reason_ = NO_CANCEL;
      MSD_LOG(INFO, "[IntRequest::update] Ask for Int changing lane to right");
    }
    MSD_LOG(INFO, "[IntRequest::update] counter_right_: %d", counter_right_);
  } else if (request_manager_input.ego_blinker ==
                 maf_endpoint::LeverStatus::LEVER_STATE_OFF &&
             request_ != NO_CHANGE) {
    if (request_manager_input.has_target_lane &&
        request_manager_input.on_target_lane) {
      MSD_LOG(INFO, "WRDEBUG: Cancel int lc blinker when ego car on target "
                    "lane and continue lc state");

    } else {
      finish_and_clear();
      request_cancel_reason_ = MANUAL_CANCEL;
      MSD_LOG(WARN,
              "[IntRequest::update] %s:%d finish request, no blinker and not "
              "on target",
              __FUNCTION__, __LINE__);
      counter_left_ = 0;
      counter_right_ = 0;
    }
  } else if (request_manager_input.ego_blinker ==
             maf_endpoint::LeverStatus::LEVER_STATE_OFF) {
    finish();
    request_cancel_reason_ = MANUAL_CANCEL;
    MSD_LOG(WARN, "[IntRequest::update] %s:%d finish request, no blinker",
            __FUNCTION__, __LINE__);
    counter_left_ = 0;
    counter_right_ = 0;
  }

  if (request_manager_input.has_origin_lane &&
      request_manager_input.on_origin_lane && v_ego * Kkph2m_s < int_limit_v) {
    counter_cancel_vel_++;
    if (counter_cancel_vel_ > 2) {
      request_cancel_reason_ = UNSUITABLE_VEL;
      finish_and_clear();
      MSD_LOG(WARN, "[IntRequest::update] %s:%d finish request, low vel",
              __FUNCTION__, __LINE__);
    }
    MSD_LOG(INFO, "[IntRequest::update] counter_cancel_vel_: %d",
            counter_cancel_vel_);
  } else {
    counter_cancel_vel_ = 0;
    request_cancel_reason_ = NO_CANCEL;
  }
}

void IntRequest::finish_and_clear() {
  finish();

  // clear ego blinker when ilc finished
  need_clear_ilc_ego_blinker_ = true;
}

void IntRequest::restore_context(const IntRequestContext &context) {
  request_ = context.request;
  turn_light_ = context.turn_light;
  counter_left_ = context.counter_left;
  counter_right_ = context.counter_right;
  tstart_ = context.tstart;
  tfinish_ = context.tfinish;
}

void IntRequest::save_context(IntRequestContext &context) const {
  context.request = request_;
  context.turn_light = turn_light_;
  context.counter_left = counter_left_;
  context.counter_right = counter_right_;
  context.tstart = tstart_;
  context.tfinish = tfinish_;
}

LCRequestManager::LCRequestManager() {
  request_manager_output_ = make_shared<RequestManagerOutput>();
  int_request_ = make_shared<IntRequest>(request_manager_output_);
  map_request_ = make_shared<MapRequest>(request_manager_output_);
  act_request_ = make_shared<ActRequest>(request_manager_output_);
}

void LCRequestManager::finish_request() {
  if (request_source_ == INT_REQUEST) {
    int_request_->finish_and_clear();
    int_request_->reset();
  } else if (request_source_ == MAP_REQUEST) {
    map_request_->finish();
    map_request_->reset();
  } else if (request_source_ == ACT_REQUEST) {
    act_request_->finish();
    act_request_->reset();
  }

  request_ = NO_CHANGE;
  request_source_ = NO_REQUEST;
  turn_light_ = 0;
}

LaneChangeDirection LCRequestManager::cal_lane_change_direction(
    const MapInfoManager &map_info_mgr) {
  static Lane last_l_lane;
  static Lane last_c_lane;
  static Lane last_r_lane;
  LaneChangeDirection lane_chane_direction = LaneChangeDirection::NONE;

  if (map_info_mgr.clane_.exist()) {
    std::vector<Lane *> lanes{&last_l_lane, &last_c_lane, &last_r_lane};
    double max_coincide = 0;
    double max_coincide_rate = 0;
    Lane *matched_lane = nullptr;
    std::array<double, 2> intercepts = map_info_mgr.clane_.intercepts();
    for (const auto lane : lanes) {
      if (lane->exist() != true)
        continue;
      if ((std::min(intercepts[0], lane->intercepts()[0]) -
               std::max(intercepts[1], lane->intercepts()[1]) >
           max_coincide) &&
          (intercepts[0] - intercepts[1] > 0)) {
        max_coincide = std::min(intercepts[0], lane->intercepts()[0]) -
                       std::max(intercepts[1], lane->intercepts()[1]);
        if (max_coincide / (intercepts[0] - intercepts[1]) > 0.8) {
          max_coincide_rate = max_coincide / (intercepts[0] - intercepts[1]);
          matched_lane = lane;
        }
      }
    }

    if (matched_lane == nullptr) {
      lane_chane_direction = LaneChangeDirection::NONE;
    } else {
      if (matched_lane->position() == LanePosition::LEFT_POS) {
        lane_chane_direction = LaneChangeDirection::LEFT;
      } else if (matched_lane->position() == LanePosition::CURR_POS) {
        lane_chane_direction = LaneChangeDirection::NONE;
      } else if (matched_lane->position() == LanePosition::RIGHT_POS) {
        lane_chane_direction = LaneChangeDirection::RIGHT;
      } else {
        lane_chane_direction = LaneChangeDirection::NONE;
      }
    }
  } else {
    lane_chane_direction = LaneChangeDirection::NONE;
  }

  // update history
  last_l_lane = map_info_mgr.llane_;
  last_c_lane = map_info_mgr.clane_;
  last_r_lane = map_info_mgr.rlane_;

  return lane_chane_direction;
}

const RequestManagerInput &
LCRequestManager::update_input(bool enable_ilc, bool enable_alc,
                               bool enable_recommend_alc,
                               std::shared_ptr<WorldModel> world_model,
    VirtualLaneManager &virtual_lane_mgr, LaneTracksManager &lane_tracks_mgr,
    int lc_status) {
  if (need_clear_ilc_ego_blinker()) {
    // clear ego blinker when ilc finished
    maf_vehicle_status::VehicleLight vehicle_light{};
    vehicle_light.vehicle_light_data.turn_signal.value =
        maf_endpoint::LeverStatus::LEVER_STATE_OFF;
    world_model->set_ego_blinker(vehicle_light);
    reset_need_clear_ilc_ego_blinker();
  }

  auto request_manager_input =
      PlanningContext::Instance()->mutable_request_manager_input();

  // params
  auto &params = request_manager_input->params;
  params.min_dist_to_intsect = MIN_DIST_TO_INTSECT;
  params.min_dist_to_ramp = MIN_DIST_TO_RAMP;
  params.min_alc_speed = MIN_ALC_SPEED;
  params.efficiency_cost_filter_time = EFFICIENCY_COST_FILTER_TIME;
  params.blocked_unit_cost = BLOCKED_UNIT_COST;
  params.conservative_unit_cost = CONSERVATIVE_UNIT_COST;
  params.min_safe_dist = MIN_SAFE_DIST;
  params.none_to_trigger_thresh = NONE_TO_TRIGGER_THRESH;
  params.balance_factor = BALANCE_FACTOR;
  params.min_saft_back_one_ttc = MIN_SAFE_BACK_ONE_TTC;
  params.car_following_factor = CAR_FOLLOWING_FACTOR;
  params.base_ttc_factor = BASE_TTC_FACTOR;
  params.min_wait_time_after_change = MIN_WAIT_TIME_AFTER_CHANGE;

  // ego related
  request_manager_input->ego_length =
      ConfigurationContext::Instance()->get_vehicle_param().length;
  request_manager_input->enable_ilc = enable_ilc;
  request_manager_input->enable_alc = enable_alc;
  request_manager_input->enable_recommend_alc = enable_recommend_alc;
  request_manager_input->ego_vel =
      world_model->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
  request_manager_input->t_headway =
      speed_planner::NORMAL_HEADWAY_TABLE[world_model->get_navi_ttc_gear()];
  request_manager_input->lc_status = lc_status;
  request_manager_input->lc_direction =
      cal_lane_change_direction(world_model->get_map_info_manager());

  // last ego related
  static int last_lc_status = ROAD_NONE;
  request_manager_input->last_lc_status = last_lc_status;
  last_lc_status = request_manager_input->lc_status;

  // map_info
  auto &map_info = request_manager_input->map_info;
  map_info.is_left_solid_lane =
      world_model->get_map_info_manager().is_solid_line(0);
  map_info.is_right_solid_lane =
      world_model->get_map_info_manager().is_solid_line(1);
  map_info.v_curvature_limit = world_model->get_v_curv();
  map_info.is_in_vision_intsect =
      world_model->get_refline_condition().reflinecondition !=
      pass_intersection_planner::ReflineCondition::NO_INTERSECTION;
  map_info.v_cruise =
      world_model->get_map_info_manager().get_map_info().v_cruise();
  map_info.v_cruise_current =
      world_model->get_map_info_manager().get_map_info().v_cruise_current();
  map_info.v_cruise_change_dis =
      world_model->get_map_info_manager().get_map_info().v_cruise_change_dis();
  map_info.dist_to_intsect =
      world_model->get_map_info_manager().get_map_info().dist_to_intsect();
  map_info.dist_to_ramp =
      world_model->get_map_info_manager().get_map_info().dis_to_ramp();
  map_info.dist_to_tollgate =
      world_model->get_map_info_manager().get_map_info().dist_to_tollgate();
  map_info.lanes_num =
      world_model->get_map_info_manager().get_map_info().lanes_num();
  map_info.current_lane_index =
      world_model->get_map_info_manager().get_map_info().current_lane_index();
  map_info.current_lane_marks =
      world_model->get_map_info_manager().get_map_info().current_lane_marks();
  map_info.left_lane_marks =
      world_model->get_map_info_manager().get_map_info().left_lane_marks();
  map_info.right_lane_marks =
      world_model->get_map_info_manager().get_map_info().right_lane_marks();

  // ilc related
  request_manager_input->ego_blinker = world_model->get_cart_ego_state_manager()
                                           .get_cart_ego_state()
                                           .ego_blinker;
  request_manager_input->ilc_limit_velocity =
      world_model->get_ilc_limit_velocity();

  // virtual lane related
  request_manager_input->has_origin_lane = virtual_lane_mgr.has_origin_lane();
  request_manager_input->on_origin_lane =
      virtual_lane_mgr.is_on_lane(ORIGIN_LANE);
  request_manager_input->has_target_lane = virtual_lane_mgr.has_target_lane();
  request_manager_input->on_target_lane =
      virtual_lane_mgr.is_on_lane(TARGET_LANE);

  auto fill_simplified_tracked_object =
      [](std::vector<SimpilfiedTrackedObject> &simplified_tracked_objects,
         const std::vector<TrackedObject> *tracked_objects) {
        simplified_tracked_objects.clear();
        SimpilfiedTrackedObject object;
        for (const auto tracked_object : *tracked_objects) {
          object.track_id = tracked_object.track_id;
          object.type = tracked_object.type;
          object.length = tracked_object.length;
          object.width = tracked_object.width;
          object.center_x = tracked_object.center_x;
          object.center_y = tracked_object.center_y;
          object.s = tracked_object.s;
          object.l = tracked_object.l;
          object.theta = tracked_object.theta;
          object.speed_yaw = tracked_object.speed_yaw;
          object.a = tracked_object.a;
          object.v = tracked_object.v;
          object.v_lead = tracked_object.v_lead;
          object.v_rel = tracked_object.v_rel;
          object.vy_rel = tracked_object.vy_rel;
          object.d_rel = tracked_object.d_rel;
          object.y_rel = tracked_object.y_rel;

          simplified_tracked_objects.push_back(object);
        }
      };

  // obstacles
  fill_simplified_tracked_object(
      request_manager_input->front_tracks_l,
      lane_tracks_mgr.get_lane_tracks(LaneProperty::LEFT_LANE,
                                      TrackType::FRONT_TRACK));
  fill_simplified_tracked_object(
      request_manager_input->front_tracks_c,
      lane_tracks_mgr.get_lane_tracks(LaneProperty::CURRENT_LANE,
                                      TrackType::FRONT_TRACK));
  fill_simplified_tracked_object(
      request_manager_input->front_tracks_r,
      lane_tracks_mgr.get_lane_tracks(LaneProperty::RIGHT_LANE,
                                      TrackType::FRONT_TRACK));
  fill_simplified_tracked_object(
      request_manager_input->side_tracks_l,
      lane_tracks_mgr.get_lane_tracks(LaneProperty::LEFT_LANE,
                                      TrackType::SIDE_TRACK));
  fill_simplified_tracked_object(
      request_manager_input->side_tracks_r,
      lane_tracks_mgr.get_lane_tracks(LaneProperty::RIGHT_LANE,
                                      TrackType::SIDE_TRACK));

  // history results
  const auto &request_manager_output =
      PlanningContext::Instance()->request_manager_output();
  request_manager_input->ego_lane_efficiency_cost_history =
      request_manager_output.ego_lane_efficiency_cost_history;
  request_manager_input->left_lane_efficiency_cost_history =
      request_manager_output.left_lane_efficiency_cost_history;
  request_manager_input->right_lane_efficiency_cost_history =
      request_manager_output.right_lane_efficiency_cost_history;
  request_manager_input->last_lane_change_finish_time =
      request_manager_output.last_lane_change_finish_time;
  return *request_manager_input;
}

void LCRequestManager::update(
    const RequestManagerInput &request_manager_input) {
  int_request_->update(request_manager_input);
  int_request_cancel_reason_ = int_request_->get_request_cancel_reason();
  int_exist_dash_line_ = int_request_->get_exist_dash_line();
  MSD_LOG(INFO, "[AutoLaneChange] NEW LOOP START >>>>>>>>>>>>>>>>>>>>>>>>>");
  act_request_->update(request_manager_input);

  MSD_LOG(INFO,
          "fengxiaotong:: LCRequest::[int_request] [%d] [map_request] [%d] "
          "[act_request] [%d]",
          int_request_->request(), map_request_->request(),
          act_request_->request());

  auto *planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  if (request_manager_input.enable_alc &&
      (act_request_->request() == LEFT_CHANGE)) {
    planner_debug->lat_dec_info.alc_request = -1;
  } else if (request_manager_input.enable_alc &&
             (act_request_->request() == RIGHT_CHANGE)) {
    planner_debug->lat_dec_info.alc_request = 1;
  } else {
    planner_debug->lat_dec_info.alc_request = 0;
  }
  if (int_request_->request() == LEFT_CHANGE) {
    planner_debug->lat_dec_info.int_request = -1;
  } else if (int_request_->request() == RIGHT_CHANGE) {
    planner_debug->lat_dec_info.int_request = 1;
  } else {
    planner_debug->lat_dec_info.int_request = 0;
  }
  planner_debug->lat_dec_info.obstacle_gap.clear();

  request_ = NO_REQUEST;
  if (int_request_->request() != NO_CHANGE) {
    if (act_request_->request() != NO_CHANGE) {
      act_request_->finish();
      act_request_->reset();
    }
    request_ = int_request_->request();
    request_source_ = INT_REQUEST;
  } else {
    MSD_LOG(INFO,
            "request_manager_input.enable_alc = %d, enable_recommend_alc = %d",
            request_manager_input.enable_alc,
            request_manager_input.enable_recommend_alc);
    if (request_manager_input.enable_alc &&
        !request_manager_input.enable_recommend_alc) {
      request_ = act_request_->request();
      request_source_ = (request_ != NO_REQUEST) ? ACT_REQUEST : NO_REQUEST;
    } else if (!request_manager_input.enable_alc &&
               request_manager_input.enable_recommend_alc) {
      if (act_request_->request() == NO_CHANGE) {
        request_manager_output_->alc_recommend_dir = LaneChangeDirection::NONE;
      } else {
        request_manager_output_->alc_recommend_dir =
            (act_request_->request() == LEFT_CHANGE) ? LaneChangeDirection::LEFT
                                                    : LaneChangeDirection::RIGHT;
      }
    }
  }

  if (int_request_->turn_light() > 0) {
    turn_light_ = int_request_->turn_light();
  } else {
    turn_light_ = request_manager_input.enable_alc ? act_request_->turn_light()
                                                   : NO_CHANGE;
  }
}

double LCRequestManager::get_req_tstart(int source) const {
  if (source == INT_REQUEST) {
    return int_request_->tstart();
  } else if (source == MAP_REQUEST) {
    return map_request_->tstart();
  } else if (source == ACT_REQUEST) {
    return act_request_->tstart();
  }

  return DBL_MAX;
}

double LCRequestManager::get_req_tfinish(int source) const {
  if (source == INT_REQUEST) {
    return int_request_->tfinish();
  } else if (source == MAP_REQUEST) {
    return map_request_->tfinish();
  } else if (source == ACT_REQUEST) {
    return act_request_->tfinish();
  }

  return DBL_MAX;
}

void LCRequestManager::restore_context(const LCRequestManagerContext &context) {
  request_ = context.request;
  request_source_ = context.request_source;
  turn_light_ = context.turn_light;

  int_request_->restore_context(context.int_request);
  map_request_->restore_context(context.map_request);
  act_request_->restore_context(context.act_request);
}

void LCRequestManager::save_context(LCRequestManagerContext &context) const {
  context.request = request_;
  context.request_source = request_source_;
  context.turn_light = turn_light_;

  int_request_->save_context(context.int_request);
  map_request_->save_context(context.map_request);
  act_request_->save_context(context.act_request);
}

} // namespace msquare
