#include "common/traffic_light_decision.h"
#include "common/planning_context.h"
#include "common/prediction_object.h"
#include "common/world_model.h"
#include "planning/common/common.h"
// #include "pnc_trigger_status.h"

namespace msquare {

constexpr double TrafficLightDecision::kDistNoStopLine;
constexpr double TrafficLightDecision::kStopDisBuffer;
constexpr double TrafficLightDecision::kStopDisPreChecker;
constexpr double TrafficLightDecision::kThresCheckTrafficLight;
constexpr double TrafficLightDecision::kGreenBlinkingDuration;
constexpr double TrafficLightDecision::kComfortDec;
constexpr double TrafficLightDecision::kYellowDuration;
constexpr double TrafficLightDecision::kPassingStopLineBuffer;
constexpr double TrafficLightDecision::kStopLineBuffer;
constexpr double TrafficLightDecision::kCrossingDistanceBuffer;
constexpr double TrafficLightDecision::kStopChecker;
constexpr double TrafficLightDecision::kDistanceToIntersectionChecker;
constexpr double TrafficLightDecision::kDistanceToCrossingChecker;
constexpr double TrafficLightDecision::kVelToCrossingChecker;
constexpr double TrafficLightDecision::kPassedIntersectionTimeBuffer;
constexpr double TrafficLightDecision::kCOVERTimeBuffer;
// constexpr double TrafficLightDecision::kComfortMaxjerk;
// constexpr double TrafficLightDecision::kComfortMaxAcc;
constexpr double TrafficLightDecision::kDistanceToIntersectionCheckerRatio;
constexpr int TrafficLightDecision::flag_light_status_OFF;
constexpr int TrafficLightDecision::flag_light_status_YELLOW;
constexpr double TrafficLightDecision::kLongCrossingLength;
constexpr double TrafficLightDecision::kTrafficLightInField;
constexpr double TrafficLightDecision::kGREENBLINKINGTimeBuffer;
constexpr double TrafficLightDecision::kYELLOWTimeBuffer;
constexpr double TrafficLightDecision::kStopCheckerDistance;
constexpr double TrafficLightDecision::kStopCheckerFollowObs;
constexpr int TrafficLightDecision::FlagLonFollowObs;
constexpr double TrafficLightDecision::IntersectionLowSpeedDistance;
constexpr double TrafficLightDecision::kGREENBLINKINGByLonFollow;
constexpr int TrafficLightDecision::kCOVERWaitTime;
constexpr double TrafficLightDecision::kFlowDistance;
constexpr double TrafficLightDecision::kFlowYawRelativeFrenet;
constexpr double TrafficLightDecision::kFlowComfortDec;
constexpr double TrafficLightDecision::kFlowBaselineL;
constexpr double TrafficLightDecision::kDistPlanningBuffer;
constexpr double TrafficLightDecision::kDistComfortBuffer;
constexpr double TrafficLightDecision::kComfortDecEmer;
constexpr double TrafficLightDecision::kCOVERGREENdist;
TrafficLightDecision::TrafficLightDecision() {}

void TrafficLightDecision::init() {
  current_state_ = CurrentState::INIT;
  dist_to_stopline_ = kDistNoStopLine;
  state_cycles_ = 1;
  is_passed_stopline_ = false;
  dist_to_stopline_pre_ = kDistNoStopLine;
  stop_flag_ = false;
  stop_point_ = kDistNoStopLine;
}

void TrafficLightDecision::update(
    const std::shared_ptr<WorldModel> &world_model) {

  dbw_status_ = world_model->get_dbw_status();

  bool true_dbw_status = world_model->get_vehicle_dbw_status();
  if (!true_dbw_status) {
    MSD_LOG(INFO,
            "TrafficlightDicision -- dbw init traffic_light decision state");
    init();
  }

  auto &map_info = world_model->get_map_info();
  auto &traffic_light = world_model->get_traffic_light_info();
  auto is_in_intersection = map_info.is_in_intersection();

  if (is_in_intersection_pre_ && !is_in_intersection) {
    passed_intersection_time_ = MTIME()->timestamp().sec();
  }

  if (traffic_light.valid) {
    intersection_decision(map_info.distance_to_crossing());

    std::vector<maf_perception_interface::TrafficLightPatternEnum>
        light_patterns;
    auto &traffic_light_groups = map_info.traffic_light_groups();
    if (traffic_light_groups.size() > 0) {
      for (auto &mpa_traffic_light : traffic_light_groups[0].traffic_lights) {
        light_patterns.push_back(mpa_traffic_light.pattern);
      }
    }

    double ego_vel =
        world_model->get_cart_ego_state_manager().get_cart_ego_state().ego_vel;
    double ego_acc =
        world_model->get_cart_ego_state_manager().get_cart_ego_state().ego_acc;

    decision_compute(map_info.distance_to_stop_line(),
                     map_info.curr_intsect_task(), light_patterns,
                     traffic_light.info, ego_vel, ego_acc, is_in_intersection,
                     map_info.crossing_length());
  }

  if (intersection_decision_flag_ &&
      (traffic_light.header >
       (passed_intersection_time_ + kPassedIntersectionTimeBuffer))) {
    stop_flag_ = true;
    stop_point_ = 0.0;
    RED_LIGHT_STOP_CLASS_ = 1;
  }
  dbw_status_pre_ = dbw_status_;
  is_in_intersection_pre_ = is_in_intersection;

  double current_time = MTIME()->timestamp().sec();
  if (current_time - traffic_light.header >= kTrafficLightInField) {
    is_traffic_light_in_field_ = false;
  } else {
    is_traffic_light_in_field_ = true;
  }

  PlanningResult &planning_result =
      PlanningContext::Instance()->mutable_planning_status()->planning_result;

  planning_result.traffic_light_state = get_current_state();
  planning_result.stop_flag = get_stop_flag();
  planning_result.is_passed_stop_line = get_is_passed_stop_line();
  planning_result.dist_to_stop = get_stop_point();

  int current_lane_index = map_info.current_lane_index();
  int lanes_num = map_info.lanes_num();
  auto traffic_light_direction = map_info.traffic_light_direction();
  bool left_direct_exist =
      ((map_info.left_lane_marks() & traffic_light_direction) ||
       (map_info.current_lane_index() == map_info.lanes_num() - 1 &&
        traffic_light_direction == Direction::UNKNOWN));
  bool right_direct_exist =
      (map_info.right_lane_marks() & traffic_light_direction ||
       traffic_light_direction == Direction::UNKNOWN);
  curr_direct_traffic_ =
      ((map_info.current_lane_marks() & traffic_light_direction) ||
       traffic_light_direction == Direction::UNKNOWN);
  int baseline_left = 0;
  int baseline_right = 0;

  MSD_LOG(INFO, "TRAFFIC_L -- TEST -- DEBUG -- curr_direct_traffic_: %d ",
          curr_direct_traffic_);
  if ((current_lane_index > 0) && (left_direct_exist)) {
    baseline_left = 1;
  }
  if ((current_lane_index < lanes_num - 1) && (right_direct_exist)) {
    baseline_right = 1;
  }

  if (baseline_left == 1) {
    auto curr_baseline_left = world_model->get_baseline_info(-1);
    auto &obstacle_manager_left = curr_baseline_left->obstacle_manager();
    MSD_LOG(INFO, "COVERTEST2 - when baseline_left = 1");
    for (const auto *obstacle_left :
         obstacle_manager_left.get_obstacles().Items()) {
      auto obstacle_left_decision =
          curr_baseline_left->obstacle_manager().find_obstacle(
              obstacle_left->Id());
      if (obstacle_left_decision == nullptr) {
        continue;
        MSD_LOG(INFO, "COVERTEST - obstacle == nullptr");
      }
      if (!obstacle_left_decision->IsFrenetInvalid()) {
        MSD_LOG(INFO, "COVERTEST - left status 0");
        if (curr_baseline_left->is_obstacle_intersect_with_lane(
                obstacle_left)) {
          MSD_LOG(INFO, "COVERTEST - left status 1");
          if (obstacle_left_decision->PerceptionSLBoundary().end_s <
              curr_baseline_left->get_adc_sl_boundary().end_s) {
            MSD_LOG(INFO, "COVERTEST - left status 2");
            if (curr_baseline_left->get_adc_sl_boundary().end_s -
                    obstacle_left_decision->PerceptionSLBoundary().end_s <
                kFlowDistance) {
              MSD_LOG(INFO, "COVERTEST - left status 3");
              if (abs(obstacle_left_decision->Yaw_relative_frenet()) <
                  kFlowYawRelativeFrenet) {
                MSD_LOG(INFO, "COVERTEST - left status 4");
                if (abs(obstacle_left_decision->PerceptionSLBoundary().end_l) <
                    kFlowBaselineL) {
                  auto aim_car_left_v = obstacle_left_decision->speed();
                  auto aim_car_left_s =
                      curr_baseline_left->get_adc_sl_boundary().end_s -
                      obstacle_left_decision->PerceptionSLBoundary().end_s +
                      map_info.distance_to_crossing();
                  MSD_LOG(INFO, "COVERTEST22 - for logic obstacle: %d ",
                          obstacle_left->Id());
                  MSD_LOG(INFO, "COVERTEST2 - aim_car_left_v: %f ",
                          aim_car_left_v);
                  MSD_LOG(INFO, "COVERTEST2 - aim_car_left_s: %f ",
                          aim_car_left_s);
                  auto s_1_left =
                      obstacle_left_decision->PerceptionSLBoundary().end_s;
                  auto s_2_left =
                      curr_baseline_left->get_adc_sl_boundary().end_s;
                  auto l_1_left =
                      obstacle_left_decision->PerceptionSLBoundary().end_l;
                  auto l_2_left =
                      curr_baseline_left->get_adc_sl_boundary().end_l;
                  MSD_LOG(INFO, "COVERTEST - s_1_left: %f ", s_1_left);
                  // MSD_LOG(INFO, "COVERTEST - s_2_left: %f ", s_2_left);
                  MSD_LOG(INFO, "COVERTEST - l_1_left: %f ", l_1_left);
                  // MSD_LOG(INFO, "COVERTEST - l_2_left: %f ", l_2_left);
                  if (aim_car_left_v * aim_car_left_v / (2 * aim_car_left_s) >
                      kFlowComfortDec) {
                    is_left_car_go_ = true;
                    MSD_LOG(INFO, "COVERTEST22 - is_left_car_go = true");
                  } else {
                    is_left_car_go_ = false;
                    MSD_LOG(INFO, "COVERTEST2 - is_left_car_go = false");
                  }
                  break;
                }
              }
            }
          }
        }
      }
    }
  }

  if (baseline_right == 1) {
    auto curr_baseline_right = world_model->get_baseline_info(1);
    auto &obstacle_manager_right = curr_baseline_right->obstacle_manager();
    MSD_LOG(INFO, "COVERTEST2 - when baseline_right = 1");
    for (const auto *obstacle_right :
         obstacle_manager_right.get_obstacles().Items()) {
      auto obstacle_right_decision =
          curr_baseline_right->obstacle_manager().find_obstacle(
              obstacle_right->Id());
      if (obstacle_right_decision == nullptr) {
        continue;
        MSD_LOG(INFO, "COVERTEST - obstacle == nullptr");
      }
      if (!obstacle_right_decision->IsFrenetInvalid()) {
        MSD_LOG(INFO, "COVERTEST - right status 0");
        if (curr_baseline_right->is_obstacle_intersect_with_lane(
                obstacle_right)) {
          MSD_LOG(INFO, "COVERTEST - right status 1");
          if (obstacle_right_decision->PerceptionSLBoundary().end_s <
              curr_baseline_right->get_adc_sl_boundary().end_s) {
            MSD_LOG(INFO, "COVERTEST - right status 2");
            if (curr_baseline_right->get_adc_sl_boundary().end_s -
                    obstacle_right_decision->PerceptionSLBoundary().end_s <
                50) {
              MSD_LOG(INFO, "COVERTEST - right status 3");
              if (abs(obstacle_right_decision->Yaw_relative_frenet()) <
                  kFlowYawRelativeFrenet) {
                MSD_LOG(INFO, "COVERTEST - right status 4");
                if (abs(obstacle_right_decision->PerceptionSLBoundary()
                            .start_l) < kFlowBaselineL) {
                  auto aim_car_right_v = obstacle_right_decision->speed();
                  auto aim_car_right_s =
                      curr_baseline_right->get_adc_sl_boundary().end_s -
                      obstacle_right_decision->PerceptionSLBoundary().end_s +
                      map_info.distance_to_crossing();
                  MSD_LOG(INFO, "COVERTEST22 - for logic obstacle: %d ",
                          obstacle_right->Id());
                  MSD_LOG(INFO, "COVERTEST2 - aim_car_right_v: %f ",
                          aim_car_right_v);
                  MSD_LOG(INFO, "COVERTEST2 - aim_car_right_s: %f ",
                          aim_car_right_s);
                  auto s_1_right =
                      obstacle_right_decision->PerceptionSLBoundary().end_s;
                  auto s_2_right =
                      curr_baseline_right->get_adc_sl_boundary().end_s;
                  auto l_1_right =
                      obstacle_right_decision->PerceptionSLBoundary().end_l;
                  auto l_2_right =
                      curr_baseline_right->get_adc_sl_boundary().end_l;
                  MSD_LOG(INFO, "COVERTEST - s_1_right: %f ", s_1_right);
                  // MSD_LOG(INFO, "COVERTEST - s_2_right: %f ", s_2_right);
                  MSD_LOG(INFO, "COVERTEST - l_1_right: %f ", l_1_right);
                  // MSD_LOG(INFO, "COVERTEST - l_2_right: %f ", l_2_right);
                  if (aim_car_right_v * aim_car_right_v /
                          (2 * aim_car_right_s) >
                      kFlowComfortDec) {
                    is_right_car_go_ = true;
                    MSD_LOG(INFO, "COVERTEST22 - is_right_car_go = true");
                  } else {
                    is_right_car_go_ = false;
                    MSD_LOG(INFO, "COVERTEST2 - is_right_car_go = false");
                  }
                  break;
                }
              }
            }
          }
        }
      }
    }
  }
}

void TrafficLightDecision::intersection_decision(
    const double &distance_to_intersection) {
  intersection_decision_flag_ = false;
  int light_status;
  double duration;
  double remaining;
  get_traffic_light_status(light_status, duration, remaining);

  auto planning_intersection =
      PlanningContext::Instance()->mutable_planning_status();
  size_t lon_follow_obs_intersection = 0;
  if (planning_intersection) {
    std::unordered_set<uint32_t> id_set;
    for (auto ids :
         planning_intersection->planning_result.lon_follow_obstacles) {
      id_set.insert(inverse_hash_prediction_id(ids));
    }
    lon_follow_obs_intersection = id_set.size();
  }
  // MSD_LOG(INFO, "TrafficlightDicision -- INTERSPEED lon_follow_obs_len: %d",
  // lon_follow_obs_intersection);

  auto inter_plan_data =
      PlanningContext::Instance()->longitudinal_motion_planner_output();
  SpeedPoint speed_point_start;
  SpeedPoint speed_point_end;
  EvaluateByTime_right_start =
      inter_plan_data.EvaluateByTime(0.001, &speed_point_start);
  EvaluateByTime_right_end =
      inter_plan_data.EvaluateByTime(3.001, &speed_point_end);
  if (EvaluateByTime_right_start) {
    lon_Evaluate_start = speed_point_start.s;
  }
  if (EvaluateByTime_right_end) {
    lon_Evaluate_end = speed_point_end.s;
  }
  lon_Evaluate_intersection = lon_Evaluate_end - lon_Evaluate_start;
  MSD_LOG(INFO, "INTERSPEED dynamic length: %f , %f , %f , %d , %f",
          distance_to_intersection, crossing_length_,
          min(kDistanceToIntersectionChecker,
              kDistanceToIntersectionCheckerRatio * crossing_length_),
          lon_follow_obs_intersection, lon_Evaluate_intersection);
  // MSD_LOG(INFO, "INTERSPEED lon_Evaluate_intersection: %f",
  // lon_Evaluate_intersection);
  if ((light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED ||
       light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW ||
       (light_status ==
            TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN_BLINKING &&
        crossing_length_ > kLongCrossingLength)) &&
      is_inside_intersection_ && is_traffic_light_in_field_ &&
      (ego_vel_ < kStopChecker ||
       (ego_vel_ < kStopCheckerDistance &&
        (lon_Evaluate_intersection + abs(distance_to_intersection) <
         IntersectionLowSpeedDistance)) ||
       (ego_vel_ < kStopCheckerFollowObs &&
        lon_follow_obs_intersection > FlagLonFollowObs)) &&
      distance_to_intersection >
          min(kDistanceToIntersectionChecker,
              kDistanceToIntersectionCheckerRatio * crossing_length_) &&
      (!stop_flag_)) {
    intersection_decision_flag_ = true;
    MSD_LOG(INFO, "INTERSPEED dynamic intersection_decision_flag_");
  } else if (stop_flag_ && current_state_ == CurrentState::CROSSING &&
             light_status ==
                 TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN) {
    stop_flag_ = false;
    stop_point_ = kDistNoStopLine;
    RED_LIGHT_STOP_CLASS_ = 0;
  }
}

void TrafficLightDecision::get_traffic_light_status(int &light_status,
                                                    double &duration_time,
                                                    double &remain_time) {
  bool bhave_left_arrow = false;
  bool bhave_right_arrow = false;
  for (const auto &tmp : light_patterns_) {
    if (tmp.value ==
        TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_LEFT_ARROW) {
      bhave_left_arrow = true;
    }
    if (tmp.value ==
        TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_RIGHT_ARROW) {
      bhave_right_arrow = true;
    }
  }

  int local_light_status = TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF;
  // int light_pattern = 0;
  int straight_local_light_status =
      TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF;
  double duration = -1.0;
  double remaining = 100000.0;

  if (traffic_light_.size() == 0) {
    local_light_status = TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN;
  } else if ((current_direction_ == Direction::TURN_LEFT ||
              current_direction_ == Direction::U_TURN_LEFT) &&
             bhave_left_arrow) {
    auto left_arrow_it = std::find_if(
        traffic_light_.begin(), traffic_light_.end(),
        [](const maf_perception_interface::TrafficLightDecision &data) {
          return data.pattern.value ==
                 TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_LEFT_ARROW;
        });
    auto solid_circle_it = std::find_if(
        traffic_light_.begin(), traffic_light_.end(),
        [](const maf_perception_interface::TrafficLightDecision &data) {
          return data.pattern.value ==
                 TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_SOLID_CIRCLE;
        });
    if (left_arrow_it != traffic_light_.end() &&
        left_arrow_it->status.value !=
            TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF) {
      local_light_status = left_arrow_it->status.value;
      duration = left_arrow_it->duration;
      remaining = left_arrow_it->remaining;
    } else if (solid_circle_it != traffic_light_.end()) {
      local_light_status = solid_circle_it->status.value;
      duration = solid_circle_it->duration;
      remaining = solid_circle_it->remaining;
    }
    // for (auto &item : traffic_light_) {
    //   if (item.pattern.value == TRAFFIC_LIGHT_PATTERN_LEFT_ARROW) {
    //     if ()
    //     local_light_status = item.status.value;
    //     // light_pattern = item.pattern.value;
    //     duration = item.duration;
    //     remaining = item.remaining;
    //     break;
    //   }
    // }
  } else if (((current_direction_ == Direction::TURN_LEFT ||
               current_direction_ == Direction::U_TURN_LEFT) &&
              (!bhave_left_arrow)) ||
             current_direction_ == Direction::GO_STRAIGHT) {
    for (auto &item : traffic_light_) {
      if (item.pattern.value ==
              TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_SOLID_CIRCLE ||
          item.pattern.value ==
              TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_UP_ARROW) {
        local_light_status = item.status.value;
        // light_pattern = item.pattern.value;
        duration = item.duration;
        remaining = item.remaining;
        break;
      }
    }
  } else if (current_direction_ == Direction::TURN_RIGHT) {
    turn_right_flag_ = true;
    if (bhave_right_arrow) {
      for (auto &item : traffic_light_) {
        if (item.pattern.value ==
            TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_RIGHT_ARROW) {
          local_light_status = item.status.value;
          // light_pattern = item.pattern.value;
          duration = item.duration;
          remaining = item.remaining;
          break;
        }
      }
    } else {
      for (auto &item : traffic_light_) {
        if (item.pattern.value ==
                TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_SOLID_CIRCLE ||
            item.pattern.value ==
                TrafficLightPatternEnum::TRAFFIC_LIGHT_PATTERN_UP_ARROW) {
          if (curr_direct_traffic_) {
            straight_local_light_status = item.status.value;
            MSD_LOG(INFO, "TRAFFIC_LANE -- TEST -- DEBUG -- in logic");
            // light_pattern = item.pattern.value;
          } else {
            local_light_status = item.status.value;
            MSD_LOG(INFO, "TRAFFIC_LANE -- TEST -- DEBUG -- out logic");
          }
          duration = item.duration;
          remaining = item.remaining;
          break;
        }
      }
    }
  }

  light_status = local_light_status;
  straight_light_status_at_right_ = straight_local_light_status;
  duration_time = duration;
  remain_time = remaining;
}

void TrafficLightDecision::decision_compute(
    double dist_to_stopline, int direction,
    const std::vector<maf_perception_interface::TrafficLightPatternEnum>
        &light_patterns,
    const vector<maf_perception_interface::TrafficLightDecision> &traffic_light,
    double ego_vel, double ego_acc, bool is_inside_intersection,
    double crossing_length) {
  dist_to_stopline_ = dist_to_stopline;
  crossing_length_ = crossing_length;
  current_direction_ = direction;
  light_patterns_ = light_patterns;
  traffic_light_ = traffic_light;
  ego_vel_ = ego_vel;
  ego_acc_ = ego_acc;
  is_inside_intersection_ = is_inside_intersection;

  state_machine();
  state_cycles_ += 1;
  state_cycles_ = std::min(state_cycles_, 100);
  dist_to_stopline_pre_ = dist_to_stopline;
}

double TrafficLightDecision::compute_jerk(
    const std::pair<double, double> comfort_param) {
  double time_stop_1;
  double time_stop_2;
  double time_stop;
  double time_stop_max;
  double ret = 0.0;
  time_stop_max = fabs((comfort_param.first - ego_acc_) / comfort_param.second);
  time_stop_1 = (-ego_acc_ - sqrt(ego_acc_ * ego_acc_ -
                                  2.0 * comfort_param.second * ego_vel_)) /
                comfort_param.second;
  time_stop_2 = (-ego_acc_ + sqrt(ego_acc_ * ego_acc_ -
                                  2.0 * comfort_param.second * ego_vel_)) /
                comfort_param.second;
  if ((time_stop_1 < 0 && time_stop_2 < 0) ||
      (time_stop_1 < 0 && time_stop_2 > time_stop_max) ||
      (time_stop_1 > time_stop_max && time_stop_2 < 0) ||
      (time_stop_1 > time_stop_max && time_stop_2 > time_stop_max) ||
      ((ego_acc_ * ego_acc_ - 2 * comfort_param.second * ego_vel_) < 0)) {
    // MSD_LOG(INFO, "jerk case 1");
    time_stop =
        fabs((ego_vel_ + (ego_acc_ - comfort_param.first) * time_stop_max +
              1 / 2.0 * comfort_param.second * pow(time_stop_max, 2)) /
             (-comfort_param.first));
    ret = 1 / 2.0 * (comfort_param.first - ego_acc_) * pow(time_stop_max, 2) -
          1 / 3.0 * comfort_param.second * pow(time_stop_max, 3) +
          (ego_vel_ + (ego_acc_ - comfort_param.first) * time_stop_max +
           1 / 2.0 * comfort_param.second * pow(time_stop_max, 2)) *
              time_stop +
          1 / 2.0 * comfort_param.first * pow(time_stop, 2);
  } else if ((time_stop_2 < 0 || time_stop_2 > time_stop_max) &&
             (time_stop_1 > 0 && time_stop_1 < time_stop_max)) {
    // MSD_LOG(INFO, "jerk case 2");
    time_stop = time_stop_1;
    ret = ego_vel_ * time_stop + 1 / 2.0 * ego_acc_ * pow(time_stop, 2) +
          1 / 6.0 * comfort_param.second * pow(time_stop, 3);
  } else if ((time_stop_1 < 0 || time_stop_1 > time_stop_max) &&
             (time_stop_2 > 0 && time_stop_2 < time_stop_max)) {
    // MSD_LOG(INFO, "jerk case 3");
    time_stop = time_stop_2;
    ret = ego_vel_ * time_stop + 1 / 2.0 * ego_acc_ * pow(time_stop, 2) +
          1 / 6.0 * comfort_param.second * pow(time_stop, 3);
  } else if ((time_stop_1 > 0 && time_stop_1 < time_stop_max) &&
             (time_stop_2 > 0 && time_stop_2 < time_stop_max)) {
    // MSD_LOG(INFO, "jerk case 4");
    time_stop = max(time_stop_1, time_stop_2);
    ret = ego_vel_ * time_stop + 1 / 2.0 * ego_acc_ * pow(time_stop, 2) +
          1 / 6.0 * comfort_param.second * pow(time_stop, 3);
  }
  return ret;
}

std::pair<double, double>
TrafficLightDecision::compute_comfort_param(bool ignore_length) {
  std::pair<double, double> ret;
  auto planning_status = PlanningContext::Instance()->mutable_planning_status();
  size_t lon_follow_obs_len = 0;
  if (planning_status) {
    std::unordered_set<uint32_t> id_set;
    for (auto ids : planning_status->planning_result.lon_follow_obstacles) {
      id_set.insert(inverse_hash_prediction_id(ids));
    }
    lon_follow_obs_len = id_set.size();
  }
  if (ignore_length) {
    ret.first = -1.5;
    ret.second = -1.9;
    // MSD_LOG(INFO, "jerkfit ignore_length");
  } else if (crossing_length_ > kLongCrossingLength && lon_follow_obs_len < 2) {
    ret.first = -1.7;
    ret.second = -2.1;
    // MSD_LOG(INFO, "jerkfit long_crossing");
  } else if (crossing_length_ > kLongCrossingLength &&
             lon_follow_obs_len >= 2) {
    ret.first = -1.8;
    ret.second = -2.2;
    // MSD_LOG(INFO, "jerkfit lon_follow_obs_len: %d", lon_follow_obs_len);
  } else {
    ret.first = -1.6;
    ret.second = -2.0;
    // MSD_LOG(INFO, "jerkfit short_crossing");
  }
  return ret;
}

void TrafficLightDecision::state_machine() {
  MSD_LOG(INFO, "TrafficlightDicision--debug dist_to_stopline_: %f",
          dist_to_stopline_);
  if (current_state_ == CurrentState::INIT) {
    if (state_cycles_ == 1) {
      MSD_LOG(INFO, "TrafficlightDicision--debug  init");
    } else if (state_cycles_ > 1) {
      current_state_ = CurrentState::LANE_KEEPING;
      MSD_LOG(INFO, "TrafficlightDicision--debug from init to lane_keeping ");
    }
  } else if (current_state_ == CurrentState::LANE_KEEPING) {
    if (dist_to_stopline_ < kThresCheckTrafficLight) {
      if (current_direction_ != Direction::UNKNOWN) {
        entered_intersection_ = false;
        if (light_patterns_.size() == 0) {
          current_state_ = CurrentState::CROSSING;
        } else {
          current_state_ = CurrentState::APPROACH_STOPLINE;
        }
      }
      // else {
      //   MSD_LOG(INFO, "TrafficlightDicision--debug  UNDEFINED direction!!!");
      // }
    }
    // else {
    //   MSD_LOG(INFO, "TrafficlightDicision--debug dist_to_stopline_: %f",
    //           dist_to_stopline_);
    // }
  } else if (current_state_ == CurrentState::APPROACH_STOPLINE) {
    int light_status;
    double duration;
    double remaining;
    get_traffic_light_status(light_status, duration, remaining);
    //  0: OFF, 1: GREEB, 2: GREENBLINKING, 3: YELLOW, 4:RED, 5: YELLOWBLINKING
    //  6 : UNKNOWN
    if ((light_status ==
         TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN_BLINKING) ||
        (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) ||
        (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED)) {
      OFF_FLAG_ = false;
      MSD_LOG(INFO, "OFF_FLAG_ = false");
      // traffic_light_status_trigger = true;
    }
    bool action_stop =
        (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED);
    bool action_go =
        (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN ||
         (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF &&
          OFF_FLAG_ == true));
    bool action_reconsider =
        (light_status ==
             TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN_BLINKING ||
         (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW &&
          YELLOW_FLAG_));
    bool action_slow_down =
        ((light_status ==
          TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW_BLINKING) ||
         (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW &&
          (!YELLOW_FLAG_)));
    bool action_reconsider_go = false;
    double time_can_go{0.0};
    double time_can_go_new{0.0};

    auto planning_num = PlanningContext::Instance()->mutable_planning_status();
    size_t lon_follow_obs_num = 0;
    if (planning_num) {
      std::unordered_set<uint32_t> id_set;
      for (auto ids : planning_num->planning_result.lon_follow_obstacles) {
        id_set.insert(inverse_hash_prediction_id(ids));
      }
      lon_follow_obs_num = id_set.size();
    }
    // MSD_LOG(INFO, "NEWGB PLAN num - 1: %d", lon_follow_obs_num);

    if (light_status ==
        TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN_BLINKING) {
      MSD_LOG(INFO, "PLANNING light_status = GREENBLINKING");
      if (-0.0000001 < duration && duration < kGreenBlinkingDuration) {
        time_can_go_new = kGreenBlinkingDuration - duration -
                          kGREENBLINKINGByLonFollow * lon_follow_obs_num;
        MSD_LOG(INFO, "NNEWGB PLANINNG GREENBLINKING time num: %f , %d",
                time_can_go_new, lon_follow_obs_num);
      } else {
        time_can_go_new = kGREENBLINKINGTimeBuffer;
        MSD_LOG(INFO, "PLANNING kGREENBLINKINGTimeBuffer");
      }
    }

    if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) {
      if (-0.0000001 < duration && duration < kYELLOWTimeBuffer) {
        time_can_go_new = kYELLOWTimeBuffer - duration;
        MSD_LOG(INFO, "PLANNING GREENBLINKING YELLOW");
      } else {
        time_can_go_new = 0;
        MSD_LOG(INFO, "PLANNING GREENBLINKING YELLOW time=0");
      }
    }

    MSD_LOG(INFO, "NEWGB PLAN time_can_go_new - 1: %f", time_can_go_new);

    auto lon_plan_data =
        PlanningContext::Instance()->longitudinal_motion_planner_output();
    SpeedPoint speed_point_1;
    EvaluateByTime_right_1 =
        lon_plan_data.EvaluateByTime(0.001, &speed_point_1);
    if (EvaluateByTime_right_1) {
      lon_Evaluate_1 = speed_point_1.s;
    }

    SpeedPoint speed_point_time;
    EvaluateByTime_right_time =
        lon_plan_data.EvaluateByTime(time_can_go_new, &speed_point_time);
    if (EvaluateByTime_right_time) {
      lon_Evaluate_time = speed_point_time.s;
    }
    lon_s_Evaluate_time = lon_Evaluate_time - lon_Evaluate_1;
    MSD_LOG(INFO, "PLANNING lon_Evaluate_time: %f", lon_s_Evaluate_time);

    if (action_reconsider) {
      double dist_can_go_new = compute_jerk(compute_comfort_param(false));

      if (-0.0000001 < duration && duration < kGreenBlinkingDuration) {
        time_can_go = kYellowDuration + (kGreenBlinkingDuration - duration);
      } else {
        time_can_go = kYellowDuration;
      }

      if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) {
        time_can_go = remaining;
      }

      double dist_can_go = ego_vel_ * time_can_go;
      double dist_can_go_old = ego_vel_ * ego_vel_ / (2.0 * kComfortDec);
      double dynamic_comfort = kComfortDecEmer;
      if (ego_vel_ > 5.55 && ego_vel_ < 16.66) {
        dynamic_comfort = 3.5 - 0.09 * ego_vel_;
      } else if (ego_vel_ >= 16.66) {
        dynamic_comfort = kComfortDec;
      }
      MSD_LOG(INFO, "dynamic_comfort ego_vel_ dynamic_comfort : %f , %f",
              ego_vel_, dynamic_comfort);

      // action_reconsider_go =
      //     (dist_can_go_new > dist_to_stopline_) && (dist_can_go >
      //     dist_to_stopline_);
      // action_reconsider_go =
      //     (lon_s_Evaluate_time > dist_to_stopline_) && (dist_can_go_new >
      //     dist_to_stopline_);
      action_reconsider_go =
          ((lon_s_Evaluate_time > dist_to_stopline_) &&
           (dist_can_go_new > dist_to_stopline_)) ||
          ((lon_s_Evaluate_time - dist_to_stopline_ > kDistPlanningBuffer) &&
           (dist_can_go_new - dist_to_stopline_ > kDistComfortBuffer)) ||
          (ego_vel_ * ego_vel_ / (2.0 * dynamic_comfort) > dist_to_stopline_);

      if ((lon_s_Evaluate_time - dist_to_stopline_ > kDistPlanningBuffer) &&
              (dist_can_go_new - dist_to_stopline_ > kDistComfortBuffer) ||
          (ego_vel_ * ego_vel_ / (2.0 * dynamic_comfort) > dist_to_stopline_)) {
        MSD_LOG(INFO, "tjerk NEWBG PLANNING hotfix3 debug");
        // TriggerStatus trigger_status;
        // trigger_status.dbw_status = true;
        // trigger_status.start_time = 10;
        // trigger_status.end_time = 0;
        // trigger_status.scenario_name = "COVER_GREEN";
        // PncTriggerStatus::Instance()->set_status(trigger_status);
      }
      if ((lon_s_Evaluate_time - dist_to_stopline_ > kDistPlanningBuffer) &&
          (dist_can_go_new - dist_to_stopline_ > kDistComfortBuffer)) {
        MSD_LOG(INFO, "tjerk NEWBG PLANNING hotfix3 debug -- 1");
      }
      if (ego_vel_ * ego_vel_ / (2.0 * dynamic_comfort) > dist_to_stopline_) {
        MSD_LOG(INFO, "tjerk NEWBG PLANNING hotfix3 debug -- 2");
      }

      action_reconsider_go_FLAG_ = action_reconsider_go;

      MSD_LOG(INFO, "tjerkdebug : %f , %f , %f , %f", dist_to_stopline_,
              dist_can_go_old, dist_can_go, dist_can_go_new);
    }
    if ((light_status ==
         TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN) &&
        (!turn_right_flag_)) {
      current_state_ = CurrentState::COVER_LIGHT;
    }
    if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF &&
        OFF_FLAG_ == false) {
      MSD_LOG(INFO, "OFF_FLAG_ = false && light_status = OFF");
      if (!action_reconsider_go_FLAG_) {
        MSD_LOG(INFO, "OFF stop");
        current_state_ = CurrentState::RED_LIGHT_STOP;
        RED_LIGHT_STOP_CLASS_ = 6;
      } else if ((dist_to_stopline_ < kCrossingDistanceBuffer) &&
                 (light_status_pre_ ==
                      TrafficLightStatusEnum::
                          TRAFFIC_LIGHT_STATUS_GREEN_BLINKING ||
                  light_status_pre_ ==
                      TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF) &&
                 action_reconsider_go_FLAG_) {
        MSD_LOG(INFO, "OFF go");
        compute_output(false);
        current_state_ = CurrentState::CROSSING;
        RED_LIGHT_STOP_CLASS_ = 0;
      }
    }
    if (action_reconsider && action_reconsider_go) {
      is_GREENBLINKING_go_ = true;
      MSD_LOG(INFO, "NNEWGB PLAN is_GREENBLINKING_go_ - 1: %d",
              is_GREENBLINKING_go_);
    }
    if (action_stop || (action_reconsider && (!action_reconsider_go))) {
      MSD_LOG(INFO, "tjerk dynamic GREENBLINKING stop 1");
      current_state_ = CurrentState::RED_LIGHT_STOP;
      if (light_status ==
          TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN_BLINKING) {
        RED_LIGHT_STOP_CLASS_ = 2;
      } else if (light_status ==
                 TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) {
        RED_LIGHT_STOP_CLASS_ = 3;
      } else if (light_status ==
                 TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED) {
        RED_LIGHT_STOP_CLASS_ = 4;
      }
    } else if ((dist_to_stopline_ < kCrossingDistanceBuffer) &&
               (action_go ||
                (action_reconsider && action_reconsider_go))) { // decided to go
      MSD_LOG(INFO, "tjerk dynamic GREENBLINKING GO 2");
      compute_output(false);
      current_state_ = CurrentState::CROSSING;
      RED_LIGHT_STOP_CLASS_ = 0;
      MSD_LOG(INFO, "NEWGB PLAN crossing - flag -2");
    } else if (dist_to_stopline_ < kCrossingDistanceBuffer &&
               action_slow_down) { // slow down
      // MSD_LOG(INFO, "SetStopLineWrtLight: slow down");
      current_state_ = CurrentState::CROSSING;
    }

    if (light_status != TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN) {
      light_status_pre_ = light_status;
    }
  } else if (current_state_ == CurrentState::RED_LIGHT_STOP) {
    int light_status;
    double duration;
    double remaining;
    get_traffic_light_status(light_status, duration, remaining);

    // if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED) {
    //   traffic_light_status_trigger = false;
    // }
    is_GREENBLINKING_go_ = false;
    // MSD_LOG(INFO, "NEWGB PLAN is_GREENBLINKING_stop_ - 1: %d",
    // is_GREENBLINKING_go_);
    if (light_status ==
        TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW_BLINKING) {
      // MSD_LOG(INFO, "========  Traffic light yellow blinking  ========");
      compute_output(false);
      RED_LIGHT_STOP_CLASS_ = 0;
      is_passed_stopline_ = false;
      current_state_ = CurrentState::APPROACH_STOPLINE;
    } else if (light_status ==
               TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN) {
      // MSD_LOG(INFO,
      //         "========  Traffic light turns green or goes off  ========");
      compute_output(false);
      RED_LIGHT_STOP_CLASS_ = 0;
      is_passed_stopline_ = false;
      current_state_ = CurrentState::APPROACH_STOPLINE;
    } else if (light_status ==
               TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF) {
      if (number_light_status_OFF_ > flag_light_status_OFF) {
        compute_output(false);
        RED_LIGHT_STOP_CLASS_ = 0;
        current_state_ = CurrentState::APPROACH_STOPLINE;
        OFF_FLAG_ = true;
        MSD_LOG(INFO, "RED_LIGHT_STOP - OFF - go");
      } else {
        compute_output(true);
        MSD_LOG(INFO, "RED_LIGHT_STOP - OFF - stop");
        RED_LIGHT_STOP_CLASS_ = 6;
      }
      number_light_status_OFF_++;
      MSD_LOG(INFO, "RED_LIGHT_STOP number_light_status_OFF_: %d",
              number_light_status_OFF_);
    } else {
      compute_output(true);
    }
    if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) {
      if (number_light_status_YELLOW_ > flag_light_status_YELLOW) {
        compute_output(false);
        RED_LIGHT_STOP_CLASS_ = 0;
        current_state_ = CurrentState::APPROACH_STOPLINE;
        YELLOW_FLAG_ = false;
        MSD_LOG(INFO, "RED_LIGHT_STOP - YELLOW_num > flag - go");
      }
      // number_light_status_YELLOW_ ++;
      MSD_LOG(INFO, "RED_LIGHT_STOP number_light_status_YELLOW_: %d",
              number_light_status_YELLOW_);
    }
    if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED) {
      YELLOW_FLAG_ = true;
      number_light_status_YELLOW_ = 0;
      MSD_LOG(INFO, "RED_LIGHT_STOP number_light_status_YELLOW_ - reset: %d",
              number_light_status_YELLOW_);
    }
    if (light_status != TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN) {
      light_status_pre_ = light_status;
    }
  } else if (current_state_ == CurrentState::COVER_LIGHT) {
    int light_status;
    double duration;
    double remaining;
    double dist_can_go_cover = compute_jerk(compute_comfort_param(true));
    get_traffic_light_status(light_status, duration, remaining);
    is_GREENBLINKING_go_ = false;

    // MSD_LOG(INFO, "COVER jerkfit: %f , %f", dist_can_go_cover,
    // dist_to_stopline_);

    if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN) {
      MSD_LOG(INFO, "COVER_state_number_ ++");
      COVER_state_number_++;
    }
    MSD_LOG(INFO,
            "NEWGB PLAN is_GREENBLINKING_stop_ dist_COVER_state_number_: %d , "
            "%d ,  %f , %f",
            is_GREENBLINKING_go_, COVER_state_number_, dist_can_go_cover,
            dist_to_stopline_);
    // MSD_LOG(INFO, "SHORT COVER_state_number_ %d", COVER_state_number_);
    double time_can_go = 0.0;
    if (light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN ||
        light_status == TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF) {
      MSD_LOG(INFO, "COVER SEARCH 1");
      compute_output(false);
      RED_LIGHT_STOP_CLASS_ = 0;
      current_state_ = CurrentState::APPROACH_STOPLINE;
    } else if (light_status == TrafficLightStatusEnum::
                                   TRAFFIC_LIGHT_STATUS_GREEN_BLINKING ||
               light_status ==
                   TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) {
      time_can_go = kCOVERTimeBuffer;
      double dist_can_go = ego_vel_ * time_can_go;
      if ((dist_can_go_cover > dist_to_stopline_) &&
          (dist_can_go > dist_to_stopline_)) {
        MSD_LOG(INFO, "COVER SEARCH 2-1");
        compute_output(false);
        RED_LIGHT_STOP_CLASS_ = 0;
        current_state_ = CurrentState::APPROACH_STOPLINE;
      } else {
        MSD_LOG(INFO, "COVER SEARCH 2-4");
        compute_output(true);
        current_state_ = CurrentState::RED_LIGHT_STOP;
        if (light_status ==
            TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN_BLINKING) {
          RED_LIGHT_STOP_CLASS_ = 2;
        } else if (light_status ==
                   TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) {
          RED_LIGHT_STOP_CLASS_ = 3;
        }
      }
    } else if (light_status ==
               TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW_BLINKING) {
      MSD_LOG(INFO, "COVER SEARCH 5");
      compute_output(false);
      RED_LIGHT_STOP_CLASS_ = 0;
      current_state_ = CurrentState::APPROACH_STOPLINE;
    } else if (light_status ==
               TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED) {
      MSD_LOG(INFO, "COVER SEARCH 4");
      compute_output(true);
      current_state_ = CurrentState::RED_LIGHT_STOP;
      RED_LIGHT_STOP_CLASS_ = 4;
    } else if (light_status ==
               TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN) {
      if (light_status_pre_ ==
              TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_GREEN ||
          light_status_pre_ ==
              TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_OFF ||
          light_status_pre_ ==
              TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW_BLINKING) {
        if (is_left_car_go_ || is_right_car_go_ ||
            dist_can_go_cover > dist_to_stopline_ ||
            (COVER_state_number_ < kCOVERWaitTime &&
             dist_to_stopline_ > kCOVERGREENdist)) {
          MSD_LOG(INFO, "SHORT COVERTEST22 SEARCH FLOW GO - GREEN");
          compute_output(false);
          // TriggerStatus trigger_status;
          // trigger_status.dbw_status = true;
          // trigger_status.start_time = 10;
          // trigger_status.end_time = 0;
          // trigger_status.scenario_name = "COVER";
          // PncTriggerStatus::Instance()->set_status(trigger_status);
          // } else if (dist_to_stopline_ < kCrossingDistanceBuffer) {
          //   MSD_LOG(INFO, "COVER SEARCH 6-1-CROSSING");
          //   current_state_ = CurrentState::CROSSING;
          if (dist_to_stopline_ < kCrossingDistanceBuffer) {
            MSD_LOG(INFO, "COVER SEARCH 6-1-CROSSING");
            current_state_ = CurrentState::CROSSING;
          }
        } else {
          // if (dist_can_go_cover > dist_to_stopline_){
          //   MSD_LOG(INFO, "COVER SEARCH 6-1");
          //   compute_output(false);
          //   RED_LIGHT_STOP_CLASS_ = 0;
          // } else {
          //   MSD_LOG(INFO, "COVER SEARCH 6-Comfort");
          //   compute_output(true);
          //   RED_LIGHT_STOP_CLASS_ = 5;
          // }
          MSD_LOG(INFO, "COVER SEARCH 6-Comfort");
          compute_output(true);
          RED_LIGHT_STOP_CLASS_ = 5;
        }
      } else if (light_status_pre_ == TrafficLightStatusEnum::
                                          TRAFFIC_LIGHT_STATUS_GREEN_BLINKING ||
                 light_status_pre_ ==
                     TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_YELLOW) {
        if (dist_to_stopline_ < kCrossingDistanceBuffer) {
          MSD_LOG(INFO, "COVER SEARCH 6-2-CROSSING");
          current_state_ = CurrentState::CROSSING;
        } else {
          if (((ego_vel_ * kCOVERTimeBuffer) > dist_to_stopline_) &&
              dist_can_go_cover > dist_to_stopline_) {
            MSD_LOG(INFO, "COVER SEARCH 6-2-1");
            compute_output(false);
            RED_LIGHT_STOP_CLASS_ = 0;
          } else {
            MSD_LOG(INFO, "COVER SEARCH 6-2-4");
            compute_output(true);
            RED_LIGHT_STOP_CLASS_ = 5;
          }
        }
      } else if (light_status_pre_ ==
                 TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_RED) {
        MSD_LOG(INFO, "COVER SEARCH 6-4");
        compute_output(true);
        RED_LIGHT_STOP_CLASS_ = 5;
      } else if (light_status_pre_ ==
                 TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN) {
        MSD_LOG(INFO, "COVER SEARCH 6-6");
        // compute_output(true);
        // RED_LIGHT_STOP_CLASS_ = 5;
        if (is_left_car_go_ || is_right_car_go_) {
          MSD_LOG(INFO, "SHORT COVERTEST22 SEARCH FLOW GO - COVER");
          compute_output(false);
          if (dist_to_stopline_ < kCrossingDistanceBuffer) {
            MSD_LOG(INFO, "COVER SEARCH 6-6-CROSSING");
            current_state_ = CurrentState::CROSSING;
          }
          // TriggerStatus trigger_status;
          // trigger_status.dbw_status = true;
          // trigger_status.start_time = 10;
          // trigger_status.end_time = 0;
          // trigger_status.scenario_name = "COVER2";
        } else if (COVER_state_number_ > kCOVERWaitTime) {
          MSD_LOG(INFO, "SHORT COVER SEARCH 6-6");
          compute_output(true);
          RED_LIGHT_STOP_CLASS_ = 5;
        } else {
          MSD_LOG(INFO, "SHORT COVER SEARCH 6-6-0");
          compute_output(false);
          if (dist_to_stopline_ < kCrossingDistanceBuffer) {
            MSD_LOG(INFO, "COVER SEARCH 6-6-0-CROSSING");
            current_state_ = CurrentState::CROSSING;
          }
        }
      }
    }

    if (light_status != TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN) {
      light_status_pre_ = light_status;
    }
  }
  // else if (current_state_ == CurrentState::INTO_WAIT_ZONE) {
  //   // todo: impl later
  //   // pass
  // }

  if (is_inside_intersection_) {
    entered_intersection_ = true;
    COVER_state_number_ = 0;
    light_status_pre_ = TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN;
    // traffic_light_status_trigger = false;
    is_GREENBLINKING_go_ = false;
    MSD_LOG(INFO, "NEWGB PLAN is_GREENBLINKING_go_ - 2: %d",
            is_GREENBLINKING_go_);
  }

  if (entered_intersection_ && (!is_inside_intersection_)) {
    current_state_ = CurrentState::LANE_KEEPING;
    compute_output(false);
    is_passed_stopline_ = false;
    OFF_FLAG_ = true;
    YELLOW_FLAG_ = true;
    number_light_status_OFF_ = 0;
    number_light_status_YELLOW_ = 0;
    straight_light_status_at_right_ = 0;
    RED_LIGHT_STOP_CLASS_ = 0;
    turn_right_flag_ = false;
    COVER_state_number_ = 0;
    action_reconsider_go_FLAG_ = false;
    light_status_pre_ = TrafficLightStatusEnum::TRAFFIC_LIGHT_STATUS_UNKNOWN;
    is_left_car_go_ = false;
    is_right_car_go_ = false;

    MSD_LOG(INFO, "number_OFF_YELLOW: %d , %d", number_light_status_OFF_,
            number_light_status_YELLOW_);
  }

  MSD_LOG(INFO, "CLASS - RED_LIGHT_STOP_CLASS_ %d , %d", RED_LIGHT_STOP_CLASS_,
          straight_light_status_at_right_);
}

void TrafficLightDecision::compute_output(const bool &stop_decision) {
  if (stop_decision) {
    stop_point_ = std::max(0.0, set_stopline_distance() - kStopDisBuffer);
    stop_flag_ = true;
  } else {
    stop_point_ = kDistNoStopLine;
    stop_flag_ = false;
  }
}

double TrafficLightDecision::set_stopline_distance() {
  double stopline_dist = dist_to_stopline_;
  if ((dist_to_stopline_ - dist_to_stopline_pre_) > kPassingStopLineBuffer &&
      std::fabs(dist_to_stopline_pre_) < kStopDisPreChecker) {
    is_passed_stopline_ = true;
  }

  if (is_passed_stopline_) {
    stopline_dist = 0;
  }
  stopline_dist = std::max(stopline_dist, 0.0);
  return stopline_dist + kStopLineBuffer;
}

} // namespace msquare
