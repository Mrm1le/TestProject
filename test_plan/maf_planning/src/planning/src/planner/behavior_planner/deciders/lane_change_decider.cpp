#include "planner/behavior_planner/deciders/lane_change_decider.h"
#include "common/config_context.h"

namespace msquare {

LaneChangeDecider::LaneChangeDecider(const TaskConfig &config)
    : Decider(config) {
  // mph_assert(config.has_lane_change_decider_config());
}

void LaneChangeDecider::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

void LaneChangeDecider::reset(const TaskConfig &config) {
  Task::reset(config);

  called_in_state_machine_ = false;
  lead_car_.start_s = 0.0;
  lead_car_.end_s = 0.0;
  lead_car_.intention = "none";
  lead_car_.temp_lead = -1;
  lead_car_.temp_lead_v_rel = 30.0;
  obstacle_on_target_.clear();
  target_gap_ = {-10, -10};
  target_gap_cost_ = std::numeric_limits<double>::max();
  min_l_threshold_ = 1.3;
  max_l_threshold_ = 5.0;
  most_front_car_ = {-1, 1, 1000.0, 100.0, 100.0, 20.0};
  most_rear_car_ = {-2, 1, -1100.0, -110.0, -110.0, -20.0};
  gap_list_.clear();
  cur_lane_.clear();
  target_lane_.clear();
  most_front_static_obs_dis_ = 0.0;

  overtake_obs_id_ = -5;
  lc_wait_time_ = 0.0;
  average_gap_length_ = 20.0;
  lane_change_cross_num_ = 0;
  target_lane_invisible_s_sections_.clear();
  is_most_front_gap_invisible_ = true;
  is_most_front_gap_exist_and_invisible_ = false;
  is_target_gap_frozen_ = false;
}

void LaneChangeDecider::unset() {
  obstacle_on_target_.clear();
  gap_list_.clear();
  cur_lane_.clear();
  target_lane_.clear();
  target_lane_invisible_s_sections_.clear();
}

double LaneChangeDecider::calc_equation_ttc(const double &a, const double &b,
                                            const double &c) {
  if (b * b - 4 * a * c < 0)
    return std::numeric_limits<double>::max();
  if (std::fabs(a) < 1e-8 && std::fabs(b) < 1e-8)
    return std::numeric_limits<double>::max();
  if (std::fabs(a) < 1e-8)
    return -c / b;
  return (-0.5 * b / a + std::sqrt((b * b - 4 * a * c) / (4 * a * a)));
}

double LaneChangeDecider::calc_predict_drel(const Obstacle *object,
                                            const PathPoint &egopoint) {
  auto &trajectory = object->Trajectory();
  double start_s = 0.0;
  double ego_length =
      ConfigurationContext::Instance()->get_vehicle_param().length;
  auto bounding_box = object->PerceptionBoundingBox();
  double object_length = bounding_box.length();
  for (auto point : trajectory) {
    if ((point.relative_time < 0.1) && (point.relative_time > -0.1)) {
      start_s = point.path_point.s;
    }
  }
  double min = std::numeric_limits<double>::max();
  double drel = 1.0;
  for (auto point : trajectory) {
    double delta_x = point.path_point.x - egopoint.x;
    double delta_y = point.path_point.y - egopoint.y;
    if ((delta_x * delta_x + delta_y * delta_y) < min) {
      drel = start_s - point.path_point.s;
      min = delta_x * delta_x + delta_y * delta_y;
      if ((drel < 0) && (-drel > 0.5 * (ego_length + object_length))) {
        drel = drel + 0.5 * (ego_length + object_length);
      };
      if ((drel < 0) && (-drel < 0.5 * (ego_length + object_length))) {
        drel = 0;
      };
      if ((drel > 0) && (drel > 0.5 * (ego_length + object_length))) {
        drel = drel - 0.5 * (ego_length + object_length);
      };
      if ((drel > 0) && (drel < 0.5 * (ego_length + object_length))) {
        drel = 0;
      };
    }
  }
  return drel;
}

bool LaneChangeDecider::updata_real_obstacle_on_target(UpdateType rqt,
                                                       int origin_id,
                                                       int target_id) {
  real_obstacle_on_target_.clear();
  const auto &merge_info = world_model_->get_map_info().next_merge_info();
  const auto &planning_status = context_->planning_status();
  current_lane_id_ = planning_status.lane_status.target_lane_id;
  if (rqt == EXTERNAL) {
    current_lane_id_ = origin_id;
    target_lane_id_ = target_id;
  } else {
    MSD_LOG(INFO, "fengxiaotong::lc_decide inter tar_id[%d] cur_id[%d]",
            target_lane_id_, current_lane_id_);
  }
  if ((world_model_->get_baseline_info(current_lane_id_) != nullptr) &&
      (world_model_->get_baseline_info(target_lane_id_) != nullptr)) {
  } else {
    return false;
  }
  auto &obstacles = world_model_->get_baseline_info(current_lane_id_)
                        ->obstacle_manager()
                        .get_obstacles()
                        .Items();
  // get target lane pred to avoid frenet invalid
  auto &obstacles_target = world_model_->get_baseline_info(target_lane_id_)
                               ->obstacle_manager()
                               .get_obstacles()
                               .Items();
  auto &lateral_obstacle = world_model_->mutable_lateral_obstacle();
  const auto &prev_follow_obstacles = PlanningContext::Instance()
                                          ->planning_status()
                                          .planning_result.lon_follow_obstacles;
  auto target_baseline = world_model_->get_baseline_info(target_lane_id_);
  auto target_lane_adc_sl = target_baseline->get_adc_sl_boundary();
  auto &target_lane_path_data = target_baseline->get_path_data();
  PathPoint adc_point;
  adc_point.x = target_baseline->get_ego_state().ego_pose.x;
  adc_point.y = target_baseline->get_ego_state().ego_pose.y;
  double adc_path_s =
      target_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto lane_border = target_lane_path_data.getWidth(adc_path_s);
  double ego_s = world_model_->get_baseline_info(current_lane_id_)
                     ->get_adc_sl_boundary()
                     .end_s;
  double ego_s_start = world_model_->get_baseline_info(current_lane_id_)
                           ->get_adc_sl_boundary()
                           .start_s;
  double v_ego = world_model_->get_baseline_info(current_lane_id_)
                     ->get_ego_state()
                     .ego_vel;
  double a_ego = world_model_->get_baseline_info(current_lane_id_)
                     ->get_ego_state()
                     .ego_acc;
  double lateral_space =
      std::max(std::max(target_lane_adc_sl.start_l - lane_border.second, 0.0),
               std::max(lane_border.first - target_lane_adc_sl.end_l, 0.0));
  double most_front_obstacle_in_target_line_dis{0.0};
  double nearest_obstacle_in_solid_line_dis{std::numeric_limits<double>::max()};
  double closest_fast_approach_car_behind_distance{
      std::numeric_limits<double>::max()};
  auto curr_lane_adc_sl =
      world_model_->get_baseline_info(current_lane_id_)->get_adc_sl_boundary();
  nearest_ttc_ = std::numeric_limits<double>::max();
  cloest_rear_dis_ = std::numeric_limits<double>::max();
  cloest_rear_dis_p1_ = std::numeric_limits<double>::min();
  cloest_front_dis_ = std::numeric_limits<double>::max();
  cloest_front_dis_p1_ = std::numeric_limits<double>::max();
  cloest_front_ttc_ = std::numeric_limits<double>::max();
  cloest_front_nttc_ = std::numeric_limits<double>::max();
  cloest_rear_ttc_ = std::numeric_limits<double>::max();
  cloest_rear_nttc_ = std::numeric_limits<double>::max();
  // find target lane object and follow object
  for (auto &obstacle : obstacles) {
    SLBoundary sl_boundary = obstacle->PerceptionSLBoundary();
    if (!obstacle->IsFrenetInvalid() && is_on_target(obstacle)) {
      if (sl_boundary.start_s - ego_s >
          most_front_obstacle_in_target_line_dis) {
        most_front_obstacle_in_target_line_ = obstacle;
        most_front_obstacle_in_target_line_dis = sl_boundary.start_s - ego_s;
      }
      if (sl_boundary.start_s - ego_s < dis_to_change_point_) {
        TargetObstacle target_obs;
        target_obs.id = obstacle->Id();
        target_obs.type = (int)obstacle->Type();
        target_obs.accel = obstacle->acceleration();
        if (sl_boundary.start_s - ego_s > 0) {
          target_obs.d_rel = sl_boundary.start_s - ego_s;
        } else if (sl_boundary.end_s - ego_s_start < 0) {
          target_obs.d_rel = sl_boundary.end_s - ego_s_start;
        } else {
          target_obs.d_rel = 0.0;
        }
        MSD_LOG(INFO,
                "LC_DECIDER:: no predict update [%d] start s [%.2f] end s "
                "[%.2f] ego_s [%.2f]",
                obstacle->Id(), target_obs.start_s, target_obs.end_s,
                ego_s_start);
        MSD_LOG(INFO,
                "LC_DECIDER:: no predict  curr_lane_adc_sl.start_s [%.2f] "
                "cloest_rear_dis_ [%.2f]",
                curr_lane_adc_sl.start_s, cloest_rear_dis_);
        if (target_obs.d_rel >= 0.0 && target_obs.d_rel < cloest_front_dis_) {
          cloest_front_dis_ = target_obs.d_rel;
          double temp_pstart_s =
              sl_boundary.start_s - v_ego + obstacle->speed();
          double temp_pend_s = sl_boundary.end_s - v_ego + obstacle->speed();
          if (temp_pstart_s - ego_s > 0) {
            cloest_front_dis_p1_ = temp_pstart_s - ego_s;
          } else if (temp_pend_s - ego_s_start < 0) {
            cloest_front_dis_p1_ = temp_pend_s - ego_s_start;
          } else {
            cloest_front_dis_p1_ = 0.0;
          }
          closest_car_front_id = obstacle->Id();
          double tmp_ttc =
              calc_equation_ttc((a_ego - obstacle->acceleration()) * 0.5,
                                v_ego - obstacle->speed(), -target_obs.d_rel);
          if (tmp_ttc >= 0 && tmp_ttc < cloest_front_ttc_)
            cloest_front_ttc_ = tmp_ttc;
          if (abs(v_ego - obstacle->speed()) > 0.01) {
            tmp_ttc = target_obs.d_rel / (v_ego - obstacle->speed());
            if (tmp_ttc >= 0 && tmp_ttc < cloest_front_nttc_)
              cloest_front_nttc_ = tmp_ttc;
          }
        }
        if (std::fabs(target_obs.d_rel) < 1e-8) {
          nearest_ttc_ = 0.0;
        } else {
          double tmp_ttc =
              calc_equation_ttc((a_ego - obstacle->acceleration()) * 0.5,
                                v_ego - obstacle->speed(), -target_obs.d_rel);
          if (tmp_ttc >= 0 && tmp_ttc < nearest_ttc_)
            nearest_ttc_ = tmp_ttc;
        }
        // calc_equation_ttc
        target_obs.start_s = sl_boundary.start_s;
        target_obs.end_s = sl_boundary.end_s;
        if (sl_boundary.end_s < curr_lane_adc_sl.start_s) {
          if (curr_lane_adc_sl.start_s - sl_boundary.end_s < cloest_rear_dis_) {
            MSD_LOG(INFO,
                    "LC_DECIDER:: update_rear curr_lane_adc_sl.start_s [%.2f] "
                    "cloest_rear_dis_ [%.2f]",
                    curr_lane_adc_sl.start_s, cloest_rear_dis_);
            cloest_rear_dis_ = curr_lane_adc_sl.start_s - sl_boundary.end_s;
            double temp_pstart_s =
                sl_boundary.start_s - v_ego + obstacle->speed();
            double temp_pend_s = sl_boundary.end_s - v_ego + obstacle->speed();
            if (temp_pstart_s - ego_s > 0) {
              cloest_rear_dis_p1_ = temp_pstart_s - ego_s;
            } else if (temp_pend_s - ego_s_start < 0) {
              cloest_rear_dis_p1_ = temp_pend_s - ego_s_start;
            } else {
              cloest_rear_dis_p1_ = 0.0;
            }
            closest_car_id_ = obstacle->Id();
            double tmp_ttc =
                calc_equation_ttc((a_ego - obstacle->acceleration()) * 0.5,
                                  v_ego - obstacle->speed(), -target_obs.d_rel);
            if (tmp_ttc >= 0 && tmp_ttc < cloest_rear_ttc_)
              cloest_rear_ttc_ = tmp_ttc;
            if (abs(v_ego - obstacle->speed()) > 0.01) {
              tmp_ttc = target_obs.d_rel / (v_ego - obstacle->speed());
              if (tmp_ttc >= 0 && tmp_ttc < cloest_rear_nttc_)
                cloest_rear_nttc_ = tmp_ttc;
            }
          }
          // TODO: fengxiaotong, big size may need not influence min_gap_s
          if (obstacle->PerceptionBoundingBox().width() >
              lateral_space + 0.10) {
            min_gap_s_ = std::max(min_gap_s_, sl_boundary.end_s + 1.e-2);
          }
          double follow_dis = curr_lane_adc_sl.start_s - sl_boundary.end_s;
          constexpr double kTTC = 0.4;
          double ref_follow_dis =
              (2 * obstacle->speed() *
                   std::cos(obstacle->Yaw_relative_frenet()) -
               v_ego) *
              kTTC;
          if (follow_dis < ref_follow_dis) {
            MSD_LOG(INFO,
                    "LaneChangdeDecider: obs[%d] fast approach no space to be "
                    "follower",
                    obstacle->Id());
            if (!exist_fast_approach_car_from_behind_) {
              exist_fast_approach_car_from_behind_ = true;
              closest_fast_approach_car_id_ = obstacle->Id();
              closest_fast_approach_car_behind_distance = follow_dis;
            } else {
              if (follow_dis < closest_fast_approach_car_behind_distance) {
                closest_fast_approach_car_id_ = obstacle->Id();
                closest_fast_approach_car_behind_distance = follow_dis;
              }
            }
          }
        }
        target_obs.v_rel =
            obstacle->speed() * std::cos(obstacle->Yaw_relative_frenet()) -
            v_ego;
        target_obs.intention = identify_intention(obstacle);
        MSD_LOG(INFO, "LaneChangeDecider: obs[%d] on target lane intention %s",
                obstacle->Id(), target_obs.intention.c_str());
        obstacle_on_target_.push_back(target_obs);
        real_obstacle_on_target_.emplace_back(obstacle->Id());
        if (obstacle->IsStatic()) {
          most_front_static_obs_dis_ =
              std::max(most_front_static_obs_dis_,
                       obstacle->PerceptionSLBoundary().end_s);
        }
        // MSD_LOG(INFO, "gap obstacle[%d] in target_lane", obstacle->Id());
      } else { // car in solid line
        if (sl_boundary.start_s - ego_s < nearest_obstacle_in_solid_line_dis) {
          nearest_obstacle_in_solid_line_ = obstacle;
          nearest_obstacle_in_solid_line_dis = sl_boundary.start_s - ego_s;
        }
      }
    }
  }
  for (auto &obstacle : obstacles_target) {
    // to avoid frenet invalid , need consider prediction
    if (std::find(real_obstacle_on_target_.begin(),
                  real_obstacle_on_target_.end(),
                  obstacle->Id()) != real_obstacle_on_target_.end())
      continue;
    if (obstacle->IsFrenetInvalid() && is_pred_on_target(obstacle, v_ego) &&
        !obstacle->IsPredictFrenetInvalid()) {
      SLBoundary pred_sl_boundary = obstacle->PredictionSLBoundary();
      pred_sl_boundary.start_s -=
          obstacle->PredSLBoundaryRelativeTime() * v_ego;
      pred_sl_boundary.end_s -= obstacle->PredSLBoundaryRelativeTime() * v_ego;
      if (pred_sl_boundary.start_s - ego_s >
          most_front_obstacle_in_target_line_dis) {
        most_front_obstacle_in_target_line_ = obstacle;
        most_front_obstacle_in_target_line_dis =
            pred_sl_boundary.start_s - ego_s;
      }
      if (pred_sl_boundary.start_s - ego_s < dis_to_change_point_) {
        TargetObstacle target_obs;
        TrackedObject temp_car{};
        if (lateral_obstacle.find_track(obstacle->Id(), temp_car)) {
          target_obs.d_rel = temp_car.d_rel;
          MSD_LOG(INFO, "LC_DECIDER:: find lateral_obstacle [%.2f]",
                  target_obs.d_rel);
        } else {
          target_obs.d_rel = calc_predict_drel(obstacle, adc_point);
          MSD_LOG(INFO, "LC_DECIDER:: calc_predict_drel [%.2f]",
                  target_obs.d_rel);
        }
        if (target_obs.d_rel < 0) {
          auto &box = obstacle->PerceptionBoundingBox();
          double length = box.length();
          target_obs.end_s = ego_s_start + target_obs.d_rel;
          target_obs.start_s = target_obs.end_s - length;
        }
        MSD_LOG(
            INFO,
            "LC_DECIDER:: update [%d] start s [%.2f] end s [%.2f] ego_s [%.2f]",
            obstacle->Id(), target_obs.start_s, target_obs.end_s, ego_s_start);
        MSD_LOG(INFO,
                "LC_DECIDER:: curr_lane_adc_sl.start_s [%.2f] cloest_rear_dis_ "
                "[%.2f]",
                curr_lane_adc_sl.start_s, cloest_rear_dis_);
        target_obs.id = obstacle->Id();
        target_obs.type = (int)obstacle->Type();
        target_obs.accel = obstacle->acceleration();
        if (target_obs.end_s < curr_lane_adc_sl.end_s) {
          if (curr_lane_adc_sl.start_s - target_obs.end_s < cloest_rear_dis_) {
            MSD_LOG(INFO,
                    "LC_DECIDER:: update_rear curr_lane_adc_sl.start_s [%.2f] "
                    "cloest_rear_dis_ [%.2f]",
                    curr_lane_adc_sl.start_s, cloest_rear_dis_);

            cloest_rear_dis_ = curr_lane_adc_sl.start_s - target_obs.end_s;
            double temp_pstart_s =
                target_obs.start_s - v_ego + obstacle->speed();
            double temp_pend_s = target_obs.end_s - v_ego + obstacle->speed();
            if (temp_pstart_s - ego_s > 0) {
              cloest_rear_dis_p1_ = temp_pstart_s - ego_s;
            } else if (temp_pend_s - ego_s_start < 0) {
              cloest_rear_dis_p1_ = temp_pend_s - ego_s_start;
            } else {
              cloest_rear_dis_p1_ = 0.0;
            }
            closest_car_id_ = obstacle->Id();
            double tmp_ttc =
                calc_equation_ttc((a_ego - obstacle->acceleration()) * 0.5,
                                  v_ego - obstacle->speed(), -target_obs.d_rel);
            if (tmp_ttc >= 0 && tmp_ttc < cloest_rear_ttc_)
              cloest_rear_ttc_ = tmp_ttc;
            if (abs(v_ego - obstacle->speed()) > 0.01) {
              tmp_ttc = target_obs.d_rel / (v_ego - obstacle->speed());
              if (tmp_ttc >= 0 && tmp_ttc < cloest_rear_nttc_)
                cloest_rear_nttc_ = tmp_ttc;
            }
          }
          // TODO: fengxiaotong, big size may need not influence min_gap_s
          if (obstacle->PerceptionBoundingBox().width() >
              lateral_space + 0.10) {
            MSD_LOG(INFO,
                    "LaneChangeDecider: pred obs[%d] width %f no space to be "
                    "leader",
                    obstacle->Id(), obstacle->PerceptionBoundingBox().width());
            min_gap_s_ = std::max(min_gap_s_, pred_sl_boundary.end_s + 1.e-2);
          }
          double follow_dis = curr_lane_adc_sl.start_s - target_obs.end_s;
          constexpr double kTTC = 0.4;
          double ref_follow_dis =
              (2 * obstacle->speed() *
                   std::cos(obstacle->Yaw_relative_frenet()) -
               v_ego) *
              kTTC;
          if (follow_dis < ref_follow_dis) {
            MSD_LOG(
                INFO,
                "LaneChangdeDecider: pred obs[%d] fast approach no space to be "
                "follower",
                obstacle->Id());
            if (!exist_fast_approach_car_from_behind_) {
              exist_fast_approach_car_from_behind_ = true;
              exist_fast_approach_car_from_behind_ = obstacle->Id();
              closest_fast_approach_car_behind_distance = follow_dis;
            } else {
              if (follow_dis < closest_fast_approach_car_behind_distance) {
                exist_fast_approach_car_from_behind_ = obstacle->Id();
                closest_fast_approach_car_behind_distance = follow_dis;
              }
            }
          }
        }
        target_obs.v_rel =
            obstacle->speed() * std::cos(obstacle->Yaw_relative_frenet()) -
            v_ego;
        target_obs.intention = identify_intention(obstacle);
        MSD_LOG(INFO,
                "LaneChangeDecider: pred obs[%d] on target lane intention %s",
                obstacle->Id(), target_obs.intention.c_str());
        obstacle_on_target_.push_back(target_obs);
        real_obstacle_on_target_.emplace_back(obstacle->Id());
        if (obstacle->IsStatic()) {
          most_front_static_obs_dis_ =
              std::max(most_front_static_obs_dis_,
                       obstacle->PerceptionSLBoundary().end_s);
        }
        // MSD_LOG(INFO, "gap obstacle[%d] in target_lane", obstacle->Id());
      } else { // car in solid line
        if (pred_sl_boundary.start_s - ego_s <
            nearest_obstacle_in_solid_line_dis) {
          nearest_obstacle_in_solid_line_ = obstacle;
          nearest_obstacle_in_solid_line_dis = pred_sl_boundary.start_s - ego_s;
        }
      }
    }
  }
  auto planner_debug = PlanningContext::Instance()->mutable_planner_debug();
  planner_debug->lat_dec_info.obstacle_gap.clear();
  for (auto obj : obstacle_on_target_) {
    GapObstacleDebugInfo temp_obj;
    temp_obj.id = obj.id;
    temp_obj.drel = obj.d_rel;
    temp_obj.speed = obj.v_rel;
    temp_obj.accel = obj.accel;
    if (obj.id == closest_car_front_id) {
      temp_obj.gap_info = 1;
    } else if (obj.id == closest_car_id_) {
      temp_obj.gap_info = 2;
    } else {
      temp_obj.gap_info = 0;
    }
    planner_debug->lat_dec_info.obstacle_gap.emplace_back(temp_obj);
  }
  return true;
}

TaskStatus LaneChangeDecider::process() {
  MLOG_PROFILING(name_.c_str());
  return TaskStatus::STATUS_SUCCESS;
  if (world_model_ == nullptr) {
    MSD_LOG(INFO, "LaneChangeDecider: world model is none!");
    return TaskStatus::STATUS_FAILED;
  }

  // id = -1 stands for virtual front/back obstacle; id = -10 stands for no
  // available gap
  target_gap_ = std::make_pair(-10, -10);
  target_gap_cost_ = std::numeric_limits<double>::max();
  gap_list_.clear();
  cur_lane_.clear();
  target_lane_.clear();

  const auto &planning_status = context_->planning_status();
  const auto &state_machine_out = context_->state_machine_output();

  bool merge_or_split_scenario = false;
  const auto &merge_info = world_model_->get_map_info().next_merge_info();
  if (merge_info.orientation.value != Orientation::UNKNOWN &&
      !merge_info.is_continue && !merge_info.is_split) {
    merge_or_split_scenario = true;
  }
  MSD_LOG(INFO, "LaneChangeDecider: xyz merge direction %d %d %d %f",
          merge_info.orientation.value, merge_info.is_continue,
          merge_info.is_split, merge_info.distance);

  // only enable in lane change preparation stage
  if (planning_status.lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      planning_status.lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION) {
    lc_wait_time_ =
        planning_status.lane_status.change_lane.lane_change_wait_time;
    gap_selection_scenario_ = "LANE_CHANGE";
    if (!config_.lane_change_decider_config().execute_in_lane_change_scenario) {
      return TaskStatus::STATUS_SUCCESS;
    }
  } else if (planning_status.lane_status.status ==
                 LaneStatus::Status::LANE_KEEP &&
             merge_or_split_scenario) {
    gap_selection_scenario_ = "LANE_MERGE";
    MSD_LOG(INFO, "LaneChangeDecider: merge direction %d",
            merge_info.orientation.value);
    if (!config_.lane_change_decider_config().execute_in_lane_merge_scenario ||
        world_model_->get_map_info().is_in_intersection()) {
      return TaskStatus::STATUS_SUCCESS;
    }
  } else {
    context_->mutable_planning_status()
        ->lane_status.change_lane.target_gap_obs = target_gap_;
    return TaskStatus::STATUS_SUCCESS;
  }
  current_lane_id_ = planning_status.lane_status.target_lane_id;
  dis_to_change_point_ = std::numeric_limits<double>::max();
  const auto &map_info = world_model_->get_map_info();
  if (map_info.is_on_highway()) {
    scenario_type_ = ScenarioType::HIGHWAY;
  } else {
    scenario_type_ = ScenarioType::URBAN;
  }
  // bool is_in_dash_line = false;
  if (gap_selection_scenario_ == "LANE_CHANGE") {
    if (planning_status.lane_status.change_lane.direction == "left") {
      lane_change_direction_ = 1;
      target_lane_id_ = current_lane_id_ - 1;
      if (planning_status.lane_status.change_lane.path_id == 0) {
        dis_to_change_point_ = map_info.get_distance_to_dash_line("right");
      } else {
        dis_to_change_point_ = map_info.get_distance_to_dash_line("left");
      }
    } else if (context_->planning_status().lane_status.change_lane.direction ==
               "right") {
      lane_change_direction_ = -1;
      target_lane_id_ = current_lane_id_ + 1;
      if (planning_status.lane_status.change_lane.path_id == 0) {
        // TODO: fengxiaotong (target id == 0) may not means on target lane
        dis_to_change_point_ = map_info.get_distance_to_dash_line("left");
      } else {
        dis_to_change_point_ = map_info.get_distance_to_dash_line("right");
      }
    }
    dis_to_change_point_ =
        std::min(dis_to_change_point_, map_info.lc_end_dis());
  } else if (gap_selection_scenario_ == "LANE_MERGE") {
    dis_to_change_point_ = merge_info.distance;
    if (merge_info.orientation.value == Orientation::LEFT) {
      lane_change_direction_ = -1;
      target_lane_id_ = current_lane_id_ + 1;
    } else if (merge_info.orientation.value == Orientation::RIGHT) {
      lane_change_direction_ = 1;
      target_lane_id_ = current_lane_id_ - 1;
    } else {
      MSD_LOG(INFO, "LaneChangeDecider: invalid merge direction %d",
              merge_info.orientation.value);
      context_->mutable_planning_status()
          ->lane_status.change_lane.target_gap_obs = target_gap_;
      return TaskStatus::STATUS_SUCCESS;
    }
  } else {
    MSD_LOG(INFO, "LaneChangeDecider: invalid gap_selection_scenario");
    context_->mutable_planning_status()
        ->lane_status.change_lane.target_gap_obs = target_gap_;
    return TaskStatus::STATUS_SUCCESS;
  }

  MSD_LOG(INFO, "LaneChangeDecider: dis_to_change_point %f v_limit %f",
          dis_to_change_point_, planning_status.v_limit);
  map_v_limit_ = world_model_->get_map_info().v_cruise();
  constexpr double kMinReservedLCTime = 10.0;
  // v_limit_ = planning_status.v_limit;
  bool is_pre_in_wait_stage =
      PlanningContext::Instance()->planning_status().lane_status.status ==
          LaneStatus::Status::LANE_CHANGE &&
      PlanningContext::Instance()
              ->planning_status()
              .lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
  v_limit_ = context_->planning_status().v_limit;
  lead_car_.id =
      PlanningContext::Instance()->planning_status().planning_result.id_yield;
  if (lead_car_.id <= 0 &&
      planning_status.lane_status.change_lane.set_virtual_obstacle) {
    lead_car_.id = -3;
  }

  if (!world_model_->get_baseline_info(current_lane_id_)) {
    MSD_LOG(INFO, "LaneChangeDecider: invalid origin_lane_id %d",
            current_lane_id_);
  }
  if (!world_model_->get_baseline_info(target_lane_id_)) {
    MSD_LOG(INFO, "LaneChangeDecider: invalid target_lane_id %d",
            target_lane_id_);
  }
  auto curr_baseline = world_model_->get_baseline_info(current_lane_id_);
  auto target_baseline = world_model_->get_baseline_info(target_lane_id_);

  if (curr_baseline == nullptr || !curr_baseline->is_valid()) {
    if (gap_selection_scenario_ == "LANE_MERGE") {
      return TaskStatus::STATUS_SUCCESS;
    }
    MSD_LOG(INFO, "LaneChangeDecider: curr_baseline is nullptr!");
    return TaskStatus::STATUS_FAILED;
  }
  if (target_baseline == nullptr || !target_baseline->is_valid()) {
    if (gap_selection_scenario_ == "LANE_MERGE") {
      return TaskStatus::STATUS_SUCCESS;
    }
    MSD_LOG(INFO, "LaneChangeDecider: target line is nullptr!");
    return TaskStatus::STATUS_FAILED;
  }

  // judge if ego car is in target lane
  if (target_baseline && target_baseline->is_valid()) {
    auto target_lane_adc_sl_boundary = target_baseline->get_adc_sl_boundary();
    constexpr double kMinLaneDis = 0.1;
    double adc_lane_dis = std::min(target_lane_adc_sl_boundary.end_l, 0.0) -
                          std::max(target_lane_adc_sl_boundary.start_l, 0.0);
    MSD_LOG(INFO, "LaneChangeDecider: adc in target lane adc_lane_dis %f",
            adc_lane_dis);
    if (std::abs(adc_lane_dis) < kMinLaneDis) {
      context_->mutable_planning_status()
          ->lane_status.change_lane.target_gap_obs = {-1, -2};
      target_gap_ = {-1, -2};
      return TaskStatus::STATUS_SUCCESS;
    }
  }

  auto target_lane_adc_sl = target_baseline->get_adc_sl_boundary();
  auto &target_lane_path_data = target_baseline->get_path_data();
  PathPoint adc_point;
  adc_point.x = target_baseline->get_ego_state().ego_pose.x;
  adc_point.y = target_baseline->get_ego_state().ego_pose.y;
  double adc_path_s =
      target_lane_path_data.discretized_path().QueryMatchedS(adc_point);
  auto lane_border = target_lane_path_data.getWidth(adc_path_s);
  double lateral_space =
      std::max(std::max(target_lane_adc_sl.start_l - lane_border.second, 0.0),
               std::max(lane_border.first - target_lane_adc_sl.end_l, 0.0));
  double overlap_dis = std::min(target_lane_adc_sl.end_l, lane_border.first) -
                       std::max(target_lane_adc_sl.start_l, lane_border.second);
  MSD_LOG(INFO,
          "LaneChangeDecider: overtake adc lateral_space %f overlap_dis %f",
          lateral_space, overlap_dis);
  // if (overlap_dis > 1.0) {
  //   context_->mutable_planning_status()->lane_status.change_lane.target_gap_obs
  //   = {-1, -2}; target_gap_ = std::make_pair(-1, -2); return
  //   TaskStatus::STATUS_SUCCESS;
  // }
  if (dis_to_change_point_ < 1.e-1) {
    if (gap_selection_scenario_ == "LANE_MERGE") {
      return TaskStatus::STATUS_SUCCESS;
    }
    MSD_LOG(INFO, "LaneChangeDecider: dis to change point is too small!");
    return TaskStatus::STATUS_FAILED;
  }
  double dis_to_stopline = world_model_->get_map_info().distance_to_stop_line();
  const double kMinDetectDis = 50.0;
  auto adc_sl_boundary = baseline_info_->get_adc_sl_boundary();
  bool curr_direct_exist =
      ((world_model_->get_map_info().current_lane_marks() &
        world_model_->get_map_info().traffic_light_direction()) ||
       world_model_->get_map_info().traffic_light_direction() ==
           Direction::UNKNOWN ||
       (world_model_->get_map_info().current_lane_marks() ==
            Direction::UNKNOWN &&
        world_model_->get_map_info().current_lane_type() == LaneType::NORMAL));
  bool must_execute_map_lc =
      !context_->planning_status()
           .lane_status.change_lane.is_active_lane_change &&
      !curr_direct_exist;
  auto target_lane_frenet_coord = curr_baseline->get_frenet_coord();
  if (target_lane_frenet_coord == nullptr) {
    if (gap_selection_scenario_ == "LANE_MERGE") {
      return TaskStatus::STATUS_SUCCESS;
    }
    MSD_LOG(INFO, "LaneChangeDecider: target_lane_frenet_coord is none!");
    return TaskStatus::STATUS_FAILED;
  }

  world_model_->trans_lane_points(curr_baseline->get_raw_refline_points(),
                                  cur_lane_, target_lane_frenet_coord, false);
  world_model_->trans_lane_points(target_baseline->get_raw_refline_points(),
                                  target_lane_, target_lane_frenet_coord,
                                  false);
  double ego_s = curr_baseline->get_adc_sl_boundary().end_s;
  double v_ego = curr_baseline->get_ego_state().ego_vel;

  if (lead_car_.id > 0) {
    auto lead_obs =
        curr_baseline->mutable_obstacle_manager().find_obstacle(lead_car_.id);
    if (lead_obs == nullptr) {
      lead_car_.id = -1;
    } else {
      SLBoundary sl_boundary = lead_obs->PerceptionSLBoundary();
      lead_car_.start_s = sl_boundary.start_s;
      lead_car_.end_s = sl_boundary.end_s;
      if (sl_boundary.start_s - ego_s > 0) {
        lead_car_.d_rel = sl_boundary.start_s - ego_s;
      } else if (sl_boundary.end_s - ego_s < 0) {
        lead_car_.d_rel = sl_boundary.end_s - ego_s;
      } else {
        lead_car_.d_rel = 0.0;
      }
      lead_car_.v_rel =
          lead_obs->speed() * std::cos(lead_obs->Yaw_relative_frenet()) - v_ego;
    }
  } else if (lead_car_.id == -3) {
    lead_car_.v_rel = -v_ego;
    lead_car_.d_rel =
        planning_status.lane_status.change_lane.virtual_obstacle_distance;
    lead_car_.start_s =
        curr_baseline->get_adc_sl_boundary().end_s + lead_car_.d_rel;
    lead_car_.end_s = lead_car_.start_s + 0.1;
  }

  min_gap_s_ = 0.0;

  obstacle_on_target_.clear();
  most_front_car_.start_s += ego_s;
  most_front_car_.end_s += ego_s;
  most_rear_car_.start_s += ego_s;
  most_rear_car_.end_s += ego_s;
  real_obstacle_on_target_.clear();
  const auto &prev_follow_obstacles = PlanningContext::Instance()
                                          ->planning_status()
                                          .planning_result.lon_follow_obstacles;
  nearest_obstacle_in_solid_line_ = nullptr;
  most_front_obstacle_in_target_line_ = nullptr;
  exist_fast_approach_car_from_behind_ = false;
  closest_car_id_ = -200;
  closest_fast_approach_car_id_ = -200;
  cloest_rear_ttc_ = std::numeric_limits<double>::max();
  cloest_front_ttc_ = std::numeric_limits<double>::max();
  cloest_rear_nttc_ = std::numeric_limits<double>::max();
  cloest_rear_nttc_ = std::numeric_limits<double>::max();
  auto rqt_temp = updata_real_obstacle_on_target(INTERNAL, 0, 0);
  if (closest_fast_approach_car_id_ != closest_car_id_) {
    exist_fast_approach_car_from_behind_ = false;
  }
  target_lane_traffic_flow_info_.target_lane_cars = real_obstacle_on_target_;
  if (most_front_obstacle_in_target_line_) {
    target_lane_traffic_flow_info_.most_front_car_vel =
        most_front_obstacle_in_target_line_->speed();
    MSD_LOG(INFO, "LaneChangeDecider: most_front_car %d most_front_car_vel %f",
            most_front_obstacle_in_target_line_->Id(),
            most_front_obstacle_in_target_line_->speed());
  }
  if (nearest_obstacle_in_solid_line_) {
    most_front_car_.id = nearest_obstacle_in_solid_line_->Id();
    most_front_car_.d_rel =
        nearest_obstacle_in_solid_line_->PerceptionSLBoundary().start_s - ego_s;
    most_front_car_.start_s =
        nearest_obstacle_in_solid_line_->PerceptionSLBoundary().start_s;
    most_front_car_.end_s =
        nearest_obstacle_in_solid_line_->PerceptionSLBoundary().end_s;
    most_front_car_.v_rel =
        nearest_obstacle_in_solid_line_->speed() *
            std::cos(nearest_obstacle_in_solid_line_->Yaw_relative_frenet()) -
        v_ego;
    MSD_LOG(INFO, "LaneChangeDecider: nearest_obstacle_in_solid_line [%d]",
            most_front_car_.id);
  }
  obstacle_on_target_.push_back(most_front_car_);
  obstacle_on_target_.push_back(most_rear_car_);
  MSD_LOG(INFO, "LaneChangeDecider: min_gap_s %f", min_gap_s_);

  bool enable_nearest_gap_policy = false;
  GapInfo nearest_front_gap, nearest_rear_gap;
  nearest_front_gap.valid = false;
  nearest_rear_gap.valid = false;
  double nearest_front_gap_dis{std::numeric_limits<double>::max()},
      nearest_rear_gap_dis{std::numeric_limits<double>::lowest()};

  std::sort(obstacle_on_target_.begin(), obstacle_on_target_.end(),
            compare_distance_drel);
  for (int i = 0; i < obstacle_on_target_.size() - 2; i++) {
    if (obstacle_on_target_.at(i).id > 0 ||
        obstacle_on_target_.at(i).id == -2) {
      for (int j = i + 1; j < (int)obstacle_on_target_.size() - 1; j++) {
        if (obstacle_on_target_.at(j).intention == "keep" ||
            obstacle_on_target_.at(j).intention == "none") {
          obstacle_on_target_.at(i).temp_lead = obstacle_on_target_.at(j).id;
          obstacle_on_target_.at(i).temp_lead_v_rel =
              obstacle_on_target_.at(j).v_rel;
          break;
        }
      }
    }
  }

  // get invisible section for target lane
  if (obstacle_on_target_.size() > 2) {
    constexpr double kValidFusionDis = 70.0;
    std::vector<std::pair<double, double>> blocked_s_sections;
    std::vector<std::pair<double, double>> blocked_vel_sections;
    std::vector<std::pair<double, double>> invisible_s_sections;
    std::vector<int> ignore_obs_id;
    auto target_lane_ego_sl = target_baseline->get_ego_state().ego_frenet;
    auto most_front_real_car =
        target_baseline->obstacle_manager().find_obstacle(
            obstacle_on_target_.at(obstacle_on_target_.size() - 2).id);
    double front_gap_s_end =
        std::min(target_lane_ego_sl.x + kValidFusionDis,
                 target_baseline->get_frenet_coord()->GetLength());
    front_gap_s_end = std::min(
        front_gap_s_end,
        dis_to_change_point_ + target_baseline->get_adc_sl_boundary().end_s);
    // MSD_LOG(INFO, "LaneChangeDecider xyzxyz most front obstacle[%d] find dis
    // %f ref %f ",
    //   most_front_real_car->Id(),
    //   most_front_real_car->PerceptionSLBoundary().end_s, front_gap_s_end);
    if (most_front_real_car && !most_front_real_car->IsFrenetInvalid() &&
        most_front_real_car->PerceptionSLBoundary().end_s < front_gap_s_end &&
        most_front_real_car->PerceptionSLBoundary().end_s > 0.0) {
      double left_width{3.8 / 2.0}, right_width{-3.8 / 2.0};
      double shift_side_l =
          target_lane_ego_sl.y > 0.0 ? left_width : right_width;
      double shift_l =
          target_lane_ego_sl.y > 0.0
              ? shift_side_l -
                    ConfigurationContext::Instance()->get_vehicle_param().width
              : shift_side_l +
                    ConfigurationContext::Instance()->get_vehicle_param().width;
      double s_begin = most_front_real_car->PerceptionSLBoundary().end_s;
      double s_end =
          std::min(most_front_real_car->PerceptionSLBoundary().end_s + 15.0,
                   target_baseline->get_frenet_coord()->GetLength());
      planning_math::IntervalMethodSolution<double> interval_methods;
      invisible_s_sections = interval_methods.merge(invisible_s_sections);
      constexpr double kFilterLength = 0.2;
      interval_methods.filter(invisible_s_sections, kFilterLength,
                              std::numeric_limits<double>::infinity());
      std::pair<double, double> most_front_min_gap_range{
          most_front_real_car->PerceptionSLBoundary().end_s, front_gap_s_end};
      auto complement_sections = interval_methods.complement(
          invisible_s_sections, most_front_min_gap_range.first,
          most_front_min_gap_range.second);
      constexpr double kMinGapLength = 8.0;
      // for (auto sec : complement_sections) {
      //   MSD_LOG(INFO, "LaneChangeDecider xyzxyz sec %f %f", sec.first,
      //   sec.second);
      // }
      interval_methods.filter(complement_sections, kMinGapLength,
                              std::numeric_limits<double>::infinity());
      if (!complement_sections.empty()) {
        is_most_front_gap_invisible_ = false;
        MSD_LOG(INFO, "LaneChangeDecider most front obstacle[%d] visible",
                most_front_real_car->Id());
      } else {
        MSD_LOG(INFO, "LaneChangeDecider most front obstacle[%d] invisible",
                most_front_real_car->Id());
        is_most_front_gap_exist_and_invisible_ = true;
      }
    }
  }
  MSD_LOG(INFO, "LaneChangeDecider: front gap visible info %d %d",
          is_most_front_gap_exist_and_invisible_, is_most_front_gap_invisible_);

  average_gap_length_ = 0.0;
  double min_gap_length{std::numeric_limits<double>::max()};
  double max_gap_length{0.0};
  double max_gap_v = 0.0;
  int gap_num = 0;
  bool ignore_obs_for_overlap{false};
  double ego_s_start = world_model_->get_baseline_info(current_lane_id_)
                           ->get_adc_sl_boundary()
                           .start_s;
  double a_ego = world_model_->get_baseline_info(current_lane_id_)
                     ->get_ego_state()
                     .ego_acc;
  for (int i = 0; i < (int)obstacle_on_target_.size() - 1; i++) {
    if (obstacle_on_target_.at(i).d_rel >= 0.0 &&
        obstacle_on_target_.at(i).d_rel < cloest_front_dis_) {
      cloest_front_dis_ = obstacle_on_target_.at(i).d_rel;
      double temp_pstart_s =
          obstacle_on_target_.at(i).start_s + obstacle_on_target_.at(i).v_rel;
      double temp_pend_s =
          obstacle_on_target_.at(i).end_s + obstacle_on_target_.at(i).v_rel;
      if (temp_pstart_s - ego_s > 0) {
        cloest_front_dis_p1_ = temp_pstart_s - ego_s;
      } else if (temp_pend_s - ego_s_start < 0) {
        cloest_front_dis_p1_ = temp_pend_s - ego_s_start;
      } else {
        cloest_front_dis_p1_ = 0.0;
      }
      closest_car_front_id = obstacle_on_target_.at(i).id;
    }
    if (obstacle_on_target_.at(i + 1).d_rel >= 0.0 &&
        obstacle_on_target_.at(i + 1).d_rel < cloest_front_dis_) {
      cloest_front_dis_ = obstacle_on_target_.at(i + 1).d_rel;
      double temp_pstart_s = obstacle_on_target_.at(i + 1).start_s +
                             obstacle_on_target_.at(i + 1).v_rel;
      double temp_pend_s = obstacle_on_target_.at(i + 1).end_s +
                           obstacle_on_target_.at(i + 1).v_rel;
      if (temp_pstart_s - ego_s > 0) {
        cloest_front_dis_p1_ = temp_pstart_s - ego_s;
      } else if (temp_pend_s - ego_s_start < 0) {
        cloest_front_dis_p1_ = temp_pend_s - ego_s_start;
      } else {
        cloest_front_dis_p1_ = 0.0;
      }
      closest_car_front_id = obstacle_on_target_.at(i + 1).id;
    }
    if (obstacle_on_target_.at(i).d_rel > 70.0) {
      break;
    }
    auto rear_obs_id = obstacle_on_target_.at(i).id;
    if (std::find(prev_follow_obstacles.begin(), prev_follow_obstacles.end(),
                  rear_obs_id) != prev_follow_obstacles.end() &&
        obstacle_on_target_.at(i).intention == "keep") {
      MSD_LOG(INFO, "LaneChangeDecider: obstacle[%d] follow ", rear_obs_id);
      break;
    }
    auto obstacle = curr_baseline->mutable_obstacle_manager().find_obstacle(
        obstacle_on_target_.at(i + 1).id);
    if (obstacle && !obstacle->IsFrenetInvalid() &&
        obstacle->PerceptionSLBoundary().start_s < min_gap_s_) {
      ignore_obs_for_overlap = true;
      continue;
    }

    GapInfo gap_info = check_gap_valid(obstacle_on_target_.at(i),
                                       obstacle_on_target_.at(i + 1));
    auto obstacle_rear =
        curr_baseline->mutable_obstacle_manager().find_obstacle(
            obstacle_on_target_.at(i).id);
    if (obstacle_on_target_.at(i).id == closest_car_id_) {
      double d_rel_tmp =
          obstacle_rear->PerceptionSLBoundary().end_s - ego_s_start;
      double tmp_ttc =
          calc_equation_ttc((a_ego - obstacle_rear->acceleration()) * 0.5,
                            v_ego - obstacle_rear->speed(), -d_rel_tmp);
      if (tmp_ttc >= 0 && tmp_ttc < cloest_rear_ttc_)
        cloest_rear_ttc_ = tmp_ttc;
      if (abs(v_ego - obstacle_rear->speed()) > 0.01) {
        tmp_ttc = d_rel_tmp / (v_ego - obstacle_rear->speed());
        if (tmp_ttc >= 0 && tmp_ttc < cloest_front_nttc_)
          cloest_front_nttc_ = tmp_ttc;
      }
    }
    if (obstacle_on_target_.at(i).id > 0 &&
        obstacle_on_target_.at(i + 1).id > 0) {
      gap_num++;
      average_gap_length_ += obstacle_on_target_.at(i + 1).start_s -
                             obstacle_on_target_.at(i).end_s;
      min_gap_length =
          std::min(min_gap_length, obstacle_on_target_.at(i + 1).start_s -
                                       obstacle_on_target_.at(i).end_s);
      max_gap_length =
          std::max(max_gap_length, obstacle_on_target_.at(i + 1).start_s -
                                       obstacle_on_target_.at(i).end_s);
    }

    if (gap_info.valid) {
      if (gap_info.front_id ==
              PlanningContext::Instance()
                  ->planning_status()
                  .lane_status.change_lane.target_gap_obs.first ||
          gap_info.rear_id ==
              PlanningContext::Instance()
                  ->planning_status()
                  .lane_status.change_lane.target_gap_obs.second) {
        gap_info.cost = std::min(0.95 * gap_info.cost, gap_info.cost - 0.2);
        if (PlanningContext::Instance()
                ->state_machine_output()
                .lc_invalid_reason == "valid cnt below threshold") {
          gap_info.cost *= 0.2;
        }
        max_gap_v =
            std::max(obstacle ? obstacle->speed() : map_v_limit_, max_gap_v);
        max_gap_length =
            std::max(max_gap_length, obstacle_on_target_.at(i + 1).start_s -
                                         obstacle_on_target_.at(i).end_s);
      }
      gap_list_.push_back(gap_info);
      // MSD_LOG(INFO, "zzzz gap [%d %d] cost %f", gap_info.front_id,
      // gap_info.rear_id, gap_info.cost);
    }
  }
  if (gap_num > 0) {
    average_gap_length_ /= gap_num;
  } else {
    average_gap_length_ = 30.0;
  }
  target_lane_traffic_flow_info_.min_gap_length = min_gap_length;
  target_lane_traffic_flow_info_.max_gap_length = max_gap_length;
  MSD_LOG(INFO, "LaneChangeDecider: max_gap_v %f max_gap_length %f", max_gap_v,
          max_gap_length);
  if (lc_wait_time_ > 20.0 && max_gap_v / map_v_limit_ < 0.5 &&
      max_gap_length < 15.0) {
    enable_nearest_gap_policy = true;
  }
  if (gap_list_.size() > 0) {
    std::sort(gap_list_.begin(), gap_list_.end(), compare_cost_asc);
    bool find_adjacent_gap{false};
    if (enable_nearest_gap_policy) {
      MSD_LOG(INFO, "LaneChangeDecider: enable_nearest_gap_policy!");
      for (auto gap_info : gap_list_) {
        auto rear_car = curr_baseline->mutable_obstacle_manager().find_obstacle(
            gap_info.rear_id);
        auto front_car =
            curr_baseline->mutable_obstacle_manager().find_obstacle(
                gap_info.front_id);
        bool is_gap_behind_adc{false}, is_gap_front_adc{false};
        if (front_car) {
          if (adc_sl_boundary.end_s >
              front_car->PerceptionSLBoundary().start_s) {
            if (front_car->PerceptionSLBoundary().start_s -
                    adc_sl_boundary.end_s >
                nearest_rear_gap_dis) {
              nearest_rear_gap_dis = front_car->PerceptionSLBoundary().end_s -
                                     adc_sl_boundary.start_s;
              is_gap_behind_adc = true;
              nearest_rear_gap = gap_info;
            }
          }
        }
        if (rear_car) {
          if (adc_sl_boundary.start_s <
              rear_car->PerceptionSLBoundary().end_s) {
            if (rear_car->PerceptionSLBoundary().end_s -
                    adc_sl_boundary.start_s <
                nearest_front_gap_dis) {
              nearest_front_gap_dis = rear_car->PerceptionSLBoundary().start_s -
                                      adc_sl_boundary.end_s;
              is_gap_front_adc = true;
              nearest_front_gap = gap_info;
            }
          }
        }
        if (!is_gap_behind_adc && !is_gap_front_adc) {
          target_gap_ = std::make_pair(gap_info.front_id, gap_info.rear_id);
          target_gap_cost_ = gap_info.cost;
          MSD_LOG(INFO, "LaneChangeDecider: find nearest gap [%d %d]",
                  gap_info.front_id, gap_info.rear_id);
          find_adjacent_gap = true;
          break;
        }
      }
      if (!find_adjacent_gap) {
        if (nearest_rear_gap.valid) {
          if (nearest_front_gap.valid &&
              nearest_front_gap.cost < nearest_rear_gap.cost) {
            target_gap_ = std::make_pair(nearest_front_gap.front_id,
                                         nearest_front_gap.rear_id);
            target_gap_cost_ = nearest_front_gap.cost;
            MSD_LOG(INFO, "LaneChangeDecider: choose nearest front gap [%d %d]",
                    target_gap_.first, target_gap_.second);
          } else {
            target_gap_ = std::make_pair(nearest_rear_gap.front_id,
                                         nearest_rear_gap.rear_id);
            target_gap_cost_ = nearest_rear_gap.cost;
            MSD_LOG(INFO, "LaneChangeDecider: choose nearest rear gap [%d %d]",
                    target_gap_.first, target_gap_.second);
          }
        } else if (nearest_front_gap.valid) {
          target_gap_ = std::make_pair(nearest_front_gap.front_id,
                                       nearest_front_gap.rear_id);
          target_gap_cost_ = nearest_front_gap.cost;
          MSD_LOG(INFO, "LaneChangeDecider: choose nearest front gap [%d %d]",
                  target_gap_.first, target_gap_.second);
        }
      }
    } else {
      target_gap_ =
          std::make_pair(gap_list_.at(0).front_id, gap_list_.at(0).rear_id);
      target_gap_cost_ = gap_list_.at(0).cost;
    }
  }
  if (target_gap_.first == -10 && target_gap_.second == -10) {
    const auto &cur_planning_status =
        PlanningContext::Instance()->planning_status();
    bool is_in_lc_back = cur_planning_status.lane_status.status ==
                             LaneStatus::Status::LANE_CHANGE &&
                         cur_planning_status.lane_status.change_lane.status ==
                             ChangeLaneStatus::Status::CHANGE_LANE_BACK;
    if (overlap_dis > 0.5 && ignore_obs_for_overlap) {
      if (!exist_fast_approach_car_from_behind_) {
        target_gap_ = std::make_pair(-1, -2);
        target_gap_cost_ = 0.0;
        return TaskStatus::STATUS_SUCCESS;
      } else {
        target_gap_ = std::make_pair(-10, -10);
        target_gap_cost_ = 0.0;
        return TaskStatus::STATUS_SUCCESS;
      }
    }
  }
  // protect gap behind invisible static front car
  if (gap_selection_scenario_ == "LANE_CHANGE" &&
      is_most_front_gap_exist_and_invisible_ &&
      dis_to_change_point_ > map_v_limit_ * kMinReservedLCTime) {
    auto target_car = curr_baseline->mutable_obstacle_manager().find_obstacle(
        target_gap_.first);
    if (target_car && !target_car->IsFrenetInvalid() &&
        target_car->PerceptionSLBoundary().start_s > adc_sl_boundary.end_s) {
      double dis_to_front_car =
          target_car->PerceptionSLBoundary().start_s - adc_sl_boundary.end_s;
      constexpr double kSoftDec = -1;
      constexpr double kDecCoefficient = -0.8;
      double obs_frenet_v =
          target_car->speed() * std::cos(target_car->Yaw_relative_frenet());
      double accept_dec =
          kSoftDec + kDecCoefficient *
                         (1.0 - clip(obs_frenet_v / map_v_limit_, 0.0, 0.999));
      double soft_dec_dis =
          (obs_frenet_v - curr_baseline->get_ego_state().ego_vel) *
          std::abs(obs_frenet_v - curr_baseline->get_ego_state().ego_vel) /
          (2 * kSoftDec);
      double min_reference_lc_dis = 7.0;
      // todo: use obs_frenet_v or speed considering front car cut out scenario
      MSD_LOG(INFO,
              "LaneChangeDecider: gap [%d %d] frozen debug dis_to_front_car %f "
              "min_reference_lc_dis %f soft_dec_dis %f accept_dec %f "
              "obs_frenet_v %f",
              target_gap_.first, target_gap_.second, dis_to_front_car,
              min_reference_lc_dis, soft_dec_dis, accept_dec, obs_frenet_v);
      if (dis_to_front_car > std::max(min_reference_lc_dis, soft_dec_dis) &&
          obs_frenet_v < map_v_limit_ * 0.25) {
        MSD_LOG(INFO, "LaneChangeDecider: gap [%d %d] frozen",
                target_gap_.first, target_gap_.second);
        is_target_gap_frozen_ = true;
      }
    }
  }

  context_->mutable_planning_status()->lane_status.change_lane.target_gap_obs =
      target_gap_;
  return TaskStatus::STATUS_SUCCESS;
}
bool LaneChangeDecider::is_pred_on_target(const Obstacle *obstacle,
                                          const double v_ego) {
  if (obstacle->IsPredictFrenetInvalid())
    return false;
  auto sl_boundary = obstacle->PredictionSLBoundary();
  sl_boundary.start_s -= obstacle->PredSLBoundaryRelativeTime() * v_ego;
  sl_boundary.end_s -= obstacle->PredSLBoundaryRelativeTime() * v_ego;
  double ego_s = world_model_->get_baseline_info(current_lane_id_)
                     ->get_adc_sl_boundary()
                     .end_s;
  // if (sl_boundary.start_s - ego_s > dis_to_change_point_) {
  //   return false;
  // }
  double s = (sl_boundary.end_s + sl_boundary.start_s) / 2.0;
  PathPoint obs_point;
  obs_point.x = obstacle->PredictionBoundingBox().center().x();
  obs_point.y = obstacle->PredictionBoundingBox().center().y();
  auto cur_baselane = world_model_->get_baseline_info(current_lane_id_);
  double obs_path_s =
      cur_baselane->get_path_data().discretized_path().QueryMatchedS(obs_point);
  auto cur_lane_width = cur_baselane->get_path_data().getWidth(obs_path_s);
  auto target_baselane = world_model_->get_baseline_info(target_lane_id_);
  double target_obs_path_s =
      target_baselane->get_path_data().discretized_path().QueryMatchedS(
          obs_point);
  auto target_lane_width =
      target_baselane->get_path_data().getWidth(target_obs_path_s);
  if (world_model_->get_map_info().is_in_intersection()) {
    constexpr double kMaxHalfLaneWidth = 1.3;
    target_lane_width.first =
        clip(target_lane_width.first, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
    target_lane_width.second =
        clip(target_lane_width.second, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
    cur_lane_width.first =
        clip(cur_lane_width.first, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
    cur_lane_width.second =
        clip(cur_lane_width.second, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
  }
  // max_l_threshold_ = calc_lane_width(s, cur_lane_) / 2.0 + calc_lane_width(s,
  // target_lane_) / 2.0 + 1.3; min_l_threshold_ = calc_lane_width(s, cur_lane_)
  // / 2.0 + calc_lane_width(s, target_lane_) / 2.0 - 1.3;
  constexpr double kMinLateralGap = 2.5;
  constexpr double kProximalLateralGap = 0.3;
  double lateral_avoid_buffer = kMinLateralGap + v_limit_ * 0.01;
  if (lane_change_direction_ > 0) {
    // left
    max_l_threshold_ = cur_lane_width.first + lateral_avoid_buffer;
    if (sl_boundary.start_s > ego_s) {
      min_l_threshold_ = cur_lane_width.first +
                         (target_lane_width.first - target_lane_width.second) -
                         lateral_avoid_buffer;
    } else {
      min_l_threshold_ = cur_lane_width.first + kProximalLateralGap;
    }
  } else {
    // right
    max_l_threshold_ = -cur_lane_width.second + lateral_avoid_buffer;
    if (sl_boundary.start_s > ego_s) {
      min_l_threshold_ = -cur_lane_width.second +
                         (target_lane_width.first - target_lane_width.second) -
                         lateral_avoid_buffer;
    } else {
      min_l_threshold_ = -cur_lane_width.second + kProximalLateralGap;
    }
  }
  if (obstacle->Type() == ObjectType::CONE_BUCKET) {
    if (lane_change_direction_ > 0) {
      min_l_threshold_ = cur_lane_width.first - kProximalLateralGap;
    } else {
      min_l_threshold_ = -cur_lane_width.second - kProximalLateralGap;
    }
  }
  // MSD_LOG(INFO, "zzzzz TBDEBUG: %f %f %d %f %f cur_lane_width[%f %f]
  // target_lane_width [%f %f] %f %f", s, max_l_threshold_,
  // lane_change_direction_, sl_boundary.start_l, sl_boundary.end_l,
  //     cur_lane_width.first, cur_lane_width.second, target_lane_width.first,
  //     target_lane_width.second, calc_lane_width(s, cur_lane_),
  //     calc_lane_width(s, target_lane_));
  if (lane_change_direction_ < 0) {
    // right
    sl_boundary.end_l -= target_lane_width.first - cur_lane_width.second;
    sl_boundary.start_l -= target_lane_width.first - cur_lane_width.second;
  } else {
    // left
    sl_boundary.end_l += -target_lane_width.second + cur_lane_width.first;
    sl_boundary.start_l += -target_lane_width.second + cur_lane_width.first;
  }

  if (lane_change_direction_ > 0 &&
      sl_boundary.end_l > lane_change_direction_ * min_l_threshold_ &&
      sl_boundary.start_l < lane_change_direction_ * max_l_threshold_) {
    return true;
  } else if (lane_change_direction_ < 0 &&
             sl_boundary.start_l < lane_change_direction_ * min_l_threshold_ &&
             sl_boundary.end_l > lane_change_direction_ * max_l_threshold_) {
    return true;
  }
  return false;
}
bool LaneChangeDecider::is_on_target(const Obstacle *obstacle) {
  auto sl_boundary = obstacle->PerceptionSLBoundary();
  double ego_s = world_model_->get_baseline_info(current_lane_id_)
                     ->get_adc_sl_boundary()
                     .end_s;
  double ego_s_start = world_model_->get_baseline_info(current_lane_id_)
                           ->get_adc_sl_boundary()
                           .start_s;
  double ego_v = world_model_->get_baseline_info(current_lane_id_)
                     ->get_ego_state()
                     .ego_vel;
  // if (sl_boundary.start_s - ego_s > dis_to_change_point_) {
  //   return false;
  // }
  double s = (sl_boundary.end_s + sl_boundary.start_s) / 2.0;
  PathPoint obs_point;
  obs_point.x = obstacle->PerceptionBoundingBox().center().x();
  obs_point.y = obstacle->PerceptionBoundingBox().center().y();
  auto cur_baselane = world_model_->get_baseline_info(current_lane_id_);
  double obs_path_s =
      cur_baselane->get_path_data().discretized_path().QueryMatchedS(obs_point);
  auto cur_lane_width = cur_baselane->get_path_data().getWidth(obs_path_s);
  auto target_baselane = world_model_->get_baseline_info(target_lane_id_);
  double target_obs_path_s =
      target_baselane->get_path_data().discretized_path().QueryMatchedS(
          obs_point);
  auto target_lane_width =
      target_baselane->get_path_data().getWidth(target_obs_path_s);
  if (world_model_->get_map_info().is_in_intersection()) {
    constexpr double kMaxHalfLaneWidth = 1.3;
    target_lane_width.first =
        clip(target_lane_width.first, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
    target_lane_width.second =
        clip(target_lane_width.second, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
    cur_lane_width.first =
        clip(cur_lane_width.first, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
    cur_lane_width.second =
        clip(cur_lane_width.second, -kMaxHalfLaneWidth, kMaxHalfLaneWidth);
  }
  // max_l_threshold_ = calc_lane_width(s, cur_lane_) / 2.0 + calc_lane_width(s,
  // target_lane_) / 2.0 + 1.3; min_l_threshold_ = calc_lane_width(s, cur_lane_)
  // / 2.0 + calc_lane_width(s, target_lane_) / 2.0 - 1.3;
  constexpr double kMinLateralGap = 2.5;
  constexpr double kProximalLateralGap = 0.3;
  constexpr double kSafeDistance = 4.0;
  double safe_distance =
      kSafeDistance * std::abs(obstacle->speed() - ego_v) * 2.0;
  double lateral_avoid_buffer = kMinLateralGap + v_limit_ * 0.01;
  if (lane_change_direction_ > 0) {
    max_l_threshold_ = cur_lane_width.first + lateral_avoid_buffer;
    if (obstacle->PerceptionSLBoundary().start_s > ego_s) {
      min_l_threshold_ = cur_lane_width.first +
                         (target_lane_width.first - target_lane_width.second) -
                         lateral_avoid_buffer;
    } else {
      min_l_threshold_ = cur_lane_width.first + kProximalLateralGap;
    }
  } else {
    max_l_threshold_ = -cur_lane_width.second + lateral_avoid_buffer;
    if (obstacle->PerceptionSLBoundary().start_s > ego_s) {
      min_l_threshold_ = -cur_lane_width.second +
                         (target_lane_width.first - target_lane_width.second) -
                         lateral_avoid_buffer;
    } else {
      min_l_threshold_ = -cur_lane_width.second + kProximalLateralGap;
    }
  }
  if (sl_boundary.start_s < safe_distance + ego_s ||
      sl_boundary.end_s > ego_s_start - safe_distance) {
    if (lane_change_direction_ > 0)
      max_l_threshold_ = cur_lane_width.first +
                         (target_lane_width.first - target_lane_width.second -
                          kProximalLateralGap);
    else
      max_l_threshold_ = -cur_lane_width.second +
                         (target_lane_width.first - target_lane_width.second -
                          kProximalLateralGap);
  }
  if (obstacle->Type() == ObjectType::CONE_BUCKET) {
    if (lane_change_direction_ > 0) {
      min_l_threshold_ = cur_lane_width.first - kProximalLateralGap;
    } else {
      min_l_threshold_ = -cur_lane_width.second - kProximalLateralGap;
    }
  }

  if (lane_change_direction_ > 0 &&
      sl_boundary.end_l > lane_change_direction_ * min_l_threshold_ &&
      sl_boundary.start_l < lane_change_direction_ * max_l_threshold_) {
    return true;
  } else if (lane_change_direction_ < 0 &&
             sl_boundary.start_l < lane_change_direction_ * min_l_threshold_ &&
             sl_boundary.end_l > lane_change_direction_ * max_l_threshold_) {
    return true;
  }
  return false;
}

std::string LaneChangeDecider::identify_intention(const Obstacle *obstacle) {
  // current lane frenet system
  if (!obstacle->HasTrajectory()) {
    return "keep";
  }

  auto cur_baseline = world_model_->get_baseline_info(current_lane_id_);
  auto target_baseline = world_model_->get_baseline_info(target_lane_id_);
  auto begin_sl_boundary = obstacle->PerceptionSLBoundary();
  double end_time = 4.0;
  auto end_cart_point = obstacle->GetPointAtTime(end_time);
  auto end_sl_boundary =
      obstacle->GetSLBoundary(cur_baseline->get_frenet_coord(), end_cart_point);

  auto &cur_lane_path_data = cur_baseline->get_path_data();
  auto &target_lane_path_data = target_baseline->get_path_data();
  PathPoint obs_point;
  obs_point.x = obstacle->PerceptionBoundingBox().center().x();
  obs_point.y = obstacle->PerceptionBoundingBox().center().y();
  double begin_obs_path_s =
      target_lane_path_data.discretized_path().QueryMatchedS(obs_point);
  double cur_begin_obs_path_s =
      cur_lane_path_data.discretized_path().QueryMatchedS(obs_point);
  obs_point.x = end_cart_point.path_point.x;
  obs_point.y = end_cart_point.path_point.y;
  double end_obs_path_s =
      target_lane_path_data.discretized_path().QueryMatchedS(obs_point);
  double cur_end_obs_path_s =
      cur_lane_path_data.discretized_path().QueryMatchedS(obs_point);
  auto begin_lane_border = target_lane_path_data.getWidth(begin_obs_path_s);
  auto end_lane_border = target_lane_path_data.getWidth(end_obs_path_s);
  auto cur_begin_lane_border = cur_lane_path_data.getWidth(begin_obs_path_s);
  auto cur_end_lane_border = cur_lane_path_data.getWidth(end_obs_path_s);

  if (lane_change_direction_ > 0) {
    begin_lane_border.first +=
        cur_begin_lane_border.first - begin_lane_border.second;
    begin_lane_border.second = cur_begin_lane_border.first;
    end_lane_border.first += cur_end_lane_border.first - end_lane_border.second;
    end_lane_border.second = cur_end_lane_border.first;
  } else {
    begin_lane_border.second -=
        begin_lane_border.first - cur_begin_lane_border.second;
    begin_lane_border.first = cur_begin_lane_border.second;
    end_lane_border.second -=
        end_lane_border.first - cur_end_lane_border.second;
    end_lane_border.first = cur_end_lane_border.second;
  }
  double begin_overlap_dis =
      std::min(begin_sl_boundary.end_l, begin_lane_border.first) -
      std::max(begin_sl_boundary.start_l, begin_lane_border.second);
  double end_overlap_dis =
      std::min(end_sl_boundary.end_l, end_lane_border.first) -
      std::max(end_sl_boundary.start_l, end_lane_border.second);
  double cur_begin_overlap_dis =
      std::min(begin_sl_boundary.end_l, cur_begin_lane_border.first) -
      std::max(begin_sl_boundary.start_l, cur_begin_lane_border.second);
  double cur_end_overlap_dis =
      std::min(end_sl_boundary.end_l, cur_end_lane_border.first) -
      std::max(end_sl_boundary.start_l, cur_end_lane_border.second);
  // MSD_LOG(INFO, "LaneChangeDecider obstacle[%d] select begin_overlap_dis %f
  // ratio  end_overlap_dis %f ratio ", obstacle->Id(), begin_overlap_dis,
  // end_overlap_dis); MSD_LOG(INFO, "LaneChangeDecider obstacle[%d] select cur
  // begin_overlap_dis %f ratio  end_overlap_dis %f ratio ", obstacle->Id(),
  // cur_begin_overlap_dis, cur_end_overlap_dis);
  constexpr double kDefaultLeavingDis = 0.7;
  constexpr double kDefaultCutInRatio = 0.4;
  constexpr double kDefaultLaneKeepRatio = 0.2;
  if (std::max(begin_overlap_dis, 0.0) - std::max(end_overlap_dis, 0.0) >
      kDefaultLeavingDis) {
    if (std::max(cur_end_overlap_dis, 0.0) -
                std::max(cur_begin_overlap_dis, 0.0) >
            kDefaultLeavingDis &&
        cur_end_overlap_dis /
                (1.e-3 + end_sl_boundary.end_l - end_sl_boundary.start_l) >
            kDefaultCutInRatio) {
      MSD_LOG(INFO,
              "LaneChangeDecider obstacle [%d] in target lane is "
              "heading_to_current_lane ",
              obstacle->Id());
      return "heading_to_current_lane";
    } else if (end_overlap_dis < 0.2 ||
               end_overlap_dis / (1.e-3 + end_sl_boundary.end_l -
                                  end_sl_boundary.start_l) <
                   kDefaultLaneKeepRatio) {
      MSD_LOG(INFO,
              "LaneChangeDecider obstacle [%d] in target lane is "
              "heading_to_further_lane ",
              obstacle->Id());
      return "heading_to_further_lane";
    }
  }

  return "keep";
}

bool LaneChangeDecider::is_gap_insertable(const GapInfo &gap) {
  auto overtake_obstacle =
      baseline_info_->mutable_obstacle_manager().find_obstacle(gap.rear_id);
  auto follow_obstacle =
      baseline_info_->mutable_obstacle_manager().find_obstacle(gap.front_id);
  auto adc_sl = baseline_info_->get_adc_sl_boundary();
  double follow_buffer{0.0}, overtake_buffer{0.0};
  double v_cruise = map_v_limit_;
  if (follow_obstacle) {
    constexpr double kDefaultfollowBuffer = 0.5;
    constexpr double kFollowTimeBuffer = 0.8;
    constexpr double kMaxFollowBuffer = 20.0;
    constexpr double ksoftLCDec = 1.0;
    double obs_frenet_v = follow_obstacle->speed() *
                          std::cos(follow_obstacle->Yaw_relative_frenet());
    if (obs_frenet_v < v_cruise - 10.0 / 3.6) {
      return false;
    }
    follow_buffer = std::min(
        kMaxFollowBuffer,
        kDefaultfollowBuffer +
            std::max(2 * baseline_info_->get_ego_state().ego_vel - obs_frenet_v,
                     0.0) *
                kFollowTimeBuffer);
    follow_buffer = std::max(
        follow_buffer,
        std::pow(std::min(0.0, obs_frenet_v -
                                   baseline_info_->get_ego_state().ego_vel),
                 2) /
            (2 * ksoftLCDec));
    if (adc_sl.end_s + follow_buffer >
        follow_obstacle->PerceptionSLBoundary().start_s) {
      return false;
    }
  }
  if (overtake_obstacle) {
    constexpr double kDefaultOvertakeBuffer = 0.5;
    constexpr double kOvertakeTimeBuffer = 0.8;
    constexpr double kMinOvertakeTimeBuffer = 0.4;
    constexpr double kMinLKTimeBuffer = 2.0;
    constexpr double kMaxOvertakeBuffer = 8.0;
    constexpr double kMinReservedTimeForLC = 8.0;
    double obs_frenet_v = overtake_obstacle->speed() *
                          std::cos(overtake_obstacle->Yaw_relative_frenet());
    // double time_buffer = kOvertakeTimeBuffer;
    //     double reserved_time = dis_to_change_point_ /
    //     PlanningContext::Instance()->planning_status().v_limit -
    //     kMinLKTimeBuffer; time_buffer = clip(reserved_time /
    //     kMinReservedTimeForLC, 1.0, 0.0) * (kOvertakeTimeBuffer -
    //     kMinOvertakeTimeBuffer) + kMinOvertakeTimeBuffer;
    double overtake_buffer = std::min(
        kMaxOvertakeBuffer,
        kDefaultOvertakeBuffer +
            std::max(2 * obs_frenet_v - baseline_info_->get_ego_state().ego_vel,
                     0.0) *
                kOvertakeTimeBuffer);
    if (adc_sl.start_s <
        overtake_obstacle->PerceptionSLBoundary().end_s + overtake_buffer) {
      return false;
    }
  }
  return true;
}

GapInfo LaneChangeDecider::check_gap_valid(const TargetObstacle &rear_car,
                                           const TargetObstacle &front_car) {
  double buffer = 1.0;
  double t_gap = 0.1;
  double car_length =
      ConfigurationContext::Instance()->get_vehicle_param().length;
  double ego_car_length = car_length + buffer;
  double v_ego = baseline_info_->get_ego_state().ego_vel;
  GapInfo gap_info;
  TargetObstacle base_car;
  double v_ego_p, v_ego_p_rel;
  double safety_distance;
  double ego_s = world_model_->get_baseline_info(current_lane_id_)
                     ->get_adc_sl_boundary()
                     .end_s;
  auto ego_sl =
      world_model_->get_baseline_info(current_lane_id_)->get_adc_sl_boundary();
  auto attempt_to_overtake_obs =
      baseline_info_->obstacle_manager().find_obstacle(overtake_obs_id_);

  gap_info.front_id = front_car.id;
  gap_info.rear_id = rear_car.id;
  gap_info.acc_valid = false;
  double base_d_rel{0.0};
  double current_gap_v_limit = v_limit_;
  bool is_in_merge_scene = gap_selection_scenario_ == "LANE_MERGE";

  if (front_car.start_s >
          ego_sl.end_s + std::max(0.0, -front_car.v_rel) * 0.2 &&
      std::max(0.0, rear_car.v_rel) * 0.2 + rear_car.end_s < ego_sl.start_s) {
    const auto &map_info = world_model_->get_map_info();
    // current_gap_v_limit = std::min(map_info.v_cruise(), front_car.v_rel +
    // v_ego);
    current_gap_v_limit = std::max(
        current_gap_v_limit, std::min(front_car.v_rel + v_ego, map_v_limit_));
  }

  // if (std::abs(front_car.d_rel) < std::abs(std::min(rear_car.d_rel +
  // ConfigurationContext::Instance()->get_vehicle_param().length,
  // front_car.d_rel - 0.1))) {
  if (front_car.start_s - ego_sl.end_s < ego_sl.start_s - rear_car.end_s) {
    base_car = front_car;
    base_d_rel = front_car.start_s - ego_sl.end_s;
    v_ego_p = std::min(v_ego + base_car.v_rel - 1.0, current_gap_v_limit);
    v_ego_p_rel = std::min(base_car.v_rel - 1.0, current_gap_v_limit - v_ego);
  } else {
    base_car = rear_car;
    base_d_rel = rear_car.end_s - ego_sl.start_s;
    if (front_car.v_rel > rear_car.v_rel + 3.0 && rear_car.v_rel < 0.0) {
      gap_info.acc_valid = true;
    }
    v_ego_p = std::min(v_ego + base_car.v_rel + 1.0, current_gap_v_limit);
    v_ego_p_rel = std::min(base_car.v_rel + 1.0, current_gap_v_limit - v_ego);
  }
  if ((front_car.type == (int)ObjectType::TRANSPORT_TRUNK ||
       front_car.type == (int)ObjectType::BUS ||
       front_car.type == (int)ObjectType::ENGINEER_TRUCK) &&
      (rear_car.type == (int)ObjectType::TRANSPORT_TRUNK ||
       rear_car.type == (int)ObjectType::BUS ||
       rear_car.type == (int)ObjectType::ENGINEER_TRUCK)) {
    t_gap = 0.2;
  }
  safety_distance = buffer + v_ego_p * t_gap;
  gap_info.base_car_id = base_car.id;

  double acc_time =
      calc_time_for_lane_change(base_car, front_car, gap_info, safety_distance,
                                current_gap_v_limit - v_ego - 0.5);
  v_ego_p = clip(v_ego_p, v_ego - acc_time * 1.0, v_ego + acc_time * 1.0);
  v_ego_p_rel = clip(v_ego_p_rel, -acc_time * 1.0, acc_time * 1.0);
  double mssf =
      std::pow(std::max(-(front_car.v_rel - v_ego_p_rel), 0.0), 2) / 2.0 +
      safety_distance;
  double mssr =
      (rear_car.v_rel - v_ego_p_rel) * 2.0 +
      std::pow(std::max((rear_car.v_rel - v_ego_p_rel), 0.0), 2) / 2.0 +
      safety_distance;
  // ROS_INFO("TBDEBUG: acc_time %f", acc_time);
  double v_target_p = 100.0;
  double d_p = 100.0;
  if (lead_car_.id > 0) {
    constexpr double kMinLCRange = 6.0;
    double gap_p = (acc_time + 1.0) *
                       (lead_car_.v_rel + v_ego - (v_ego + rear_car.v_rel)) +
                   lead_car_.start_s - rear_car.end_s - kMinLCRange;
    d_p = gap_p;
  }
  auto rear_obstacle =
      baseline_info_->obstacle_manager().find_obstacle(rear_car.id);
  bool is_rear_obstacle_static =
      rear_obstacle ? rear_obstacle->IsStatic() : true;
  // if (v_ego_p_rel < rear_car.v_rel && !is_rear_obstacle_static &&
  // rear_car.end_s - ego_sl.start_s > - std::max(mssr + ego_car_length,
  // safety_distance + ego_car_length)){
  //   d_p = -1;
  // }

  double mss = mssf + mssr + ego_car_length;
  double mss0 = 2 * safety_distance + ego_car_length;
  constexpr double kMaxGapPredictTime = 5.0;
  double gap_predict_time = std::min(acc_time, kMaxGapPredictTime);
  if ((front_car.v_rel + v_ego < 0.5 && rear_car.v_rel + v_ego < 0.5) ||
      std::abs(front_car.v_rel - rear_car.v_rel) < 0.5) {
    gap_predict_time = 0.0;
  }
  double gap_length = front_car.start_s - rear_car.end_s +
                      gap_predict_time * (front_car.v_rel - rear_car.v_rel);
  double lc_end_dis = dis_to_change_point_;
  double l_end_dis = 100.0;
  if (lc_end_dis < 200.0) {
    double gap_end_dis = lc_end_dis - acc_time * (v_ego + rear_car.v_rel) -
                         (rear_car.end_s - ego_sl.end_s) - safety_distance;
    l_end_dis = gap_end_dis;
  }
  MSD_LOG(INFO,
          "LaneChangeDecider[check_gap_valid]: gap [%d %d] rear v %f "
          "temp_lead_v %f temp_lead %d ego_v %f rear_dis %f v_limit_ %f",
          front_car.id, rear_car.id, rear_car.v_rel, rear_car.temp_lead_v_rel,
          rear_car.temp_lead, v_ego, rear_car.d_rel, current_gap_v_limit);
  MSD_LOG(INFO,
          "LaneChangeDecider[check_gap_valid]: gap lc_debug: gap %f mss0 %f "
          "mss %f d_p %f v_target_p %f v_ego_p %f acc_time %f v_ego_p_rel %f "
          "min_rear_dis %f acc_valid %d",
          gap_length, mss0, mss, d_p, v_target_p, v_ego_p, acc_time,
          v_ego_p_rel,
          std::max(mssr + ego_car_length, safety_distance + ego_car_length),
          gap_info.acc_valid);
  MSD_LOG(
      INFO,
      "LaneChangeDecider[check_gap_valid]: base id %d base_d_rel %f front s %f",
      base_car.id, base_d_rel, front_car.start_s);
  if (gap_length > mss0 && gap_length > mss && d_p > 0.0 &&
      v_target_p >= v_ego_p && acc_time < std::numeric_limits<double>::max()) {
    gap_info.valid = true;
    if (gap_info.acc_valid) {
      v_ego_p = std::min(std::max(0.0, v_ego + front_car.v_rel - 1.0),
                         current_gap_v_limit);
      v_ego_p_rel = v_ego_p - v_ego;
    }
    v_ego_p =
        std::max(0.0, std::min(v_ego_p, rear_car.temp_lead_v_rel + v_ego));
    if (front_car.id != -1) {
      v_ego_p =
          std::min(v_ego_p, target_lane_traffic_flow_info_.most_front_car_vel);
    }
    v_ego_p_rel = v_ego_p - v_ego;
    MSD_LOG(INFO, "LaneChangeDecider[check_gap_valid]: gap v_ego_p %f",
            v_ego_p);
    double dis_offset =
        (gap_info.rear_id == base_car.id) ? car_length : -car_length;
    // double gap_cost =  ((10.0 > rear_car.end_s - ego_sl.start_s) &&
    // (front_car.start_s - ego_sl.end_s > -10.0)) ?
    //                   -std::min(std::max(front_car.start_s - rear_car.end_s
    //                   - 20.0, 0.0), 20.0) : 0.0;
    double gap_distance{0.0};
    if (rear_car.end_s > ego_sl.start_s) {
      gap_distance = rear_car.end_s - ego_sl.start_s;
    }
    if (front_car.start_s < ego_sl.end_s) {
      gap_distance = std::max(gap_distance, ego_sl.end_s - front_car.start_s);
    }
    MSD_LOG(INFO,
            "LaneChangeDecider[check_gap_valid]: gap_distance %f length %f "
            "predict length %f",
            gap_distance, front_car.start_s - rear_car.end_s,
            (front_car.v_rel - rear_car.v_rel) * acc_time);
    // constexpr double kMaxGapLengthCost = 15.0;
    // double gap_length_cost = kMaxGapLengthCost - std::min(kMaxGapLengthCost,
    //     std::max(front_car.start_s - rear_car.end_s +
    //     std::min((front_car.v_rel - rear_car.v_rel) * acc_time, 5.0) - 5.0,
    //     0.0));
    double min_gap_length = std::max(mss, mss0);
    constexpr double kDefaultTTC = 0.3;
    double front_safe_buffer{0.0}, rear_safe_buffer{0.0};
    if (front_car.id > 0) {
      front_safe_buffer = (front_car.v_rel + v_ego) * kDefaultTTC;
    }
    if (rear_car.id > 0) {
      rear_safe_buffer = (rear_car.v_rel + v_ego) * kDefaultTTC;
    }
    double safe_gap_length =
        min_gap_length + front_safe_buffer + rear_safe_buffer;
    double kMaxGapLengthCost = 2.0;
    double gap_length_cost = kMaxGapLengthCost *
                             clip(safe_gap_length - gap_length, 0.0,
                                  safe_gap_length - min_gap_length + 1.e-2) /
                             (safe_gap_length - min_gap_length + 1.e-2);
    // double optimality_cost = std::pow(std::abs(v_limit_ - v_ego_p), 2);
    constexpr double kSpeedCostPower = 0.5;
    constexpr double kIntialSlope = 0.5;
    constexpr double kMasGapRefDis = 20.0;
    constexpr double kMaxDisCost = 10.0;
    double speed_bias =
        std::pow(kIntialSlope / kSpeedCostPower, 1.0 / (kSpeedCostPower - 1.0));
    double min_ratio = std::pow(speed_bias, kSpeedCostPower);
    double max_ratio = std::pow(speed_bias + 1.0, kSpeedCostPower);
    double optimality_cost =
        (1 - clip((std::pow(v_ego_p / v_limit_ + speed_bias, kSpeedCostPower) -
                   min_ratio) /
                      (max_ratio - min_ratio),
                  0.0, 1.0)) *
        50.0;
    optimality_cost +=
        (1 - clip(front_car.d_rel + 10.0, 0.0, kMasGapRefDis) / kMasGapRefDis) *
        kMaxDisCost;
    // add lon distance reward
    // if (front_car.id == -1) {
    //   constexpr double kMostFrontGapReward = 2.0;
    //   optimality_cost -= kMostFrontGapReward;
    // }
    // constexpr double kDisCoefficient = 0.02;
    // optimality_cost += clip(rear_car.end_s - ego_sl.start_s, -20.0, 100.0) *
    // kDisCoefficient;
    double time_efficiency_cost = acc_time * 0.5;
    double feasibility_cost = gap_length_cost;
    double v_cruise = map_v_limit_;
    double kMinLCLength = v_cruise * 8.0;
    double kMaxLCLength = v_cruise * 20.0;
    constexpr double kMinLCTime = 10.0;
    constexpr double kMaxLCTime = 20.0;
    constexpr double kMinLKTime = 5.0;
    double curr_lane_change_ref = dis_to_change_point_ / lane_change_cross_num_;
    MSD_LOG(INFO, "LaneChangeDecider[check_gap_valid]: curr_lane_change_ref %f",
            curr_lane_change_ref);
    double kRiskAppetite = 1.0;
    if (is_in_merge_scene) {
      feasibility_cost *= 2;
      double time_for_adjust =
          curr_lane_change_ref /
              PlanningContext::Instance()->planning_status().v_limit -
          kMinLKTime;
      time_efficiency_cost = clip(acc_time - time_for_adjust, 0.0,
                                  acc_time + std::abs(time_for_adjust)) *
                             0.5;
    }
    if (scenario_type_ == ScenarioType::URBAN) {
      kRiskAppetite =
          0.0 + 1.0 *
                    (kMaxLCLength -
                     clip(curr_lane_change_ref, kMinLCLength, kMaxLCLength)) /
                    (kMaxLCLength - kMinLCLength);
    } else if (scenario_type_ == ScenarioType::HIGHWAY) {
      kRiskAppetite =
          2.0 *
          (0.0 +
           1.0 * (clip(lc_wait_time_, kMinLCTime, kMaxLCTime) - kMinLCTime) /
               (kMaxLCTime - kMinLCTime));
      if (curr_lane_change_ref < 800.0) {
        double max_lc_time = clip(
            curr_lane_change_ref /
                    PlanningContext::Instance()->planning_status().v_limit -
                kMinLKTime,
            kMinLCTime, kMaxLCTime);
        kRiskAppetite = std::max(
            kRiskAppetite,
            5.0 * (0.1 + 0.9 *
                             (kMaxLCTime -
                              clip(max_lc_time, kMinLCTime, kMaxLCTime)) /
                             (kMaxLCTime - kMinLCTime)));
        MSD_LOG(INFO,
                "LaneChangeDecider[check_gap_valid]: max_lc_time %f "
                "kRiskAppetite %f acc_time %f",
                max_lc_time, kRiskAppetite, acc_time);
      }
    }
    MSD_LOG(INFO,
            "LaneChangeDecider gap kRiskAppetite %f lc_wait_time_ %f gap_cost "
            "%f feasibility_cost %f optimality_cost %f time_efficiency_cost %f",
            kRiskAppetite, lc_wait_time_, gap_length_cost, feasibility_cost,
            optimality_cost, time_efficiency_cost);
    double cost = 0.0;
    if (is_gap_insertable(gap_info)) {
      MSD_LOG(INFO,
              "LaneChangeDecider[check_gap_valid]: gap [%d %d] is insertable "
              "gap vel [%f %f]",
              gap_info.front_id, gap_info.rear_id, front_car.v_rel,
              rear_car.v_rel);
      cost = feasibility_cost + kRiskAppetite * time_efficiency_cost;
    } else if (std::abs(rear_car.v_rel + v_ego) < 0.5 &&
               (front_car.id > 0 || !is_most_front_gap_invisible_)) {
      MSD_LOG(INFO,
              "LaneChangeDecider[check_gap_valid]: gap [%d %d] is sure "
              "insertable gap vel [%f %f]",
              gap_info.front_id, gap_info.rear_id, front_car.v_rel,
              rear_car.v_rel);
      cost = optimality_cost + kRiskAppetite * time_efficiency_cost;
    } else if (!context_->lateral_behavior_planner_output().must_change_lane &&
               front_car.id == -1 && is_most_front_gap_invisible_) {
      cost = 0;
      gap_info.valid = false;
    } else {
      cost = optimality_cost + feasibility_cost +
             kRiskAppetite * time_efficiency_cost;
    }
    if (l_end_dis < 0.0) {
      cost += 100.0;
      if (rear_car.start_s > ego_sl.end_s) {
        cost += 900.0;
      }
    }
    constexpr double kDisPenalization = 1.e+5;
    if (rear_car.start_s - ego_sl.start_s > curr_lane_change_ref) {
      // cost += (rear_car.start_s - ego_sl.start_s - curr_lane_change_ref) *
      // kDisPenalization;
      gap_info.valid = false;
      gap_info.cost = 0;
    }
    gap_info.cost = cost;
  } else {
    gap_info.valid = false;
    gap_info.cost = 0;
  }
  auto front_obs =
      baseline_info_->mutable_obstacle_manager().find_obstacle(front_car.id);
  if (!context_->state_machine_output().must_change_lane && front_obs &&
      front_obs->Type() == ObjectType::CONE_BUCKET && gap_info.valid) {
    gap_info.cost += 10000;
  }
  // check if in traffic jam
  if (gap_info.rear_id == -2 && gap_info.front_id > 0) {
    if (front_car.v_rel + v_ego < 2.0 && acc_time > 15.0 &&
        front_car.start_s - ego_sl.end_s < -5.0) {
      MSD_LOG(INFO,
              "LaneChangeDecider[check_gap_valid]: last gap is unacceptable");
      gap_info.valid = false;
      gap_info.cost = 0;
    }
  }
  auto overtake_obstacle =
      baseline_info_->mutable_obstacle_manager().find_obstacle(
          gap_info.rear_id);
  auto follow_obstacle =
      baseline_info_->mutable_obstacle_manager().find_obstacle(
          gap_info.front_id);

  // ignore gap with cone bucket front
  // if (front_obs && front_obs->Type() == ObjectType::CONE_BUCKET) {
  //   gap_info.cost = std::numeric_limits<double>::max();
  //   gap_info.valid = false;
  // }
  MSD_LOG(INFO,
          "LaneChangeDecider[check_gap_valid]: gap [%d %d] l_end_dis %f cost "
          "%f valid %d",
          front_car.id, rear_car.id, l_end_dis, gap_info.cost, gap_info.valid);

  return gap_info;
}

double LaneChangeDecider::calc_time_for_lane_change(
    TargetObstacle base_car, TargetObstacle front_car, GapInfo gap_info,
    const double safety_distance, const double max_v) {
  bool is_base_car_rear = gap_info.rear_id == base_car.id;
  double v_ego = baseline_info_->get_ego_state().ego_vel;
  // if (front_car.end_s < most_front_static_obs_dis_ + 1.e-2 &&
  // front_car.start_s < baseline_info_->get_adc_sl_boundary().end_s) {
  //   return std::numeric_limits<double>::infinity();
  // }
  double v_diff_max = (is_base_car_rear)
                          ? std::min(3.0, max_v - base_car.v_rel)
                          : std::min(3.0, v_ego + base_car.v_rel - 0.5);
  double v_diff_end = std::min(1.0, v_diff_max);
  if (gap_info.acc_valid) {
    v_diff_max = std::max(
        std::min(front_car.v_rel - 1.0, max_v) - base_car.v_rel, v_diff_max);
    v_diff_end = v_diff_max;
  }
  double a = 0.8;
  double d, d_offset, vrel;
  double acc_time = 0.0;
  double mss = 100.0;
  auto ego_sl =
      world_model_->get_baseline_info(current_lane_id_)->get_adc_sl_boundary();
  if (is_base_car_rear) {
    // d_offset = clip(base_car.d_rel +
    // ConfigurationContext::Instance()->get_vehicle_param().length, 0,
    // ConfigurationContext::Instance()->get_vehicle_param().length) +
    // ConfigurationContext::Instance()->get_vehicle_param().length;
    d = safety_distance + base_car.end_s - ego_sl.start_s;
    vrel = -1 * base_car.v_rel;
    mss = (base_car.v_rel) * 2.0 +
          std::pow(std::max((base_car.v_rel), 0.0), 2) / 2.0 + safety_distance +
          ConfigurationContext::Instance()->get_vehicle_param().length;
    MSD_LOG(INFO,
            "zzz gap debug basecar v_rel %f v_ego %f safety_distance %f d %f",
            base_car.v_rel, v_ego, safety_distance, d);
  } else {
    a = 1.6;
    // d_offset = clip(base_car.d_rel,
    // -ConfigurationContext::Instance()->get_vehicle_param().length, 0.0);
    d = safety_distance - (base_car.start_s - ego_sl.end_s);
    vrel = base_car.v_rel;
    mss = std::pow(std::max(-(base_car.v_rel), 0.0), 2) / 2.0 + safety_distance;
  }
  mss = std::max(0.0, mss);

  if ((mss <= ego_sl.start_s - base_car.end_s && is_base_car_rear) ||
      (mss <= base_car.start_s - ego_sl.end_s && !is_base_car_rear) || d <= 0) {
    MSD_LOG(INFO, "zzz gap debug [%d %d] mss %f d %f", front_car.id,
            base_car.id, mss, base_car.d_rel);
    return acc_time;
  }
  if (v_diff_max <= 0.0) {
    acc_time = std::numeric_limits<double>::infinity();
    return acc_time;
  }
  if (vrel > v_diff_max) {
    double d_dec_max = (std::pow(vrel, 2) - std::pow(v_diff_end, 2)) / (2 * a);
    if (d > d_dec_max) {
      acc_time = (d - d_dec_max) / v_diff_max + (vrel - v_diff_end) / a;
    } else {
      acc_time =
          (vrel - std::sqrt(std::max(0.0, std::pow(vrel, 2) - 2 * d * a))) / a;
    }
  } else {
    d += std::pow(std::min(vrel, 0.0), 2) / (2 * a);
    double d_dec = std::pow(v_diff_max - v_diff_end, 2) / (2 * a);
    if (d <
        d_dec + (std::pow(v_diff_max, 2) - std::pow(std::max(vrel, 0.0), 2)) /
                    (2 * a)) {
      double v_m = std::sqrt(d + std::pow(std::max(vrel, 0.0), 2) / (2 * a) +
                             std::pow(v_diff_end, 2) / (2 * a));
      if (v_m >= vrel && v_m >= v_diff_end) {
        acc_time = (v_m - vrel + v_m - v_diff_end) / a;
      } else if (vrel > v_diff_end) {
        acc_time =
            (vrel - std::sqrt(std::max(0.0, std::pow(vrel, 2) - 2 * d * a))) /
            a;
      } else {
        acc_time = (std::sqrt(std::pow(vrel, 2) + 2 * d * a) - vrel) / a;
      }
    } else {
      acc_time = (d + std::pow(v_diff_max - std::max(vrel, 0.0), 2) / (2 * a) +
                  d_dec) /
                     v_diff_max +
                 std::abs(std::min(vrel / a, 0.0));
    }
  }
  return acc_time;
}

double LaneChangeDecider::clip(const double x, const double lo,
                               const double hi) {
  return std::max(lo, std::min(hi, x));
}

double LaneChangeDecider::calc_lane_width(
    const double &s, const std::vector<RefPointFrenet> &ref_line) {
  double lane_width = 3.8;
  if (ref_line.size() == 0) {
    return lane_width;
  } else if (s < ref_line.front().s) {
    lane_width = ref_line.front().lane_width;
    return clip(lane_width, 3.0, 4.0);
  } else if (s > ref_line.back().s) {
    lane_width = ref_line.back().lane_width;
    return clip(lane_width, 3.0, 4.0);
  }
  for (int i = 0; i < (int)ref_line.size() - 1; i++) {
    if (s >= ref_line.at(i).s && s <= ref_line.at(i + 1).s) {
      double k =
          (s - ref_line.at(i).s) / (ref_line.at(i + 1).s - ref_line.at(i).s);
      lane_width =
          ref_line.at(i).lane_width +
          (ref_line.at(i + 1).lane_width - ref_line.at(i).lane_width) * k;
    }
  }
  return clip(lane_width, 3.0, 4.0);
}

double LaneChangeDecider::calc_desired_distance(const double v_lead,
                                                const double v_ego) {
  double t_gap = interp(v_ego, _T_GAP_VEGO_BP, _T_GAP_VEGO_V);
  double d_offset = 4.0;
  return d_offset + std::max(v_lead, 0.0) * t_gap;
}

double LaneChangeDecider::calc_desired_speed(const double d_lead,
                                             const double d_des,
                                             const double v_lead_unp) {
  // *** compute desired speed ***
  // the desired speed curve is divided in 4 portions:
  // 1-constant
  // 2-linear to regain distance
  // 3-linear to shorten distance
  // 4-parabolic (constant decel)
  const double max_runaway_speed = -2.;

  double v_lead = std::max(v_lead_unp, 0.0);

  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);

  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));
  double v_rel_des = 0.0;
  if (d_lead < d_des) {
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_lead - d_des);
    double v_rel_des_2 = (d_lead - d_des) * l_slope / 3.0;
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else if (d_lead < d_des + x_linear_to_parabola) {
    v_rel_des = (d_lead - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else {
    v_rel_des = std::sqrt(2 * (d_lead - d_des - x_parabola_offset) * p_slope);
  }
  double v_target = v_rel_des + v_lead;

  return v_target;
}

double LaneChangeDecider::interp(double x, const std::vector<double> &xp,
                                 const std::vector<double> &fp) {
  const int N = xp.size() - 1;

  if (x < xp[0]) {
    return fp[0];
  }
  for (int i = 0; i <= N; ++i) {
    if (x < xp[i]) {
      return ((x - xp[i - 1]) * (fp[i] - fp[i - 1]) / (xp[i] - xp[i - 1]) +
              fp[i - 1]);
    }
  }

  return fp[N];
}

bool LaneChangeDecider::compare_distance_asc(const TargetObstacle &obs1,
                                             const TargetObstacle &obs2) {
  return obs1.start_s < obs2.start_s;
}
bool LaneChangeDecider::compare_distance_drel(const TargetObstacle &obs1,
                                              const TargetObstacle &obs2) {
  if (std::fabs(obs1.d_rel - obs2.d_rel) < 1e-5) {
    return obs1.start_s < obs2.start_s;
  } else {
    return obs1.d_rel < obs2.d_rel;
  }
}
bool LaneChangeDecider::compare_cost_asc(const GapInfo &gap1,
                                         const GapInfo &gap2) {
  return gap1.cost < gap2.cost;
}
} // namespace msquare
