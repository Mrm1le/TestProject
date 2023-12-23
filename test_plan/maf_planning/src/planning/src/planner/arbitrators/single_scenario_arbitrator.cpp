#include "planner/arbitrators/single_scenario_arbitrator.h"
#include "common/config_context.h"
#include "planner/scenarios/scenario_factory.h"
#include <cstdlib>

namespace msquare {

void SingleScenarioArbitrator::init(
    const ScenarioFacadeConfig &scenario_config) {
  scenario_facade_ = ScenarioFactory::CreateScenarioFacade(scenario_config);
  scenario_facade_->init();
  // preset scenario context
  auto scenario_context = scenario_facade_->get_scenario_context();
  // todo(@zzd) construct obs_decision_manager in scenario context
  *scenario_context->mutable_planning_status() =
      PlanningContext::Instance()->planning_status();
  scenario_context->mutable_planning_status()->planning_success = false;
  // scenario_context->mutable_lateral_behavior_planner_output() =
  //     PlanningContext::Instance()->lateral_behavior_planner_output();
}

void SingleScenarioArbitrator::init_from_type(
    const ScenarioFacadeConfig::ScenarioFacadeType scenario_type) {
  scenario_facade_ =
      ScenarioFactory::ProduceScenarioWithSpecifiedType(scenario_type);
}

ScenarioFacade &SingleScenarioArbitrator::get_mutable_scenario_facade() const {
  return *scenario_facade_;
}

std::shared_ptr<Arbitrator> SingleScenarioArbitrator::get_best_solution() {
  return shared_from_this();
}

// std::shared_ptr<SingleScenarioArbitrator>
// SingleScenarioArbitrator::shared_from_this() {
//   return
//   std::static_pointer_cast<SingleScenarioArbitrator>(Arbitrator::shared_from_this());
// }

void SingleScenarioArbitrator::arbitrate() {
  best_arbitrator_ = nullptr;
  // std::shared_ptr<Arbitrator> best_arbitrator = nullptr;
  if (!scenario_facade_->process(world_model_)) {
    MSD_LOG(INFO, "arbitrator[%s] failed", name_.c_str());
    optimal_solution_cost_ = std::numeric_limits<double>::infinity();
    higher_level_cost_ = std::numeric_limits<double>::infinity();
    optimal_scenario_context_ = scenario_facade_->get_scenario_context();
  } else {
    optimal_scenario_context_ = scenario_facade_->get_scenario_context();
    post_process();
    optimal_solution_cost_ = 0.0;
    higher_level_cost_ = 0.0;
    calculate_higher_level_cost();
    // auto s = get_best_solution();
    // best_arbitrator_ =
    // std::dynamic_pointer_cast<SingleScenarioArbitrator>(shared_from_this());
  }
}

bool SingleScenarioArbitrator::invocation_condition() const { return true; }

bool SingleScenarioArbitrator::commitment_condition() const { return true; }

void SingleScenarioArbitrator::calculate_higher_level_cost() {
  if (cost_func_) {
    optimal_solution_cost_ =
        cost_func_(optimal_scenario_context_, world_model_);
    MSD_LOG(INFO, "arbitrator [%s] use dynamic cost func %f", name_.c_str(),
            optimal_solution_cost_);
    higher_level_cost_ = optimal_solution_cost_;
    return;
  }
  return;
  if (optimal_scenario_context_ == nullptr) {
    MSD_LOG(INFO, "arbitrator[%s] null", name_.c_str());
    optimal_solution_cost_ = std::numeric_limits<double>::infinity();
    return;
  }
  const auto &speed_data =
      optimal_scenario_context_->longitudinal_motion_planner_output();
  const auto &planning_status = optimal_scenario_context_->planning_status();
  const auto &baseline_info = scenario_facade_->get_baseline_info();
  if (!baseline_info || !baseline_info->is_valid()) {
    return;
  }
  constexpr double kLaneChangeNeededCost = 10.0;  // km/h
  constexpr double kLaneChangeManeuverCost = 5.0; // km/h
  higher_level_cost_ = 0.0;

  // calculate average speed cost
  const auto &lane_status = planning_status.lane_status;
  bool is_in_lc_preparation_stage =
      lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
  double average_speed = 0.0;
  // TODO: fengxiaotong val real or relative?
  if (is_in_lc_preparation_stage) {
    auto gap = planning_status.lane_status.change_lane.target_gap_obs;
    if (gap.first == -10 || gap.second == -10) { // no valid gap
      average_speed = 0.0;
    } else if (gap.first == -1) {
      average_speed = 15.0;
    } else if (gap.first > 0) {
      auto obs_gap_front =
          baseline_info->mutable_obstacle_manager().find_obstacle(gap.first);
      if (obs_gap_front != nullptr) {
        const auto &obs_traj = obs_gap_front->Trajectory();
        if (obs_gap_front->IsStatic() || obs_traj.empty() ||
            std::abs(obs_traj.back().relative_time -
                     obs_traj.front().relative_time) < 1.e-1) {
          average_speed = 0.0;
        } else {
          average_speed =
              (obs_traj.back().path_point.s - obs_traj.front().path_point.s) /
              (obs_traj.back().relative_time - obs_traj.front().relative_time);
        }
      }
    }
    // TODO: replace by real map v_limit for target_lane
    average_speed =
        std::min(average_speed, world_model_->get_map_info().v_cruise());
  } else {
    SpeedPoint sp;
    if (speed_data.EvaluateByTime(FLAGS_lon_decision_time_horizon, &sp)) {
      average_speed =
          (sp.s - speed_data.front().s) / FLAGS_lon_decision_time_horizon;
    }
  }
  higher_level_cost_ -= average_speed * 3.6;
  MSD_LOG(INFO, "arbitrator[%s] average speed %f %d", name_.c_str(),
          average_speed, is_in_lc_preparation_stage);

  // calculate lane change need cost
  auto target_lane_id = planning_status.lane_status.change_lane.path_id;
  auto target_baseline = world_model_->get_baseline_info(target_lane_id);
  if (!target_baseline || !target_baseline->is_target_lane()) {
    MSD_LOG(INFO, "arbitrator[%s] not target lane", name_.c_str());
    higher_level_cost_ += kLaneChangeNeededCost;
  }

  // calculate lane change maneuver cost
  if (scenario_facade_->get_baseline_info()->is_change_lane()) {
    higher_level_cost_ += kLaneChangeManeuverCost;
  }

  // calculate collision cost
  auto scenario_context = scenario_facade_->get_scenario_context();
  bool is_traj_safe = true;
  if (true) {
    is_traj_safe = check_trajectory_safety(
        optimal_scenario_context_->planning_status().planning_result,
        scenario_facade_->get_baseline_info(),
        scenario_facade_->get_scenario_context());
  }
  if (!is_traj_safe) {
    MSD_LOG(INFO, "arbitrator[%s] not safe", name_.c_str());
    if (scenario_context->planning_status().scheme_stage ==
        SchemeStage::BACKUP) {
      MSD_LOG(ERROR, "arbitrator[%s] not safe in primary stage", name_.c_str());
    }
    higher_level_cost_ = std::numeric_limits<double>::infinity();
    optimal_solution_cost_ = higher_level_cost_;
    return;
  }

  optimal_solution_cost_ = higher_level_cost_;
}

bool SingleScenarioArbitrator::check_trajectory_safety(
    const PlanningResult &planning_result,
    std::shared_ptr<BaseLineInfo> baseline_info,
    std::shared_ptr<ScenarioFacadeContext> context) {
  if (planning_result.traj_pose_array.empty()) {
    return false;
  }
  if (planning_result.traj_vel_array.empty()) {
    return false;
  }
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    return false;
  }
  auto &obstacle_manager = baseline_info->obstacle_manager();
  auto &obstacle_decision_manager =
      context->mutable_obstacle_decision_manager();
  std::vector<int> blocking_id;

  double kDeltaT = 0.2;
  constexpr double kMinCheckTime = 1.0;
  double check_time_length;
  bool is_in_lane_change = false;
  if (context->planning_status().lane_status.status ==
          LaneStatus::Status::LANE_CHANGE &&
      context->planning_status().lane_status.change_lane.status ==
          ChangeLaneStatus::Status::IN_CHANGE_LANE) {
    check_time_length = 2.5;
    auto gap =
        context->planning_status().lane_status.change_lane.target_gap_obs;
    is_in_lane_change = true;
  } else {
    check_time_length = 4.0;
  }
  MSD_LOG(INFO, "arbitrator check_time_length %f", check_time_length);
  check_time_length = std::max(check_time_length, kMinCheckTime);
  int planning_step = static_cast<int>(kDeltaT / FLAGS_trajectory_density);
  int num_planning_step = static_cast<int>(
      std::min(FLAGS_trajectory_time_length, check_time_length) / kDeltaT);
  int num_sample_step = static_cast<int>(
      std::min(FLAGS_trajectory_time_length, check_time_length) /
      FLAGS_trajectory_density);
  int max_index_range = static_cast<int>(kDeltaT / FLAGS_trajectory_density);

  std::shared_ptr<FrenetCoordinateSystem> frenet_coord =
      baseline_info->get_frenet_coord();
  EgoState ego_state = baseline_info->get_ego_state();
  int position_matched_index =
      baseline_info->get_ego_state_manager().query_nearst_point_with_buffer(
          planning_result.traj_pose_array, ego_state.ego_pose.x,
          ego_state.ego_pose.y, 1.0e-6);

  double block_s_end = 200.0;
  std::vector<int> clear_obs_id;
  int last_consider_index = 0;
  double min_adc_check_length = std::min(1.0, ConfigurationContext::Instance()
                                                  ->get_vehicle_param()
                                                  .front_edge_to_center);
  const auto &speed_data = planning_result.traj_vel_array;

  // find all obstacles given decisions
  std::vector<int> interested_obstacles;
  for (const auto *ptr_obstacle : obstacle_manager.get_obstacles().Items()) {
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(ptr_obstacle->Id());
    const ObjectDecisionType &lat_decision =
        ptr_obstacle_decision->LateralDecision();
    const ObjectDecisionType &lon_decision =
        ptr_obstacle_decision->LongitudinalDecision();

    if (!lat_decision.has_nudge() && !lon_decision.has_stop() &&
        !lon_decision.has_follow() && !lon_decision.has_yield() &&
        !lon_decision.has_overtake()) {
      continue;
    } else {
      interested_obstacles.push_back(ptr_obstacle->Id());
    }
  }
  std::vector<int> obs_no_need_to_check;

  for (int i = position_matched_index;
       i <
       std::min(num_sample_step, (int)planning_result.traj_pose_array.size());
       i += planning_step) {
    Point2D cart_pose, frenet_pose;
    cart_pose.x = planning_result.traj_pose_array[i].position_enu.x;
    cart_pose.y = planning_result.traj_pose_array[i].position_enu.y;
    PathPoint adc_point;
    adc_point.x = cart_pose.x;
    adc_point.y = cart_pose.y;
    double cart_yaw = planning_result.traj_pose_array[i].heading_yaw;

    if (std::abs(speed_data[i].distance -
                 speed_data[last_consider_index].distance) <
            min_adc_check_length &&
        last_consider_index != 0 &&
        last_consider_index != num_sample_step - 1 &&
        std::abs(i - last_consider_index) < max_index_range) {
      continue;
    }
    last_consider_index = i;

    (void)frenet_coord->CartCoord2FrenetCoord(cart_pose, frenet_pose);
    double theta_ref = frenet_coord->GetRefCurveHeading(frenet_pose.x);
    double frenet_theta =
        planning_result.traj_pose_array[i].heading_yaw - theta_ref;

    double dx1 = cos(frenet_theta) *
                 ConfigurationContext::Instance()->get_vehicle_param().length /
                 2.;
    double dy1 = sin(frenet_theta) *
                 ConfigurationContext::Instance()->get_vehicle_param().length /
                 2.;
    double dx2 = sin(frenet_theta) *
                 ConfigurationContext::Instance()->get_vehicle_param().width /
                 2.;
    double dy2 = cos(frenet_theta) *
                 ConfigurationContext::Instance()->get_vehicle_param().width /
                 2.;

    SLBoundary adc_sl;
    adc_sl.start_s = frenet_pose.x - fabs(dx1) - fabs(dx2);
    adc_sl.start_l = frenet_pose.y - fabs(dy1) - fabs(dy2);
    adc_sl.end_s = frenet_pose.x + fabs(dx1) + fabs(dx2);
    adc_sl.end_l = frenet_pose.y + fabs(dy1) + fabs(dy2);

    double relative_time =
        (i - position_matched_index) / planning_step * kDeltaT;

    if (is_in_lane_change && relative_time > kMinCheckTime) {
      auto &path_data = baseline_info->get_path_data();
      double adc_path_s = path_data.discretized_path().QueryMatchedS(adc_point);
      auto lane_border = path_data.getWidth(adc_path_s);
      // MSD_LOG(INFO, "arbitrator finish lc [%f %f] [%f %f] %f",
      // adc_sl.start_l, adc_sl.end_l, lane_border.second, lane_border.first,
      //       (i - position_matched_index) / planning_step * kDeltaT);
      if (adc_sl.start_l > lane_border.second &&
          adc_sl.end_l < lane_border.first) {
        break;
      }
    }

    block_s_end = adc_sl.end_s;
    planning_math::Vec2d adc_box_center(cart_pose.x, cart_pose.y);
    planning_math::Box2d adc_box(
        adc_box_center, cart_yaw,
        ConfigurationContext::Instance()->get_vehicle_param().length + 0.2,
        ConfigurationContext::Instance()->get_vehicle_param().width + 0.2);
    planning_math::Polygon2d adc_polygon(adc_box);

    for (auto obs_id : interested_obstacles) {
      if (!obs_no_need_to_check.empty() &&
          std::find(obs_no_need_to_check.begin(), obs_no_need_to_check.end(),
                    obs_id) != obs_no_need_to_check.end()) {
        continue;
      }
      auto *ptr_obstacle = obstacle_manager.find_obstacle(obs_id);
      if (ptr_obstacle == nullptr) {
        continue;
      }
      auto ptr_obstacle_decision =
          obstacle_decision_manager.find_obstacle_decision(ptr_obstacle->Id());
      if (ptr_obstacle_decision == nullptr) {
        continue;
      }
      const ObjectDecisionType &lat_decision =
          ptr_obstacle_decision->LateralDecision();
      const ObjectDecisionType &lon_decision =
          ptr_obstacle_decision->LongitudinalDecision();

      if (!lat_decision.has_nudge() && !lon_decision.has_stop() &&
          !lon_decision.has_follow() && !lon_decision.has_yield() &&
          !lon_decision.has_overtake()) {
        continue;
      }
      bool is_lat_need_nudge = false;
      bool is_lon_need_evade = false;
      bool is_lon_overtake = false;
      if (lat_decision.has_nudge()) {
        if (relative_time >= lat_decision.nudge().start_time &&
            relative_time <= lat_decision.nudge().start_time +
                                 lat_decision.nudge().time_buffer) {
          is_lat_need_nudge = true;
        }
      }
      if (lon_decision.has_stop()) {
        is_lon_need_evade = true;
      }
      if (lon_decision.has_yield()) {
        if (relative_time >= lon_decision.yield().start_time &&
            relative_time <= lon_decision.yield().start_time +
                                 lon_decision.yield().time_buffer) {
          is_lon_need_evade = true;
        }
      }
      if (lon_decision.has_overtake()) {
        if (relative_time >= lon_decision.overtake().start_time &&
            relative_time <= lon_decision.overtake().start_time +
                                 lon_decision.overtake().time_buffer) {
          is_lon_need_evade = true;
          is_lon_overtake = true;
        }
      }
      if (lon_decision.has_follow()) {
        if (relative_time >= lon_decision.follow().start_time &&
            relative_time <= lon_decision.follow().start_time +
                                 lon_decision.follow().time_buffer) {
          is_lon_need_evade = true;
        }
      }

      if (!is_lat_need_nudge && !is_lon_need_evade) {
        continue;
      }

      if (is_lon_overtake && is_in_lane_change) {
        if (ptr_obstacle->has_sl_polygon_seq()) {
          PolygonWithT polygon_t;
          if (ptr_obstacle->sl_polygon_seq().EvaluateByTime(relative_time,
                                                            &polygon_t)) {
            auto &path_data = baseline_info->get_path_data();
            double adc_path_s =
                path_data.discretized_path().QueryMatchedS(adc_point);
            auto lane_border = path_data.getWidth(adc_path_s);
            double lateral_space =
                std::max(std::max(adc_sl.start_l - lane_border.second, 0.0),
                         std::max(lane_border.first - adc_sl.end_l, 0.0));
            if (adc_sl.start_s > polygon_t.second.max_x() + 0.5 &&
                lateral_space <
                    ptr_obstacle->PerceptionBoundingBox().width() - 0.3) {
              obs_no_need_to_check.push_back(ptr_obstacle->Id());
              continue;
            }
          }
        }
      }

      planning_math::Polygon2d obs_polygon;
      if (ptr_obstacle->IsStatic() || ptr_obstacle->Trajectory().empty()) {
        obs_polygon = ptr_obstacle->PerceptionPolygon();
      } else {
        TrajectoryPoint traj_point =
            ptr_obstacle->GetPointAtTime(relative_time);
        obs_polygon = ptr_obstacle->GetPolygonAtPoint(traj_point);
      }
      if (obs_polygon.points().empty() || !obs_polygon.is_convex()) {
        continue;
      }
      if (obs_polygon.HasOverlap(adc_polygon)) {
        blocking_id.push_back(ptr_obstacle->Id());
        TrajectoryPoint traj_point =
            ptr_obstacle->GetPointAtTime(relative_time);
        PolygonWithT polygon_t;
        (void)ptr_obstacle->sl_polygon_seq().EvaluateByTime(relative_time,
                                                            &polygon_t);
        MSD_LOG(INFO,
                "arbitrator trajectory safety check: obstacle[%d] collide [%f "
                "%f] [%f %f]",
                ptr_obstacle->Id(), traj_point.path_point.x,
                traj_point.path_point.y,
                0.5 * polygon_t.second.min_x() + 0.5 * polygon_t.second.max_x(),
                0.5 * polygon_t.second.min_y() +
                    0.5 * polygon_t.second.max_y());
        break;
      }
    }

    if (!blocking_id.empty()) {
      MSD_LOG(
          INFO,
          "arbitrator trajectory safety check: blocking s : %f blocking t : %f",
          block_s_end, relative_time);
      MSD_LOG(INFO, "arbitrator ego_sl [%f %f] cart [%f %f]", frenet_pose.x,
              frenet_pose.y, cart_pose.x, cart_pose.y);
      MSD_LOG(INFO, "arbitrator obs_sl [%f %f] cart [%f %f]", frenet_pose.x,
              frenet_pose.y, cart_pose.x, cart_pose.y);
      break;
    }
  }

  if (!blocking_id.empty()) {
    return false;
  }
  return true;
}

} // namespace msquare
