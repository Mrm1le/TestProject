#include "planner/arbitrators/arbitrator_cost_strategy.h"
#include "common/config_context.h"
#include "common/math/math_utils.h"

namespace msquare {

// util::Factory<ArbitratorCostType, ArbitratorCostFunc,
//               std::unordered_map<ArbitratorCostType, ArbitratorCostFunc,
//               std::hash<int>>>
//     ArbitratorCostStrategy::cost_factory_;

ArbitratorCostFuncMap ArbitratorCostStrategy::cost_func_map_;

void ArbitratorCostStrategy::set_arbitrator_cost_func(
    std::shared_ptr<Arbitrator> arbitrator, ArbitratorCostType cost_type) {
  MSD_LOG(INFO, "reset arbitrator[%s] cost func type %d",
          arbitrator->name().c_str(), int(cost_type));
  if (cost_func_map_.find(cost_type) == cost_func_map_.end()) {
    return;
  }
  arbitrator->set_cost_function(cost_func_map_[cost_type]);
  return;
}

void ArbitratorCostStrategy::init() {
  // constrcut cost func map
  cost_func_map_[ArbitratorCostType::NORMAL] =
      [](const std::shared_ptr<ScenarioFacadeContext> context,
         const std::shared_ptr<WorldModel> world_model) {
        return ArbitratorCostStrategy::normal_arbitrator_cost(context,
                                                              world_model);
      };
  cost_func_map_[ArbitratorCostType::UNPROTECTED_MANEUVER] =
      [](const std::shared_ptr<ScenarioFacadeContext> context,
         const std::shared_ptr<WorldModel> world_model) {
        return ArbitratorCostStrategy::unprotected_maneuver_arbitrator_cost(
            context, world_model);
      };
  cost_func_map_[ArbitratorCostType::NONESSENTIAL_LANECHANGE_MANEUVER] =
      [](const std::shared_ptr<ScenarioFacadeContext> context,
         const std::shared_ptr<WorldModel> world_model) {
        return ArbitratorCostStrategy::
            nonessential_lane_change_maneuver_arbitrator_cost(context,
                                                              world_model);
      };
}

double ArbitratorCostStrategy::normal_arbitrator_cost(
    const std::shared_ptr<ScenarioFacadeContext> context,
    const std::shared_ptr<WorldModel> world_model) {
  if (context == nullptr || world_model == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  const auto &speed_data = context->longitudinal_motion_planner_output();
  const auto &planning_status = context->planning_status();
  auto frenet_lane_id = planning_status.lane_status.target_lane_id;
  auto baseline_info = world_model->get_baseline_info(frenet_lane_id);
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    return 0.0;
  }
  constexpr double kLaneChangeNeededCost = 20.0;   // km/h
  constexpr double kLaneChangeManeuverCost = 10.0; // km/h
  constexpr double kLaneChangeManeuverCostCoefficient = 0.025;
  constexpr double kLaneChangeApproachGapCostCoefficient = 0.0;
  constexpr double kNotRequiredMapLaneChangeReward = 2.0; // km/h
  double cost = 0.0;

  // calculate average speed cost
  const auto &lane_status = planning_status.lane_status;
  bool is_in_lc_preparation_stage =
      lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
  bool is_in_lc_change_stage =
      lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      lane_status.change_lane.status ==
          ChangeLaneStatus::Status::IN_CHANGE_LANE;
  const auto &pre_lane_status =
      PlanningContext::Instance()->planning_status().lane_status;
  bool is_pre_in_lk_stage =
      pre_lane_status.status == LaneStatus::Status::LANE_KEEP;
  bool is_pre_in_wait_stage =
      pre_lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      pre_lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
  bool is_pre_in_change_stage =
      pre_lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      pre_lane_status.change_lane.status ==
          ChangeLaneStatus::Status::IN_CHANGE_LANE;
  double average_speed = 0.0;
  double map_v_limit = world_model->get_map_info().v_cruise();
  if (is_in_lc_preparation_stage) {
    auto gap = planning_status.lane_status.change_lane.target_gap_obs;
    auto most_front_car_vel = planning_status.lane_status.change_lane
                                  .target_lane_traffic_flow.most_front_car_vel;
    if (gap.first == -10 || gap.second == -10) { // no valid gap
      average_speed = 0.0;
    } else if (gap.first == -1) {
      average_speed = map_v_limit;
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
          // average_speed = (obs_traj.back().path_point.s -
          // obs_traj.front().path_point.s) /
          //   (obs_traj.back().relative_time - obs_traj.front().relative_time);
          average_speed = obs_traj.back().v;
        }
        if (is_pre_in_lk_stage && !obs_gap_front->IsFrenetInvalid()) {
          if (obs_gap_front->PerceptionSLBoundary().end_s <
              baseline_info->get_adc_sl_boundary().start_s) {
            average_speed =
                std::min(average_speed,
                         context->planning_status().planning_result.v_set);
          }
        }
      }
      average_speed = std::min(average_speed, most_front_car_vel);
    }
    // TODO: replace by real map v_limit for target_lane
    average_speed = std::min(average_speed, map_v_limit);
    if (world_model->get_traffic_light_decision()->get_stop_flag()) {
      average_speed = -10.0;
    }
  } else {
    // SpeedPoint sp;
    // if (speed_data.EvaluateByTime(FLAGS_lon_decision_time_horizon, &sp)) {
    //   average_speed = (sp.s - speed_data.front().s) /
    //   FLAGS_lon_decision_time_horizon;
    // }
    average_speed = context->planning_status().planning_result.v_set;
    if (context->planning_status().planning_result.id_yield < 0 ||
        context->planning_status().planning_result.vl_yield > average_speed) {
      average_speed =
          std::min(context->planning_status().planning_result.vl_yield,
                   planning_status.v_limit);
    } else {
      average_speed = std::min(average_speed, planning_status.v_limit);
    }
  }
  cost -= average_speed * 3.6;
  MSD_LOG(INFO, "arbitrator average speed %f %d", average_speed,
          is_in_lc_preparation_stage);

  // calculate lane change need cost
  auto target_lane_id = planning_status.lane_status.change_lane.path_id;
  auto target_baseline = world_model->get_baseline_info(target_lane_id);
  if (!target_baseline || !target_baseline->is_target_lane()) {
    MSD_LOG(INFO, "arbitrator not target lane %f", kLaneChangeNeededCost);
    cost += kLaneChangeNeededCost;
  }

  // calculate lane change maneuver cost
  if (target_baseline->is_change_lane()) {
    cost += kLaneChangeManeuverCostCoefficient *
            baseline_info->get_ego_state().ego_vel * 3.6;
    MSD_LOG(INFO, "arbitrator change lane %f",
            kLaneChangeManeuverCostCoefficient *
                baseline_info->get_ego_state().ego_vel * 3.6);
    if (is_in_lc_preparation_stage) {
      cost += kLaneChangeApproachGapCostCoefficient *
              baseline_info->get_ego_state().ego_vel * 3.6;
    }
    if (context->lateral_behavior_planner_output().lc_request_source ==
        "map_request") {
      cost -= kNotRequiredMapLaneChangeReward;
      MSD_LOG(INFO, "arbitrator map change lane reward %f",
              -kNotRequiredMapLaneChangeReward);
    }
  }

  // calculate collision cost
  bool is_traj_safe = true;
  if (true) {
    auto traj_check_result = check_trajectory_safety(baseline_info, context);
    context->mutable_planning_status()->planning_result.traj_collision_info =
        traj_check_result;
    double kMinavoidDisForLC = 0.05;
    double kStateChangeBufferForLC = 0.05;
    double kMinAvoidDis = 0.0;
    double min_avoid_dis = 0.0;
    // if (!is_pre_in_change_stage && is_in_lc_change_stage) {
    //   min_avoid_dis = kMinavoidDisForLC + kStateChangeBufferForLC;
    // } else if (is_in_lc_change_stage) {
    //   min_avoid_dis = kMinavoidDisForLC;
    // } else {
    //   min_avoid_dis = kMinAvoidDis;
    // }
    is_traj_safe = traj_check_result.min_obs_dis.distance > min_avoid_dis;
  }
  if (!is_traj_safe) {
    // MSD_LOG(INFO, "arbitrator[%s] not safe", name_.c_str());
    if (context->planning_status().scheme_stage == SchemeStage::BACKUP) {
      // MSD_LOG(ERROR, "arbitrator[%s] not safe in primary stage",
      // name_.c_str());
    }
    cost = std::numeric_limits<double>::infinity();
    return cost;
  }
  return cost;
}

double
ArbitratorCostStrategy::nonessential_lane_change_maneuver_arbitrator_cost(
    const std::shared_ptr<ScenarioFacadeContext> context,
    const std::shared_ptr<WorldModel> world_model) {
  if (context == nullptr || world_model == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  const auto &speed_data = context->longitudinal_motion_planner_output();
  const auto &planning_status = context->planning_status();
  auto frenet_lane_id = planning_status.lane_status.target_lane_id;
  auto baseline_info = world_model->get_baseline_info(frenet_lane_id);
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    return 0.0;
  }
  constexpr double kLaneChangeNeededCost = 20.0;   // km/h
  constexpr double kLaneChangeManeuverCost = 10.0; // km/h
  constexpr double kAvoidCutInCost = 20.0;         // km/h
  constexpr double kDefaultLatDis = 0.1;
  constexpr double kMinLatDis = -0.0;
  constexpr double kLaneChangeManeuverCostCoefficient = 0.125;
  double cost = 0.0;

  // calculate average speed cost
  const auto &lane_status = planning_status.lane_status;
  bool is_lane_keep_stage = lane_status.status == LaneStatus::Status::LANE_KEEP;
  bool is_in_lc_preparation_stage =
      lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
  bool is_in_lc_change_stage =
      lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      lane_status.change_lane.status ==
          ChangeLaneStatus::Status::IN_CHANGE_LANE;
  const auto &pre_lane_status =
      PlanningContext::Instance()->planning_status().lane_status;
  bool is_pre_in_lk_stage =
      pre_lane_status.status == LaneStatus::Status::LANE_KEEP;
  bool is_pre_in_wait_stage =
      pre_lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      pre_lane_status.change_lane.status ==
          ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION;
  bool is_pre_in_change_stage =
      pre_lane_status.status == LaneStatus::Status::LANE_CHANGE &&
      pre_lane_status.change_lane.status ==
          ChangeLaneStatus::Status::IN_CHANGE_LANE;
  double average_speed = 0.0;
  double map_v_limit = world_model->get_map_info().v_cruise();
  if (is_in_lc_preparation_stage) {
    auto gap = planning_status.lane_status.change_lane.target_gap_obs;
    if (gap.first == -10 || gap.second == -10) { // no valid gap
      average_speed = 0.0;
    } else if (gap.first == -1) {
      average_speed = map_v_limit;
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
          // average_speed = (obs_traj.back().path_point.s -
          // obs_traj.front().path_point.s) /
          //   (obs_traj.back().relative_time - obs_traj.front().relative_time);
          average_speed = obs_traj.back().v;
        }
      }
    }
    // TODO: replace by real map v_limit for target_lane
    average_speed = std::min(average_speed, map_v_limit);
  } else {
    average_speed = context->planning_status().planning_result.v_set;
    if (context->planning_status().planning_result.id_yield < 0 ||
        context->planning_status().planning_result.vl_yield > average_speed) {
      average_speed =
          std::min(context->planning_status().planning_result.vl_yield,
                   planning_status.v_limit);
    } else {
      average_speed = std::min(average_speed, planning_status.v_limit);
    }
  }
  cost -= average_speed * 3.6;
  MSD_LOG(INFO, "arbitrator average speed %f %d %d", average_speed * 3.6,
          is_in_lc_preparation_stage, is_lane_keep_stage);
  auto follow_obstacles =
      context->planning_status().planning_result.lon_follow_obstacles;
  auto nudge_obstacles =
      context->planning_status().planning_result.lat_nudge_obstacles;

  // calculate lane change need cost
  auto target_lane_id = planning_status.lane_status.change_lane.path_id;
  auto target_baseline = world_model->get_baseline_info(target_lane_id);
  if (!target_baseline || !target_baseline->is_target_lane()) {
    MSD_LOG(INFO, "arbitrator not target lane %f", kLaneChangeNeededCost);
    cost += kLaneChangeNeededCost;
  }

  // calculate lane change maneuver cost
  if (target_baseline->is_change_lane()) {
    cost += kLaneChangeManeuverCostCoefficient *
            baseline_info->get_ego_state().ego_vel * 3.6;
    MSD_LOG(INFO, "arbitrator change lane cost %f",
            kLaneChangeManeuverCostCoefficient *
                baseline_info->get_ego_state().ego_vel * 3.6);
  }

  // calculate collision cost
  bool is_traj_safe = true;
  auto traj_check_result = check_trajectory_safety(baseline_info, context);
  context->mutable_planning_status()->planning_result.traj_collision_info =
      traj_check_result;
  double kMinavoidDisForLC = 0.05;
  double kStateChangeBufferForLC = 0.05;
  double kMinAvoidDis = 0.0;
  double min_avoid_dis = 0.0;
  if (!is_pre_in_change_stage && is_in_lc_change_stage) {
    min_avoid_dis = kMinavoidDisForLC + kStateChangeBufferForLC;
  } else if (is_in_lc_change_stage) {
    min_avoid_dis = kMinavoidDisForLC;
  } else {
    min_avoid_dis = kMinAvoidDis;
  }
  is_traj_safe = traj_check_result.min_obs_dis.distance > min_avoid_dis;
  MSD_LOG(INFO,
          " arbitrator check trajectory safety info min_obs id %d t %f dis %f",
          traj_check_result.min_obs_dis.obs_id,
          traj_check_result.min_obs_dis.time,
          traj_check_result.min_obs_dis.distance);
  MSD_LOG(INFO,
          " arbitrator check trajectory safety info min_obs_lat_dis id %d t %f "
          "dis %f",
          traj_check_result.min_obs_lat_dis.obs_id,
          traj_check_result.min_obs_lat_dis.time,
          traj_check_result.min_obs_lat_dis.distance);
  MSD_LOG(INFO,
          " arbitrator check trajectory safety info min_obs_lon_dis id %d t %f "
          "dis %f",
          traj_check_result.min_obs_lon_dis.obs_id,
          traj_check_result.min_obs_lon_dis.time,
          traj_check_result.min_obs_lon_dis.distance);
  if (!is_traj_safe) {
    MSD_LOG(INFO, "arbitrator not safe");
    if (context->planning_status().scheme_stage == SchemeStage::BACKUP) {
      // MSD_LOG(ERROR, "arbitrator[%s] not safe in primary stage",
      // name_.c_str());
    }
    cost = std::numeric_limits<double>::infinity();
    return cost;
  } else {
    double v_cruise = map_v_limit;
    double v_set = context->planning_status().planning_result.v_set;
    double v_ego = baseline_info->get_ego_state().ego_vel;
    constexpr double kAcceptableVLoss = 4.0;
    if (is_lane_keep_stage &&
        (traj_check_result.min_obs_lat_dis.distance > kDefaultLatDis &&
         (v_set / (1.e-2 + std::min(v_cruise, v_ego)) > 0.5 ||
          v_set - std::min(v_cruise, v_ego) > -kAcceptableVLoss))) {
      cost = std::numeric_limits<double>::lowest();
      return cost;
    }
    cost += (1.0 - clip(traj_check_result.min_obs_lat_dis.distance,
                        kDefaultLatDis, kMinLatDis) /
                       kDefaultLatDis) *
            kAvoidCutInCost;
    MSD_LOG(INFO, "arbitrator avoid obs cost %f",
            (1.0 - clip(traj_check_result.min_obs_lat_dis.distance,
                        kDefaultLatDis, kMinLatDis) /
                       kDefaultLatDis) *
                kAvoidCutInCost);
  }
  return cost;
}

TrajCollisionCheckInfo ArbitratorCostStrategy::check_trajectory_safety(
    std::shared_ptr<BaseLineInfo> baseline_info,
    std::shared_ptr<ScenarioFacadeContext> context) {
  const auto &planning_result = context->planning_status().planning_result;
  double ini_lon_error = context->planning_status().planning_result.lon_error;
  ini_lon_error = clip(ini_lon_error, 2.0, -2.0);
  constexpr double kCollideDis = -1.0;
  TrajCollisionCheckInfo result;
  double min_obs_dis = std::numeric_limits<double>::max();
  if (planning_result.traj_pose_array.empty()) {
    result.min_obs_dis.distance = kCollideDis;
    return result;
  }
  if (planning_result.traj_vel_array.empty()) {
    result.min_obs_dis.distance = kCollideDis;
    return result;
  }
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    result.min_obs_dis.distance = kCollideDis;
    return result;
  }
  auto &obstacle_manager = baseline_info->obstacle_manager();
  auto &obstacle_decision_manager =
      context->mutable_obstacle_decision_manager();
  std::vector<int> blocking_id;

  double kDeltaT = 0.2;
  constexpr double kMinCheckTime = 0.8;
  double check_time_length;
  bool is_in_lane_change = false;
  bool is_in_lane_change_back = false;
  if (context->planning_status().lane_status.status ==
          LaneStatus::Status::LANE_CHANGE &&
      context->planning_status().lane_status.change_lane.status ==
          ChangeLaneStatus::Status::IN_CHANGE_LANE) {
    check_time_length = 2.5;
    auto gap =
        context->planning_status().lane_status.change_lane.target_gap_obs;
    is_in_lane_change = true;
  } else if (context->planning_status().lane_status.status ==
                 LaneStatus::Status::LANE_CHANGE &&
             context->planning_status().lane_status.change_lane.status ==
                 ChangeLaneStatus::Status::CHANGE_LANE_BACK) {
    is_in_lane_change_back = true;
    check_time_length = 3.0;
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
  double ego_vel = ego_state.planning_init_point.v;
  double theta = baseline_info->get_frenet_coord()->GetRefCurveHeading(
      baseline_info->get_adc_sl_boundary().end_s);
  double ego_frenet_vel = ego_vel * std::cos(ego_state.ego_pose.theta - theta);
  // size_t position_matched_index =
  //     baseline_info->get_ego_state_manager().query_nearst_point_with_buffer(
  //     planning_result.traj_pose_array, ego_state.ego_pose.x,
  //     ego_state.ego_pose.y, 1.0e-6);

  double block_s_end = 200.0;
  std::vector<int> clear_obs_id;
  size_t last_consider_index = 0;
  double min_adc_check_length = std::min(1.0, ConfigurationContext::Instance()
                                                  ->get_vehicle_param()
                                                  .front_edge_to_center);
  const auto &speed_data = planning_result.traj_vel_array;

  double gap_length_before_change_lane{25.0};
  constexpr double kMinFollowTimeBuffer = 0.2;
  double follow_time_buffer = kMinFollowTimeBuffer;
  auto lane_status = PlanningContext::Instance()->planning_status().lane_status;
  if (lane_status.status == LaneStatus::LANE_KEEP ||
      (lane_status.status == LaneStatus::LANE_CHANGE &&
       lane_status.change_lane.status ==
           ChangeLaneStatus::Status::CHANGE_LANE_PREPARATION)) {
    auto target_gap = lane_status.change_lane.target_gap_obs;
    auto follow_obs = obstacle_manager.find_obstacle(target_gap.first);
    auto overtake_obs = obstacle_manager.find_obstacle(target_gap.second);
    if (follow_obs && !follow_obs->IsFrenetInvalid() && overtake_obs &&
        !overtake_obs->IsFrenetInvalid()) {
      gap_length_before_change_lane =
          follow_obs->PerceptionSLBoundary().start_s -
          overtake_obs->PerceptionSLBoundary().end_s;
      follow_time_buffer =
          clip((gap_length_before_change_lane -
                ConfigurationContext::Instance()->get_vehicle_param().length) /
                   25.0,
               1.0, 0.25) *
          0.8;
    } else {
      follow_time_buffer = 0.8;
    }
  }

  // find all obstacles given decisions
  std::vector<int> interested_obstacles;
  std::vector<int> obstacles_at_back_side;
  for (const auto *ptr_obstacle : obstacle_manager.get_obstacles().Items()) {
    auto ptr_obstacle_decision =
        obstacle_decision_manager.find_obstacle_decision(ptr_obstacle->Id());
    const ObjectDecisionType &lat_decision =
        ptr_obstacle_decision->LateralDecision();
    const ObjectDecisionType &lon_decision =
        ptr_obstacle_decision->LongitudinalDecision();
    // @Fixme
    // if (is_in_lane_change) {
    //   // check whether there is cone bucket as leader for lane change
    //   if (ptr_obstacle && ptr_obstacle->Type() == ObjectType::CONE_BUCKET &&
    //   lon_decision.has_follow()) {
    //     if (!ptr_obstacle->IsFrenetInvalid() &&
    //     ptr_obstacle->PerceptionSLBoundary().start_s < 5.0 +
    //     baseline_info->get_adc_sl_boundary().end_s) {
    //       MSD_LOG(INFO, "arbitrator lane change trajectory follow cone
    //       bucket[%d]", ptr_obstacle->Id()); result.min_obs_dis.distance =
    //       kCollideDis; return result;
    //     }
    //   }
    // }

    if (!lat_decision.has_nudge() && !lon_decision.has_stop() &&
        !lon_decision.has_follow() && !lon_decision.has_yield() &&
        !lon_decision.has_overtake()) {
      continue;
    } else {
      interested_obstacles.push_back(ptr_obstacle->Id());
      if (!ptr_obstacle->IsFrenetInvalid()) {
        auto perception_sl_boundary = ptr_obstacle->PerceptionSLBoundary();
        auto ego_sl = ego_state.ego_frenet;
        double theta_refline =
            (perception_sl_boundary.start_l + perception_sl_boundary.end_l >
             2.0 * ego_sl.y)
                ? std::atan2(perception_sl_boundary.end_l - ego_sl.y,
                             perception_sl_boundary.end_s - ego_sl.x)
                : std::atan2(perception_sl_boundary.start_l - ego_sl.y,
                             perception_sl_boundary.end_s - ego_sl.x);
        bool is_in_cone = std::fabs(planning_math::WrapAngle(theta_refline) -
                                    M_PI) < M_PI / 6.0;
        if (is_in_cone) {
          obstacles_at_back_side.push_back(ptr_obstacle->Id());
        }
      }
    }
  }
  std::vector<int> obs_no_need_to_check;
  auto init_adc_sl = baseline_info->get_adc_sl_boundary();
  init_adc_sl.start_s -= ini_lon_error;
  init_adc_sl.end_s -= ini_lon_error;

  int min_traj_size =
      static_cast<int>(std::min(planning_result.traj_pose_array.size(),
                                planning_result.traj_vel_array.size()));

  for (size_t i = 0; i < std::min(num_sample_step, min_traj_size);
       i += planning_step) {
    Point2D cart_pose, frenet_pose;
    cart_pose.x = planning_result.traj_pose_array[i].position_enu.x;
    cart_pose.y = planning_result.traj_pose_array[i].position_enu.y;
    PathPoint adc_point;
    adc_point.x = cart_pose.x;
    adc_point.y = cart_pose.y;
    double cart_yaw = planning_result.traj_pose_array[i].heading_yaw;
    double cur_adc_vel = planning_result.traj_vel_array[i].target_velocity;
    double cur_relative_time = speed_data[i].relative_time;
    double cur_theta = frenet_coord->GetRefCurveHeading(clip(
        baseline_info->get_adc_sl_boundary().end_s + speed_data[i].distance,
        frenet_coord->GetLength(), 0.0));
    double cur_ego_frenet_vel = cur_adc_vel * std::cos(cart_yaw - cur_theta);

    if (std::abs(speed_data[i].distance -
                 speed_data[last_consider_index].distance) <
            min_adc_check_length &&
        last_consider_index != 0 &&
        last_consider_index != num_sample_step - 1 &&
        std::abs(static_cast<int>(i) - static_cast<int>(last_consider_index)) <
            max_index_range) {
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
    adc_sl.start_s = frenet_pose.x - fabs(dx1) - fabs(dx2) - ini_lon_error;
    adc_sl.start_l = frenet_pose.y - fabs(dy1) - fabs(dy2);
    adc_sl.end_s = frenet_pose.x + fabs(dx1) + fabs(dx2) - ini_lon_error;
    adc_sl.end_l = frenet_pose.y + fabs(dy1) + fabs(dy2);

    double relative_time = cur_relative_time;
    auto &path_data = baseline_info->get_path_data();
    double adc_path_s = path_data.discretized_path().QueryMatchedS(adc_point);
    auto lane_border = path_data.getWidth(adc_path_s);
    if (is_in_lane_change && relative_time > kMinCheckTime) {
      // MSD_LOG(INFO, "zzzzz arbitrator finish lc [%f %f] [%f %f] %f",
      // adc_sl.start_l, adc_sl.end_l, lane_border.second, lane_border.first,
      //       (i - position_matched_index) / planning_step * kDeltaT);
      double overlap_dis = std::min(adc_sl.end_l, lane_border.first) -
                           std::max(adc_sl.start_l, lane_border.second);
      double lateral_space =
          std::max(std::max(adc_sl.start_l - lane_border.second, 0.0),
                   std::max(lane_border.first - adc_sl.end_l, 0.0));
      // if (adc_sl.start_l > lane_border.second && adc_sl.end_l <
      // lane_border.first) {
      //   break;
      // }
      constexpr double kOverlapDisRatio = 0.8;
      constexpr double kMinLateralSpace = 0.4;
      if (overlap_dis / (1.e-3 + adc_sl.end_l - adc_sl.start_l) >
              kOverlapDisRatio ||
          lateral_space < kMinLateralSpace) {
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
      bool is_lon_follow = false;
      if (lat_decision.has_nudge()) {
        if (relative_time >= lat_decision.nudge().start_time &&
            relative_time <= lat_decision.nudge().start_time +
                                 lat_decision.nudge().time_buffer) {
          if (ptr_obstacle->PerceptionSLBoundary().end_s < adc_sl.start_s) {
            double consider_time =
                0.5 + (1 - clip((adc_sl.start_s -
                                 ptr_obstacle->PerceptionSLBoundary().end_s) /
                                    3.0,
                                1.0, 0.0)) *
                          2.0;
            if (relative_time < consider_time) {
              is_lat_need_nudge = true;
            }
          } else {
            is_lat_need_nudge = true;
          }
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
          is_lon_follow = true;
        }
      }

      if (!is_lat_need_nudge && !is_lon_need_evade) {
        continue;
      }
      // if (!ptr_obstacle->IsFrenetInvalid() &&
      // ptr_obstacle->PerceptionSLBoundary() < init_adc_sl.start_s - 2.0)
      if (std::find(obstacles_at_back_side.begin(),
                    obstacles_at_back_side.end(),
                    ptr_obstacle->Id()) != obstacles_at_back_side.end()) {
        constexpr double kMaxCheckTimeForBackSide = 1.5;
        if (relative_time > kMaxCheckTimeForBackSide) {
          continue;
        }
      }
      bool is_collide = false;
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
            if (relative_time < 1.0 &&
                adc_sl.start_s > polygon_t.second.max_x() + 0.5 &&
                lateral_space <
                    ptr_obstacle->PerceptionBoundingBox().width() + 0.2) {
              obs_no_need_to_check.push_back(ptr_obstacle->Id());
              MSD_LOG(INFO, "arbitrator traj checker: obstacle[%d] ignored",
                      ptr_obstacle->Id());
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

      TrajectoryPoint traj_point = ptr_obstacle->GetPointAtTime(relative_time);
      PolygonWithT polygon_t;
      bool has_polygon = ptr_obstacle->sl_polygon_seq().EvaluateByTime(
          relative_time, &polygon_t);
      if (has_polygon && adc_sl.start_s < polygon_t.second.max_x() + 0.2 &&
          adc_sl.end_s + 0.2 >
              polygon_t.second.min_x()) { // longitudinal overlap
        double lat_dis;
        lat_dis = std::max(adc_sl.start_l, polygon_t.second.min_y()) -
                  std::min(adc_sl.end_l, polygon_t.second.max_y());
        if (lat_dis < result.min_obs_lat_dis.distance) {
          result.min_obs_lat_dis.distance = lat_dis;
          result.min_obs_lat_dis.obs_id = ptr_obstacle->Id();
          result.min_obs_lat_dis.time = relative_time;
        }
      }

      if (has_polygon && adc_sl.start_l < polygon_t.second.max_y() + 0.1 &&
          adc_sl.end_l + 0.1 > polygon_t.second.min_y()) { // lateral overlap
        double lon_dis = std::max(adc_sl.start_s, polygon_t.second.min_x()) -
                         std::min(adc_sl.end_s, polygon_t.second.max_x());
        if (lon_dis < result.min_obs_lon_dis.distance) {
          result.min_obs_lon_dis.distance = lon_dis;
          result.min_obs_lon_dis.obs_id = ptr_obstacle->Id();
          result.min_obs_lon_dis.time = relative_time;
        }
        if (is_lon_follow && (is_in_lane_change || is_in_lane_change_back) &&
            baseline_info->is_obstacle_intersect_with_lane(ptr_obstacle)) {
          double theta_ref =
              baseline_info->get_frenet_coord()->GetRefCurveHeading(
                  polygon_t.second.min_x());
          double obstacle_vel =
              traj_point.v *
              std::cos(traj_point.velocity_direction - theta_ref);
          constexpr double kMaxFollowBuffer = 5.0;
          double follow_buffer = std::min(
              kMaxFollowBuffer,
              (2 * cur_ego_frenet_vel - obstacle_vel) * follow_time_buffer);
          if (obs_polygon.DistanceTo(adc_polygon) < follow_buffer) {
            MSD_LOG(INFO,
                    "arbitrator traj checker: target lane leader obstacle[%d] "
                    "time %f follow buffer %f lon_dis %f",
                    ptr_obstacle->Id(), relative_time, follow_buffer,
                    polygon_t.second.min_x() - adc_sl.end_s);
            is_collide = true;
            blocking_id.push_back(ptr_obstacle->Id());
            MSD_LOG(INFO,
                    "arbitrator trajectory safety check: target lane leader "
                    "obstacle[%d] collide [%f %f] [%f %f]",
                    ptr_obstacle->Id(), traj_point.path_point.x,
                    traj_point.path_point.y,
                    0.5 * polygon_t.second.min_x() +
                        0.5 * polygon_t.second.max_x(),
                    0.5 * polygon_t.second.min_y() +
                        0.5 * polygon_t.second.max_y());
            result.min_obs_dis.distance = kCollideDis;
            result.min_obs_dis.obs_id = ptr_obstacle->Id();
            result.min_obs_dis.time = relative_time;
            break;
          }
        }
      }

      double init_obs_s = ptr_obstacle->PerceptionSLBoundary().end_s;
      double init_obs_v =
          ptr_obstacle->speed() * std::cos(ptr_obstacle->Yaw_relative_frenet());
      // bool is_obs_on_baseline =  ptr_obstacle->PerceptionSLBoundary().start_l
      // > lane_border.second &&
      //       ptr_obstacle->PerceptionSLBoundary().end_l  < lane_border.first;
      bool is_cur_obs_on_baseline =
          has_polygon ? polygon_t.second.min_y() > lane_border.second &&
                            polygon_t.second.max_y() < lane_border.first
                      : false;
      bool is_lateral_collide =
          has_polygon ? polygon_t.second.max_y() > adc_sl.start_l &&
                            polygon_t.second.min_y() < adc_sl.end_l
                      : false;
      if (is_cur_obs_on_baseline && is_in_lane_change &&
          init_obs_s < init_adc_sl.start_s) {
        constexpr double kTTC = 0.1;
        constexpr double kSoftObsDec = -2.0;
        double obs_frenet_v =
            traj_point.v * std::cos(traj_point.velocity_direction -
                                    frenet_coord->GetRefCurveHeading(
                                        polygon_t.second.max_x()));
        double min_follow_distance = std::min(
            1.0, std::max(obs_frenet_v - cur_ego_frenet_vel, 0.0) * kTTC);
        if (is_lateral_collide &&
            obs_polygon.DistanceTo(adc_polygon) < min_follow_distance &&
            relative_time > 0.1) {
          double invasion_distance = 0.0;
          if (adc_sl.start_l < lane_border.first &&
              adc_sl.end_l > lane_border.second) {
            invasion_distance = std::min(adc_sl.end_l, lane_border.first) -
                                std::max(adc_sl.start_l, lane_border.second);
          }
          constexpr double kMaxInvasionDis = 0.5;
          if (invasion_distance > 0.0) {
            double dec = kSoftObsDec *
                         clip(invasion_distance / kMaxInvasionDis, 1.0, 0.0);
            constexpr double kMinFollowDis = 0.0;
            double min_avoid_s = adc_sl.start_s - kMinFollowDis - init_obs_s;
            if (min_avoid_s > 0) {
              double average_obs_v = min_avoid_s / relative_time;
              double cur_dec =
                  2.0 * (average_obs_v - init_obs_v) / relative_time;
              if (init_obs_v + cur_dec * relative_time > 0.0 && cur_dec > dec) {
                is_collide = false;
              } else {
                MSD_LOG(INFO,
                        "arbitrator trajectory safety check: invasion_distance "
                        "%f dec : %f  %f average_obs_v %f min_avoid_s %f "
                        "relative_time %f init_obs_v %f",
                        invasion_distance, cur_dec, dec, average_obs_v,
                        min_avoid_s, relative_time, init_obs_v);
                is_collide = true;
              }
            }
          } else {
            MSD_LOG(INFO,
                    "arbitrator trajectory safety check: relative distance : "
                    "%f  %f",
                    obs_polygon.DistanceTo(adc_polygon), min_follow_distance);
            is_collide = true;
          }
        }
      } else {
        if (obs_polygon.HasOverlap(adc_polygon)) {
          MSD_LOG(INFO, "arbitrator trajectory safety check: overlap");
          is_collide = true;
        } else {
          min_obs_dis =
              std::min(min_obs_dis, obs_polygon.DistanceTo(adc_polygon));
        }
      }

      if (is_collide) {
        blocking_id.push_back(ptr_obstacle->Id());
        MSD_LOG(INFO,
                "arbitrator trajectory safety check: obstacle[%d] collide [%f "
                "%f] [%f %f]",
                ptr_obstacle->Id(), traj_point.path_point.x,
                traj_point.path_point.y,
                0.5 * polygon_t.second.min_x() + 0.5 * polygon_t.second.max_x(),
                0.5 * polygon_t.second.min_y() +
                    0.5 * polygon_t.second.max_y());
        result.min_obs_dis.distance = kCollideDis;
        result.min_obs_dis.obs_id = ptr_obstacle->Id();
        result.min_obs_dis.time = relative_time;
        break;
      } else {
        // min_obs_dis = std::min(min_obs_dis,
        // obs_polygon.DistanceTo(adc_polygon));
      }
    }

    if (!blocking_id.empty()) {
      MSD_LOG(
          INFO,
          "arbitrator trajectory safety check: blocking s : %f blocking t : %f",
          block_s_end, relative_time);
      MSD_LOG(INFO, "arbitrator ego_sl [%f %f] cart [%f %f]", frenet_pose.x,
              frenet_pose.y, cart_pose.x, cart_pose.y);
      break;
    }
  }
  return result;
}

// get the probability of not collding with probabilistic prediction trajectory
double ArbitratorCostStrategy::unprotected_maneuver_arbitrator_cost(
    const std::shared_ptr<ScenarioFacadeContext> context,
    const std::shared_ptr<WorldModel> world_model) {
  const auto &planning_result = context->planning_status().planning_result;
  const auto &planning_status = context->planning_status();
  auto frenet_lane_id = planning_status.lane_status.target_lane_id;
  auto baseline_info = world_model->get_baseline_info(frenet_lane_id);
  constexpr double kCollideDis = -1.0;
  constexpr double kCollideProb = std::numeric_limits<double>::max();
  double min_obs_dis = std::numeric_limits<double>::max();
  if (planning_result.traj_pose_array.empty()) {
    return 0.0;
  }
  if (planning_result.traj_vel_array.empty()) {
    return 0.0;
  }
  if (baseline_info == nullptr || !baseline_info->is_valid()) {
    return 0.0;
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
  MSD_LOG(INFO, "arbitrator probability check_time_length %f",
          check_time_length);
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
  size_t position_matched_index =
      baseline_info->get_ego_state_manager().query_nearst_point_with_buffer(
          planning_result.traj_pose_array, ego_state.ego_pose.x,
          ego_state.ego_pose.y, 1.0e-6);

  double block_s_end = 200.0;
  std::vector<int> clear_obs_id;
  size_t last_consider_index = 0;
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
  double prob_of_collide = 0.0;
  double prob_of_collide_cur_time = 0.0;
  double prob_of_collide_with_obs_cur_time = 0.0;
  constexpr double kGamma = 0.99;

  for (size_t i = position_matched_index;
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

    prob_of_collide_cur_time = 0.0;
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
                    ptr_obstacle->PerceptionBoundingBox().width() + 0.2) {
              obs_no_need_to_check.push_back(ptr_obstacle->Id());
              continue;
            }
          }
        }
      }

      planning_math::Polygon2d obs_polygon;
      planning_math::Vec2d obs_center;
      double obs_std1{0.0}, obs_std2{0.0};
      if (ptr_obstacle->IsStatic() || ptr_obstacle->Trajectory().empty()) {
        obs_polygon = ptr_obstacle->PerceptionPolygon();
        obs_center = ptr_obstacle->PerceptionBoundingBox().center();
      } else {
        TrajectoryPoint traj_point =
            ptr_obstacle->GetPointAtTime(relative_time);
        obs_polygon = ptr_obstacle->GetPolygonAtPoint(traj_point);
        obs_center.set_x(traj_point.path_point.x);
        obs_center.set_y(traj_point.path_point.y);
        obs_std1 = traj_point.sigma_x;
        obs_std2 = traj_point.sigma_y;
      }
      obs_std1 = std::max(0.05, obs_std1);
      obs_std2 = std::max(0.05, obs_std2);
      if (obs_polygon.points().empty() || !obs_polygon.is_convex()) {
        continue;
      }
      double dis_to_obs{0.0};
      if (obs_polygon.HasOverlap(adc_polygon)) {
        blocking_id.push_back(ptr_obstacle->Id());
        TrajectoryPoint traj_point =
            ptr_obstacle->GetPointAtTime(relative_time);
        PolygonWithT polygon_t;
        (void)ptr_obstacle->sl_polygon_seq().EvaluateByTime(relative_time,
                                                            &polygon_t);
        // MSD_LOG(INFO, "arbitrator trajectory safety check: obstacle[%d]
        // collide [%f %f] [%f %f]", ptr_obstacle->Id(),
        //     traj_point.path_point.x, traj_point.path_point.y, 0.5 *
        //     polygon_t.second.min_x() + 0.5 * polygon_t.second.max_x(), 0.5 *
        //     polygon_t.second.min_y() + 0.5 * polygon_t.second.max_y());
        prob_of_collide_with_obs_cur_time = 1.0;
        dis_to_obs = 0.0;
      } else {
        dis_to_obs = obs_polygon.DistanceTo(adc_polygon);
      }

      planning_math::Vec2d adc_center{adc_point.x, adc_point.y};
      double center_dis_to_obs =
          std::max(0.1, adc_center.DistanceTo(obs_center));
      planning_math::Vec2d projection_vector =
          (adc_center - obs_center) * (dis_to_obs / center_dis_to_obs);
      prob_of_collide_with_obs_cur_time = planning_math::Gaussian2d(
          0.0, 0.0, obs_std1, obs_std2, projection_vector.x(),
          projection_vector.y(), 0.0);
      // MSD_LOG(INFO, "arbitrator adc_center[%f %f] obs_center[%d %f %f %f]
      // dis_to_obs %f std1 %f std 2 %f x %f y %f prob %f", adc_center.x(),
      // adc_center.y(),
      //   ptr_obstacle->Id(), relative_time, obs_center.x(), obs_center.y(),
      //   dis_to_obs, obs_std1, obs_std2, projection_vector.x(),
      //   projection_vector.y(), prob_of_collide_with_obs_cur_time);
      prob_of_collide_cur_time +=
          ptr_obstacle->Prob() * prob_of_collide_with_obs_cur_time;
    }

    prob_of_collide += std::pow(kGamma, (i - position_matched_index)) *
                       prob_of_collide_cur_time;
  }
  return prob_of_collide;
}

} // namespace msquare
