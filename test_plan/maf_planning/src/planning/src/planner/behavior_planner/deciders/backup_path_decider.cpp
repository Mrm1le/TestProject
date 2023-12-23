#include "planner/behavior_planner/deciders/backup_path_decider.h"
#include "common/config_context.h"
#include "common/math/vec2d.h"
#include "common/planning_context.h"
#include "planning/common/common.h"

namespace msquare {

BackUpPathDecider::BackUpPathDecider(const TaskConfig &config)
    : Decider(config) {
  // mph_assert(config.has_backup_path_decider_config());
}

void BackUpPathDecider::init(std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}

void BackUpPathDecider::reset(const TaskConfig &config) { Task::reset(config); }

void BackUpPathDecider::clear_longitudinal_decisions() {
  // clear longitudinal decisions
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  for (auto *ptr_obstacle_decision :
       obstacle_decision_manager.get_obstacles_decision().Items()) {
    if (ptr_obstacle_decision->HasLongitudinalDecision()) {
      auto mutable_ptr_obstacle_decision =
          obstacle_decision_manager.find_obstacle_decision(
              ptr_obstacle_decision->Id());
      if (mutable_ptr_obstacle_decision != nullptr) {
        mutable_ptr_obstacle_decision->ClearLongitudinalDecision();
      }
    }
  }
}

void BackUpPathDecider::unset() {}

void BackUpPathDecider::clear_lateral_decisions() {
  // clear lateral decisions
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  for (auto *ptr_obstacle_decision :
       obstacle_decision_manager.get_obstacles_decision().Items()) {
    if (ptr_obstacle_decision->HasLateralDecision()) {
      auto mutable_ptr_obstacle_decision =
          obstacle_decision_manager.find_obstacle_decision(
              ptr_obstacle_decision->Id());
      if (mutable_ptr_obstacle_decision != nullptr) {
        mutable_ptr_obstacle_decision->ClearLateralDecision();
      }
    }
  }
}

bool BackUpPathDecider::check_trajectory_safety(
    const PlanningResult &planning_result,
    std::shared_ptr<BaseLineInfo> baseline_info,
    std::shared_ptr<ScenarioFacadeContext> context) {
  if (planning_result.traj_pose_array.empty()) {
    return false;
  }
  if (planning_result.traj_vel_array.empty()) {
    return false;
  }
  auto &obstacle_manager = baseline_info->obstacle_manager();
  auto &obstacle_decision_manager =
      context->mutable_obstacle_decision_manager();
  std::vector<int> blocking_id;

  double kDeltaT = 0.2;
  int planning_step = static_cast<int>(kDeltaT / FLAGS_trajectory_density);
  int num_planning_step =
      static_cast<int>(FLAGS_trajectory_time_length / kDeltaT);
  int num_sample_step =
      static_cast<int>(FLAGS_trajectory_time_length / FLAGS_trajectory_density);
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
  double min_adc_check_length = std::min(2.0, ConfigurationContext::Instance()
                                                  ->get_vehicle_param()
                                                  .front_edge_to_center);
  const auto &last_speed_data = planning_result.traj_vel_array;

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

  for (int i = position_matched_index;
       i <
       std::min(num_sample_step, (int)planning_result.traj_pose_array.size());
       i += planning_step) {
    Point2D cart_pose, frenet_pose;
    cart_pose.x = planning_result.traj_pose_array[i].position_enu.x;
    cart_pose.y = planning_result.traj_pose_array[i].position_enu.y;
    double cart_yaw = planning_result.traj_pose_array[i].heading_yaw;

    if (std::abs(last_speed_data[i].distance -
                 last_speed_data[last_consider_index].distance) <
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

    /* TODO consider all corners
    std::array<Point2D, 4> adc_corners;
    adc_corners[0].x = frenet_pose.x + dx1 - dx2;
    adc_corners[0].y = frenet_pose.y + dy1 + dy2;
    adc_corners[1].x = frenet_pose.x + dx1 + dx2;
    adc_corners[1].y = frenet_pose.y + dy1 - dy2;
    adc_corners[2].x = frenet_pose.x - dx1 + dx2;
    adc_corners[2].y = frenet_pose.y - dy1 - dy2;
    adc_corners[3].x = frenet_pose.x - dx1 - dx2;
    adc_corners[3].y = frenet_pose.y - dy1 + dy2;
    */

    constexpr double KLonBuffer = 1.0;
    constexpr double KLatBuffer = 0.3;
    SLBoundary adc_sl;
    adc_sl.start_s = frenet_pose.x - fabs(dx1) - fabs(dx2) - KLonBuffer;
    adc_sl.start_l = frenet_pose.y - fabs(dy1) - fabs(dy2) - KLatBuffer;
    adc_sl.end_s = frenet_pose.x + fabs(dx1) + fabs(dx2) + KLonBuffer;
    adc_sl.end_l = frenet_pose.y + fabs(dy1) + fabs(dy2) + KLatBuffer;
    block_s_end = adc_sl.end_s;
    planning_math::Vec2d adc_box_center(cart_pose.x, cart_pose.y);
    planning_math::Box2d adc_box(
        adc_box_center, cart_yaw,
        ConfigurationContext::Instance()->get_vehicle_param().length + 0.2,
        ConfigurationContext::Instance()->get_vehicle_param().width + 0.2);
    planning_math::Polygon2d adc_polygon(adc_box);

    for (auto obs_id : interested_obstacles) {
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
      double relative_time =
          (i - position_matched_index) / planning_step * kDeltaT;
      bool is_lat_need_nudge = false;
      bool is_lon_need_evade = false;
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
        MSD_LOG(INFO, "trajectory safety check: obstacle[%d] collide",
                ptr_obstacle->Id());
        break;
      }
    }

    if (!blocking_id.empty()) {
      MSD_LOG(INFO, "trajectory safety check: blocking s : %f", block_s_end);
      break;
    }
  }

  if (!blocking_id.empty()) {
    return false;
  }
  return true;
}

bool BackUpPathDecider::check_last_published_trajectory() {
  const auto &pre_planning_result =
      PlanningContext::Instance()->planning_status().pre_planning_result;
  if (pre_planning_result.traj_pose_array.empty()) {
    clear_longitudinal_decisions();
    clear_lateral_decisions();
    return false;
  }

  auto &obstacle_manager = baseline_info_->obstacle_manager();
  auto &obstacle_decision_manager =
      context_->mutable_obstacle_decision_manager();
  std::vector<int> blocking_id;

  double kDeltaT = 0.2;
  int planning_step = static_cast<int>(kDeltaT / FLAGS_trajectory_density);
  int num_planning_step =
      static_cast<int>(FLAGS_trajectory_time_length / kDeltaT);
  int num_sample_step =
      static_cast<int>(FLAGS_trajectory_time_length / FLAGS_trajectory_density);
  int max_index_range = static_cast<int>(kDeltaT / FLAGS_trajectory_density);

  std::shared_ptr<FrenetCoordinateSystem> frenet_coord =
      baseline_info_->get_frenet_coord();
  EgoState ego_state = baseline_info_->get_ego_state();
  int position_matched_index =
      baseline_info_->get_ego_state_manager().query_nearst_point_with_buffer(
          pre_planning_result.traj_pose_array, ego_state.ego_pose.x,
          ego_state.ego_pose.y, 1.0e-6);

  double block_s_end = 200.0;
  std::vector<int> clear_obs_id;
  int last_consider_index = 0;
  double min_adc_check_length = std::min(2.0, ConfigurationContext::Instance()
                                                  ->get_vehicle_param()
                                                  .front_edge_to_center);
  const auto &last_speed_data = pre_planning_result.traj_vel_array;
  for (int i = position_matched_index;
       i < std::min(num_sample_step,
                    (int)pre_planning_result.traj_pose_array.size());
       i += planning_step) {
    Point2D cart_pose, frenet_pose;
    cart_pose.x = pre_planning_result.traj_pose_array[i].position_enu.x;
    cart_pose.y = pre_planning_result.traj_pose_array[i].position_enu.y;

    if (std::abs(last_speed_data[i].distance -
                 last_speed_data[last_consider_index].distance) <
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
        pre_planning_result.traj_pose_array[i].heading_yaw - theta_ref;

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

    /* TODO consider all corners
    std::array<Point2D, 4> adc_corners;
    adc_corners[0].x = frenet_pose.x + dx1 - dx2;
    adc_corners[0].y = frenet_pose.y + dy1 + dy2;
    adc_corners[1].x = frenet_pose.x + dx1 + dx2;
    adc_corners[1].y = frenet_pose.y + dy1 - dy2;
    adc_corners[2].x = frenet_pose.x - dx1 + dx2;
    adc_corners[2].y = frenet_pose.y - dy1 - dy2;
    adc_corners[3].x = frenet_pose.x - dx1 - dx2;
    adc_corners[3].y = frenet_pose.y - dy1 + dy2;
    */

    constexpr double KLonBuffer = 1.0;
    constexpr double KLatBuffer = 0.3;
    SLBoundary adc_sl;
    adc_sl.start_s = frenet_pose.x - fabs(dx1) - fabs(dx2) - KLonBuffer;
    adc_sl.start_l = frenet_pose.y - fabs(dy1) - fabs(dy2) - KLatBuffer;
    adc_sl.end_s = frenet_pose.x + fabs(dx1) + fabs(dx2) + KLonBuffer;
    adc_sl.end_l = frenet_pose.y + fabs(dy1) + fabs(dy2) + KLatBuffer;
    block_s_end = adc_sl.end_s;

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
      }

      double relative_time =
          (i - position_matched_index) / planning_step * kDeltaT;
      SLBoundary sl_boundary;
      if (ptr_obstacle->has_sl_polygon_seq()) {
        auto sl_polygon_seq = ptr_obstacle->sl_polygon_seq();
        PolygonWithT cur_sl_polygon_t;
        if (sl_polygon_seq.EvaluateByTime(relative_time, &cur_sl_polygon_t)) {
          sl_boundary.start_s = cur_sl_polygon_t.second.min_x();
          sl_boundary.end_s = cur_sl_polygon_t.second.max_x();
          sl_boundary.start_l = cur_sl_polygon_t.second.min_y();
          sl_boundary.end_l = cur_sl_polygon_t.second.max_y();
        }
      } else {
        TrajectoryPoint traj_point =
            ptr_obstacle->GetPointAtTime(relative_time);
        sl_boundary = ptr_obstacle->GetSLBoundary(frenet_coord, traj_point);
      }

      if (sl_boundary.start_s > adc_sl.end_s ||
          sl_boundary.end_s < adc_sl.start_s) {
        continue;
      } else {
        if (sl_boundary.start_l > adc_sl.end_l ||
            sl_boundary.end_l < adc_sl.start_l) {
          continue;
        } else {
          blocking_id.push_back(ptr_obstacle->Id());
          if (lat_decision.has_nudge()) {
            MSD_LOG(INFO, "clear lat decision id : %d", ptr_obstacle->Id());
            clear_obs_id.push_back(ptr_obstacle->Id());

            ObjectDecisionType nudge_decision;
            auto mutable_nudge_decision = nudge_decision.mutable_nudge();
            mutable_nudge_decision->is_longitunidal_ignored = false;
            mutable_nudge_decision->start_time = 0.0;
            mutable_nudge_decision->time_buffer = 0.0;
            mutable_nudge_decision->type = lat_decision.nudge().type;
            mutable_nudge_decision->distance_l =
                lat_decision.nudge().distance_l;
            nudge_decision.setDecisionSource("backup_decider");
            (void)obstacle_decision_manager.add_lateral_decision(
                "backup_decider", ptr_obstacle->Id(), nudge_decision);
          }
          auto &lateral_output =
              context_->mutable_lateral_behavior_planner_output();
          for (auto &offset : lateral_output.s_r_offset) {
            if (std::get<0>(offset) > frenet_pose.x) {
              std::get<1>(offset) = 0;
            }
          }
          break;
        }
      }
    }

    if (!blocking_id.empty()) {
      MSD_LOG(INFO, "back up blocking s : %f", block_s_end);
      break;
    }
  }

  if (!blocking_id.empty()) {
    for (const auto *ptr_obstacle : obstacle_manager.get_obstacles().Items()) {
      auto ptr_obstacle_decision =
          obstacle_decision_manager.find_obstacle_decision(ptr_obstacle->Id());
      if (!ptr_obstacle_decision->LateralDecision().has_nudge()) {
        continue;
      }
      TrajectoryPoint traj_init_point = ptr_obstacle->GetPointAtTime(0.0);
      SLBoundary sl_boundary =
          ptr_obstacle->GetSLBoundary(frenet_coord, traj_init_point);
      if (sl_boundary.start_s > block_s_end) {
        MSD_LOG(INFO, "clear lat decision id : %d", ptr_obstacle->Id());
        auto mutable_ptr_obstacle_decision =
            obstacle_decision_manager.find_obstacle_decision(
                ptr_obstacle_decision->Id());
        mutable_ptr_obstacle_decision->ClearLateralDecision();
      }
    }
    clear_longitudinal_decisions();
    context_->mutable_planning_status()->planning_result.id_clear.assign(
        clear_obs_id.begin(), clear_obs_id.end());
    return false;
  }
  clear_longitudinal_decisions();
  context_->mutable_planning_status()->planning_result.id_clear.assign(
      clear_obs_id.begin(), clear_obs_id.end());
  return true;
}

TaskStatus BackUpPathDecider::process() {
  MLOG_PROFILING(name_.c_str());
  if (world_model_ == nullptr || !baseline_info_ ||
      !baseline_info_->is_valid()) {
    MSD_LOG(INFO, "world model is none!");
    clear_longitudinal_decisions();
    clear_lateral_decisions();
    return TaskStatus::STATUS_FAILED;
  }

  context_->mutable_planning_status()->scheme_stage = SchemeStage::SECONDARY;
  bool check_last_plan = check_last_published_trajectory();

  if (check_last_plan) {
    if (PlanningContext::Instance()
            ->planning_status()
            .backup_consecutive_loops <= kBackUpFreezeLoop) {
      MSD_LOG(WARN, "publish last Planning Message!");
      context_->mutable_planning_status()->planning_result =
          PlanningContext::Instance()->planning_status().pre_planning_result;
      context_->mutable_planning_status()->planning_success = true;
      ++(context_->mutable_planning_status()->backup_consecutive_loops);
      return TaskStatus::STATUS_SUCCESS_BREAK;
    } else {
      context_->mutable_planning_status()->scheme_stage = SchemeStage::BACKUP;
      context_->mutable_planning_status()->backup_reason = "OverLoop";
      clear_lateral_decisions();
      return TaskStatus::STATUS_SUCCESS;
    }
  }
  context_->mutable_planning_status()->scheme_stage = SchemeStage::BACKUP;
  context_->mutable_planning_status()->backup_reason = "ObsBlock";
  return TaskStatus::STATUS_SUCCESS;
}

} // namespace msquare
