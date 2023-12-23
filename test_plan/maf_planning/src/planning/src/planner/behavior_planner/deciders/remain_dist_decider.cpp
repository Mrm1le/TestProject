#include "planner/behavior_planner/deciders/remain_dist_decider.h"

#include "common/sbp_obstacle_point.h"

#include "common/math/math_utils.h"
#include "planning/common/common.h"

namespace msquare {
namespace parking {

namespace {
constexpr int kSafeUssId = -1;
constexpr double kSafeRemainDistance = 1.2;
constexpr int kDangerUssId = 9;
constexpr double kConsiderPathDistance = 2.5;

constexpr double kOutSideSlotThreshold = 5.0; // 3.5
constexpr double kHighSpeedThreshold = 0.4;
constexpr double kHighKappaThreshold = 0.1;
} // namespace

RemainDistDecider::RemainDistDecider(
    const std::shared_ptr<WorldModel> &world_model) {
  world_model_ = world_model;
}

bool RemainDistDecider::MakeDecision(
    const EgoModelManager &ego_model, const bool is_moved,
    FreespacePoint *const ptr_lead_point,
    std::vector<std::vector<std::pair<double, double>>>
        *const ptr_vec_debug_sl_points,
    double *const ptr_side_safe_threshold,
    OutHoleParam *const ptr_out_hole_param) {
  // 1. init
  out_hole_param_ = ptr_out_hole_param;
  // 1. add latitude buff for special case
  std::string debug_string = ", [rm] ";
  bool is_reverse =
      (PlanningContext::Instance()->planning_status().planning_result.gear ==
       GearState::REVERSE);
  // 1.1. add extra latitude buff in turnning scene
  double turnning_extra_buffer = 0.0;
  const auto &mpc_trajectory = PlanningContext::Instance()
                                   ->longitudinal_behavior_planner_output()
                                   .refactored_mpc_trajectory;
  if (!mpc_trajectory.empty()) {
    const auto &ego_pose = mpc_trajectory.at(0);
    auto aheading_pose = mpc_trajectory.at(0);
    if (mpc_trajectory.size() > 10) {
      aheading_pose = mpc_trajectory.at(10);
    }
    const auto &parking_slot = PlanningContext::Instance()
                                   ->mutable_parking_behavior_planner_output()
                                   ->parking_slot_info;
    planning_math::LineSegment2d slot_center_line = parking_slot.center_line;
    planning_math::Vec2d pseudo_target_slot = slot_center_line.start();
    planning_math::Vec2d vec2d_ego_pose(ego_pose.x, ego_pose.y, ego_pose.theta);
    bool is_outside_slot = slot_center_line.unit_direction().InnerProd(
                               vec2d_ego_pose - pseudo_target_slot) > kOutSideSlotThreshold
                               ? true
                               : false;

    if (is_moved && std::abs(aheading_pose.kappa()) > kHighKappaThreshold &&
        std::abs(world_model_->get_ego_state().ego_vel) > kHighSpeedThreshold && 
        is_outside_slot) {
      turnning_extra_buffer = 0.05;
      debug_string +=
          ",kap" + std::to_string(aheading_pose.kappa()).substr(0, 5);
    }
  }

  const double risky_lat =
      CarParams::GetInstance()->lat_inflation() * 0.5 + turnning_extra_buffer;
  const double secure_lat = CarParams::GetInstance()->lat_inflation() * 0.9;
  const double risky_thres = std::max(risky_lat, 0.05);
  const double secure_thres = std::max(secure_lat, 0.05);
  const int zigzag_num =
      PlanningContext::Instance()->planning_status().zigzag_num;
  bool use_secure_absolutely =
      (!is_reverse && zigzag_num < 1); // obstacle type extend in add point
  double side_safe_threshold = risky_thres;
  if (use_secure_absolutely) {
    side_safe_threshold = secure_thres;
  }
  *ptr_side_safe_threshold = side_safe_threshold;
  debug_string += ",side_buf" + std::to_string(side_safe_threshold).substr(0, 5) + ", ";

  // 2. constuct grid map
  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_lot == nullptr) {
    return false;
  }
  double pilliar_extra_threshold = secure_lat - risky_lat;
  mc_footprint_model_ = PlanningContext::Instance()->lon_mc_footprint_model();
  bool status = constructGridMap(ego_model, pilliar_extra_threshold);
  if (!status)
    return false;

  // 3. get sl
  bool is_ego_pose_collision = false, is_plan_path_collision = false,
       is_mpc_collision = false;

  std::pair<double, double> danger_sl = std::make_pair(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
  bool is_checking_plan_path = true;
  if (!is_moved) {
    // ego pose
    std::vector<std::pair<double, double>> ego_sl_points;
    std::pair<double, double> ego_danger_sl = std::make_pair(
        std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    Pose2DTrajectory plan_path;
    plan_path.emplace_back(world_model_->get_ego_state().ego_pose);
    is_ego_pose_collision = getSlBoundary(
        plan_path, is_reverse, is_checking_plan_path, side_safe_threshold,
        &ego_sl_points, &ego_danger_sl, &debug_string);
    debug_string += "[not_move]";
    if (is_ego_pose_collision) {
      danger_sl = ego_danger_sl;
      debug_string += "_coll";
    }

    // for debug plot
    // plan path
    std::vector<std::pair<double, double>> plan_path_sl_points;
    std::pair<double, double> plan_path_danger_sl = std::make_pair(
        std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    // TODO(ckl): clear plan path
    const Pose2DTrajectory &debug_plan_path =
        PlanningContext::Instance()
            ->longitudinal_behavior_planner_output()
            .plan_path; // use no extend plan path
    // const Pose2DTrajectory plan_path;

    is_checking_plan_path = true;
    is_plan_path_collision = getSlBoundary(
        debug_plan_path, is_reverse, is_checking_plan_path, side_safe_threshold,
        &plan_path_sl_points, &plan_path_danger_sl, &debug_string);
    // hack vec_sl for debug visualize
    if (!ego_sl_points.empty())
      // second + 0.1 is hack number
      ego_sl_points.emplace_back(ego_sl_points.at(0).first + 0.1, ego_sl_points.at(0).second + 0.1);
    ptr_vec_debug_sl_points->emplace_back(ego_sl_points);
    ptr_vec_debug_sl_points->emplace_back(plan_path_sl_points);
  } else {
    // plan path
    std::vector<std::pair<double, double>> plan_path_sl_points;
    std::pair<double, double> plan_path_danger_sl = std::make_pair(
        std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    // TODO(ckl): clear plan path
    const Pose2DTrajectory &plan_path =
        PlanningContext::Instance()
            ->longitudinal_behavior_planner_output()
            .plan_path; // use no extend plan path
    // const Pose2DTrajectory plan_path;

    is_checking_plan_path = true;
    is_plan_path_collision = getSlBoundary(
        plan_path, is_reverse, is_checking_plan_path, side_safe_threshold,
        &plan_path_sl_points, &plan_path_danger_sl, &debug_string);

    // mpc_trajectory
    std::vector<std::pair<double, double>> mpc_sl_points;
    std::pair<double, double> mpc_danger_sl = std::make_pair(
        std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    const Pose2DTrajectory &mpc_trajectory =
        PlanningContext::Instance()
            ->longitudinal_behavior_planner_output()
            .refactored_mpc_trajectory;
    is_checking_plan_path = false;
    is_mpc_collision = getSlBoundary(
        mpc_trajectory, is_reverse, is_checking_plan_path, side_safe_threshold,
        &mpc_sl_points, &mpc_danger_sl, &debug_string);

    ptr_vec_debug_sl_points->emplace_back(mpc_sl_points);
    ptr_vec_debug_sl_points->emplace_back(plan_path_sl_points);

    if (is_plan_path_collision || is_mpc_collision) {
      if (plan_path_danger_sl.first < mpc_danger_sl.first) {
        danger_sl = plan_path_danger_sl;
      } else {
        danger_sl = mpc_danger_sl;
      }
    }
  }

  // // 4. get remain distance
  // // TODO: can de deleted ?
  // ObjectDecisionType object_decision;
  // auto mutable_lead_decision = object_decision.mutable_lead();
  // mutable_lead_decision->distance_s = result.s;
  // mutable_lead_decision->min_distance = result.min_distance;
  // mutable_lead_decision->is_collision = result.is_collision;
  // mutable_lead_decision->type = ObjectType::NOT_KNOW;
  // point_ptr->AddParkingLongitudinalDecision("point_decision",
  //                                           object_decision);

  ptr_lead_point->d_rel =
      std::numeric_limits<double>::max(); // TODO(): need use this?
  if (is_ego_pose_collision || is_plan_path_collision || is_mpc_collision) {
    ptr_lead_point->id = kDangerUssId;
    ptr_lead_point->d_rel = danger_sl.first;
    ptr_lead_point->d_path = danger_sl.second;
    ptr_lead_point->s = danger_sl.first;
    ptr_lead_point->l = danger_sl.second;
    ptr_lead_point->is_collision = true;
    // TODO(): add danger obstacle xy
    // ptr_lead_point->x = obs.x;
    if (is_ego_pose_collision) {
      ptr_lead_point->collision_type = CollisionType::MPC;
    } else {
      if (is_plan_path_collision && is_mpc_collision) {
        ptr_lead_point->collision_type = CollisionType::BOTH;
      } else if (is_plan_path_collision) {
        ptr_lead_point->collision_type = CollisionType::PLANPATH;
      } else {
        ptr_lead_point->collision_type = CollisionType::MPC;
      }
    }
  } else {
    ptr_lead_point->id = kSafeUssId;
    ptr_lead_point->d_rel = kSafeRemainDistance;
    ptr_lead_point->d_path = kSafeRemainDistance;
    ptr_lead_point->s = kSafeRemainDistance;
    ptr_lead_point->l = kSafeRemainDistance;
    ptr_lead_point->is_collision = false;
    ptr_lead_point->collision_type = CollisionType::COLLISIONFREE;
  }

    // special case
  if (!is_moved)
    GetOutHole(
        *PlanningContext::Instance()->mutable_is_planner_update_plan_path(),
        *ptr_vec_debug_sl_points, side_safe_threshold, ptr_lead_point,
        PlanningContext::Instance()->mutable_planning_debug_info());

  *PlanningContext::Instance()->mutable_planning_debug_info() += debug_string;
  return true;
}

bool RemainDistDecider::GetOutHole(
    const bool is_planner_update_plan_path,
    const std::vector<std::vector<std::pair<double, double>>>
        &vec_debug_sl_points, const double side_safe_threshold, 
    FreespacePoint *const ptr_lead_point, string *const ptr_debug_str) {
  // counter
  bool is_getting_out_hole_duration = false;
  if (is_planner_update_plan_path) {
    ++out_hole_param_->getting_out_count;
  }
  if (out_hole_param_->getting_out_count > 0) {
    if (++out_hole_param_->getting_out_count <
        out_hole_param_->kGettingOutHoldCountThreshold) {
      is_getting_out_hole_duration = true;
    } else {
      out_hole_param_->getting_out_count = 0;
    }
  }

  // checkout plan path and mpc path
  if (vec_debug_sl_points.size() < 2)
    return true;
  bool is_sl_rising = true;
  for (size_t i = 0; i < 2; ++i) {
    const auto &sl_points = vec_debug_sl_points.at(i);

    // each path point donot too danger
    for (size_t j = 0; j < sl_points.size(); ++j) {
      // only check 0.3m path
      if (sl_points.at(j).first > out_hole_param_->kCheckingRisingPathLength)
        break;
      if (sl_points.at(j).second < out_hole_param_->kSafeLateralThreshold) {
        is_sl_rising = false;
        break;
      }
    }

    if (sl_points.size() < 2) {
      is_sl_rising = false;
      return true;
    }
    // obstacle lateral dist need rising thend
    for (size_t j = 1; j < sl_points.size(); ++j) {
      // only check 0.3m path
      if (sl_points.at(j - 1).first >
          out_hole_param_->kCheckingRisingPathLength)
        break;
      if (!((sl_points.at(j).second - sl_points.at(j - 1).second) >
            out_hole_param_->kSafeLateralDeltaDistThreshold)) {
        is_sl_rising = false;
        break;
      }
      // need get out danger threshold in one step
      else if ((sl_points.at(j).second - sl_points.at(j - 1).second) <
                     out_hole_param_->kSafeLateralDeltaDistThreshold &&
                 sl_points.at(j).second < side_safe_threshold) {
        is_sl_rising = false;
        break;
      }
    }
  }

  if (out_hole_param_->getting_out_count) {
    *ptr_debug_str += "\n,hole(" + std::to_string(out_hole_param_->getting_out_count) + ","
       + std::to_string(is_getting_out_hole_duration) + "," + std::to_string(is_sl_rising) + ") ";
  }

  const auto &mpc_sl_points = vec_debug_sl_points.at(0);
  if (mpc_sl_points.at(0).second < side_safe_threshold &&
      is_getting_out_hole_duration && is_sl_rising &&
      std::abs(world_model_->get_ego_state().ego_vel) <
          out_hole_param_->kSlowEgoVel) {
    ptr_lead_point->d_rel =
        out_hole_param_->kSafeRemainDist; // hack moving d_ref

    *ptr_debug_str += ",OUT_HOLE! ";
  }
  return true;
}

bool RemainDistDecider::constructGridMap(const EgoModelManager &ego_model,
                                         const double pilliar_extra) {
  // init grid map prepare
  if (PlanningContext::Instance()
          ->parking_behavior_planner_output()
          .parking_lot == nullptr) {
    return false;
  }
  const auto &slot_box = PlanningContext::Instance()
                             ->parking_behavior_planner_output()
                             .parking_lot->getBox();

  grid::HAS2SearchNode end_node(grid::HAS2SearchNode::PathType::LINE,
                                slot_box.center().x(), slot_box.center().y(),
                                slot_box.heading());
  // end of init

  auto last_time = MTIME()->timestamp().ms();
  //// parameter
  double deviation = 0.0;
  bool is_reverse =
      (PlanningContext::Instance()->planning_status().planning_result.gear ==
       GearState::REVERSE);
  if (is_reverse) {
    deviation = 0.0;
  } else {
    deviation = VehicleParam::Instance()->front_edge_to_center;
  }
  const double map_size = 10.5;
  const double obs_to_ego_threshold_square = map_size * 0.5 * map_size * 0.5;
  const Pose2D &ego_pose = world_model_->get_ego_state().ego_pose;
  const double rectify_x = ego_pose.x + std::cos(ego_pose.theta) * deviation;
  const double rectify_y = ego_pose.y + std::sin(ego_pose.theta) * deviation;

  double grid_resolution = 0.025;
  // max error caused by grid
  double extra_inflation = 0.025 * std::sqrt(2) * 0.5 + 0.02; // 0.0353553
  int roundedNumber = static_cast<int>(std::round(map_size / 0.025));
  // 5. get obstacle
  const auto &obs_tmp = world_model_->obstacle_manager().get_points().Items();
  vector<Obstacle> vec_pillar_origin;
  std::vector<planning_math::Vec2d> vec_high_obs;
  std::vector<planning_math::Vec2d> vec_low_obs;
  vec_high_obs.reserve(400);
  vec_low_obs.reserve(400);
  for (const auto &pt : obs_tmp) {
    double obs_to_ego_square = planning_math::sum_square(
        pt->point().x - rectify_x, pt->point().y - rectify_y);
    if (obs_to_ego_square < obs_to_ego_threshold_square) {
      if (pt->point().z < ego_model.chassis_height()) {
        vec_low_obs.emplace_back(pt->point().x, pt->point().y);
      } else {
        vec_high_obs.emplace_back(pt->point().x, pt->point().y);
      }
    }
    if (pt->Type() == ObjectType::FREESPACE) {
      vec_pillar_origin.emplace_back(*pt);
    }
  }
  std::vector<SbpObstaclePtr> obs_ptrs;
  obs_ptrs.emplace_back(std::make_shared<SbpObstaclePoint>(vec_high_obs));
  // add low obstacle
  SbpObstaclePtr low_sbp_obstacle =
      std::make_shared<SbpObstaclePoint>(vec_low_obs);
  low_sbp_obstacle->setHeightType(ObstacleHeightType::LOW);
  obs_ptrs.emplace_back(low_sbp_obstacle);

  // extra pillar obstacles near slot
  std::vector<planning_math::Vec2d> extra_pillar_pts;
  if (!vec_pillar_origin.empty()) {
    getExtraPillarObstacle(pilliar_extra, vec_pillar_origin,
                           &extra_pillar_pts); // pilliar_extra
    obs_ptrs.push_back(std::make_shared<SbpObstaclePoint>(extra_pillar_pts));
  }

  // 6. run grid
  obs_grid_ = std::make_shared<grid::ObstacleGrid>();
  obs_grid_->init(roundedNumber, roundedNumber, grid_resolution,
                  rectify_x - (map_size / 2), rectify_y - (map_size / 2));
  obs_grid_->constructFromSbpObstacleMultiHeights(
      obs_ptrs, *mc_footprint_model_, end_node, extra_inflation,
      extra_inflation);
  auto time_stamp = MTIME()->timestamp().ms() - last_time;
  std::string debug_string;
  debug_string += "\n[vp]con_grid=" + std::to_string(time_stamp).substr(0, 5)
     + "ms,origin_pt=" + std::to_string(obs_tmp.size()) + ",grid_pt=" + std::to_string(obs_ptrs.size())
     + ".(";
  int i = 0;
  for (const auto &it : obs_ptrs) {
    if (++i > 2) break;
    debug_string += std::to_string(it->getDiscretePoints(0.1).size()) + ",";
  }
  debug_string += ")\n";

  *PlanningContext::Instance()->mutable_planning_debug_info() += debug_string;
  return true;
}

bool RemainDistDecider::getSlBoundary(
    const Pose2DTrajectory &path, const bool is_reverse,
    const bool is_checking_plan_path, const double side_safe_threshold,
    std::vector<std::pair<double, double>> *const ptr_sl_points,
    std::pair<double, double> *const ptr_danger_sl,
    std::string *const ptr_debug_string) {
  // delta_s
  ptr_sl_points->reserve(path.size());
  double delta_s = 0.0;
  double s = 0.0;
  double last_s = 0.0;

  // get sl points
  for (size_t i = 0; i < path.size(); ++i) {
    const auto &path_point = path.at(i);
    if (std::abs(path_point.s - last_s) < delta_s && i > 0) {
      continue;
    }
    if (path_point.s > kConsiderPathDistance) break;

    double obs_dist =
        obs_grid_->getMinDistance(path_point, mc_footprint_model_);
    obs_dist = max(obs_dist, 0.0);
    ptr_sl_points->emplace_back(std::make_pair(path_point.s, obs_dist));
    last_s = path_point.s;
  }

  for (size_t i = 0; i < ptr_sl_points->size(); ++i) {
    const auto &sl = ptr_sl_points->at(i);
    if (sl.second <= side_safe_threshold) {
      *ptr_danger_sl = sl;

      // debug info
      if (is_checking_plan_path)
        *ptr_debug_string += ",pp_coll";
      else
        *ptr_debug_string += ",mpc_coll";

      double prev_delta_s = 0.0;
      if (i > 0) {
        const auto &prev_sl = ptr_sl_points->at(i - 1);
        prev_delta_s = sl.first - prev_sl.first;
      }
      *ptr_debug_string += ",delS" + std::to_string(prev_delta_s).substr(0, 5);
      return true;
    }
  }

  for (const auto &sl : *ptr_sl_points) {
  }
  return false;
}

void RemainDistDecider::getExtraPillarObstacle(
    const double pilliar_extra, const vector<Obstacle> &vec_pillar_origin,
    std::vector<planning_math::Vec2d> *const ptr_extra_pillar_pts) {
  const auto &parking_slot = PlanningContext::Instance()
                                 ->mutable_parking_behavior_planner_output()
                                 ->parking_slot_info;
  planning_math::LineSegment2d slot_center_line = parking_slot.center_line;

  for (const auto &pillar_point : vec_pillar_origin) {
    if (slot_center_line.DistanceTo(planning_math::Vec2d(
            pillar_point.point().x, pillar_point.point().y)) > 2.5) {
      continue;
    }

    planning_math::Vec2d obs_relative(
        pillar_point.point().x - slot_center_line.start().x(),
        pillar_point.point().y - slot_center_line.start().y());
    planning_math::Vec2d slot_left_direction =
        slot_center_line.unit_direction().rotate(M_PI_2);
    double is_left_side =
        slot_center_line.unit_direction().CrossProd(obs_relative) > 0 ? -1.0
                                                                      : 1.0;
    planning_math::Vec2d shift_vector =
        (double)is_left_side * slot_left_direction * pilliar_extra;

    planning_math::Vec2d obs_pt(pillar_point.point().x, pillar_point.point().y);
    planning_math::Vec2d extra_obs = obs_pt + shift_vector;
    ptr_extra_pillar_pts->emplace_back(std::move(extra_obs));
  }
  // debug extra obstacle points
  PlanningContext::Instance()->mutable_vec_extra_obstacle_points()->clear();
  PlanningContext::Instance()
      ->mutable_vec_extra_obstacle_points()
      ->emplace_back(*ptr_extra_pillar_pts);
}

} // namespace parking
} // namespace msquare
