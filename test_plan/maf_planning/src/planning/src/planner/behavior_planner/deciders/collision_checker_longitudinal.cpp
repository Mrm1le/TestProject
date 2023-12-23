#include "common/math/math_utils.h"
#include "planner/behavior_planner/deciders/collision_checker.h"
#include <iostream>
// #include <iostream>
namespace msquare {

namespace parking {

namespace {
constexpr double kModelSwitchHeight = 0.2;
constexpr double kSafeRemainDistance = 1.2;

}

void CollisionCarParams::update() {
  front_edge_to_center = VehicleParam::Instance()->front_edge_to_center;
  length = VehicleParam::Instance()->length;
  half_length = length / 2.0;
  width = VehicleParam::Instance()->width;

  back_comp_r = 0.0;
  back_comp_d = length - front_edge_to_center;
  comp_length_r = front_edge_to_center - half_length;
  comp_length_d = front_edge_to_center - half_length + 0.5 * back_comp_d;
}
bool CollisionChecker::isCollision(ObsPtsWithId &pts,
                                   const PolygonsWithPose &polygons) {
  if (polygons.empty()) {
    return false;
  }
  // collision check is not needed when traversed length is larger than this
  const double consider_s = 2.5;
  bool global_is_collision = false;

  for (auto &p_wrap : pts) {
    double s = 0;
    double min_dis = std::numeric_limits<double>::max();
    double temp_dis = std::numeric_limits<double>::max();
    double collision_threshold = p_wrap.collision_threshold;
    CollisionCheckStatus &result = p_wrap.result;
    if (result.is_collision) {
      continue;
    }
    result.is_valid = true;
    result.s = std::numeric_limits<double>::max();
    result.min_distance = std::numeric_limits<double>::max();
    const planning_math::Vec2d &p = p_wrap.p;
    PolygonsWithPose::const_iterator iter = polygons.begin();
    bool is_init = true;

    for (const auto &polygon : polygons) { // std::get<0>(obs)
      // the sum is usually not so large, so we use 'sqrt' here, not slow
      // 'hypot'
      s += std::sqrt(msquare::planning_math::sum_square(
          std::get<2>(polygon).x - std::get<2>(*iter).x,
          std::get<2>(polygon).y - std::get<2>(*iter).y));
      if (p_wrap.z() > kModelSwitchHeight) {
        temp_dis = std::get<0>(polygon).DistanceTo(p);
      } else {
        temp_dis = std::get<1>(polygon).DistanceTo(p);
      }
      if (temp_dis < min_dis) {
        result.s = s;
        min_dis = temp_dis;
        if (min_dis < collision_threshold) {
          result.min_distance = result.is_collision
                                    ? std::min(min_dis, result.min_distance)
                                    : min_dis;
          p_wrap.ego_danger_location = std::get<2>(polygon);
          result.is_collision = true;
          global_is_collision = true;
          break;
        }
      }
      if (s > consider_s) {
        break;
      }
      if (is_init) {
        is_init = false;
        continue;
      }
      iter++;
    } // end polygons
  }   // end pts
  return global_is_collision;
}

void CollisionChecker::remainDisCheck(
    ObsPtsWithId &pts, const Pose2DTrajectory &mpc_traj,
    const Pose2DTrajectory &plan_traj, bool is_reverse, bool is_moved,
    const Pose2D &ego_pose, FreespacePoint *const ptr_lead_point,
    std::string *const ptr_debug_str,
    std::vector<std::pair<double, double>> *const ptr_old_mpc_sl_points) {
  param_.update();
  double back_comp_length =
      is_reverse ? param_.back_comp_r : param_.back_comp_d;

  set_params(param_.front_edge_to_center - param_.half_length, back_comp_length,
             is_reverse);
  ego_model_.set_model_type(EgoModelType::HEXADECAGON);

  PathPoint pp;

  // check ego
  if (!is_moved) {
    PolygonsWithPose ego_polygons;
    ego_polygons.reserve(1);
    pp.x = ego_pose.x;
    pp.y = ego_pose.y;
    pp.theta = ego_pose.theta;
    const planning_math::Polygon2d hexadecagon_model =
        ego_model_.get_ego_model_polygon(EgoModelType::HEXADECAGON, pp);
    const planning_math::Polygon2d wheel_base_model =
        ego_model_.get_ego_model_polygon(EgoModelType::WHEEL_BASE, pp);
    ego_polygons.emplace_back(
        std::make_tuple(hexadecagon_model, wheel_base_model, ego_pose));
    if (isCollision(pts, ego_polygons)) {
      *ptr_debug_str += "[old] plan_path collision!";
      ptr_lead_point->collision_type = CollisionType::MPC;
      return;
    }
  }

  // check mpc trajectory
  PolygonsWithPose mpc_polygons;
  mpc_polygons.reserve(mpc_traj.size());
  for (auto &p : mpc_traj) {
    pp.x = p.x;
    pp.y = p.y;
    pp.theta = p.theta;
    const planning_math::Polygon2d hexadecagon_model =
        ego_model_.get_ego_model_polygon(EgoModelType::HEXADECAGON, pp);
    const planning_math::Polygon2d wheel_base_model =
        ego_model_.get_ego_model_polygon(EgoModelType::WHEEL_BASE, pp);
    mpc_polygons.emplace_back(
        std::make_tuple(hexadecagon_model, wheel_base_model, p));
  }
  isCollision(pts, mpc_polygons);

  for (auto &pt : pts) {
    if (pt.result.is_collision) {
      *ptr_debug_str += "[old] plan_path collision!";
      ptr_lead_point->collision_type = CollisionType::MPC;
      break;
    }
  }

  // check plan trajectory
  PolygonsWithPose plan_polygons;
  plan_polygons.reserve(plan_traj.size());
  for (auto &p : plan_traj) {
    pp.x = p.x;
    pp.y = p.y;
    pp.theta = p.theta;
    const planning_math::Polygon2d hexadecagon_model =
        ego_model_.get_ego_model_polygon(EgoModelType::HEXADECAGON, pp);
    const planning_math::Polygon2d wheel_base_model =
        ego_model_.get_ego_model_polygon(EgoModelType::WHEEL_BASE, pp);
    plan_polygons.emplace_back(
        std::make_tuple(hexadecagon_model, wheel_base_model, p));
  }
  isCollision(pts, plan_polygons);
  // for(const auto& polygon : mpc_polygons) {
  //   for (const auto& pt: polygon.first.points()) {
  //     MSD_LOG(ERROR, "polygon is x: %f, %f", pt.x(), pt.y());
  //   }
  // }
  // for (const auto& data : pts) {
  //   MSD_LOG(ERROR, "pt result is: %d, remain is: %f, id %d, x:%f , x:%f",
  //   data.result.is_collision, data.result.s, data.p.Id(),data.p.x(), data.p.y());
  // }
  ptr_lead_point->collision_type = CollisionType::PLANPATH;

  return;
}

} // namespace parking

} // namespace msquare
