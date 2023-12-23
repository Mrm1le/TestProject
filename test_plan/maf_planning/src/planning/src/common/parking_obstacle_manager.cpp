#include "common/parking_obstacle_manager.h"
#include "common/parking_world_model.h"
#include <cassert>
#include <tuple>

#include "common/math/math_utils.h"

using namespace std;

namespace msquare {

namespace parking {

bool ObstacleManager::init(const std::shared_ptr<WorldModel> &world_model) {
  mph_assert(nullptr != world_model);
  world_model_ = world_model;
  // lateral_obstacle_ =
  // std::make_shared<LateralObstacle>(world_model->get_ego_state(),
  //     world_model->get_mutable_map_info());
  return true;
}

void ObstacleManager::clear() {
  obstacles_ = IndexedList<int, Obstacle>();
  static_obstacles_ = IndexedList<int, Obstacle>();
  points_ = IndexedList<int, Obstacle>();
  all_points_ = IndexedList<int, Obstacle>();
  uss_points_ = IndexedList<int, Obstacle>();
  lines_ = IndexedList<int, Obstacle>();
  // freespace_obstacles_ = IndexedList<int, Obstacle>();
  groundline_obstacles_ = IndexedList<int, Obstacle>();
  pillar_obstacles_ = IndexedList<int, Obstacle>();
  gate_obstacles_ = IndexedList<int, Obstacle>();
  road_border_obstacles_ = IndexedList<int, Obstacle>();
  all_obstacles_ptr_.clear();
}

Obstacle *ObstacleManager::add_obstacle(const Obstacle &obstacle) {
  return obstacles_.Add(obstacle.Id(), obstacle);
}

Obstacle *ObstacleManager::find_obstacle(int object_id) {
  return obstacles_.Find(object_id);
}

const Obstacle *ObstacleManager::find_obstacle(int object_id) const {
  return obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_obstacles() const {
  return obstacles_;
}

Obstacle *ObstacleManager::add_static_obstacle(const Obstacle &obstacle) {
  return static_obstacles_.Add(obstacle.Id(), obstacle);
}

Obstacle *ObstacleManager::find_static_obstacle(int object_id) {
  return static_obstacles_.Find(object_id);
}

const Obstacle *ObstacleManager::find_static_obstacle(int object_id) const {
  return static_obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &
ObstacleManager::get_static_obstacles() const {
  return static_obstacles_;
}

Obstacle *ObstacleManager::add_groundline_obstacle(const Obstacle &obstacle) {
  return groundline_obstacles_.Add(obstacle.Id(), obstacle);
}

Obstacle *ObstacleManager::find_groundline_obstacle(int object_id) {
  return groundline_obstacles_.Find(object_id);
}

const Obstacle *ObstacleManager::find_groundline_obstacle(int object_id) const {
  return groundline_obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &
ObstacleManager::get_groundline_obstacles() const {
  return groundline_obstacles_;
}

Obstacle *ObstacleManager::add_point(const Obstacle &obstacle) {
  return points_.Add(obstacle.Id(), obstacle);
}

Obstacle *ObstacleManager::find_point(int object_id) {
  return points_.Find(object_id);
}

const Obstacle *ObstacleManager::find_point(int object_id) const {
  return points_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_points() const {
  return points_;
}

Obstacle *ObstacleManager::add_all_point(const Obstacle &obstacle) {
  return all_points_.Add(obstacle.Id(), obstacle);
}

Obstacle *ObstacleManager::find_all_point(int object_id) {
  return all_points_.Find(object_id);
}

const Obstacle *ObstacleManager::find_all_point(int object_id) const {
  return all_points_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_all_points() const {
  return all_points_;
}

Obstacle *ObstacleManager::add_uss_point(const Obstacle &obstacle) {
  return uss_points_.Add(obstacle.Id(), obstacle);
}

Obstacle *ObstacleManager::find_uss_point(int object_id) {
  return uss_points_.Find(object_id);
}

const Obstacle *ObstacleManager::find_uss_point(int object_id) const {
  return uss_points_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_uss_points() const {
  return uss_points_;
}

Obstacle *ObstacleManager::add_line(const Obstacle &obstacle) {
  return lines_.Add(obstacle.Id(), obstacle);
}

Obstacle *ObstacleManager::find_line(int object_id) {
  return lines_.Find(object_id);
}

const Obstacle *ObstacleManager::find_line(int object_id) const {
  return lines_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_lines() const {
  return lines_;
}

// Obstacle *ObstacleManager::add_freespace_obstacle(const Obstacle &obstacle) {
//   return freespace_obstacles_.Add(obstacle.Id(), obstacle);
// }

Obstacle *ObstacleManager::find_from_all_obstacle(int object_id) {
  auto obstacle = obstacles_.Find(object_id);
  if (obstacle != nullptr) {
    return obstacle;
  }
  // obstacle = freespace_obstacles_.Find(object_id);
  // if (obstacle != nullptr) {
  //   return obstacle;
  // }
  obstacle = gate_obstacles_.Find(object_id);
  if (obstacle != nullptr) {
    return obstacle;
  }
  return groundline_obstacles_.Find(object_id);
}

const Obstacle *ObstacleManager::find_from_all_obstacle(int object_id) const {
  auto obstacle = obstacles_.Find(object_id);
  if (obstacle != nullptr) {
    return obstacle;
  }
  // obstacle = freespace_obstacles_.Find(object_id);
  // if (obstacle != nullptr) {
  //   return obstacle;
  // }
  obstacle = gate_obstacles_.Find(object_id);
  if (obstacle != nullptr) {
    return obstacle;
  }
  return groundline_obstacles_.Find(object_id);
}

// const IndexedList<int, Obstacle> &
// ObstacleManager::get_freespace_obstacles() const {
//   return freespace_obstacles_;
// }

const std::vector<const Obstacle *> &ObstacleManager::get_all_obstacles() {
  if (all_obstacles_ptr_.size() !=
      obstacles_.Items().size() + gate_obstacles_.Items().size()) {
    all_obstacles_ptr_.clear();

    // for (const auto &item : freespace_obstacles_.Items()) {
    //   all_obstacles_ptr_.push_back(item);
    // }

    for (const auto &item : obstacles_.Items()) {
      all_obstacles_ptr_.push_back(item);
    }

    for (const auto &item : gate_obstacles_.Items()) {
      all_obstacles_ptr_.push_back(item);
    }
    for (const auto &item : pillar_obstacles_.Items()) {
      all_obstacles_ptr_.push_back(item);
    }
    for (const auto &item : road_border_obstacles_.Items()) {
      all_obstacles_ptr_.push_back(item);
    }
  }
  return all_obstacles_ptr_;
}

Obstacle *ObstacleManager::add_pillar(const Obstacle &obstacle) {
  return pillar_obstacles_.Add(obstacle.Id(), obstacle);
}

const Obstacle *ObstacleManager::find_pillar(int object_id) const {
  return pillar_obstacles_.Find(object_id);
}

Obstacle *ObstacleManager::find_pillar(int object_id) {
  return pillar_obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_pillars() const {
  return pillar_obstacles_;
}

Obstacle *ObstacleManager::add_gate(const Obstacle &obstacle) {
  return gate_obstacles_.Add(obstacle.Id(), obstacle);
}

const Obstacle *ObstacleManager::find_gate(int object_id) const {
  return gate_obstacles_.Find(object_id);
}

Obstacle *ObstacleManager::find_gate(int object_id) {
  return gate_obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_gates() const {
  return gate_obstacles_;
}

Obstacle *ObstacleManager::add_road_border(const Obstacle &obstacle) {
  return road_border_obstacles_.Add(obstacle.Id(), obstacle);
}

const Obstacle *ObstacleManager::find_road_border(int object_id) const {
  return road_border_obstacles_.Find(object_id);
}

Obstacle *ObstacleManager::find_road_border(int object_id) {
  return road_border_obstacles_.Find(object_id);
}

const IndexedList<int, Obstacle> &ObstacleManager::get_road_borders() const {
  return road_border_obstacles_;
}

bool ObstacleManager::add_lateral_decision(const std::string &tag, int id,
                                           const ObjectDecisionType &decision) {
  auto obstacle = obstacles_.Find(id);
  if (!obstacle) {
    // std::cerr << "failed to find obstacle" << std::endl;
    return false;
  }
  obstacle->AddLateralDecision(tag, decision);
  return true;
}

bool ObstacleManager::add_parking_lateral_decision(
    const std::string &tag, int id, const ObjectDecisionType &decision) {
  // auto obstacle = obstacles_.Find(id);
  auto obstacle = find_from_all_obstacle(id);
  if (!obstacle) {
    // std::cerr << "failed to find obstacle" << std::endl;
    return false;
  }
  obstacle->AddParkingLateralDecision(tag, decision);
  return true;
}

bool ObstacleManager::add_longitudinal_decision(
    const std::string &tag, int id, const ObjectDecisionType &decision) {
  auto obstacle = obstacles_.Find(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }
  obstacle->AddLongitudinalDecision(tag, decision);
  return true;
}

bool ObstacleManager::add_parking_longitudinal_decision(
    const std::string &tag, int id, const ObjectDecisionType &decision) {
  auto obstacle = obstacles_.Find(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }
  obstacle->AddParkingLongitudinalDecision(tag, decision);
  return true;
}

bool ObstacleManager::add_parking_static_obs_longitudinal_decision(
    const std::string &tag, int id, const ObjectDecisionType &decision) {
  auto pillar_obstacle = pillar_obstacles_.Find(id);
  auto road_border_obstacle = road_border_obstacles_.Find(id);
  auto gate_obstacle = gate_obstacles_.Find(id);
  if (!pillar_obstacle) {
    if (!road_border_obstacle) {
      if (!gate_obstacle) {
        // std::cerr << "Failed to find obstacle" << std::endl;
        return false;
      }
      gate_obstacle->AddParkingLongitudinalDecision(tag, decision);
      return true;
    }
    road_border_obstacle->AddParkingLongitudinalDecision(tag, decision);
    return true;
  }
  pillar_obstacle->AddParkingLongitudinalDecision(tag, decision);
  return true;
}

bool ObstacleManager::set_blocking(int id, bool blocking) {
  // auto obstacle = obstacles_.Find(id);
  auto obstacle = find_from_all_obstacle(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }
  obstacle->SetBlockingObstacle(blocking);
  return true;
}

bool ObstacleManager::set_beside_intersation(int id) {
  if (false && world_model_->is_parking_lvp()) {
    return true;
  }
  auto obstacle = find_obstacle(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }
  bool is_beside_intersation = false;
  auto slboundary = obstacle->PerceptionSLBoundary();
  for (double s = slboundary.start_s - 1.0; s < slboundary.end_s + 1.0;
       s += 0.5) {
    if (world_model_->get_refline_manager()->is_intersection(s))
      is_beside_intersation = true;
  }
  obstacle->SetBesideIntersection(is_beside_intersation);
  return true;
}

bool ObstacleManager::set_road_status(int id) {
  auto obstacle = find_obstacle(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }

  const SLBoundary &obs_sl = obstacle->PerceptionSLBoundary();
  double left_border_dist_start = planning_math::interps(
      world_model_->get_refline_manager()->get_left_road_border_distance(),
      world_model_->get_refline_manager()->get_s(), obs_sl.start_s);
  double left_border_dist_end = planning_math::interps(
      world_model_->get_refline_manager()->get_left_road_border_distance(),
      world_model_->get_refline_manager()->get_s(), obs_sl.end_s);
  double right_border_dist_start = planning_math::interps(
      world_model_->get_refline_manager()->get_right_road_border_distance(),
      world_model_->get_refline_manager()->get_s(), obs_sl.start_s);
  double right_border_dist_end = planning_math::interps(
      world_model_->get_refline_manager()->get_right_road_border_distance(),
      world_model_->get_refline_manager()->get_s(), obs_sl.end_s);

  double right_bound =
      -std::max(right_border_dist_start, right_border_dist_end);
  double left_bound = std::max(left_border_dist_start, left_border_dist_end);

  // obstacle->SetLeftSpaceBorder(left_border_dist_start, left_border_dist_end,
  // obs_sl); obstacle->SetRightSpaceBorder(right_border_dist_start,
  // right_border_dist_end, obs_sl);

  (void)set_road_border_distance(id);

  if (false && world_model_->is_parking_lvp()) {
    // std::cout<<"No road status in PRO"<<std::endl;
    return true;
  }

  obstacle->SetAcrossRoadBorder(
      ((left_bound - obs_sl.start_l) > 0.5 && left_bound - obs_sl.end_l < 0) ||
      ((obs_sl.end_l - right_bound > 0.5) &&
       (obs_sl.start_l - right_bound < 0)));

  obstacle->SetInRoad(
      (obs_sl.end_l > right_bound && obs_sl.end_l < left_bound) &&
      (obs_sl.start_l > right_bound) && (obs_sl.start_l < left_bound));

  obstacle->SetBesideRoad(
      ((obs_sl.end_l - right_bound) < 0.5 && obs_sl.end_l < left_bound) ||
      ((left_bound - obs_sl.start_l) < 0.5 && obs_sl.start_l > right_bound));

  obstacle->SetInRoadLoose(
      (obs_sl.start_l > right_bound && obs_sl.start_l < left_bound) ||
      (obs_sl.end_l > right_bound) && (obs_sl.end_l < left_bound));

  if (obstacle->IsAcrossRoadBorder()) {
    if (obs_sl.start_l < right_bound && obs_sl.end_l > right_bound) {
      obstacle->SetDistAcrossRoadBoader(std::abs(obs_sl.end_l - right_bound));
    } else if (obs_sl.start_l < left_bound && obs_sl.end_l > left_bound) {
      obstacle->SetDistAcrossRoadBoader(std::abs(obs_sl.start_l - left_bound));
    }
  } else {
    obstacle->SetDistAcrossRoadBoader(0.0);
  }

  double middle_left_l_start =
      left_border_dist_start -
      ((left_border_dist_start + right_border_dist_start) - 2.0) / 2.0;
  double middle_right_r_start =
      right_border_dist_start +
      ((left_border_dist_start + right_border_dist_start) - 2.0) / 2.0;
  double middle_left_l_end =
      left_border_dist_end -
      ((left_border_dist_end + right_border_dist_end) - 2.0) / 2.0;
  double middle_right_r_end =
      right_border_dist_end +
      ((left_border_dist_end + right_border_dist_end) - 2.0) / 2.0;

  double middle_right_bound =
      -std::max(middle_right_r_start, middle_right_r_end);
  double middle_left_bound = std::max(middle_left_l_start, middle_left_l_end);

  double middle_l =
      std::max((left_border_dist_start - right_border_dist_start) / 2.0,
               (left_border_dist_end - right_border_dist_end) / 2.0);

  obstacle->SetInMiddleRoad(
      (obs_sl.end_l > middle_right_bound && obs_sl.end_l < middle_left_bound) ||
      (obs_sl.start_s < middle_left_bound &&
       obs_sl.start_l > middle_right_bound));

  obstacle->SetInRoadSide(
      (obs_sl.start_l < (middle_l + 0.2) || obs_sl.end_l < (middle_l + 0.2))
          ? 1
          : 2);

  obstacle->SetInRoadType(
      world_model_->get_refline_manager()->is_carriage_way(obs_sl.start_s) ||
              world_model_->get_refline_manager()->is_carriage_way(obs_sl.end_s)
          ? 2
          : 1);

  // std::cout << "obs_contruct: " << obs_sl.start_l << " " << obs_sl.end_l << "
  // " << middle_l
  // << "  " << obstacle->RoadSide() << std::endl;

  return true;
}

bool ObstacleManager::set_road_border_distance(int id, bool using_credible) {
  auto obstacle = find_obstacle(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }

  const SLBoundary &obs_sl = obstacle->PerceptionSLBoundary();
  const std::vector<Point3D> &sl_corners = obs_sl.corners;
  bool having_credible_border = false;
  double left_border_distance_credible_min = 1e19;
  double right_border_distance_credible_min = 1e19;
  double left_border_distance_uncredible_min = 1e19;
  double right_border_distance_uncredible_min = 1e19;
  for (const auto &corner : sl_corners) {
    double left_border_dist = planning_math::interps(
        world_model_->get_refline_manager()->get_left_road_border_distance(),
        world_model_->get_refline_manager()->get_s(), corner.x);
    double right_border_dist = planning_math::interps(
        world_model_->get_refline_manager()->get_right_road_border_distance(),
        world_model_->get_refline_manager()->get_s(), corner.x);
    if (!world_model_->get_refline_manager()->is_intersection(corner.x)) {
      having_credible_border = true;
      left_border_distance_credible_min = std::min(
          left_border_dist - corner.y, left_border_distance_credible_min);
      right_border_distance_credible_min = std::min(
          right_border_dist + corner.y, right_border_distance_credible_min);
    } else {
      left_border_distance_uncredible_min = std::min(
          left_border_dist - corner.y, left_border_distance_uncredible_min);
      right_border_distance_uncredible_min = std::min(
          right_border_dist + corner.y, right_border_distance_uncredible_min);
    }
    // std::cout << "road_border: " << left_border_distance_credible_min << " "
    // << right_border_distance_credible_min
    // << " " << left_border_distance_uncredible_min << " " <<
    // right_border_distance_uncredible_min << std::endl;
  }
  if (using_credible) {
    if (having_credible_border) {
      obstacle->SetLeftSpaceBorder(left_border_distance_credible_min);
      obstacle->SetRightSpaceBorder(right_border_distance_credible_min);
    } else {
      obstacle->SetLeftSpaceBorder(left_border_distance_uncredible_min);
      obstacle->SetRightSpaceBorder(right_border_distance_uncredible_min);
    }
  } else {
    obstacle->SetLeftSpaceBorder(std::min(left_border_distance_uncredible_min,
                                          left_border_distance_credible_min));
    obstacle->SetRightSpaceBorder(std::min(right_border_distance_uncredible_min,
                                           right_border_distance_credible_min));
  }
  return true;
}

bool ObstacleManager::set_virtual_box(
    int id, const planning_math::Box2d &perception_bounding_box_virtual,
    const SLBoundary &perception_sl_bounding_box_virtual) {
  auto obstacle = find_obstacle(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }
  obstacle->SetVirtualBoxFlag(true);
  obstacle->SetPerceptionBoundingBoxVirtual(perception_bounding_box_virtual);
  obstacle->SetPerceptionSlBoundaryVirtual(perception_sl_bounding_box_virtual);
  // set left and right distance to boundary
  double left_border_dist_start = planning_math::interps(
      world_model_->get_refline_manager()->get_left_road_border_distance(),
      world_model_->get_refline_manager()->get_s(),
      perception_sl_bounding_box_virtual.start_s);
  double left_border_dist_end = planning_math::interps(
      world_model_->get_refline_manager()->get_left_road_border_distance(),
      world_model_->get_refline_manager()->get_s(),
      perception_sl_bounding_box_virtual.end_s);
  obstacle->SetLeftSpaceBorder(left_border_dist_start, left_border_dist_end,
                               perception_sl_bounding_box_virtual);
  double right_border_dist_start = planning_math::interps(
      world_model_->get_refline_manager()->get_right_road_border_distance(),
      world_model_->get_refline_manager()->get_s(),
      perception_sl_bounding_box_virtual.start_s);
  double right_border_dist_end = planning_math::interps(
      world_model_->get_refline_manager()->get_right_road_border_distance(),
      world_model_->get_refline_manager()->get_s(),
      perception_sl_bounding_box_virtual.end_s);
  obstacle->SetRightSpaceBorder(right_border_dist_start, right_border_dist_end,
                                perception_sl_bounding_box_virtual);
  // std::cout << "virtual: " << id << " " <<
  // perception_sl_bounding_box_virtual.start_s << " " <<
  // perception_sl_bounding_box_virtual.end_s
  // << " " << perception_sl_bounding_box_virtual.start_l << " " <<
  // perception_sl_bounding_box_virtual.end_l
  // << " " << left_border_dist_start << " " << left_border_dist_end
  // << " " << right_border_dist_start << " " << right_border_dist_end
  // << std::endl;
  return true;
}

// void ObstacleManager::add_freespace_with_filter(
//     const std::vector<std::shared_ptr<Obstacle>> freespace) {
//   // divide into groups
//   std::vector<const Obstacle *> tmp;
//   for (size_t i = 0; i < freespace.size(); i++) {
//     bool flag = false;
//     auto &ego_box = world_model_->get_ego_state().ego_box;
//     if (ego_box.DistanceTo(freespace[i]->PerceptionBoundingBox()) < 1.0)
//       add_freespace_obstacle(*freespace[i]);
//     else {
//       for (auto &obs : obstacles_.Items()) {
//         auto obs_box = obs->PerceptionBoundingBox();
//         if (obs->Type() == ObjectType::COUPE) {
//           obs_box.LongitudinalExtend(1.0);
//           obs_box.LateralExtend(1.0);
//         } else if (obs->Type() == ObjectType::PEDESTRIAN) {
//           obs_box.LongitudinalExtend(3.0);
//           obs_box.LateralExtend(3.0);
//         }
//         auto freespace_box = freespace[i]->PerceptionBoundingBox();
//         if (obs_box.DistanceTo(freespace_box) < 0.5) {
//           flag = true;
//           break;
//         }
//       }
//       if (!flag)
//         add_freespace_obstacle(*freespace[i]);
//     }
//   }
//   // delete
// }

// const std::unordered_map<int, const Obstacle*>
// &ObstacleManager::get_fcp_svp_relation() const{
//   return fcp_svp_relation_;
// }

// const Obstacle *ObstacleManager::find_relative_obstacle(int object_id) const{
//   return fcp_svp_relation_[object_id];
// }

std::vector<const Obstacle *> ObstacleManager::get_static_obstacle() {
  std::vector<const Obstacle *> static_obstacles_;
  for (auto obj : obstacles_.Items()) {
    if (obj->Type() != ObjectType::NOT_KNOW && obj->IsStatic() == 1) {
      static_obstacles_.emplace_back(obj);
    }
  }
  std::sort(static_obstacles_.begin(), static_obstacles_.end(),
            [](const Obstacle *a, const Obstacle *b) {
              return a->PerceptionSLBoundary().start_s >
                     b->PerceptionSLBoundary().start_s;
            });
  return static_obstacles_;
}

std::vector<const Obstacle *> ObstacleManager::get_dynamic_obstacle() {
  std::vector<const Obstacle *> dynamic_obstacles_;
  for (auto obj : obstacles_.Items()) {
    if (obj->Type() != ObjectType::NOT_KNOW && obj->IsStatic() != 1) {
      dynamic_obstacles_.emplace_back(obj);
    }
  }
  std::sort(dynamic_obstacles_.begin(), dynamic_obstacles_.end(),
            [](const Obstacle *a, const Obstacle *b) {
              return a->PerceptionSLBoundary().start_s >
                     b->PerceptionSLBoundary().start_s;
            });
  return dynamic_obstacles_;
}

std::vector<const Obstacle *>
ObstacleManager::get_static_obstacle_slimit(const double s_min,
                                            const double s_max) {
  std::vector<const Obstacle *> static_obstacles_;
  for (auto obj : obstacles_.Items()) {
    if (obj->Type() != ObjectType::NOT_KNOW && obj->IsStatic() == 1) {
      if (obj->PerceptionSLBoundary().start_s > s_min &&
          obj->PerceptionSLBoundary().end_s < s_max)
        static_obstacles_.emplace_back(obj);
    }
  }
  std::sort(static_obstacles_.begin(), static_obstacles_.end(),
            [](const Obstacle *a, const Obstacle *b) {
              return a->PerceptionSLBoundary().start_s >
                     b->PerceptionSLBoundary().start_s;
            });
  return static_obstacles_;
}

std::vector<const Obstacle *>
ObstacleManager::get_dynamic_obstacle_slimit(const double s_min,
                                             const double s_max) {
  std::vector<const Obstacle *> dynamic_obstacles_;
  for (auto obj : obstacles_.Items()) {
    if (obj->Type() != ObjectType::NOT_KNOW && obj->IsStatic() != 1) {
      if (obj->PerceptionSLBoundary().start_s > s_min &&
          obj->PerceptionSLBoundary().end_s < s_max)
        dynamic_obstacles_.emplace_back(obj);
    }
  }
  std::sort(dynamic_obstacles_.begin(), dynamic_obstacles_.end(),
            [](const Obstacle *a, const Obstacle *b) {
              return a->PerceptionSLBoundary().start_s >
                     b->PerceptionSLBoundary().start_s;
            });
  return dynamic_obstacles_;
}

// void ObstacleManager::erase_st_boundaries() {
//   for (const auto *obstacle : obstacles_.Items()) {
//     auto *obstacle_ptr = obstacles_.Find(obstacle->Id());
//     obstacle_ptr->EraseStBoundary();
//   }
// }

bool ObstacleManager::obj_direction_filter() {
  const double PI = 3.141592653589793238463;
  // auto ego_pose = world_model_->get_ego_state().ego_pose;
  // int obj_dir_ego = 0, obj_dir_frenet = 0;
  double angle_diff_thres = 0.785;
  SLBoundary sl_boundary;
  for (auto &obj : obstacles_.Items()) {
    // sl_boundary = obj->PerceptionSLBoundary();
    // if(std::abs(sl_boundary.start_l)>10 || obj->GetCartePosewrtEgo().x > 10
    // || obj->GetCartePosewrtEgo().y > 10){
    //   continue;
    // }
    double obj_theta_ego = obj->GetCartePosewrtEgo().theta;
    // sl_boundary =obj->PerceptionSLBoundary();
    double obj_theta_frenet = obj->Yaw_relative_frenet();
    // if(obj->Id()==1010955){
    //   std::cout << "lxrdebug id : to ego : to frenet = "<<obj->Id()<<" : "
    //   <<obj_theta_ego<<" : "<<obj_theta_frenet <<std::endl;
    // }
    if (obj->Type() == ObjectType::COUPE) {
      if (obstacles_dir_filter[obj->Id()].first.size() >= DIRCTION_CNT_MAX) {
        if (*obstacles_dir_filter[obj->Id()].first.begin() == 1) {
          obstacles_dir_count[obj->Id()].first.first--;
        } else if (*obstacles_dir_filter[obj->Id()].first.begin() == -1) {
          obstacles_dir_count[obj->Id()].first.second--;
        }
        if (*obstacles_dir_filter[obj->Id()].second.begin() == 1) {
          obstacles_dir_count[obj->Id()].second.first--;
        } else if (*obstacles_dir_filter[obj->Id()].second.begin() == -1) {
          obstacles_dir_count[obj->Id()].second.second--;
        }
        obstacles_dir_filter[obj->Id()].first.erase(
            obstacles_dir_filter[obj->Id()].first.begin());
        obstacles_dir_filter[obj->Id()].second.erase(
            obstacles_dir_filter[obj->Id()].second.begin());
      }

      // if(std::abs((std::fmod(obj_theta_ego,2*PI)<-PI)?(std::fmod(obj_theta_ego,2*PI)+2*PI):std::fmod(obj_theta_ego,2*PI))
      // < angle_diff_thres){
      if (std::abs(planning_math::NormalizeAngle(obj_theta_ego)) <
          angle_diff_thres) {
        obstacles_dir_filter[obj->Id()].first.push_back(1);
        obstacles_dir_count[obj->Id()].first.first++;
      } else if (std::abs(planning_math::NormalizeAngle(obj_theta_ego)) >
                 PI - angle_diff_thres) {
        obstacles_dir_filter[obj->Id()].first.push_back(-1);
        obstacles_dir_count[obj->Id()].first.second++;
      } else {
        obstacles_dir_filter[obj->Id()].first.push_back(0);
      }

      if (std::abs(planning_math::NormalizeAngle(obj_theta_frenet)) <
          angle_diff_thres) {
        obstacles_dir_filter[obj->Id()].second.push_back(1);
        obstacles_dir_count[obj->Id()].second.first++;
      } else if (std::abs(planning_math::NormalizeAngle(obj_theta_frenet)) >
                 PI - angle_diff_thres) {
        obstacles_dir_filter[obj->Id()].second.push_back(-1);
        obstacles_dir_count[obj->Id()].second.second++;
      } else {
        obstacles_dir_filter[obj->Id()].second.push_back(0);
      }
    }
  }
  // std::cout << "obsdebug filter size = " <<
  // obstacles_dir_filter.size()<<std::endl;
  std::unordered_map<
      int, std::pair<std::pair<int, int>, std::pair<int, int>>>::iterator it;
  int count = 0;
  for (it = obstacles_dir_count.begin(); it != obstacles_dir_count.end();) {
    // if(it->first == 1010955){
    //   std::cout<<"lxrdebug direction ego frenet =
    //   "<<it->second.first.first<<", "<<it->second.first.second<<",
    //   "<<it->second.second.first<<", "<<it->second.second.second<<std::endl;
    // }
    if (obstacles_.Find(it->first) == nullptr) {
      it = obstacles_dir_count.erase(it);
    } else {
      std::pair<int, int> direction;
      direction.first =
          (it->second.first.first > DIRCTION_CNT_MAX / 2)
              ? 1
              : ((it->second.first.second > DIRCTION_CNT_MAX / 2) ? -1 : 0);
      direction.second =
          (it->second.second.first > DIRCTION_CNT_MAX / 2)
              ? 1
              : ((it->second.second.second > DIRCTION_CNT_MAX / 2) ? -1 : 0);
      obstacles_.Find(it->first)->SetPoseDirection(direction.first +
                                                   direction.second);

      ++it;
      count++;
    }
  }

  // modify road border distance
  for (auto &obj : obstacles_.Items()) {
    if (obj->Type() == ObjectType::COUPE && obj->GetPoseDirection() == 0)
      (void)set_road_border_distance(obj->Id(), false);
  }

  for (auto &obj : obstacles_.Items()) {
    if (obj->Type() == ObjectType::COUPE) {
      if (obj->speed() < 0.3)
        continue;

      if (std::cos(obj->Speed_yaw_relative_ego() -
                   obj->GetCartePosewrtEgo().theta) > std::cos(PI / 3)) {
        obstacles_.Find(obj->Id())->SetGear(1);
      } else if (std::cos(obj->Speed_yaw_relative_ego() -
                          obj->GetCartePosewrtEgo().theta) <
                 std::cos(2 * PI / 3)) {
        obstacles_.Find(obj->Id())->SetGear(-1);
      }
    }
  }
  return true;
}
bool ObstacleManager::obj_speed_filter() {
  const double &ego_vel = world_model_->get_ego_state().ego_vel;
  double abs_ego_vel = std::abs(ego_vel);
  const StatusType &status_type =
      PlanningContext::Instance()->planning_status().scenario.status_type;
  int scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;
  // const PlannerType &planner_type =
  // PlanningContext::Instance()->parking_behavior_planner_output().planner_type;
  // double static_v_thres = 0.0;
  double lon_highspeed_v_thres_car = 1.0;
  double lon_highspeed_v_thres_human = 1.0;
  double lon_static_v_thres = 1.0;
  double lat_static_v_thres = 1.0;
  double lon_opposite_v_thres = -1.0;
  for (auto &obj : obstacles_.Items()) {
    double speed_yaw = obj->Speed_yaw_relative_frenet();
    double obs_v_lat_frenet = obj->PerceptionSpeed() * sin(speed_yaw);
    double obs_v_lon_frenet = obj->PerceptionSpeed() * cos(speed_yaw);
    double obs_v_lat_ego = obj->GetCarteVelwrtEgo().y;
    double obs_v_lon_ego = obj->GetCarteVelwrtEgo().x;
    if (obj->Type() == ObjectType::PEDESTRIAN) {
      obstacles_speed_filter[obj->Id()].first++;

      if (PlanningContext::Instance()->has_scene(
              scene_avp, ParkingSceneType::SCENE_TURN)) {
        lon_static_v_thres =
            interp(abs_ego_vel, _V_EGO_TURN_HUMAN_SVP, _V_THRES_TURN_HUMAN_SVP);
        lat_static_v_thres =
            interp(abs_ego_vel, _V_EGO_TURN_HUMAN_SVP, _V_THRES_TURN_HUMAN_SVP);
      } else {
        lon_static_v_thres =
            interp(abs_ego_vel, _V_EGO_AVP_HUMAN_SVP, _V_THRES_AVP_HUMAN_SVP);
        lat_static_v_thres =
            interp(abs_ego_vel, _V_EGO_AVP_HUMAN_SVP, _V_THRES_AVP_HUMAN_SVP);
      }

      std::get<0>(obstacles_speed_filter[obj->Id()].second.first) =
          obj_check_static(
              lon_static_v_thres, obs_v_lon_frenet,
              std::get<0>(obstacles_speed_filter[obj->Id()].second.first),
              ObjectType::PEDESTRIAN, false);
      std::get<1>(obstacles_speed_filter[obj->Id()].second.first) =
          obj_check_static(
              lat_static_v_thres, obs_v_lat_frenet,
              std::get<1>(obstacles_speed_filter[obj->Id()].second.first),
              ObjectType::PEDESTRIAN, false);
      std::get<2>(obstacles_speed_filter[obj->Id()].second.first) =
          obj_check_static(
              lon_highspeed_v_thres_human, obs_v_lon_frenet,
              std::get<2>(obstacles_speed_filter[obj->Id()].second.first),
              ObjectType::PEDESTRIAN, true);
      std::get<3>(obstacles_speed_filter[obj->Id()].second.first) = 0;
      std::get<0>(obstacles_speed_filter[obj->Id()].second.second) =
          obj_check_static(
              lon_static_v_thres, obs_v_lon_ego,
              std::get<0>(obstacles_speed_filter[obj->Id()].second.second),
              ObjectType::PEDESTRIAN, false);
      std::get<1>(obstacles_speed_filter[obj->Id()].second.second) =
          obj_check_static(
              lat_static_v_thres, obs_v_lat_ego,
              std::get<1>(obstacles_speed_filter[obj->Id()].second.second),
              ObjectType::PEDESTRIAN, false);
      std::get<2>(obstacles_speed_filter[obj->Id()].second.second) =
          obj_check_static(
              lon_highspeed_v_thres_human, obs_v_lon_ego,
              std::get<2>(obstacles_speed_filter[obj->Id()].second.second),
              ObjectType::PEDESTRIAN, true);
      std::get<3>(obstacles_speed_filter[obj->Id()].second.second) = 0;

    } else if (obj->Type() == ObjectType::COUPE) {
      obstacles_speed_filter[obj->Id()].first++;

      if (status_type == StatusType::APA || status_type == StatusType::APOA) {
        lon_static_v_thres =
            interp(abs_ego_vel, _V_EGO_APA_CAR_SVP, _V_THRES_APA_CAR_SVP);
        lat_static_v_thres =
            interp(abs_ego_vel, _V_EGO_APA_CAR_SVP, _V_THRES_APA_CAR_SVP);
      } else {
        if (PlanningContext::Instance()->has_scene(
                scene_avp, ParkingSceneType::SCENE_TURN)) {
          lon_static_v_thres =
              interp(abs_ego_vel, _V_EGO_TURN_CAR_SVP, _V_THRES_TURN_CAR_SVP);
          lat_static_v_thres =
              interp(abs_ego_vel, _V_EGO_TURN_CAR_SVP, _V_THRES_TURN_CAR_SVP);
        } else {
          lon_static_v_thres =
              interp(abs_ego_vel, _V_EGO_AVP_CAR_SVP, _V_THRES_AVP_CAR_SVP);
          lat_static_v_thres =
              interp(abs_ego_vel, _V_EGO_AVP_CAR_SVP, _V_THRES_AVP_CAR_SVP);
        }
      }

      std::get<0>(obstacles_speed_filter[obj->Id()].second.first) =
          obj_check_static(
              lon_static_v_thres, obs_v_lon_frenet,
              std::get<0>(obstacles_speed_filter[obj->Id()].second.first),
              ObjectType::COUPE, false);
      std::get<1>(obstacles_speed_filter[obj->Id()].second.first) =
          obj_check_static(
              lat_static_v_thres, obs_v_lat_frenet,
              std::get<1>(obstacles_speed_filter[obj->Id()].second.first),
              ObjectType::COUPE, false);
      std::get<2>(obstacles_speed_filter[obj->Id()].second.first) =
          obj_check_static(
              lon_highspeed_v_thres_car, obs_v_lon_frenet,
              std::get<2>(obstacles_speed_filter[obj->Id()].second.first),
              ObjectType::COUPE, true);
      std::get<3>(obstacles_speed_filter[obj->Id()].second.first) =
          obj_check_static(
              lon_opposite_v_thres, obs_v_lon_frenet,
              std::get<3>(obstacles_speed_filter[obj->Id()].second.first),
              ObjectType::COUPE, true);

      std::get<0>(obstacles_speed_filter[obj->Id()].second.second) =
          obj_check_static(
              lon_static_v_thres, obs_v_lon_ego,
              std::get<0>(obstacles_speed_filter[obj->Id()].second.second),
              ObjectType::COUPE, false);
      std::get<1>(obstacles_speed_filter[obj->Id()].second.second) =
          obj_check_static(
              lat_static_v_thres, obs_v_lat_ego,
              std::get<1>(obstacles_speed_filter[obj->Id()].second.second),
              ObjectType::COUPE, false);
      std::get<2>(obstacles_speed_filter[obj->Id()].second.second) =
          obj_check_static(
              lon_highspeed_v_thres_car, obs_v_lon_ego,
              std::get<2>(obstacles_speed_filter[obj->Id()].second.second),
              ObjectType::COUPE, true);
      std::get<3>(obstacles_speed_filter[obj->Id()].second.second) =
          obj_check_static(
              lon_opposite_v_thres, obs_v_lon_ego,
              std::get<3>(obstacles_speed_filter[obj->Id()].second.second),
              ObjectType::COUPE, true);
    }
  }

  std::unordered_map<
      int, std::pair<int, std::pair<std::tuple<int, int, int, int>,
                                    std::tuple<int, int, int, int>>>>::iterator
      it;
  for (it = obstacles_speed_filter.begin();
       it != obstacles_speed_filter.end();) {
    if (obstacles_.Find(it->first) == nullptr) {
      it = obstacles_speed_filter.erase(it);
    } else {
      if (obstacles_.Find(it->first)->Type() == ObjectType::COUPE) {
        if (it->second.first <= 6) {
          obstacles_.Find(it->first)->SetLonStaticWrtFrenet(-1);
          obstacles_.Find(it->first)->SetLatStaticWrtFrenet(-1);
          obstacles_.Find(it->first)->SetLonHighspeedWrtFrenet(-1);
          obstacles_.Find(it->first)->SetLonStaticWrtEgo(-1);
          obstacles_.Find(it->first)->SetLatStaticWrtEgo(-1);
          obstacles_.Find(it->first)->SetLonHighspeedWrtEgo(-1);
        } else {
          obstacles_.Find(it->first)->SetLonStaticWrtFrenet(
              std::get<0>(it->second.second.first) > STATIC_CNT_CAR);
          obstacles_.Find(it->first)->SetLatStaticWrtFrenet(
              std::get<1>(it->second.second.first) > STATIC_CNT_CAR);
          obstacles_.Find(it->first)->SetLonHighspeedWrtFrenet(
              (std::get<2>(it->second.second.first) < STATIC_CNT_CAR) &&
              (obstacles_.Find(it->first)->GetPoseDirection() > 0));

          obstacles_.Find(it->first)->SetLonStaticWrtEgo(
              std::get<0>(it->second.second.second) > STATIC_CNT_CAR);
          obstacles_.Find(it->first)->SetLatStaticWrtEgo(
              std::get<1>(it->second.second.second) > STATIC_CNT_CAR);
          obstacles_.Find(it->first)->SetLonHighspeedWrtEgo(
              (std::get<2>(it->second.second.second) < STATIC_CNT_CAR) &&
              (obstacles_.Find(it->first)->GetPoseDirection() > 0));
        }
        if (it->second.first <= 10) {
          obstacles_.Find(it->first)->SetLonOppositeWrtEgo(-1);
          obstacles_.Find(it->first)->SetLonOppositeWrtFrenet(-1);
        } else {
          obstacles_.Find(it->first)->SetLonOppositeWrtFrenet(
              (std::get<3>(it->second.second.first) > STATIC_CNT_CAR) &&
              (obstacles_.Find(it->first)->GetPoseDirection() < 0));
          obstacles_.Find(it->first)->SetLonOppositeWrtEgo(
              (std::get<3>(it->second.second.second) > STATIC_CNT_CAR) &&
              (obstacles_.Find(it->first)->GetPoseDirection() < 0));
        }
      } else if (obstacles_.Find(it->first)->Type() == ObjectType::PEDESTRIAN) {
        if (it->second.first <= 6) {
          obstacles_.Find(it->first)->SetLonStaticWrtFrenet(-1);
          obstacles_.Find(it->first)->SetLatStaticWrtFrenet(-1);
          obstacles_.Find(it->first)->SetLonHighspeedWrtFrenet(-1);
          obstacles_.Find(it->first)->SetLonStaticWrtEgo(-1);
          obstacles_.Find(it->first)->SetLatStaticWrtEgo(-1);
          obstacles_.Find(it->first)->SetLonHighspeedWrtEgo(-1);
        } else {
          obstacles_.Find(it->first)->SetLonStaticWrtFrenet(
              std::get<0>(it->second.second.first) > STATIC_CNT_PED);
          obstacles_.Find(it->first)->SetLatStaticWrtFrenet(
              std::get<1>(it->second.second.first) > STATIC_CNT_PED);
          obstacles_.Find(it->first)->SetLonHighspeedWrtFrenet(
              std::get<2>(it->second.second.first) < STATIC_CNT_PED);
          obstacles_.Find(it->first)->SetLonOppositeWrtFrenet(0);

          obstacles_.Find(it->first)->SetLonStaticWrtEgo(
              std::get<0>(it->second.second.second) > STATIC_CNT_PED);
          obstacles_.Find(it->first)->SetLatStaticWrtEgo(
              std::get<1>(it->second.second.second) > STATIC_CNT_PED);
          obstacles_.Find(it->first)->SetLonHighspeedWrtEgo(
              std::get<2>(it->second.second.second) < STATIC_CNT_PED);
          obstacles_.Find(it->first)->SetLonOppositeWrtEgo(0);
        }
        // std::cout << "id:tuple = " << obstacles_.Find(it->first)->Id()<<" :
        // <"<<std::get<0>(obstacles_speed_filter[obstacles_.Find(it->first)->Id()])<<","<<std::get<1>(obstacles_speed_filter[obstacles_.Find(it->first)->Id()])<<","<<std::get<2>(obstacles_speed_filter[obstacles_.Find(it->first)->Id()])<<">"<<std::endl;
      }
      ++it;
    }
  }

  for (auto &obj : obstacles_.Items()) {
    // [EPL3-338863] temp solution, make ofo static
    if (obj->SFType() == FusionObjectType::MSD_OBJECT_TYPE_OFO) {
      obstacles_.Find(obj->Id())->SetStatic(1);
      continue;
    }
    if ((obj->Type() == ObjectType::COUPE && VehicleParam::Instance()->car_type == "C03")
    	  || (obj->Type() == ObjectType::PEDESTRIAN)
        || (obj->Type() == ObjectType::CONE_BUCKET)) {
      continue;
    }
    if (obj->IsLonStaticWrtFrenet() != 1 || obj->IsLatStaticWrtFrenet() != 1 ||
        obj->IsLonStaticWrtEgo() != 1 || obj->IsLatStaticWrtEgo() != 1) {
      obstacles_.Find(obj->Id())->SetStatic(0);
    } else {
      obstacles_.Find(obj->Id())->SetStatic(1);
    }
  }
  return true;
}

int ObstacleManager::obj_check_static(double v_thres, double obs_speed,
                                      int count, const ObjectType obs_type,
                                      bool for_lon_highspeed) {
  if (!for_lon_highspeed) {
    if (std::abs(obs_speed) < v_thres) {
      ++count;
    } else {
      --count;
    }
  } else {
    if (obs_speed < v_thres) {
      ++count;
    } else {
      --count;
    }
  }
  int COUNT_MAX =
      (obs_type == ObjectType::COUPE) ? STATIC_CNT_CAR_MAX : STATIC_CNT_PED_MAX;
  count = std::max(0, std::min(COUNT_MAX, count));
  return count;
}

double ObstacleManager::interp(double x, const std::vector<double> &xp,
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

bool ObstacleManager::obj_intention_status_manager() {
  if (false && world_model_->is_parking_lvp()) {
    return true;
  }

  const double &ego_s = world_model_->get_ego_state().ego_frenet.x;
  // single frame properties
  for (auto &obj_cars : obstacles_.Items()) {

    SLBoundary sl_boundary_car;
    sl_boundary_car = obj_cars->PerceptionSLBoundary();

    // approaching gate
    for (auto &obj_gates : gate_obstacles_.Items()) {
      SLBoundary sl_boundary_gate;
      sl_boundary_gate = obj_gates->PerceptionSLBoundary();
      if (std::abs(sl_boundary_gate.start_l) < 5 ||
          std::abs(sl_boundary_gate.end_l) < 5) {
        // double gate_center_l = (sl_boundary_gate.start_l +
        // sl_boundary_gate.end_l)/2;
        double car_center_l =
            (sl_boundary_car.start_l + sl_boundary_car.end_l) / 2;

        bool is_infront_gate =
            ((sl_boundary_gate.start_s - sl_boundary_car.end_s) < 15 &&
             (sl_boundary_gate.end_s - sl_boundary_car.start_s) > -5);

        // Approaching gate cars(single-frame)
        if (is_infront_gate && obj_cars->GetPoseDirection() >= 0 &&
            std::abs(car_center_l) < 1.5 &&
            obj_cars->IsLonOppositeWrtFrenet() != 1) {
          obstacles_.Find(obj_cars->Id())->SetApproachingGate(true);
          continue;
          // obj_cars->SetApproachingGate(true);
        } else {
          obstacles_.Find(obj_cars->Id())->SetApproachingGate(false);
        }
        double obs_v_lon =
            obj_cars->speed() * cos(obj_cars->Speed_yaw_relative_frenet());
        if (obstacles_.Find(obj_cars->Id())->IsApproachingGate() &&
            obs_v_lon < -1.0) {
          obstacles_.Find(obj_cars->Id())->SetApproachingGate(false);
        }
      }
    }

    // Pre-process for apa/apoa check
    double obs_v_lat_frenet =
        obj_cars->speed() * sin(obj_cars->Speed_yaw_relative_frenet());
    if ((std::abs(std::abs(obj_cars->Speed_yaw_relative_frenet()) - PI / 2) <
         PI / 3) &&
        (obs_v_lat_frenet * (sl_boundary_car.start_l + sl_boundary_car.end_l) >
         0) &&
        (std::abs(obs_v_lat_frenet) > 0.2)) {
      obstacles_.Find(obj_cars->Id())->SetTowardRoadSide(1);
      obstacles_.Find(obj_cars->Id())->SetTowardRoadCenter(0);
    }

    if ((std::abs(std::abs(obj_cars->Speed_yaw_relative_frenet()) - PI / 2) <
         PI / 3) &&
        (obs_v_lat_frenet * (sl_boundary_car.start_l + sl_boundary_car.end_l) <
         0) &&
        (std::abs(obs_v_lat_frenet) > 0.2)) {
      obstacles_.Find(obj_cars->Id())->SetTowardRoadSide(0);
      obstacles_.Find(obj_cars->Id())->SetTowardRoadCenter(1);
    }
  }

  // APA status
  for (auto &obj_cars : obstacles_.Items()) {
    if (obj_cars->Type() == ObjectType::COUPE) {
      double obs_v_lon_frenet =
          obj_cars->speed() * cos(obj_cars->Speed_yaw_relative_frenet());
      SLBoundary sl_boundary_car;
      sl_boundary_car = obj_cars->PerceptionSLBoundary();
      if ((obj_cars->Type() == ObjectType::COUPE &&
           std::abs((sl_boundary_car.start_l + sl_boundary_car.end_l) / 2.0) <
               6.0 &&
           (sl_boundary_car.start_s - ego_s) < 30.0 &&
           (sl_boundary_car.start_s - ego_s) >
               2.0)) { // TBD w.r.t fusion output
        if ((obj_cars->IsTowardsRoadSide() == 1 && !obj_cars->IsBesideRoad()) ||
            ((obs_v_lon_frenet < -0.2 ||
              std::abs(std::abs(std::fmod(obj_cars->Yaw_relative_frenet(),
                                          2.0 * PI)) -
                       PI / 2) < PI / 3) &&
             !obj_cars->IsBesideRoad())) { // trigger
          obstacles_apa_filter[obj_cars->Id()].first++;
          std::get<0>(obstacles_apa_filter[obj_cars->Id()].second) = std::min(
              30, ++std::get<0>(obstacles_apa_filter[obj_cars->Id()].second));
        } else if (obj_cars->IsTowardsRoadSide() == 0) {
          if (obstacles_apa_filter.find(obj_cars->Id()) !=
              obstacles_apa_filter.end()) {
            obstacles_apa_filter[obj_cars->Id()].first++;
            std::get<0>(obstacles_apa_filter[obj_cars->Id()].second) = std::max(
                -10,
                --std::get<0>(obstacles_apa_filter[obj_cars->Id()].second));
          }
        }

        if (obj_cars->IsAcrossRoadBorder() == true) {
          std::get<1>(obstacles_apa_filter[obj_cars->Id()].second) = std::min(
              30, ++std::get<1>(obstacles_apa_filter[obj_cars->Id()].second));
        } else if (obj_cars->IsInRoad() == true) {
          std::get<1>(obstacles_apa_filter[obj_cars->Id()].second) = 0;
        } else if (obj_cars->IsBesideRoad() == true) {
          std::get<1>(obstacles_apa_filter[obj_cars->Id()].second) = std::max(
              -10, --std::get<1>(obstacles_apa_filter[obj_cars->Id()].second));
        }

        if (obj_cars->IsBesideRoad() && obj_cars->IsLatStaticWrtFrenet() == 1) {
          std::get<2>(obstacles_apa_filter[obj_cars->Id()].second)++;
        } else {
          std::get<2>(obstacles_apa_filter[obj_cars->Id()].second) = 0;
        }
      }
    }
  }
  std::unordered_map<int, std::pair<int, std::tuple<int, int, int>>>::iterator
      it_apa;
  for (it_apa = obstacles_apa_filter.begin();
       it_apa != obstacles_apa_filter.end();) {
    // std::cout<<"lxrdebug allapa_id = "<<it_apa->first<<std::endl;
    if (it_apa->first == 1001840) {
      // std::cout<<"apadebug apaid total toside across beside =
      // "<<it_apa->first<<" "<<it_apa->second.first<<"
      // "<<std::get<0>(it_apa->second.second)<<"
      // "<<std::get<1>(it_apa->second.second)<<"
      // "<<std::get<2>(it_apa->second.second)<<std::endl;
    }
    // if(it_apa->first == 1002742){
    //   std::cout <<"apadebug apaid total <tuple> = "<<it_apa->first<<"
    //   "<<it_apa->second.first<<" <"<<std::get<0>(it_apa->second.second)<<",
    //   "<<std::get<1>(it_apa->second.second)<<",
    //   "<<std::get<2>(it_apa->second.second)<<">"<<std::endl;
    // }
    if (obstacles_.Find(it_apa->first) == nullptr) {
      it_apa = obstacles_apa_filter.erase(it_apa);
    } else {
      if (it_apa->second.first > 30 &&
          (std::get<0>(it_apa->second.second) > 20 &&
           std::get<1>(it_apa->second.second) > 5)) {
        obstacles_.Find(it_apa->first)->SetIsApa(true);
        obstacles_.Find(it_apa->first)->SetIsApoa(false);
      }
      if ((std::get<2>(it_apa->second.second) > 5 &&
           obstacles_.Find(it_apa->first)->IsApa() &&
           std::abs(
               std::abs(obstacles_.Find(it_apa->first)->Yaw_relative_frenet()) -
               PI / 2) < PI / 6) ||
          std::get<0>(it_apa->second.second) == -10) {
        obstacles_.Find(it_apa->first)->SetIsApa(false);
      }
      it_apa++;
    }
  }

  // APOA status
  for (auto &obj_cars : obstacles_.Items()) {
    if (obj_cars->Type() == ObjectType::COUPE) {
      SLBoundary sl_boundary_car;
      sl_boundary_car = obj_cars->PerceptionSLBoundary();
      if ((obj_cars->Type() == ObjectType::COUPE &&
           std::abs((sl_boundary_car.start_l + sl_boundary_car.end_l) / 2.0) <
               6.0 &&
           (sl_boundary_car.start_s - ego_s) < 30.0 &&
           (sl_boundary_car.start_s - ego_s) >
               2.0)) {                              // TBD w.r.t fusion output
        if (obj_cars->IsTowardsRoadCenter() == 1) { // trigger
          obstacles_apoa_filter[obj_cars->Id()].first++;
          std::get<0>(obstacles_apoa_filter[obj_cars->Id()].second) = std::min(
              30, ++std::get<0>(obstacles_apoa_filter[obj_cars->Id()].second));
        } else if (obj_cars->IsTowardsRoadCenter() != 1) {
          if (obstacles_apoa_filter.find(obj_cars->Id()) !=
              obstacles_apoa_filter.end()) {
            obstacles_apoa_filter[obj_cars->Id()].first++;
            std::get<0>(obstacles_apoa_filter[obj_cars->Id()].second) =
                std::max(-10,
                         --std::get<0>(
                             obstacles_apoa_filter[obj_cars->Id()].second));
          }
        }

        if (obj_cars->IsAcrossRoadBorder() == true) {
          std::get<1>(obstacles_apoa_filter[obj_cars->Id()].second) = std::min(
              30, ++std::get<1>(obstacles_apoa_filter[obj_cars->Id()].second));
        } else if (obj_cars->IsBesideRoad() == true) {
          std::get<1>(obstacles_apoa_filter[obj_cars->Id()].second) = 0;
        } else if (obj_cars->IsInRoad() == true) {
          std::get<1>(obstacles_apoa_filter[obj_cars->Id()].second) = std::max(
              -10, --std::get<1>(obstacles_apoa_filter[obj_cars->Id()].second));
        }

        if (obj_cars->IsLatStaticWrtFrenet() == 1 && obj_cars->IsInRoad()) {
          std::get<2>(obstacles_apoa_filter[obj_cars->Id()].second)++;
        } else {
          std::get<2>(obstacles_apoa_filter[obj_cars->Id()].second) = 0;
        }
      }
    }
  }
  std::unordered_map<int, std::pair<int, std::tuple<int, int, int>>>::iterator
      it_apoa;
  for (it_apoa = obstacles_apoa_filter.begin();
       it_apoa != obstacles_apoa_filter.end();) {
    // std::cout <<"lxrdebug apoaid total <tuple> = "<<it_apoa->first<<"
    // "<<it_apoa->second.first<<" <"<<std::get<0>(it_apoa->second.second)<<",
    // "<<std::get<1>(it_apoa->second.second)<<",
    // "<<std::get<2>(it_apoa->second.second)<<">"<<std::endl;
    if (obstacles_.Find(it_apoa->first) == nullptr) {
      it_apoa = obstacles_apoa_filter.erase(it_apoa);
    } else {
      if (it_apoa->second.first > 30 &&
          (std::get<0>(it_apoa->second.second) > 20 &&
           std::get<1>(it_apoa->second.second) > 5)) {
        obstacles_.Find(it_apoa->first)->SetIsApoa(true);
        obstacles_.Find(it_apoa->first)->SetIsApa(false);
      }
      if (std::get<2>(it_apoa->second.second) > 5 &&
          obstacles_.Find(it_apoa->first)->IsApoa() &&
          std::abs(obstacles_.Find(it_apoa->first)->Yaw_relative_frenet()) <
              PI / 6.0) {
        obstacles_.Find(it_apoa->first)->SetIsApoa(false);
      }
      it_apoa++;
    }
  }

  // pullover status
  for (auto &obj_cars : obstacles_.Items()) {
    double speed_yaw = obj_cars->Speed_yaw_relative_frenet();
    // double obs_v_lat = obj_cars->speed() * sin(speed_yaw);
    double obs_v_lon = obj_cars->speed() * cos(speed_yaw);
    // std::cout<<"lxrdebug : obs_v_lon = "<<obs_v_lon <<std::endl;
    if (obj_cars->Type() == ObjectType::COUPE) {
      SLBoundary sl_boundary_car;
      sl_boundary_car = obj_cars->PerceptionSLBoundary();
      if (obj_cars->Type() == ObjectType::COUPE &&
          std::abs((sl_boundary_car.start_l + sl_boundary_car.end_l) / 2.0) <
              6.0 &&
          (sl_boundary_car.start_s - ego_s) < 30.0 &&
          (sl_boundary_car.start_s - ego_s) > 2.0) { // TBD w.r.t fusion output
        if (obj_cars->IsLonHighspeedWrtFrenet() == 1 &&
            (obj_cars->IsInRoad() ||
             (obj_cars->IsInBend() && obj_cars->IsAcrossRoadBorder()))) {
          obstacles_pullover_filter[obj_cars->Id()].first++;
          std::get<2>(obstacles_pullover_filter[obj_cars->Id()].second)++;

        } else if (obstacles_pullover_filter.find(obj_cars->Id()) !=
                   obstacles_pullover_filter.end()) {
          obstacles_pullover_filter[obj_cars->Id()].first++;
          if (obs_v_lon < 0.8 && obs_v_lon > -0.2) {
            std::get<0>(obstacles_pullover_filter[obj_cars->Id()].second) =
                std::min(20,
                         ++std::get<0>(
                             obstacles_pullover_filter[obj_cars->Id()].second));
          } else if (std::abs(obs_v_lon) > 0.8) {
            // std::cout <<"lxrdebug ~!!!!!!!!!"<<std::endl;
            std::get<0>(obstacles_pullover_filter[obj_cars->Id()].second) =
                std::max(-20,
                         --std::get<0>(
                             obstacles_pullover_filter[obj_cars->Id()].second));
          }

          if (obj_cars->IsLatStaticWrtFrenet() == 1) {
            std::get<1>(obstacles_pullover_filter[obj_cars->Id()].second) =
                std::min(20,
                         ++std::get<1>(
                             obstacles_pullover_filter[obj_cars->Id()].second));
          } else {
            std::get<1>(obstacles_pullover_filter[obj_cars->Id()].second) =
                std::max(-10,
                         ++std::get<1>(
                             obstacles_pullover_filter[obj_cars->Id()].second));
          }
        }
      }
    }
  }

  std::unordered_map<int, std::pair<int, std::tuple<int, int, int>>>::iterator
      it_pullover;
  for (it_pullover = obstacles_pullover_filter.begin();
       it_pullover != obstacles_pullover_filter.end();) {
    // if(it_pullover->first == 1002742){
    //   std::cout<<"lxrdebug pullid total low static inroad =
    //   "<<it_pullover->first<<" "<<it_pullover->second.first<<"
    //   "<<std::get<0>(it_pullover->second.second)<<"
    //   "<<std::get<1>(it_pullover->second.second)<<"
    //   "<<std::get<2>(it_pullover->second.second)<<std::endl;
    // }
    if (obstacles_.Find(it_pullover->first) == nullptr) {
      it_pullover = obstacles_pullover_filter.erase(it_pullover);
    } else {
      if (it_pullover->second.first > 10 &&
          std::get<0>(it_pullover->second.second) > 5) {
        obstacles_.Find(it_pullover->first)->SetIsHighspeedDecel(true);
      } else if (it_pullover->second.first > 10 &&
                 std::get<0>(it_pullover->second.second) == -20) {
        obstacles_.Find(it_pullover->first)->SetIsHighspeedDecel(false);
      }

      if (it_pullover->second.first > 10 &&
          std::get<0>(it_pullover->second.second) > 15 &&
          std::get<1>(it_pullover->second.second) > 15) {
        obstacles_.Find(it_pullover->first)->SetIsPullover(true);
      } else if (it_pullover->second.first > 10 &&
                 (std::get<0>(it_pullover->second.second) < -5 ||
                  std::get<1>(it_pullover->second.second) < 0)) {
        obstacles_.Find(it_pullover->first)->SetIsPullover(false);
      }

      it_pullover++;
    }
  }

  return true;
}

bool ObstacleManager::obj_pseudo_prediction() {

  const double ego_s = world_model_->get_ego_state().ego_frenet.x +
                       VehicleParam::Instance()->front_edge_to_center;
  frenet_coord_ = world_model_->get_frenet_coord();
  if (frenet_coord_ == nullptr)
    return false;
  const msquare::planning_math::Box2d ego_box =
      world_model_->get_ego_state().ego_box;
  Pose2D ego_pose = world_model_->get_ego_state().ego_pose;
  ego_pose.x +=
      VehicleParam::Instance()->front_edge_to_center * cos(ego_pose.theta);
  ego_pose.y +=
      VehicleParam::Instance()->front_edge_to_center * sin(ego_pose.theta);
  const double ego_vel = world_model_->get_ego_state().ego_vel;
  auto scene_avp =
      PlanningContext::Instance()->planning_status().planning_result.scene_avp;
  for (auto &obj : obstacles_.Items()) {
    bool frenet_trigger = false;
    if (world_model_->is_parking_svp()) {
      frenet_trigger = (obj->IsLonOppositeWrtFrenet() == 1 && obj->IsInRoad() &&
                        obj->Is_angle_consistent() &&
                        ((!obj->IsInBend() && !obj->IsBesideIntersection()) ||
                         PlanningContext::Instance()->has_scene(
                             scene_avp, ParkingSceneType::SCENE_RAMP) ||
                         PlanningContext::Instance()->has_scene(
                             scene_avp, ParkingSceneType::SCENE_ENTRANCE)));
    } else if (false && world_model_->is_parking_lvp()) {
      frenet_trigger = obj->IsLonOppositeWrtFrenet() == 1 && !obj->IsInBend();
    }
    // prediction traj wrt frenet
    if (obj->Type() == ObjectType::COUPE &&
        frenet_trigger) { // opposite car in straight lane and inroad
      PseudoPredictionTrajectory prediction_traj;
      SLBoundary sl_boundary;
      sl_boundary = obj->PerceptionSLBoundary(); // whether fcp obj is in frenet
                                                 // range or not
      if (std::abs(sl_boundary.start_s - ego_s) > 80 ||
          std::abs((sl_boundary.start_l + sl_boundary.end_l) / 2.0) > 5.0) {
        continue;
      }
      double speed_yaw = obj->Speed_yaw_relative_frenet();
      double obs_v_lon = obj->speed() * cos(speed_yaw);
      double obs_v_lat = obj->speed() * sin(speed_yaw);
      if (obs_v_lon < -4.0) { // suppose obs_v_lon always smaller that 5.0 in
                              // order to compensate perception error
        obs_v_lon = -4.0;
      }
      double TTC = std::min(10.0, (sl_boundary.start_s - ego_s > 0.0)
                                      ? (sl_boundary.start_s - ego_s) /
                                            (std::abs(obs_v_lon - ego_vel))
                                      : -100.0);
      prediction_traj.id = obj->Id();
      prediction_traj.type = ObjectType::COUPE;
      prediction_traj.coordinate = 1;
      prediction_traj.TTC = TTC;
      prediction_traj.direction = -1;
      prediction_traj.time_series.resize(PREDICTION_SIZE);
      prediction_traj.velocity.resize(PREDICTION_SIZE);
      prediction_traj.relative_sl.resize(PREDICTION_SIZE);
      if (TTC > 0.0 && TTC < (sl_boundary.start_s - ego_s) / ego_vel) {
        prediction_traj.meet_s = sl_boundary.start_s - ego_s + TTC * obs_v_lon;
        prediction_traj.enable = true;
        // std::cout <<"lxrdebug:start_s end_s = "<<sl_boundary.start_s<<"
        // "<<sl_boundary.end_s<<std::endl; std::cout <<"lxrdebug: egos s vlon
        // TTC meets = "<<ego_s<<" "<<sl_boundary.start_s<<" "<<obs_v_lon<<"
        // "<<TTC<<" "<<prediction_traj.meet_s<<std::endl;
        for (int i = 0; i < PREDICTION_SIZE; i++) {
          prediction_traj.time_series.at(i) = TTC * 1.5 / PREDICTION_SIZE * i;

          prediction_traj.velocity.at(i) =
              obs_v_lon; // Currently assuming that obj has constant velocity
          prediction_traj.relative_sl.at(i) =
              Point2D((sl_boundary.start_s - ego_s) +
                          prediction_traj.time_series.at(i) * (obs_v_lon),
                      sl_boundary.start_l);
        }
      } else {
        prediction_traj.enable = false;
      }

      obstacles_.Find(obj->Id())->SetPseudoPredictionTraj(prediction_traj);
      continue;
    }

    // prediction traj wrt cartesian
    if (world_model_->is_parking_svp() && obj->Type() == ObjectType::COUPE &&
        obj->IsLonOppositeWrtEgo() == 1 && obj->Is_angle_consistent() &&
        (obj->IsInBend() ||
         obj->IsBesideIntersection() &&
             !PlanningContext::Instance()->has_scene(
                 scene_avp,
                 ParkingSceneType::SCENE_RAMP))) { // opposite cars that far
                                                   // from planning frenet (add
                                                   // distance between ego and
                                                   // intersection )
      const auto &obj_box = obj->PerceptionBoundingBox();
      PseudoPredictionTrajectory prediction_traj;
      const Pose2D &obs_pose = obj->GetCartePosewrtEgo();
      const Point2D &obs_vel = obj->GetCarteVelwrtEgo();
      if (obstacles_.Find(obj->Id())->GetPseudoPredictionTraj().enable ==
          true) {
        continue;
      }
      if (obs_pose.x > 80.0 || std::abs(obs_pose.y) > 5.0) {
        continue;
      }
      double projection_factor = 1.0;
      double TTC = std::min(
          10.0, (obs_pose.x / obs_vel.x < 0)
                    ? (obs_pose.x - obj_box.length() / 2.0) /
                          std::abs(obs_vel.x - projection_factor * ego_vel)
                    : -100.0); // ego_vel is wrt frenet, a factor should be
                               // multiplied on ego_vel as projection wrt
                               // cartesian
      prediction_traj.id = obj->Id();
      prediction_traj.coordinate = 2;
      prediction_traj.type = ObjectType::COUPE;
      prediction_traj.TTC = TTC;
      prediction_traj.direction = -1;
      prediction_traj.time_series.resize(PREDICTION_SIZE);
      prediction_traj.velocity.resize(PREDICTION_SIZE);
      prediction_traj.relative_sl.resize(PREDICTION_SIZE);
      if (TTC > 0 && TTC < obs_pose.x / ego_vel) {
        prediction_traj.meet_s =
            obs_pose.x - obj_box.length() / 2.0 + TTC * obs_vel.x;
        double meet_s_in_frenet =
            std::max(0.0, std::min(prediction_traj.meet_s + ego_s,
                                   frenet_coord_->GetLength()));
        if (cos(frenet_coord_->GetRefCurveHeading(meet_s_in_frenet) -
                frenet_coord_->GetRefCurveHeading(ego_s)) > cos(PI / 3)) {
          prediction_traj.enable = true;
        } else {
          prediction_traj.enable = false;
          continue;
        }
        // std::cout <<"lxrdebug : x vx TTC meets = "<<obs_pose.x <<",
        // "<<obs_vel.x<<" "<<TTC<<" "<<prediction_traj.meet_s<<std::endl;
        for (int i = 0; i < PREDICTION_SIZE; i++) {
          prediction_traj.time_series.at(i) = TTC * 1.5 * i / PREDICTION_SIZE;
          prediction_traj.velocity.at(i) = obs_vel.x;
          prediction_traj.relative_sl.at(i) =
              Point2D(obs_pose.x - obj_box.length() / 2.0 +
                          prediction_traj.time_series.at(i) * obs_vel.x,
                      0.0);
        }
      } else {
        prediction_traj.enable = false;
      }
      obstacles_.Find(obj->Id())->SetPseudoPredictionTraj(prediction_traj);
      continue;
    }

    if (obj->Type() == ObjectType::PEDESTRIAN && obj->IsLonHighspeedWrtEgo() &&
        std::hypot(obj->GetCartePosewrtEgo().x, obj->GetCartePosewrtEgo().y) <
            7.0) {
      // std::cout<<"pedestrain prediction pose vel =
      // <"<<obj->GetCartePosewrtEgo().x<<", "<<obj->GetCartePosewrtEgo().y<<">
      // , <"<<obj->GetCarteVelwrtEgo().x<<",
      // "<<obj->GetCarteVelwrtEgo().y<<">"<<std::endl;
      PseudoPredictionTrajectory prediction_traj;
      prediction_traj.enable = true;
      prediction_traj.id = obj->Id();
      prediction_traj.type = ObjectType::PEDESTRIAN;
      prediction_traj.direction = 1;
      // calculate a pseudo prediction trajectory w.r.t Cartesian coodinate in
      // 10 seconds
      double time_interval = 0.5;
      double prediction_duration = 10.0;
      int prediction_size = int(prediction_duration / time_interval);
      for (int i = 0; i < prediction_size; i++) {
        prediction_traj.pose.emplace_back(
            Pose2D(obj->GetCartePosewrtEgo().x +
                       obj->GetCarteVelwrtEgo().x * i * time_interval,
                   obj->GetCartePosewrtEgo().y +
                       obj->GetCarteVelwrtEgo().y * i * time_interval,
                   obj->GetCartePosewrtEgo().theta));
        prediction_traj.velocity.emplace_back(obj->GetCarteVelwrtEgo().x);
        prediction_traj.time_series.emplace_back(i * time_interval);
        Point2D prediction_sl;
        Point2D prediction_xy;
        Point2D prediction_xy_rev;
        prediction_xy.x = ego_pose.x +
                          cos(ego_pose.theta) * prediction_traj.pose.at(i).x +
                          sin(ego_pose.theta) * prediction_traj.pose.at(i).y;
        prediction_xy.y = ego_pose.y -
                          sin(ego_pose.theta) * prediction_traj.pose.at(i).x +
                          cos(ego_pose.theta) * prediction_traj.pose.at(i).y;
        if (frenet_coord_->CartCoord2FrenetCoord(
                prediction_xy, prediction_sl) != TRANSFORM_FAILED)
          prediction_traj.relative_sl.emplace_back(
              Point2D(prediction_sl.x - ego_s, prediction_sl.y));
        else
          prediction_traj.relative_sl.emplace_back(Point2D(-100.0, -100.0));
      }
      obstacles_.Find(obj->Id())->SetPseudoPredictionTraj(prediction_traj);
      continue;
    }

    if (obj->Type() == ObjectType::COUPE && obj->IsLonHighspeedWrtEgo() &&
        std::abs(std::fmod(obj->GetCartePosewrtEgo().theta, PI)) < PI / 6.0 &&
        std::hypot(obj->GetCartePosewrtEgo().x, obj->GetCartePosewrtEgo().y) <
            10.0) {
      PseudoPredictionTrajectory prediction_traj;
      auto obj_box = obj->PerceptionBoundingBox();
      prediction_traj.enable = true;
      prediction_traj.id = obj->Id();
      prediction_traj.type = ObjectType::COUPE;
      prediction_traj.direction = 1;
      double time_interval = 0.5;
      double prediction_duration = 10.0;
      int prediction_size = int(prediction_duration / time_interval);
      for (int i = 0; i < prediction_size; i++) {
        prediction_traj.pose.emplace_back(Pose2D(
            obj->GetCartePosewrtEgo().x +
                cos(obj->GetCartePosewrtEgo().theta) * obj_box.length() / 2.0 +
                obj->GetCarteVelwrtEgo().x * i * time_interval,
            obj->GetCartePosewrtEgo().y +
                sin(obj->GetCartePosewrtEgo().theta) * obj_box.length() / 2.0 +
                obj->GetCarteVelwrtEgo().y * i * time_interval,
            obj->GetCartePosewrtEgo().theta));
        prediction_traj.velocity.emplace_back(obj->GetCarteVelwrtEgo().x);
        prediction_traj.time_series.emplace_back(i * time_interval);
        Point2D prediction_sl;
        Point2D prediction_xy;
        Point2D prediction_xy_rev;
        prediction_xy.x = ego_pose.x +
                          cos(ego_pose.theta) * prediction_traj.pose.at(i).x +
                          sin(ego_pose.theta) * prediction_traj.pose.at(i).y;
        prediction_xy.y = ego_pose.y -
                          sin(ego_pose.theta) * prediction_traj.pose.at(i).x +
                          cos(ego_pose.theta) * prediction_traj.pose.at(i).y;
        if (frenet_coord_->CartCoord2FrenetCoord(
                prediction_xy, prediction_sl) != TRANSFORM_FAILED)
          prediction_traj.relative_sl.emplace_back(
              Point2D(prediction_sl.x - ego_s, prediction_sl.y));
        else
          prediction_traj.relative_sl.emplace_back(Point2D(-100.0, -100.0));
      }
      obstacles_.Find(obj->Id())->SetPseudoPredictionTraj(prediction_traj);
    }
  }
  return true;
}

// process specific attributes for obstacle
bool ObstacleManager::obj_specific_manager() {
  if (false && world_model_->is_parking_lvp()) {
    return true;
  }
  const double &ego_s = world_model_->get_ego_state().ego_frenet.x;
  const Pose2D &ego_pose = world_model_->get_ego_state().ego_pose;
  const double &ego_vel = world_model_->get_ego_state().ego_vel;

  frenet_coord_ = world_model_->get_frenet_coord();
  for (auto &obj : obstacles_.Items()) {
    if (obj->Type() == ObjectType::COUPE &&
        (obj->IsBesideIntersection() || obj->IsInBend())) {
      double speed_yaw_frenet = obj->Speed_yaw_relative_frenet();
      double speed_yaw_ego = obj->Speed_yaw_relative_ego();
      const Pose2D &obj_pose = obj->GetCartePosewrtEgo();
      double dist_to_ego =
          std::sqrt(std::pow(obj_pose.x, 2) + std::pow(obj_pose.y, 2));
      // if( obj->Id()==4001339){
      //   SLBoundary sl_boundary;
      //   sl_boundary = obj->PerceptionSLBoundary();
      //   std::cout <<"lxrdebug: speed_yaw dist heading theta=
      //   "<<speed_yaw_frenet<<" "<<speed_yaw_ego<<" "<<dist_to_ego<< "
      //   "<<frenet_coord_->GetRefCurveHeading(sl_boundary.start_s) <<" "<<
      //   ego_pose.theta<<std::endl;
      // }
      if (beside_frenet_flag[obj->Id()] == 1) {
        obstacles_.Find(obj->Id())->SetIsAlongFrenet(0);
        return true;
      } else {
        if (obj->IsLonOppositeWrtEgo() == 1 &&
            (obj->IsLonOppositeWrtFrenet() != 1 ||
             obj->IsLatStaticWrtFrenet() != 1)) {
          obstacles_.Find(obj->Id())->SetIsAlongFrenet(0);
          beside_frenet_flag[obj->Id()] = 1;
          // std::cout <<"lxrdebug: flag settled !!!  "<<obj->Id()<<std::endl;
          return true;
        } else if (obj->IsLonOppositeWrtFrenet() == 1) {
          obstacles_.Find(obj->Id())->SetIsAlongFrenet(1);
        }
      }
    }
  }

  return true;
}

std::unordered_map<int, int>
ObstacleManager::obj_static_fusion_filter(double IoU_threshold,
                                          bool ralation_reverse) const {
  std::unordered_map<int, int> static_fusion_relation;
  for (auto &static_obs : static_obstacles_.Items()) {
    // std::cout<<static_obs->Id()<<std::endl;
    for (auto &obs : obstacles_.Items()) {
      if (obs->Type() == ObjectType::COUPE) {
        if (static_obs->PerceptionPolygon().DistanceTo(
                obs->PerceptionPolygon()) > 0.1)
          continue;
        if (static_obs->PerceptionPolygon().ComputeIoU(
                obs->PerceptionPolygon()) > IoU_threshold) {
          if (!ralation_reverse)
            static_fusion_relation[static_obs->Id()] = obs->Id();
          else
            static_fusion_relation[obs->Id()] = static_obs->Id();
        }
      }
    }
  }
  return static_fusion_relation;
}

} // namespace parking

} // namespace msquare
