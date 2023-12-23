#include "common/obstacle_decision_manager.h"

namespace msquare {

void ObstacleDecisionManager::clear() {
  obstacles_decision_ = IndexedList<int, ObstacleDecision>();
}

ObstacleDecision *ObstacleDecisionManager::add_obstacle_decision(
    const ObstacleDecision &obstacle_decision) {
  return obstacles_decision_.Add(obstacle_decision.Id(), obstacle_decision);
}

const ObstacleDecision *
ObstacleDecisionManager::find_obstacle_decision(int object_id) const {
  return obstacles_decision_.Find(object_id);
}

ObstacleDecision *
ObstacleDecisionManager::find_obstacle_decision(int object_id) {
  return obstacles_decision_.Find(object_id);
}

const IndexedList<int, ObstacleDecision> &
ObstacleDecisionManager::get_obstacles_decision() const {
  return obstacles_decision_;
}

const std::vector<const ObstacleDecision *>
ObstacleDecisionManager::get_object_decision(int object_id) const {
  std::vector<const ObstacleDecision *> res;
  if (obstacle_id_hash_map_.find(object_id) == obstacle_id_hash_map_.end()) {
    return res;
  } else {
    for (auto id : obstacle_id_hash_map_.at(object_id)) {
      //   auto ptr_obs_decision = find_obstacle_decision(id);
      res.push_back(find_obstacle_decision(id));
    }
    return res;
  }
  return res;
}

void ObstacleDecisionManager::set_hash_id_map(
    const std::unordered_map<int, std::vector<int>> &hash_id_map) {
  obstacle_id_hash_map_ = hash_id_map;
}

bool ObstacleDecisionManager::add_lateral_decision(
    const std::string &tag, int id, const ObjectDecisionType &decision) {
  auto obstacle_decision = obstacles_decision_.Find(id);
  if (!obstacle_decision) {
    // std::cerr << "failed to find obstacle decision" << std::endl;
    return false;
  }
  obstacle_decision->AddLateralDecision(tag, decision);
  return true;
}

bool ObstacleDecisionManager::add_longitudinal_decision(
    const std::string &tag, int id, const ObjectDecisionType &decision) {
  auto obstacle_decision = obstacles_decision_.Find(id);
  if (!obstacle_decision) {
    // std::cerr << "Failed to find obstacle" << std::endl;
    return false;
  }
  obstacle_decision->AddLongitudinalDecision(tag, decision);
  return true;
}

} // namespace msquare