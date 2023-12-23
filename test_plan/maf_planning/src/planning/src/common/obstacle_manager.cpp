#include "common/obstacle_manager.h"
#include "common/world_model.h"
#include "planner/motion_planner/common/window_smoother.h"

#include <cassert>

using namespace std;

namespace msquare {

bool ObstacleManager::init(const std::shared_ptr<WorldModel> &world_model) {
  mph_assert(nullptr != world_model);
  world_model_ = world_model;
  return true;
}

void ObstacleManager::clear() {
  world_model_ = nullptr;
  obstacles_ = IndexedList<int, Obstacle>();
  obstacle_id_hash_map_.clear();
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

bool ObstacleManager::set_st_boundary(int id, const STBoundary &boundary) {
  auto obstacle = obstacles_.Find(id);
  if (!obstacle) {
    // std::cerr << "Failed to find obstacle : " << id << std::endl;
    return false;
  } else {
    obstacle->set_path_st_boundary(boundary);
    return true;
  }
}

void ObstacleManager::erase_st_boundaries() {
  for (const auto *obstacle : obstacles_.Items()) {
    auto *obstacle_ptr = obstacles_.Find(obstacle->Id());
    if (obstacle_ptr != nullptr) {
      obstacle_ptr->EraseStBoundary();
    }
  }
  return;
}

} // namespace msquare
