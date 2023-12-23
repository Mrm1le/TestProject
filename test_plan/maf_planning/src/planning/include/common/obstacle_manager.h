#ifndef MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_MANAGER_H_
#define MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_MANAGER_H_

#include <limits>
#include <memory>
#include <string>

#include "common/config/obs_manager_config.h"
#include "common/lateral_obstacle.h"
#include "common/obstacle.h"
#include "common/tracked_object.h"

namespace msquare {

// class WorldModel;

class ObstacleManager {
public:
  // ObstacleManager();
  //~ObstacleManager() = default;

  bool init(const std::shared_ptr<WorldModel> &world_model);

  void clear();

  Obstacle *add_obstacle(const Obstacle &obstacle);

  const Obstacle *find_obstacle(int object_id) const;

  Obstacle *find_obstacle(int object_id);

  const IndexedList<int, Obstacle> &get_obstacles() const;

  void erase_st_boundaries();

  std::unordered_map<int, std::vector<int>> &mutable_hash_id_map() {
    return obstacle_id_hash_map_;
  }

  const std::unordered_map<int, std::vector<int>> &hash_id_map() const {
    return obstacle_id_hash_map_;
  }

private:
  bool set_st_boundary(int id, const STBoundary &boundary);

private:
  std::shared_ptr<WorldModel> world_model_;
  IndexedList<int, Obstacle> obstacles_;
  std::unordered_map<int, std::vector<int>> obstacle_id_hash_map_;
};

} // namespace msquare

#endif
