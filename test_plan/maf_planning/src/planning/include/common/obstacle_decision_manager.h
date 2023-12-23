#ifndef MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_DECISION_MANAGER_H_
#define MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_DECISION_MANAGER_H_

#include <limits>
#include <memory>
#include <string>

#include "common/obstacle_decision.h"
#include "common/utils/index_list.h"

namespace msquare {

class ObstacleDecisionManager {
public:
  void clear();

  ObstacleDecision *
  add_obstacle_decision(const ObstacleDecision &obstacle_decision);

  const ObstacleDecision *find_obstacle_decision(int object_id) const;

  ObstacleDecision *find_obstacle_decision(int object_id);

  const IndexedList<int, ObstacleDecision> &get_obstacles_decision() const;

  const std::vector<const ObstacleDecision *>
  get_object_decision(int object_id) const;

  bool add_lateral_decision(const std::string &tag, int id,
                            const ObjectDecisionType &decision);

  bool add_longitudinal_decision(const std::string &tag, int id,
                                 const ObjectDecisionType &decision);

  void
  set_hash_id_map(const std::unordered_map<int, std::vector<int>> &hash_id_map);

private:
  IndexedList<int, ObstacleDecision> obstacles_decision_;
  std::unordered_map<int, std::vector<int>> obstacle_id_hash_map_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_COMMON_OBSTACLE_DECISION_MANAGER_H_
