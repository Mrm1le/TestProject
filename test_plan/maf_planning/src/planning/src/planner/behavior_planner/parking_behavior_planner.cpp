#include "planner/behavior_planner/parking_behavior_planner.h"

namespace msquare {
namespace parking {

BehaviorPlanner::BehaviorPlanner(const std::shared_ptr<WorldModel> &world_model)
    : Planner(world_model) {}

BehaviorPlanner::~BehaviorPlanner() {}
} // namespace parking
} // namespace msquare
