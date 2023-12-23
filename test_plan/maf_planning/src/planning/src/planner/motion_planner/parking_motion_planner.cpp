#include "planner/motion_planner/parking_motion_planner.h"

namespace msquare {
namespace parking {
MotionPlanner::MotionPlanner(const std::shared_ptr<WorldModel> &world_model)
    : Planner(world_model) {}
} // namespace parking
} // namespace msquare
