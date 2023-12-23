#include "data_driven_planner/common/planning_result_manager.h"
#include "data_driven_planner/common/ddp_utils.h"

namespace msquare {
namespace ddp {

void PlanningResultManager::add_planning_result(
    const PlanningResult &planning_result) {
  planning_results.emplace_back(planning_result);

  if (planning_results.size() > 10) {
    planning_results.pop_front();
  }
}

bool PlanningResultManager::get_planning_ego_pose(double timestamp,
                                                  EgoPose *ego_pose) {
  bool found = false;
  constexpr double kPlanningResultShelfLife = 0.3;
  for (auto it = planning_results.rbegin();
       it != planning_results.rend() && !found; ++it) {
    auto &traj_points = it->traj_points;
    auto relative_time = timestamp - it->timestamp;
    if (traj_points.front().t <= relative_time &&
        relative_time <= traj_points.back().t &&
        traj_points.front().t + kPlanningResultShelfLife > relative_time) {
      for (size_t i = 0; i < traj_points.size() - 1; i++) {
        auto &pre_pt = traj_points[i];
        auto &next_pt = traj_points[i + 1];
        if (pre_pt.t <= relative_time && relative_time <= next_pt.t) {
          ego_pose->timestamp = timestamp;
          ego_pose->v = interpolate(pre_pt.t, pre_pt.v, next_pt.t, next_pt.v,
                                    relative_time);
          ego_pose->a = interpolate(pre_pt.t, pre_pt.a, next_pt.t, next_pt.a,
                                    relative_time);
          ego_pose->position.x = interpolate(pre_pt.t, pre_pt.x, next_pt.t,
                                             next_pt.x, relative_time);
          ego_pose->position.y = interpolate(pre_pt.t, pre_pt.y, next_pt.t,
                                             next_pt.y, relative_time);
          ego_pose->position.z = 0;
          ego_pose->heading_angle =
              interpolate_angle(pre_pt.t, pre_pt.heading_angle, next_pt.t,
                                next_pt.heading_angle, relative_time);
          ego_pose->velocity.x =
              ego_pose->v * std::cos(ego_pose->heading_angle);
          ego_pose->velocity.y =
              ego_pose->v * std::sin(ego_pose->heading_angle);
          ego_pose->velocity.z = 0;
          found = true;
          MSD_LOG(
              INFO,
              "get_planning_ego_pose successed, index: %d, relative time: %f",
              i, relative_time);
          break;
        }
      }
    }
  }

  return found;
}

} // namespace ddp
} // namespace msquare
