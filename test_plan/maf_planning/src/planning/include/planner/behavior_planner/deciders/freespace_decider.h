#ifndef MSQUARE_DECISION_PLANNING_PLANNER_FREEAPCE_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_FREEAPCE_DECIDER_H_

#include "common/math/line_segment2d.h"
#include "common/parking_obstacle_manager.h"
#include "common/parking_world_model.h"
#include "common/planning_context.h"
#include "common/utils/geometry.h"
#include "planner/message_type.h"
#include <chrono>

namespace msquare {
namespace parking {
class FreespaceDecider {
public:
  FreespaceDecider(const std::shared_ptr<WorldModel> &world_model);

  bool execute();

private:
  void set_map_boundary();
  bool get_freespace_vector();
  bool get_map_boundary();
  void feed_planning_context();

  std::vector<std::vector<std::pair<int, planning_math::Vec2d>>>
  freespace_filter(
      const std::vector<std::pair<int, planning_math::Vec2d>> &fs_pt_,
      const std::vector<planning_math::LineSegment2d> &boundarys,
      const Pose2D &ego_pose);

  std::vector<std::pair<int, planning_math::Vec2d>> fs_pt_;
  std::vector<planning_math::LineSegment2d> map_boundarys_;
  std::vector<std::vector<std::pair<int, planning_math::Vec2d>>> fs_pt_array_;
  std::shared_ptr<WorldModel> world_model_;

  const double FREESPACE_FILTER_DIST = 100.0;

  // for get map boundary
  double plan_start;

  const double SEND_INTERVAL = 2.0;
  const double MAP_BOX_LENGTH = 40.0;
  const double MAP_BOX_WIDTH = 10.0;
};
} // namespace parking
} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_FREEAPCE_DECIDER_H_
