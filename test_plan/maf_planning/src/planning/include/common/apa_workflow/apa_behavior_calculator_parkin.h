#pragma once

#include <memory>

#include "common/math/math_utils.h"
#include "common/parking_world_model.h"

namespace msquare {
namespace parking {

class APABehaviorCalculatorParkIn {

public:
  explicit APABehaviorCalculatorParkIn(
      const std::shared_ptr<WorldModel> &world_model);
  ~APABehaviorCalculatorParkIn();

public:
  bool calculate_approaching_wheel_stop(double distance_to_end,
                                        bool openspace_is_running);

  bool calculate_blocked_base_scene(bool openspace_is_running);

  bool calculate_hit_sth_base_scene(bool openspace_is_running);

  bool calculate_times_try_parkin(bool base_scene, bool is_at_last_segment,
                                  std::size_t *ptr_times_try_parking_in);

  bool
  calculate_blocked_by_obstacle_behind_in_slot(bool blocked_base_scene,
                                               const Pose2D &target_pose_lot);

  bool
  calculate_need_update_parking_slot_corners(const Pose2D &target_pose_lot);
  bool calculate_need_check_mpc_collide(const bool need_update_slot_corners,
                                        bool is_at_last_segment);
  bool
  calculate_need_dynamic_plan_adjust_tail(const bool need_check_mpc_collide);

private:
  std::shared_ptr<WorldModel> world_model_;

private:
  bool isPointBehindEgoUnsafe(const planning_math::Vec2d &point,
                              const Pose2D &ego_pose,
                              const Pose2D &target_pose);

  bool isGroundlineBehindEgoUnsafe(const std::vector<GroundLine> &groundline,
                                   const Pose2D &ego_pose,
                                   const Pose2D &target_pose);
};

} // namespace parking
} // namespace msquare
