#pragma once

#include <memory>

#include "common/math/math_utils.h"
#include "common/parking_world_model.h"

namespace msquare {
namespace parking {

struct ParkoutSceneType {
  bool is_front_collision;
  bool is_both_side_collision;
};

class APABehaviorCalculatorParkOut {

public:
  explicit APABehaviorCalculatorParkOut(
      const std::shared_ptr<WorldModel> &world_model);
  ~APABehaviorCalculatorParkOut();

public:
  bool calculate_reached_takeover_point(bool is_at_last_segment,
                                        bool openspace_is_running);

  bool calculate_parkout_target(Pose2D &out_target_pose,
                                Pose2D &out_error_tolerence);

  uint32_t calculate_available_parkout_directions(
      const ParkoutSceneType &parkout_scene_res);

  uint32_t
  calculate_recommended_parkout_direction(uint32_t available_directions);

  ParkoutSceneType
  calc_parkout_scene_type(const ParkingSlotInfo &parking_slot_info);

private:
  bool area_has_obstacle(const planning_math::Box2d &area);

  bool check_parkout_direction(uint32_t direction);

  bool calc_parkout_target_perpendicular_oblique(Pose2D &out_target_pose,
                                                 Pose2D &out_error_tolerence,
                                                 bool oblique);

  bool calc_parkout_target_parallel(Pose2D &target_pose,
                                    Pose2D &error_tolerence);

  bool is_slot_too_narrow_for_parkout(const ParkingSlotInfo &parking_slot_info);

  bool calc_two_size_length(const ParkingSlotInfo &parking_slot_info,
                            double &x_min_right, double &x_max_left);

private:
  std::shared_ptr<WorldModel> world_model_;
};

} // namespace parking
} // namespace msquare
