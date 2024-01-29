#pragma once

#include <memory>

#include "common/parking_world_model.h"
#include "common/utils/timer.h"
#include "planner/behavior_planner/parking_longitudinal_behavior_planner.h"

namespace msquare {
namespace parking {

enum class APAStrategyType : unsigned int {
  NONE = 0,
  REPLAN,
  DYNAMIC_PLAN_ADJUST_TAIL,
  DYNAMIC_PLAN_SIMPLE_MPC,
  PAUSE,
  ABANDON,
  FINISH,
  WAIT_FOR_WLC, // wait for wlc confirm before final finish
  STRATEGY_NUM
};

class APABehaviorDeciderParkIn {
public:
  struct DeciderResult {
    APAStrategyType strategy;
    std::string reason;
  };

public:
  explicit APABehaviorDeciderParkIn(
      const std::shared_ptr<WorldModel> &world_model,
      const std::shared_ptr<ParkingLongitudinalBehaviorPlanner>
          &parking_longitudinal_behavior_planner);
  ~APABehaviorDeciderParkIn();

public:
  DeciderResult strategy_hit_wheelstop(bool openspace_is_running,
                                       bool vehicle_reached_no_wheelstop);

  DeciderResult strategy_wheelstop_in_parallel_slot(bool openspace_is_running);

  DeciderResult
  strategy_blocked_and_reached(bool vehicle_reached_with_wheelstop);

  DeciderResult
  strategy_tiny_perpendicular_and_oblique_slot(bool is_at_last_segment);

  DeciderResult
  strategy_dynamic_plan_adjust_tail(bool need_dynamic_plan_adjust_tail);

  DeciderResult strategy_dynamic_plan_simple_mpc(double distance_to_end,
                                                 bool is_at_last_segment,
                                                 bool openspace_is_running);

  DeciderResult strategy_dynamic_plan_simple_mpc_collide_timeout(
      bool dynamic_plan_simple_mpc_enabled,
      bool dynamic_plan_simple_mpc_traj_success,
      bool dynamic_plan_simple_mpc_traj_collide);

  DeciderResult strategy_mpc_collide(bool need_check_mpc_collide);

  DeciderResult strategy_openspace_fallback(bool openspace_is_fallback);

  DeciderResult strategy_blocked(bool blocked_base_scene);

  DeciderResult
  strategy_traj_following_finish(bool openspace_is_finish,
                                 bool vehicle_reached_with_wheelstop);

  DeciderResult strategy_first_reverse_gear_switch(bool is_at_last_segment);

  DeciderResult strategy_assumed_blocked();

  void reset_mpc_collide_timeout();

  DeciderResult strategy_during_pause(
      const bool is_pause_status, const double trajectory_length,
      const bool is_at_last_segment, const bool vehicle_reached_with_wheelstop,
      const bool has_moved, const bool need_pause,
      const LongitudinalBehaviorPlannerOutput::RemainDistInfo
          &remain_distance_info,
      std::string *ptr_debug_string);

private:
  struct StateParkingConfig {
    double apa_simple_mpc_entry_threshold_;
  };

  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<ParkingLongitudinalBehaviorPlanner>
      parking_longitudinal_behavior_planner_;

  int times_tiny_slot_overlap_ = 0;

  StateParkingConfig state_parking_cfg_;

  bool is_dynamic_planning_activated_ = false;

  Timer dynamic_plan_mpc_block_timer_;

private:
  // check if slot has left/right car
  bool checkTwoSides(bool &is_left_car, bool &is_right_car);
};

} // namespace parking
} // namespace msquare
