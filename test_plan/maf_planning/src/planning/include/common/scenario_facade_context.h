#ifndef COMMON_SCENARIO_FACADE_CONTEXT_
#define COMMON_SCENARIO_FACADE_CONTEXT_

#include "common/config/scenario_facade_config.h"
#include "common/obstacle_decision_manager.h"
#include "common/planning_context.h"
#include "common/speed/st_graph_data.h"
#include "common/world_model.h"

namespace msquare {

class ScenarioFacadeContext {

public:
  ScenarioFacadeContext() = default;
  const PlanningStatus &planning_status() const { return planning_status_; }
  PlanningStatus *mutable_planning_status() { return &planning_status_; }

  const LongitudinalBehaviorPlannerOutput &
  longitudinal_behavior_planner_output() const {
    return longitudinal_behavior_planner_output_;
  }
  LongitudinalBehaviorPlannerOutput *
  mutable_longitudinal_behavior_planner_output() {
    return &longitudinal_behavior_planner_output_;
  }

  path_planner::PathPlannerInput *mutable_path_planner_input() {
    return &path_planner_input_;
  }
  const path_planner::PathPlannerInput &path_planner_input() const {
    return path_planner_input_;
  }

  PlannerDebug *mutable_planner_debug() { return &planner_debug_; }
  const PlannerDebug &planner_debug() const { return planner_debug_; }

  speed_planner::SpeedPlannerInput *mutable_speed_planner_input() {
    return &speed_planner_input_;
  }
  const speed_planner::SpeedPlannerInput &speed_planner_input() const {
    return speed_planner_input_;
  }

  speed_planner::LonDecisionOutput *mutable_lon_decison_output() {
    return &lon_decison_output_;
  }
  const speed_planner::LonDecisionOutput &lon_decison_output() const {
    return lon_decison_output_;
  }

  speed_planner::SpeedPlannerOutput *mutable_speed_planner_output() {
    return &speed_planner_output_;
  }
  const speed_planner::SpeedPlannerOutput &speed_planner_output() const {
    return speed_planner_output_;
  }

  gmp_interface::GeneralMotionPlannerOutput *
  mutable_general_motion_planner_output() {
    return &general_motion_planner_output_;
  }
  const gmp_interface::GeneralMotionPlannerOutput &
  general_motion_planner_output() const {
    return general_motion_planner_output_;
  }

  const SpeedData &longitudinal_motion_planner_output() const {
    return speed_data_;
  }
  SpeedData *mutable_longitudinal_motion_planner_output() {
    return &speed_data_;
  }

  const SpeedLimit &speed_limit() const { return speed_limit_; }
  SpeedLimit *mutable_speed_limit() { return &speed_limit_; }

  const LateralBehaviorPlannerOutput &lateral_behavior_planner_output() const {
    return lateral_behavior_planner_output_;
  }

  LateralBehaviorPlannerOutput &mutable_lateral_behavior_planner_output() {
    return lateral_behavior_planner_output_;
  }

  const LateralMotionPlannerOutput &lateral_motion_planner_output() const {
    return lateral_motion_planner_output_;
  }

  LateralMotionPlannerOutput &mutable_lateral_motion_planner_output() {
    return lateral_motion_planner_output_;
  }

  const ObstacleDecisionManager &obstacle_decision_manager() const {
    return obstacle_decision_manager_;
  }

  ObstacleDecisionManager &mutable_obstacle_decision_manager() {
    return obstacle_decision_manager_;
  }

  const StGraphData &st_graph_data() const { return st_graph_data_; }

  StGraphData &mutable_st_graph_data() { return st_graph_data_; }

  const IndexedList<int, Obstacle> &get_virtual_obstacles() const {
    return virtual_obstacles_;
  }

  Obstacle *add_virtual_obstacle(const Obstacle &obstacle) {
    return virtual_obstacles_.Add(obstacle.Id(), obstacle);
  }

  Obstacle *find_virtual_obstacle(int object_id) {
    return virtual_obstacles_.Find(object_id);
  }

  const Obstacle *find_virtual_obstacle(int object_id) const {
    return virtual_obstacles_.Find(object_id);
  }

  const MSDStateMachineOutput &state_machine_output() const {
    return state_machine_output_;
  }

  MSDStateMachineOutput &mutable_state_machine_output() {
    return state_machine_output_;
  }

private:
  ScenarioFacadeConfig::ScenarioFacadeType scenario_facade_type_;
  PlanningStatus planning_status_;
  path_planner::PathPlannerInput path_planner_input_;
  speed_planner::LonDecisionOutput lon_decison_output_;
  speed_planner::SpeedPlannerInput speed_planner_input_;
  speed_planner::SpeedPlannerOutput speed_planner_output_;
  gmp_interface::GeneralMotionPlannerOutput general_motion_planner_output_;
  LongitudinalBehaviorPlannerOutput longitudinal_behavior_planner_output_;
  LateralBehaviorPlannerOutput lateral_behavior_planner_output_;
  LateralMotionPlannerOutput lateral_motion_planner_output_;
  SpeedData speed_data_;
  SpeedLimit speed_limit_;
  ObstacleDecisionManager obstacle_decision_manager_;
  StGraphData st_graph_data_;

  IndexedList<int, Obstacle> virtual_obstacles_;
  PlannerDebug planner_debug_;
  MSDStateMachineOutput state_machine_output_;
};

} // namespace msquare

#endif // COMMON_SCENARIO_FACADE_CONTEXT_
