#ifndef MSQUARE_DECISION_PLANNING_COMMON_SCENARIO_MANAGER_H_
#define MSQUARE_DECISION_PLANNING_COMMON_SCENARIO_MANAGER_H_

#include <algorithm>
#include <iostream>
#include <memory>

#include "common/planning_context.h"
#include "planner/behavior_planner/deciders/backup_path_decider.h"
#include "planner/behavior_planner/deciders/lane_change_decider.h"
#include "planner/behavior_planner/deciders/path_decider.h"
#include "planner/behavior_planner/deciders/speed_boundary_decider.h"
#include "planner/behavior_planner/general_motion_planner.h"
#include "planner/behavior_planner/lateral_behavior_planner.h"
#include "planner/behavior_planner/longitudinal_behavior_planner.h"
#include "planner/message_type.h"
#include "planner/motion_planner/lateral_motion_planner.h"
#include "planner/motion_planner/longitudinal_motion_planner.h"
#include "planner/planning_config.h"
#include "world_model.h"

namespace msquare {

class ScenarioManager {
private:
  enum class ScenarioStatus : int {
    NOT_READY = 0,
    PRETREATMENT_FAIL = -1,
    READY = 1,
  };

public:
  explicit ScenarioManager(const std::shared_ptr<WorldModel> &world_model);
  ~ScenarioManager();

public:
  // execute planning algorithm
  bool execute_planning();

  // log debug info with mlog
  void log_debug_info();
  // output lat dec info for debug script
  void output_lat_dec_info();

  const std::shared_ptr<MSDStateMachine> &get_state_machine() const {
    return state_machine_;
  }

  const std::shared_ptr<WorldModel> &get_world_model() const {
    return world_model_;
  }

private:
  ScenarioStatus check_scenario_status();
  void create_lateral_decision_info_msg(std::string &msg);
  void create_lateral_decision_debug_msg(std::string &msg);
  void create_prediction_object_info_msg(std::string &msg);
  void create_map_planning_info_msg(std::string &msg);
  void create_acc_refline_msg(std::string &msg);

private:
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<MSDStateMachine> state_machine_;
  std::unique_ptr<msquare::ddp::DataDrivenPlanner> data_driven_planner_;
};

} // namespace msquare

#endif
