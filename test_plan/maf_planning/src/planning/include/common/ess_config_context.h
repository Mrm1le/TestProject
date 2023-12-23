#pragma once

#include "common/utils/macro.h"
#include "nlohmann/json.hpp"
#include <fstream>

namespace msquare {

struct EssPlannerConfig {
  double near_road_edge_thres = 3.5;
  double near_object_lateral_thres = 3.5;
  double near_forward_object_ttc = 5.0;
  double near_forward_object_distance = 50.0;
  double near_backward_object_ttc = 5.0;
  double near_backward_object_distance = 50.0;

  double driver_intention_torque = 2.0;
  double driver_intention_delta_torque = 1.0;
  double driver_override_torque = 3.0;
  double driver_longitudinal_safety_thres = 2.5;
  double driver_lateral_safety_thres = 1.0;

  double planning_longitudinal_safety_thres = 2.5;
  double planning_exit_exceed_distance = 5.0;
  double planning_min_ttc_thres = 1.0;
  double planning_lateral_safety_thres = 1.0;
  double planning_actuator_delay_time = 0.3;
  double planning_max_lateral_acc_thres = 8.0;
  double planning_max_trigger_time_sec = 5.0;
  double planning_cooling_time_sec = 15.0;

  int fcw_count_thres = 10;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    EssPlannerConfig, near_road_edge_thres, near_object_lateral_thres,
    near_forward_object_ttc, near_forward_object_distance,
    near_backward_object_ttc, near_backward_object_distance,
    driver_intention_torque, driver_intention_delta_torque,
    driver_override_torque, driver_longitudinal_safety_thres,
    driver_lateral_safety_thres, planning_longitudinal_safety_thres,
    planning_exit_exceed_distance, planning_min_ttc_thres,
    planning_lateral_safety_thres, planning_actuator_delay_time,
    planning_max_lateral_acc_thres, planning_max_trigger_time_sec,
    planning_cooling_time_sec, fcw_count_thres)

class EssConfigurationContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(EssConfigurationContext);

public:
  const EssPlannerConfig &ess_planner_config() { return ess_planner_config_; }

  void load_ess_planner_config(const std::string &config_file_dir) {
    std::string config_file_name = "ess_planner_config.json";
    std::ifstream fjson(config_file_dir +
                        "/scenario_configs_json/planner_config/" +
                        config_file_name);
    std::string json_str((std::istreambuf_iterator<char>(fjson)),
                         std::istreambuf_iterator<char>());
    nlohmann::json input_json = nlohmann::json::parse(json_str);
    ess_planner_config_ = input_json;
  }

private:
  EssPlannerConfig ess_planner_config_{};
};

} // namespace msquare
