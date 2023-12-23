#pragma once
#include "common/utils/macro.h"

namespace msquare {

struct ConfigFiles {
  std::string vehicle_param_file;
  std::string parking_task_config_file;
  std::string openspace_decider_config_file;
  std::string openspace_decider_log_path;
  std::string openspace_apa_planner_config_file;
  std::string openspace_apa_sop_planner_config_file;
  std::string log_path;
  std::string legacy_openspace_apoa_planner_config_file;
  std::string legacy_openspace_apoa_parallel_planner_config_file;
  std::string openspace_apoa_planner_config_file;
  std::string openspace_apa_parallel_planner_config_file;
  std::string openspace_apa_parallel_sop_planner_config_file;
  std::string openspace_apoa_parallel_planner_config_file;
  std::string openspace_avp_planner_config_file;
  std::string openspace_rpa_straight_planner_config_file;
  std::string teb_config_file;
  std::string teb_footprint_file;
  std::string parking_lot_config_file;
  std::string parallel_parking_slot_config_file;
  std::string lot_config_pre_apa_file;
  std::string trajectory_file;
  std::string state_parking_config_file;
  std::string parking_lateral_behavior_planner_config_file;
  std::string teb_openspace_decider_config_file;
  std::string via_point_decider_config_file;
  std::string apf_decider_config_file;
};

class PlanningConfig {
private:
  // this is a singleton class
  DECLARE_SINGLETON(PlanningConfig);

public:
  ConfigFiles *mutable_config_files() { return &config_files_; }
  const ConfigFiles &config_files() const { return config_files_; }

private:
  ConfigFiles config_files_;
};

} // namespace msquare
