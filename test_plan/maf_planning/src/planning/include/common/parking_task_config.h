#ifndef MSQUARE_DECISION_PLANNING_COMMON_PARKING_TASK_CONFIG_H_
#define MSQUARE_DECISION_PLANNING_COMMON_PARKING_TASK_CONFIG_H_

#include "mph_assert.h"
#include <string>
#include <yaml-cpp/yaml.h>

namespace msquare {

namespace parking {

class ParkingTaskConfig {
public:
  ParkingTaskConfig();
  ~ParkingTaskConfig() = default;

  bool LoadYAML();

public:
  bool enable_config = false;
  bool enable_reparkin = false;
  bool enable_control_test = false;
  int park_out_id = 0;
  int pick_up_id = 0;
  int park_in_id = 0;
  int current_task = 0;
  int done_id = 0;
  std::string parking_mode = "designate_parkin";
  std::string park_out_type = "PARKING_LOT";
  std::string pick_up_type = "HUMAN_ACCESS";
  std::string park_in_type = "PARKING_LOT";
  std::vector<int> available_id;
  std::vector<std::string> task_name_list;
  std::vector<int> task_type_list;
  std::vector<int> poi_id_list;
  std::vector<std::string> poi_type_list;
  std::vector<uint32_t> park_out_direction_list;
};

} // namespace parking

} // namespace msquare

#endif
