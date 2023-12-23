#include "common/parking_task_config.h"
#include "common/planning_config.h"
#include <cassert>
#include <iostream>

namespace msquare {

namespace parking {

using std::string;

ParkingTaskConfig::ParkingTaskConfig() {}

bool ParkingTaskConfig::LoadYAML() {
  std::string config_path =
      PlanningConfig::Instance()->config_files().parking_task_config_file;
  try {
    YAML::Node config = YAML::LoadFile(config_path);
    available_id.clear();
    task_name_list.clear();
    task_type_list.clear();
    poi_id_list.clear();
    poi_type_list.clear();
    for (const auto &id : config["available_id"]) {
      available_id.push_back(id.as<int>());
    }
    enable_config = config["enable_config"].as<bool>();
    enable_reparkin = config["enable_reparkin"].as<bool>();
    enable_control_test = config["enable_control_test"].as<bool>();
    current_task = config["current_task"].as<int>();
    parking_mode = config["parking_mode"].as<string>();
    park_out_id = config["poi_config"]["park_out"]["id"].as<int>();
    pick_up_id = config["poi_config"]["pick_up"]["id"].as<int>();
    park_in_id = config["poi_config"]["park_in"]["id"].as<int>();
    done_id = config["poi_config"]["exit"]["id"].as<int>();
    park_out_type = config["poi_config"]["park_out"]["type"].as<string>();
    pick_up_type = config["poi_config"]["pick_up"]["type"].as<string>();
    park_in_type = config["poi_config"]["park_in"]["type"].as<string>();
    for (const auto &task : config["parking_task_list"]) {
      // std::cout << "-------- task --------" << std::endl;
      // std::cout << " task_name: " << task["task_name"].as<string>() <<
      // std::endl; std::cout << " task_type: " << task["task_type"].as<int>()
      // << std::endl; std::cout << " poi_id: " << task["poi_id"].as<int>() <<
      // std::endl; std::cout << " poi_type: " << task["poi_type"].as<string>()
      // << std::endl;
      task_name_list.emplace_back(task["task_name"].as<string>());
      task_type_list.emplace_back(task["task_type"].as<int>());
      poi_id_list.emplace_back(task["poi_id"].as<int>());
      poi_type_list.emplace_back(task["poi_type"].as<string>());
      park_out_direction_list.emplace_back(
          task["park_out_direction"].as<uint32_t>());
    }
    mph_assert(task_name_list.size() == task_type_list.size());
    mph_assert(poi_id_list.size() == task_type_list.size());
    mph_assert(poi_id_list.size() == poi_type_list.size());

  } catch (const std::exception &e) {
    // std::cerr << "[Error]ParkingTaskConfig: " << e.what() << '\n';
    return false;
  }
  return true;
}

} // namespace parking

} // namespace msquare
