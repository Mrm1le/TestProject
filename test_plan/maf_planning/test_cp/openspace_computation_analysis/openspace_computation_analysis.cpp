#include "common/planning_context.h"
#include "common/sbp_strategy.h"
#include "common/utils/yaml_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"
#include <iostream>

int main(int argc, char const *argv[]) {
  using namespace msquare::parking;
  std::string car_param_file = "/home/ros/catkin_ws/src/maf_planning/resource/"
                               "config/scenario_configs_json/parking/"
                               "vehicle_param.yaml";
  std::string config_file_name = "/home/ros/catkin_ws/src/maf_planning/"
                                 "resource/config/scenario_configs_json/"
                                 "parking/apa.yaml";

  if (!msquare::HybridAstarConfig::GetInstance()->loadFile(config_file_name)) {
    return false;
  }
  if (!msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(
          config_file_name)) {
    return false;
  }
  if (!msquare::CarParams::GetInstance()->loadFile(car_param_file)) {
    return false;
  }
  if (!msquare::CarParams::GetInstance()->loadFile4Plan(config_file_name)) {
    return false;
  }
  if (!msquare::StrategyParams::GetInstance()->loadFile(config_file_name)) {
    return false;
  }
  if (!msquare::VehicleParam::Instance()->loadFile(car_param_file)) {
    return false;
  }
  YAML::Node node = YAML::LoadFile(argv[1]);
  OpenspaceDeciderOutput osd = node.as<OpenspaceDeciderOutput>();
  msquare::SbpResult result;
  planWithStrategy(msquare::StrategyParams::GetInstance(), osd, result);
  return 0;
}
