#include "common/config/vehicle_param.h"
#include "common/sbp_strategy.h"
// #include "common/utils/yaml_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <iostream>
#include "util.h"

int main(int argc, char const *argv[]) {
  using namespace msquare::parking;
  OpenspaceDeciderOutput osd;
  // YAML::Node node = YAML::LoadFile(argv[1]);
  std::ifstream fs;
  fs.open(argv[1]);
  std::string json_str;
  fs >> json_str;
  feedRequest (json_str, osd);
  msquare::SbpResult result;
  if (genOpenspacePath(osd, result, nullptr)) {
    if (!msquare::parking::GetTemporalProfile(&result)) {
      std::cout << "GetSpeedProfile from Hybrid Astar path fails" << std::endl;
    }
  }
  return 0;
}