#ifndef COMMON_CONFIG_SCENARIO_FACADE_CONFIG_H_
#define COMMON_CONFIG_SCENARIO_FACADE_CONFIG_H_

#include "common/config/task_config.h"
#include <boost/optional.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace msquare {

using namespace boost;

// specific scenario configuration
// need to customize
using TasksConfigMap =
    std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>;
using TasksConfigVector = std::vector<TaskConfig>;
typedef struct {
  std::string level{"normal"};
} FixLaneCruiseScenarioFacadeConfig;

typedef struct {
  std::string level{"normal"};
} ChangeLaneCruiseScenarioFacadeConfig;

typedef struct {
  std::string level{"normal"};
} ChangelanePreparationScenarioFacadeConfig;

typedef struct {
  std::string level{"normal"};
} InterSectionPassScenarioFacadeConfig;

typedef struct {
  std::string level{"normal"};
} UnprotectedLeftTurnScenarioFacadeConfig;

class ScenarioFacadeConfig {
public:
  explicit ScenarioFacadeConfig(){};

  ~ScenarioFacadeConfig() = default;

  typedef enum {
    FixLaneCruise = 1,
    ChangeLaneCruise = 2,
    ChangelanePreparation = 3,
    InterSectionPass = 4,
    UnprotectedLeftTurn = 5,
    ScenarioFacadeLast = 6,
  } ScenarioFacadeType;

  ScenarioFacadeType scenario_facade_type_ = {};
  optional<FixLaneCruiseScenarioFacadeConfig> fixlane_cruise_scenario_config_;
  optional<ChangeLaneCruiseScenarioFacadeConfig>
      change_lane_cruise_scenario_config_;
  optional<ChangelanePreparationScenarioFacadeConfig>
      change_lane_preparation_scenario_config_;
  optional<InterSectionPassScenarioFacadeConfig>
      intersection_pass_scenario_config_;
  optional<UnprotectedLeftTurnScenarioFacadeConfig>
      unprotected_left_turn_scenario_config_;
  TasksConfigVector tasks_config_vector_;
  TasksConfigMap tasks_config_map_;
};

const static std::string ScenarioFacadeNames
    [static_cast<int>(
         ScenarioFacadeConfig::ScenarioFacadeType::ScenarioFacadeLast) -
     1] = {"FixLaneCruise", "ChangeLaneCruise", "ChangelanePreparation",
           "InterSectionPass", "UnprotectedLeftTurn"};

} // namespace msquare

#endif // COMMON_CONFIG_SCENARIO_CONFIG_H_
