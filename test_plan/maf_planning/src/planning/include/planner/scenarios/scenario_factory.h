#ifndef MSQUARE_DECISION_PLANNING_PLANNER_SCENARIO_FACADE_FACOTRY_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_SCENARIO_FACADE_FACOTRY_H_

#include <functional>
#include <memory>
#include <unordered_map>

#include "common/config/scenario_facade_config.h"
#include "common/config_context.h"
#include "common/utils/factory.h"
#include "memory/simple_mempool.hpp"
#include "planner/scenarios/scenario_facade.h"

namespace msquare {

class ScenarioFactory {
public:
  static void Init(const ScenarioConfigMap &default_scenario_config);
  static std::shared_ptr<ScenarioFacade>
  CreateScenarioFacade(const ScenarioFacadeConfig &scenario_facade_config);
  static std::shared_ptr<ScenarioFacade> ProduceScenarioWithSpecifiedType(
      const ScenarioFacadeConfig::ScenarioFacadeType &type);
  static std::unordered_map<std::string, memory::MemoryPoolPtr> &
  get_task_mempory_pools();

private:
  static util::Factory<
      ScenarioFacadeConfig::ScenarioFacadeType, ScenarioFacade,
      std::function<ScenarioFacade *(const ScenarioFacadeConfig &config)>,
      std::function<void(ScenarioFacade *scenPtr)>>
      scenario_facade_factory_;
  static std::unordered_map<ScenarioFacadeConfig::ScenarioFacadeType,
                            ScenarioFacadeConfig, std::hash<int>>
      default_scenario_facade_configs_;

  static std::unordered_map<std::string, memory::MemoryPoolPtr>
      task_mempory_pools_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_SCENARIO_FACADE_FACOTRY_H_
