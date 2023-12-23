#include "planner/scenarios/scenario_factory.h"
#include "planner/scenarios/scenario_facade.h"

namespace msquare {

#define MAP_ITEM(s) std::make_pair(#s, s)
#define REGIST_SCENA                                                                                    \
[](const ScenarioFacadeConfig &config) -> ScenarioFacade * {                                          \
  return (ScenarioFacade *)ScenarioFactory::get_task_mempory_pools()["scefac_pool"]->Alloc(&config);  \
},                                                                                                    \
[](ScenarioFacade *sce_ptr) -> void {                                                                 \
  ScenarioFactory::get_task_mempory_pools()["scefac_pool"]->Free(sce_ptr);                            \
});

std::unordered_map<std::string, memory::MemoryPoolPtr>
    ScenarioFactory::task_mempory_pools_;

util::Factory<
    ScenarioFacadeConfig::ScenarioFacadeType, ScenarioFacade,
    std::function<ScenarioFacade *(const ScenarioFacadeConfig &config)>,
    std::function<void(ScenarioFacade *scenPtr)>>
    ScenarioFactory::scenario_facade_factory_;

std::unordered_map<ScenarioFacadeConfig::ScenarioFacadeType,
                   ScenarioFacadeConfig, std::hash<int>>
    ScenarioFactory::default_scenario_facade_configs_;

std::unordered_map<std::string, memory::MemoryPoolPtr> &
ScenarioFactory::get_task_mempory_pools() {
  return task_mempory_pools_;
}

void ScenarioFactory::Init(const ScenarioConfigMap &default_scenario_config) {

  /*init memory pool, *** be careful concurrent!! ***/
  if (task_mempory_pools_.size() == 0) {
    auto scefac_pool = std::make_shared<
        memory::SimpleMemoryPool<ScenarioFacade, ScenarioFacadeConfig>>(8);
    task_mempory_pools_.insert(MAP_ITEM(scefac_pool));
  }

  scenario_facade_factory_.Clear();
  (void)scenario_facade_factory_.Register(
      ScenarioFacadeConfig::ScenarioFacadeType::FixLaneCruise,
      REGIST_SCENA

  (void)scenario_facade_factory_.Register(
      ScenarioFacadeConfig::ScenarioFacadeType::ChangeLaneCruise,
      REGIST_SCENA

  (void)scenario_facade_factory_.Register(
      ScenarioFacadeConfig::ScenarioFacadeType::ChangelanePreparation,
      REGIST_SCENA

  default_scenario_facade_configs_ = default_scenario_config;
}

std::shared_ptr<ScenarioFacade> ScenarioFactory::CreateScenarioFacade(
    const ScenarioFacadeConfig &scenario_config) {
  return scenario_facade_factory_.CreateObject(
      scenario_config.scenario_facade_type_, scenario_config);
}

std::shared_ptr<ScenarioFacade>
ScenarioFactory::ProduceScenarioWithSpecifiedType(
    const ScenarioFacadeConfig::ScenarioFacadeType &type) {
  mph_assert(default_scenario_facade_configs_.find(type) !=
             default_scenario_facade_configs_.end());
  ScenarioFacadeConfig scenario_config =
      default_scenario_facade_configs_.at(type);
  // ConfigurationContext::Instance()->get_params().at(type);
  auto scenario_instance =
      ScenarioFactory::CreateScenarioFacade(scenario_config);
  scenario_instance->init();
  // preset scenario context
  auto scenario_context = scenario_instance->get_scenario_context();
  // todo(@zzd) construct obs_decision_manager in scenario context
  *scenario_context->mutable_planning_status() =
      PlanningContext::Instance()->planning_status();
  scenario_context->mutable_planning_status()->planning_success = false;
  scenario_context->mutable_lateral_behavior_planner_output() =
      PlanningContext::Instance()->lateral_behavior_planner_output();
  scenario_context->mutable_lateral_motion_planner_output() =
      PlanningContext::Instance()->lateral_motion_planner_output();
  *scenario_context->mutable_path_planner_input() =
      PlanningContext::Instance()->path_planner_input();
  *scenario_context->mutable_planner_debug() =
      PlanningContext::Instance()->planner_debug();
  *scenario_context->mutable_speed_planner_input() =
      PlanningContext::Instance()->speed_planner_input();
  *scenario_context->mutable_speed_planner_output() =
      PlanningContext::Instance()->speed_planner_output();
  *scenario_context->mutable_lon_decison_output() =
      PlanningContext::Instance()->lon_decison_output();
  return scenario_instance;
}

} // namespace msquare