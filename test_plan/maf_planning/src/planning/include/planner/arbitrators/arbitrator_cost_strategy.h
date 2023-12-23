#include "common/utils/factory.h"
#include "planner/arbitrators/arbitrator.h"

namespace msquare {

typedef enum {
  SAFETY_CRITERIA = 1,
  MANEUVER_CRITERIA = 2,
  TARGET_CRITERIA = 3,
  MAP_LIMIT_CRITERIA = 4,
  COMFORT_CRTTERIA = 5,
} ArbitratorCostSource;

typedef enum {
  NORMAL = 1,
  UNPROTECTED_MANEUVER = 2,
  PROTECTED_MANEUVER = 3,
  NONESSENTIAL_LANECHANGE_MANEUVER = 4,
} ArbitratorCostType;

using ArbitratorCostFuncMap =
    std::unordered_map<ArbitratorCostType, ArbitratorCostFunc, std::hash<int>>;

class ArbitratorCostStrategy {
public:
  static void init();
  static void set_arbitrator_cost_func(std::shared_ptr<Arbitrator> arbitrator,
                                       ArbitratorCostType cost_type);

private:
  static double
  normal_arbitrator_cost(const std::shared_ptr<ScenarioFacadeContext> context,
                         const std::shared_ptr<WorldModel> world_model);
  static double unprotected_maneuver_arbitrator_cost(
      const std::shared_ptr<ScenarioFacadeContext> context,
      const std::shared_ptr<WorldModel> world_model);
  static double nonessential_lane_change_maneuver_arbitrator_cost(
      const std::shared_ptr<ScenarioFacadeContext> context,
      const std::shared_ptr<WorldModel> world_model);

  // todo: calculate min obs dis
  static TrajCollisionCheckInfo
  check_trajectory_safety(std::shared_ptr<BaseLineInfo> baseline_info,
                          std::shared_ptr<ScenarioFacadeContext> context);

private:
  // static util::Factory<ArbitratorCostType, ArbitratorCostFunc,
  //                     std::unordered_map<ArbitratorCostType,
  //                     ArbitratorCostFunc, std::hash<int>>>
  //   cost_factory_;
  static ArbitratorCostFuncMap cost_func_map_;
};

} // namespace msquare