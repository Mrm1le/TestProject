#pragma once

#include "common/obstacle.h"
#include "common/obstacle_decision_manager.h"
#include "common/obstacle_manager.h"
#include "common/scenario_facade_context.h"
#include "common/world_model.h"

namespace msquare {

class PredictionReviser {
public:
  explicit PredictionReviser(std::shared_ptr<WorldModel> world_model,
                             std::shared_ptr<BaseLineInfo> baseline_info,
                             ScenarioFacadeContext *context);

  bool revise();
  bool lane_change_revise(std::shared_ptr<BaseLineInfo> target_baseline);
  std::vector<int> interactive_objects() { return interactive_objects_; }
  void revise_prediction_trajectory(Obstacle *obstacle, SpeedData speed_data);

private:
  void find_interactive_objects();
  // void revise_prediction_trajectory(Obstacle* obstacle, SpeedData
  // speed_data);
  double min_avoid_dec(Obstacle *obstacle, SpeedData speed_data);

private:
  ScenarioFacadeContext *context_;
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<BaseLineInfo> baseline_info_;
  std::vector<int> interactive_objects_;
};

} // namespace msquare
