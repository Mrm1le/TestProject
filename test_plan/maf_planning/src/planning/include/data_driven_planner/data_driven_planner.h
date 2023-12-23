#pragma once

#include <cmath>
#include <vector>

#include "common/planning_context.h"
#include "common/world_model.h"
#include "data_driven_planner/common/ddp_config.h"
#include "data_driven_planner/common/ddp_context.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include "data_driven_planner/models/vision_lane_manager.h"
#include "pnc.h"

namespace msquare {
namespace ddp {

class DdpModel;
class ModelResultManager;
// class EgoState;
// class ObstacleManager;
// class VirtualLaneManager;
// class ScenarioStateMachine;
class FusionObjectManager;
class VisionLaneManager;
// class TrafficLightInfoManager;
class PlanningResultManager;

class DataDrivenPlanner {
public:
  explicit DataDrivenPlanner(const std::shared_ptr<WorldModel> &world_model);
  virtual ~DataDrivenPlanner() = default;

  bool run();

  //  private:
  //   void set_planning_result(const PlanningResult &ddp_result,
  //                            msquare::PlanningResult &pnc_result);
  //   void clear_planning_result(msquare::PlanningResult &pnc_result);

  //   bool disable_data_driven_planning(const PlanningResult &ddp_result);

private:
  DdpConfig config_;

  std::shared_ptr<WorldModel> world_model_ = nullptr;
  std::shared_ptr<DdpModel> ddp_model_ = nullptr;

  std::shared_ptr<ModelResultManager> model_result_manager_ = nullptr;
  //   std::shared_ptr<EgoState> ego_state_ = nullptr;
  //   std::shared_ptr<ObstacleManager> obstacle_manager_ = nullptr;
  //   std::shared_ptr<msquare::ddp::VirtualLaneManager> virtual_lane_manager_ =
  //       nullptr;
  //   std::shared_ptr<msquare::ddp::ReferencePathManager>
  //   reference_path_manager_ =
  //       nullptr;
  //   std::shared_ptr<TrafficLightDecider> traffic_light_decider_ = nullptr;
  std::shared_ptr<FusionObjectManager> fusion_object_manager_ = nullptr;
  //   std::shared_ptr<TrafficLightInfoManager> traffic_light_info_manager_ =
  //       nullptr;
  std::shared_ptr<PlanningResultManager> planning_result_manager_ = nullptr;
  std::shared_ptr<EgoPoseManager> ego_pose_manager_ = nullptr;
  std::shared_ptr<VisionLaneManager> vision_lane_manager_ = nullptr;

  //   std::shared_ptr<ScenarioStateMachine> scenario_state_machine_ = nullptr;
  //   std::shared_ptr<ScenarioStateDisplay> scenario_state_display_ = nullptr;
};

} // namespace ddp
} // namespace msquare
