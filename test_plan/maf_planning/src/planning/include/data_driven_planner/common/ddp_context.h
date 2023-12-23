#pragma once

#include "mtime_core/mtime.h"

#include "common/config/vehicle_param.h"
#include "common/speed/speed_limit.h"
#include "common/utils/macro.h"
#include "data_driven_planner/common/basic_types.h"
#include "data_driven_planner/common/ddp_config.h"
#include "data_driven_planner/common/model_result_manager.h"
#include "data_driven_planner/common/planning_result_manager.h"
#include "data_driven_planner/models/ego_pose_manager.h"
#include "data_driven_planner/models/fusion_object_manager.h"
#include "data_driven_planner/models/vision_lane_manager.h"

#include "pnc/define/obstacle_decider_interface.hpp"
#include "pnc/define/planning_status.h"

namespace msquare {
namespace ddp {

class TimeDurationFlag {
public:
  explicit TimeDurationFlag(double duration_time)
      : duration_time_(duration_time), flag_(false), last_time_(-1) {}
  bool get_flag() {
    if (MTIME()->timestamp().sec() - last_time_ > duration_time_) {
      flag_ = false;
    }
    return flag_;
  }

  void set_flag(bool flag) {
    last_time_ = MTIME()->timestamp().sec();
    flag_ = flag;
  }

private:
  double duration_time_;
  bool flag_;
  double last_time_;
};

struct StatusInfo {
  std::string error_info;
  std::string debug_info;
  std::string source;
  TimeDurationFlag flag_ddp_disable_traffic_light =
      TimeDurationFlag(double(5.0));
  int planning_loop = 0;
  double timestamp_ddp = 0.0;
  double last_vel_limit = 100.0;
  bool vehicle_dbw_status = false;
};

class DdpConfigBuilder;
class ModelResultManager;
// class EgoState;
// class ObstacleManager;
// class ScenarioStateMachine;
// class VirtualLaneManager;
// class ReferencePathManager;
// class TrafficLightDecider;
class FusionObjectManager;
class VisionLaneManager;
// class TrafficLightInfoManager;
class PlanningResultManager;
class EgoPoseManager;
// class ScenarioStateDisplay;

class DdpContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(DdpContext);

public:
  const bool planning_success() const { return planning_success_; }

  bool &mutable_planning_success() { return planning_success_; }

  const bool b_dagger() const { return b_dagger_; }

  bool &mutable_b_dagger() { return b_dagger_; }

  const LatDecisionInfos &lateral_decisions_for_show() const {
    return lateral_decisions_for_show_;
  }

  LatDecisionInfos &mutable_lateral_decisions_for_show() {
    return lateral_decisions_for_show_;
  }

  const PlanningResult &planning_result() const { return planning_result_; }

  PlanningResult &mutable_planning_result() { return planning_result_; }

  const LonDecisionInfo &lon_decision_result() const {
    return lon_decision_result_;
  }
  LonDecisionInfo &mutable_lon_decision_result() {
    return lon_decision_result_;
  }

  const FaultDiagnosisInfo &fault_diagnosis_result() const {
    return fault_diagnosis_result_;
  }
  FaultDiagnosisInfo &mutable_fault_diagnosis_result() {
    return fault_diagnosis_result_;
  }

  void set_last_ddp_planning_result(
      const std::shared_ptr<PlanningResult> planning_result) {
    last_ddp_planning_result_ = planning_result;
  }

  const std::shared_ptr<PlanningResult> &last_ddp_planning_result() {
    return last_ddp_planning_result_;
  }

  const msquare::PlanningResult &bak_planning_result() const {
    return bak_planning_result_;
  }

  msquare::PlanningResult &mutable_bak_planning_result() {
    return bak_planning_result_;
  }

  const msquare::PlanningResult &last_planning_result() const {
    return last_planning_result_;
  }

  msquare::PlanningResult &mutable_last_planning_result() {
    return last_planning_result_;
  }

  // const msquare::VehicleParam &vehicle_param() const { return vehicle_param_;
  // }

  // void set_vehicle_param(const msquare::VehicleParam &vehicle_param) {
  //   vehicle_param_ = vehicle_param;
  // }

  const StatusInfo &status_info() { return status_info_; }

  StatusInfo &mutable_status_info() { return status_info_; }

  const msquare::SpeedLimit &speed_limit() { return speed_limit_; }

  msquare::SpeedLimit &mutable_speed_limit() { return speed_limit_; }

  const std::shared_ptr<DdpConfigBuilder> config_builder() {
    return config_builder_ptr_;
  }
  void
  set_config_builder(std::shared_ptr<DdpConfigBuilder> config_builder_ptr) {
    config_builder_ptr_ = config_builder_ptr;
  }

  const std::shared_ptr<DdpConfigBuilder> hnp_config_builder() {
    return hnp_config_builder_ptr_;
  }
  void set_hnp_config_builder(
      std::shared_ptr<DdpConfigBuilder> hnp_config_builder_ptr) {
    hnp_config_builder_ptr_ = hnp_config_builder_ptr;
  }

  const std::shared_ptr<ModelResultManager> &model_result_manager() {
    return model_result_manager_ptr_;
  }
  void set_model_result_manager(
      std::shared_ptr<ModelResultManager> model_result_manager_ptr) {
    model_result_manager_ptr_ = model_result_manager_ptr;
  }

  // const std::shared_ptr<EgoState> &ego_state() { return ego_state_ptr_; }
  // void set_ego_state(std::shared_ptr<EgoState> ego_state_ptr) {
  //   ego_state_ptr_ = ego_state_ptr;
  // }

  // const std::shared_ptr<ObstacleManager> &obstacle_manager() {
  //   return obstacle_manager_ptr_;
  // }
  // void set_obstacle_manager(
  //     std::shared_ptr<ObstacleManager> obstacle_manager_ptr) {
  //   obstacle_manager_ptr_ = obstacle_manager_ptr;
  // }

  // const std::shared_ptr<VirtualLaneManager> &virtual_lane_manager() {
  //   return virtual_lane_manager_ptr_;
  // }
  // void set_virtual_lane_manager(
  //     std::shared_ptr<VirtualLaneManager> virtual_lane_manager_ptr) {
  //   virtual_lane_manager_ptr_ = virtual_lane_manager_ptr;
  // }

  // const std::shared_ptr<TrafficLightDecider> &traffic_light_decider() {
  //   return traffic_light_decider_ptr_;
  // }
  // void set_traffic_light_decider(
  //     std::shared_ptr<TrafficLightDecider> traffic_light_decider_ptr) {
  //   traffic_light_decider_ptr_ = traffic_light_decider_ptr;
  // }

  const std::shared_ptr<FusionObjectManager> &fusion_object_manager() {
    return fusion_object_manager_ptr_;
  }
  void set_fusion_object_manager(
      std::shared_ptr<FusionObjectManager> fusion_object_manager_ptr) {
    fusion_object_manager_ptr_ = fusion_object_manager_ptr;
  }

  // const std::shared_ptr<TrafficLightInfoManager>
  // &traffic_light_info_manager() {
  //   return traffic_light_info_manager_ptr_;
  // }
  // void set_traffic_light_info_manager(
  //     std::shared_ptr<TrafficLightInfoManager>
  //     traffic_light_info_manager_ptr) {
  //   traffic_light_info_manager_ptr_ = traffic_light_info_manager_ptr;
  // }

  // const std::shared_ptr<ReferencePathManager> &reference_path_manager() {
  //   return reference_path_manager_ptr_;
  // }
  // void set_reference_path_manager(
  //     std::shared_ptr<ReferencePathManager> reference_path_manager_ptr) {
  //   reference_path_manager_ptr_ = reference_path_manager_ptr;
  // }

  const std::shared_ptr<PlanningResultManager> &planning_result_manager() {
    return planning_result_manager_ptr_;
  }
  void set_planning_result_manager(
      std::shared_ptr<PlanningResultManager> planning_result_manager_ptr) {
    planning_result_manager_ptr_ = planning_result_manager_ptr;
  }

  const std::shared_ptr<EgoPoseManager> &ego_pose_manager() {
    return ego_pose_manager_ptr_;
  }
  void
  set_ego_pose_manager(std::shared_ptr<EgoPoseManager> ego_pose_manager_ptr) {
    ego_pose_manager_ptr_ = ego_pose_manager_ptr;
  }

  const std::shared_ptr<VisionLaneManager> &vision_lane_manager() {
    return vision_lane_manager_ptr_;
  }
  void set_vision_lane_manager(
      std::shared_ptr<VisionLaneManager> vision_lane_manager_ptr) {
    vision_lane_manager_ptr_ = vision_lane_manager_ptr;
  }

  // const std::shared_ptr<ScenarioStateMachine> &scenario_state_machine() {
  //   return scenario_state_machine_ptr_;
  // }
  // void set_scenario_state_machine(
  //     std::shared_ptr<ScenarioStateMachine> scenario_state_machine_ptr) {
  //   scenario_state_machine_ptr_ = scenario_state_machine_ptr;
  // }

  // const std::shared_ptr<ScenarioStateDisplay> &scenario_state_display() {
  //   return scenario_state_display_ptr_;
  // }
  // void set_scenario_state_display(
  //     std::shared_ptr<ScenarioStateDisplay> scenario_state_display_ptr) {
  //   scenario_state_display_ptr_ = scenario_state_display_ptr;
  // }

  std::string log_string() {
    return status_info_.error_info + "|" + status_info_.debug_info;
  }

  void clear() {
    planning_success_ = true;
    planning_result_.raw_traj_points.clear();
    planning_result_.traj_points.clear();
    status_info_.error_info = {};
    status_info_.debug_info = {};
  }

  const odc_interface::ObstacleDeciderOutput &obstacle_decider_output() const {
    return obstacle_decider_output_;
  }
  odc_interface::ObstacleDeciderOutput *mutable_obstacle_decider_output() {
    return &obstacle_decider_output_;
  }

  const ddp::DdpTrajectory &get_obstacle_decider_ddp_trajectory() const {
    return obstacle_decider_ddp_trajectory_;
  }

  ddp::DdpTrajectory *mutable_obstacle_decider_ddp_trajectory() {
    return &obstacle_decider_ddp_trajectory_;
  }

private:
  bool planning_success_;
  bool b_dagger_;
  PlanningResult planning_result_;
  LonDecisionInfo lon_decision_result_;
  FaultDiagnosisInfo fault_diagnosis_result_;
  LatDecisionInfos lateral_decisions_for_show_;
  std::shared_ptr<PlanningResult> last_ddp_planning_result_;
  msquare::PlanningResult bak_planning_result_;
  msquare::PlanningResult last_planning_result_;
  // msquare::VehicleParam vehicle_param_;
  StatusInfo status_info_;

  std::shared_ptr<DdpConfigBuilder> config_builder_ptr_;
  std::shared_ptr<DdpConfigBuilder> hnp_config_builder_ptr_;
  std::shared_ptr<ModelResultManager> model_result_manager_ptr_;
  // std::shared_ptr<EgoState> ego_state_ptr_;
  // std::shared_ptr<ObstacleManager> obstacle_manager_ptr_;
  // std::shared_ptr<VirtualLaneManager> virtual_lane_manager_ptr_;
  // std::shared_ptr<ReferencePathManager> reference_path_manager_ptr_;
  // std::shared_ptr<ScenarioStateMachine> scenario_state_machine_ptr_;
  // std::shared_ptr<ScenarioStateDisplay> scenario_state_display_ptr_;
  // std::shared_ptr<TrafficLightDecider> traffic_light_decider_ptr_;
  std::shared_ptr<FusionObjectManager> fusion_object_manager_ptr_;
  // std::shared_ptr<TrafficLightInfoManager> traffic_light_info_manager_ptr_;
  std::shared_ptr<PlanningResultManager> planning_result_manager_ptr_;
  std::shared_ptr<EgoPoseManager> ego_pose_manager_ptr_;
  std::shared_ptr<VisionLaneManager> vision_lane_manager_ptr_;

  odc_interface::ObstacleDeciderOutput obstacle_decider_output_;
  ddp::DdpTrajectory obstacle_decider_ddp_trajectory_;

  msquare::SpeedLimit speed_limit_;
};

} // namespace ddp
} // namespace msquare
