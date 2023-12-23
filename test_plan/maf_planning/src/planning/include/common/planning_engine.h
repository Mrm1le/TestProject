#ifndef MSQUARE_DECISION_PLANNING_COMMON_PLANNING_ENGINE_H_
#define MSQUARE_DECISION_PLANNING_COMMON_PLANNING_ENGINE_H_

#include <math.h> /* asin */
#include <mutex>

#include "common/apa_workflow/parking_planning_task.hpp"
#include "common/planning_task.hpp"
#include "common/search_based_planning_utils.h"
#include "planning_task_interface.h"
// #include "common/pnc_trigger.h"
#include "common/search_based_planning_task.hpp"
#include "common/timer_task.hpp"
#include "planner/motion_planner/longitudinal_motion_planner.h"
#include "pnc/planning_engine_interface.h"
#include "worldmodel/ddmap_generator_interface.hpp"
#include "worldmodel/ddmap_generator_task.hpp"
#include "worldmodel/fusion_task.hpp"
#include "worldmodel/publisher_task.hpp"
#include "worldmodel_flowchart.h"
#include "slot_release_config.h"

using namespace msd_planning;

namespace msquare {
class PlanningEngine : public PlanningEngineInterface {
public:
  PlanningEngine(const MSDPlanningConfig &planning_config);
  ~PlanningEngine();

  virtual bool init() override;
  virtual bool start() override;
  virtual bool stop() override;
  virtual bool reset() override;
  virtual bool exit() override;

  virtual void feed_tick(uint64_t tick) override;

  virtual void keep_run() override;
  virtual void keep_stop() override;
  virtual void stop_status_manager() override;
  virtual void feed_module_status(
      const maf_framework_status::ModuleStatus &module_status) override;
  virtual void feed_chassis_report(
      const std::shared_ptr<maf_endpoint::ChassisReport> chassis_report)
      override;
  virtual void feed_wireless_charger_report(
      const std::shared_ptr<maf_endpoint::WirelessChargerReport>
          wireless_charger_report) override;
  virtual void feed_wheel_report(
      const std::shared_ptr<maf_endpoint::WheelReport> wheel_report) override;
  virtual void feed_body_report(
      const std::shared_ptr<maf_endpoint::BodyReport> body_report) override;
  virtual void feed_imu_report(
      const std::shared_ptr<maf_gps_imu::MLAImu> imu_report) override;
  virtual void
  feed_sbp_result(const maf_planning::SBPResult &sbp_result) override;
  virtual void feed_worldmodel_map(
      const std::shared_ptr<maf_worldmodel::ProcessedMap> processed_map)
      override;
  virtual void feed_worldmodel_objects(
      const std::shared_ptr<maf_worldmodel::ObjectsInterface> objects_interface)
      override;
  virtual void feed_world_model_parking_slots(
      const std::shared_ptr<maf_worldmodel::FusionAPA> parking_slots) override;
  virtual void feed_world_model_scene_objects(
      const std::shared_ptr<maf_worldmodel::SceneObjects> scene_objects)
      override;
  virtual void
  feed_fusion_objects(const std::shared_ptr<
                      maf_perception_interface::PerceptionFusionObjectResult>
                          scene_objects) override;
  virtual void feed_fusion_grond_lines(
      const std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
          ground_lines) override;
  virtual void feed_fusion_uss_grond_lines(
      const std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
          ground_lines) override;
  virtual void feed_fusion_uss(
      const std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport> uss)
      override;
  virtual void feed_traffic_light(
      const std::shared_ptr<maf_perception_interface::TrafficLightPerception>
          traffic_light_perception) override;
  virtual void feed_ego_pose(
      const std::shared_ptr<maf_mla_localization::MLALocalization> ego_pose)
      override;
  virtual void
  feed_prediction_info(const std::shared_ptr<maf_worldmodel::PredictionResult>
                           prediction_results) override;
  virtual void feed_mpc_trajectory(
      const std::shared_ptr<maf_planning::MpcTrajectoryResult> mpc_trajectory)
      override;
  virtual void feed_tasknap_slice(uint8_t *ptr, uint64_t size) override;
  virtual void feed_planning_control_cmd_request(
      const maf_system_manager::ModuleControlCmdRequest &request) override;
  virtual void feed_planning_request(
      const maf_system_manager::SysPlanningRequest &request) override;
  virtual void
  feed_planning_reset_request(const maf_std::Header &request) override;
  virtual void feed_perception_vision_lane(
      const std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_vision_lane) override;

  virtual void feed_perception_radar(
      const std::shared_ptr<maf_perception_interface::RadarPerceptionResult>
          perception_radar) override;
  virtual void
  feed_mff_info(const std::shared_ptr<maf_std::Header> mff_info) override;

  virtual void feed_perception_lidar_road_edge(
      const std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_lidar_road_edge) override;

  virtual void set_callback(MSDPlanningOutputCallback callback) override;
  virtual void set_callback(MSDPlanningTriggerCallback callback) override;
  virtual void set_callback(MSDScenarioRecognizeCallback callback) override;
  virtual void set_callback(MSDPlanningTasknapCallback callback) override;
  //   virtual void set_callback(MSDPlanningModuleStatusCallback callback)
  //   override;
  virtual void set_callback(MSDPlanningNodeStatusCallback callback) override;
  virtual void set_callback(SBPRequestCallback callback) override;
  virtual void set_callback(MSDPlanningInfoCallback callback) override;
  virtual void set_callback(MSDMdebugCallback callback) override;

  // for wm
  virtual void feed_perception_fusion_result(
      PerceptionFusionObjectResultPtr perception_fusion_result) override;
  virtual void feed_mode_switch_request(
      const maf_system_manager::SysWorldModelRequest &request) override;
  virtual void feed_system_reset_request(
      std::shared_ptr<maf_std::Header> system_reset_cmd) override;
  virtual void feed_system_control_request(
      std::shared_ptr<maf_system_manager::ModuleControlCmdRequest> &msg)
      override;
  virtual void set_processed_map_callback(MSDMapCallback callback) override;
  virtual void set_wm_info_callback(WorldmodelInfoCallback callback) override;
  virtual void
  set_matched_info_callback(MatchedEgoPositionCallback callback) override;
  virtual void
  set_objects_interface_callback(MSDObjectsCallback callback) override;

  // parking worldmodel
  virtual void feed_fusion_parking_slot(
      const std::shared_ptr<maf_perception_interface::FusionParkingSlotResult>
          fusion_parking_slots) override;
  virtual void
  set_wm_parking_slot_callback(WorldModelParkingSlotCallback callback) override;
  virtual std::shared_ptr<mtaskflow::TasknapAgent> get_tasknap_agent() override;
  virtual void
  set_simulation_sync_callback(NPPHeaderLoggerCallback callback) override;

private:
  mtaskflow::FlowMachine machine_{};
  bool is_machine_running_ = false;

  MSDPlanningConfig planning_config_{};
  uint8_t running_mode_{maf_system_manager::RunningModeEnum::MANUAL_DRIVING};

  bool enable_timer_tick_{true};
  mtaskflow::FlowPublisher<uint64_t> tick_publisher_{};

  mtaskflow::FlowPublisher<maf_framework_status::ModuleStatus>
      module_status_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_endpoint::ChassisReport>>
      chassis_report_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_endpoint::WirelessChargerReport>>
      wireless_charger_report_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_endpoint::WheelReport>>
      wheel_report_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_endpoint::BodyReport>>
      body_report_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_gps_imu::MLAImu>>
      imu_report_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_worldmodel::ProcessedMap>>
      worldmodel_map_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_worldmodel::ObjectsInterface>>
      worldmodel_objects_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_worldmodel::FusionAPA>>
      worldmodel_parking_slots_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_worldmodel::SceneObjects>>
      worldmodel_scene_objects_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>>
      fusion_objects_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>
      fusion_groundlines_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>
      fusion_uss_groundlines_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport>>
      fusion_uss_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::TrafficLightPerception>>
      traffic_light_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_mla_localization::MLALocalization>>
      ego_pose_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_worldmodel::PredictionResult>>
      prediction_info_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_planning::MpcTrajectoryResult>>
      mpc_trajectory_publisher_{};
  mtaskflow::FlowPublisher<maf_system_manager::ModuleControlCmdRequest>
      planning_control_cmd_request_publisher_{};
  mtaskflow::FlowPublisher<maf_system_manager::SysPlanningRequest>
      planning_request_publisher_{};
  mtaskflow::FlowPublisher<maf_std::Header> planning_reset_request_publisher_{};
  mtaskflow::FlowPublisher<maf_planning::SBPResult> sbp_result_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>
      perception_vision_lane_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>
      perception_lidar_road_edge_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::RadarPerceptionResult>>
      perception_radar_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_std::Header>>
      mff_info_publisher_{};
  // parking worldmodel
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::FusionParkingSlotResult>>
      fusion_parking_slot_publisher_{};
  std::shared_ptr<msd_worldmodel::WorldmodelPubTask> wm_pub_task_;
  std::shared_ptr<msd_worldmodel::worldmodel_v1::FusionTask>
      fusion_object_task_{};
  mtaskflow::FlowPublisher<PerceptionFusionObjectResultPtr>
      perception_fusion_result_publisher_{};

  std::shared_ptr<PlanningTaskInterface> parking_planning_task_{};
  std::shared_ptr<parking::SBPlanningTask> sbp_planning_task_{};
  // parking worldmodel
  std::shared_ptr<::parking::SetPSDFusionTask> pec_task_{};

  std::shared_ptr<mtaskflow::TasknapAgent> taskagent_ptr_;
  static constexpr auto PARAM_KEY_MFR_SNAPSHOT_NAME = "mfr_snapshot_planning";
};

} // namespace msquare

#endif
