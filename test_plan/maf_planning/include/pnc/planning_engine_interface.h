#ifndef MSQUARE_DECISION_PLANNING_COMMON_PLANNING_ENGINE_INTERFACE_H_
#define MSQUARE_DECISION_PLANNING_COMMON_PLANNING_ENGINE_INTERFACE_H_

#include <memory>

#include "maf/base.hpp"
#include "msd/worldmodel/worldmodel_generator.h"
#include "mtaskflow/mtaskflow.hpp"
#include "pnc.h"

namespace msquare {

enum class SceneType : unsigned char {
  URBAN = 0,
  PARKING_SVP = 1,
  HIGHWAY = 2,
  PARKING_LVP = 4,
  PARKING_APA = 5,
  REMOTE_CONTROL = 6,
  NOT_DEFINED = 7,
};

// FOR L_BCAR
enum DrivingModelConfig {
  NO_REQUEST_DrivingModel = 0,
  SAFE_DRIVING_MODEL = 1,
  STEADY_DRIVING_MODEL = 2,
  RADICAL_DRIVING_MODEL = 3,
  SELF_LEARNING_MODEL = 4,
};

struct MSDPlanningConfig {
  SceneType scene_type{SceneType::URBAN};
  bool enable_timer_tick{true};
  uint32_t underclocking_enum{10};
  bool enable_ilc{true};
  bool enable_alc{false};
  double ilc_limit_velocity{0.0};
  double ilc_limit_time{0.0};
  double cruise_velocity{120.0};
  bool is_acc_mode{false};
  bool is_ddmap{false};
  int driving_style{1};
  std::string config_file_dir{""};
  std::string mtaskflow_config_file{""};
  std::string wm_config_file{""};
  std::string wm_pec_config_file{""};
  bool init_wm{true};
};

struct MSDPlanningOutputMeta {
  uint64_t tick_count{0};
  bool succeed{false};
};

struct TopicStatus {
  std::string topic_name;
  std::string topic_type;
  double normal_hz;
  double warning_hz;
  double error_hz;
  bool hdmap;
  bool ddmap;
};

using MSDPlanningOutputCallback =
    std::function<void(const MSDPlanningOutputMeta &meta,
                       const maf_planning::Planning &planning_result,
                       const std::string &trigger_msg_id)>;
using MSDPlanningLdpOutputCallback =
    std::function<void(const std::string &planning_ldp_output)>;
using MSDPlanningTriggerCallback =
    std::function<void(const std::vector<TriggerStatus> &trigger_status)>;
// using MSDPlanningModuleStatusCallback = std::function<void(
//     const maf_framework_status::ModuleStatus &planning_module_status)>;

using MSDPlanningTasknapCallback =
    std::function<void(uint8_t *ptr, uint64_t size)>;
using MSDPlanningNodeStatusCallback = std::function<void(
    const maf_framework_status::NodeStatus &planning_node_status)>;
using MSDScenarioRecognizeCallback =
    std::function<void(const RecognizeInfo &recognize_info)>;
using SBPRequestCallback =
    std::function<void(const maf_planning::SBPRequest &sbp_request)>;
using MSDPlanningInfoCallback =
    std::function<void(const maf_std::Header &planning_info)>;
using MSDMdebugCallback = std::function<void(std::string &&log)>;

// for wm
using MLALocalizationPtr =
    std::shared_ptr<maf_mla_localization::MLALocalization>;
using PerceptionFusionObjectResultPtr =
    std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>;
using ObjectsInterfacePtr = std::shared_ptr<maf_worldmodel::ObjectsInterface>;
using ProcessedMapPtr = std::shared_ptr<maf_worldmodel::ProcessedMap>;
using NodesStatusPtr = std::shared_ptr<maf_framework_status::NodesStatus>;
using MSDMapCallback = std::function<void(ProcessedMapPtr processed_map)>;
using MSDObjectsCallback =
    std::function<void(ObjectsInterfacePtr object_interface)>;
using MSDNodesStatusCallback = std::function<void(NodesStatusPtr status)>;
using HrzResponseCallback =
    std::function<void(const maf_std::Header &response)>;
using ModeSwitchCmdCallback = std::function<void(
    const maf_system_manager::SysWorldModelResponse &response)>;
using WorldmodelInfoCallback =
    std::function<void(const maf_std::Header &response)>;
using MatchedEgoPositionCallback =
    std::function<void(const maf_std::Header &response)>;
using SystemResetCallback =
    std::function<void(const maf_std::Header &response)>;
using WorldModelParkingSlotCallback = std::function<void(
    const std::shared_ptr<maf_worldmodel::FusionAPA> &response)>;
using NPPHeaderLoggerCallback = std::function<void(std::string &&log)>;

class PlanningEngineInterface : public maf::CoreBase {
public:
  MSD_API virtual ~PlanningEngineInterface() = default;

  MSD_API static std::shared_ptr<PlanningEngineInterface>
  make(const MSDPlanningConfig &planning_config);

  MSD_API virtual void feed_tick(uint64_t tick) = 0;

  MSD_API virtual void keep_run() = 0;
  MSD_API virtual void keep_stop() = 0;
  MSD_API virtual void stop_status_manager() = 0;
  MSD_API virtual void feed_module_status(
      const maf_framework_status::ModuleStatus &module_status) = 0;
  MSD_API virtual void feed_chassis_report(
      const std::shared_ptr<maf_endpoint::ChassisReport> chassis_report) = 0;
  MSD_API virtual void feed_wireless_charger_report(
      const std::shared_ptr<maf_endpoint::WirelessChargerReport> 
      wireless_charger_report) = 0;
  MSD_API virtual void feed_wheel_report(
      const std::shared_ptr<maf_endpoint::WheelReport> wheel_report) = 0;
  MSD_API virtual void feed_body_report(
      const std::shared_ptr<maf_endpoint::BodyReport> body_report) = 0;
  MSD_API virtual void
  feed_imu_report(const std::shared_ptr<maf_gps_imu::MLAImu> imu_report) = 0;
  MSD_API virtual void
  feed_sbp_result(const maf_planning::SBPResult &sbp_result) = 0;
  MSD_API virtual void feed_worldmodel_map(
      const std::shared_ptr<maf_worldmodel::ProcessedMap> processed_map) = 0;
  MSD_API virtual void feed_worldmodel_objects(
      const std::shared_ptr<maf_worldmodel::ObjectsInterface>
          objects_interface) = 0;
  MSD_API virtual void feed_world_model_parking_slots(
      const std::shared_ptr<maf_worldmodel::FusionAPA> parking_slots) = 0;
  MSD_API virtual void feed_world_model_scene_objects(
      const std::shared_ptr<maf_worldmodel::SceneObjects> scene_objects) = 0;
  MSD_API virtual void
  feed_fusion_objects(const std::shared_ptr<
                      maf_perception_interface::PerceptionFusionObjectResult>
                          scene_objects) = 0;
  MSD_API virtual void feed_fusion_grond_lines(
      const std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
          ground_lines) = 0;
  MSD_API virtual void feed_fusion_uss_grond_lines(
      const std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
          ground_lines) = 0;
  MSD_API virtual void feed_fusion_uss(
      const std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport> uss) = 0;
  MSD_API virtual void feed_traffic_light(
      const std::shared_ptr<maf_perception_interface::TrafficLightPerception>
          traffic_light_perception) = 0;
  MSD_API virtual void
  feed_ego_pose(const std::shared_ptr<maf_mla_localization::MLALocalization>
                    ego_pose) = 0;
  MSD_API virtual void
  feed_prediction_info(const std::shared_ptr<maf_worldmodel::PredictionResult>
                           prediction_results) = 0;
  MSD_API virtual void
  feed_mpc_trajectory(const std::shared_ptr<maf_planning::MpcTrajectoryResult>
                          mpc_trajectory) = 0;
  MSD_API virtual void feed_tasknap_slice(uint8_t *ptr, uint64_t size) = 0;
  MSD_API virtual void feed_planning_control_cmd_request(
      const maf_system_manager::ModuleControlCmdRequest &request) = 0;
  MSD_API virtual void feed_planning_request(
      const maf_system_manager::SysPlanningRequest &request) = 0;
  MSD_API virtual void
  feed_planning_reset_request(const maf_std::Header &request) = 0;
  MSD_API virtual void feed_perception_vision_lane(
      const std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_vision_lane) = 0;
  MSD_API virtual void feed_perception_lidar_road_edge(
      const std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_lidar_road_edge) = 0;

  MSD_API virtual void feed_perception_radar(
      const std::shared_ptr<maf_perception_interface::RadarPerceptionResult>
          perception_radar) = 0;
  MSD_API virtual void
  feed_mff_info(const std::shared_ptr<maf_std::Header> mff_info) = 0;

  MSD_API virtual void set_callback(MSDPlanningOutputCallback callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningTriggerCallback callback) = 0;
  //   MSD_API virtual void set_callback(MSDPlanningModuleStatusCallback
  //   callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningNodeStatusCallback callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningTasknapCallback callback) = 0;
  MSD_API virtual void set_callback(MSDScenarioRecognizeCallback callback) = 0;
  MSD_API virtual void set_callback(SBPRequestCallback callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningInfoCallback callback) = 0;
  MSD_API virtual void set_callback(MSDMdebugCallback callback) = 0;

  // for wm
  MSD_API virtual void feed_perception_fusion_result(
      PerceptionFusionObjectResultPtr perception_fusion_result) = 0;
  MSD_API virtual void feed_mode_switch_request(
      const maf_system_manager::SysWorldModelRequest &request) = 0;
  MSD_API virtual void feed_system_reset_request(
      std::shared_ptr<maf_std::Header> system_reset_cmd) = 0;
  MSD_API virtual void feed_system_control_request(
      std::shared_ptr<maf_system_manager::ModuleControlCmdRequest> &msg) = 0;
  MSD_API virtual void set_processed_map_callback(MSDMapCallback callback) = 0;
  MSD_API virtual void
  set_wm_info_callback(WorldmodelInfoCallback callback) = 0;
  MSD_API virtual void
  set_matched_info_callback(MatchedEgoPositionCallback callback) = 0;
  MSD_API virtual void
  set_objects_interface_callback(MSDObjectsCallback callback) = 0;
  // parking worldmodel
  MSD_API virtual void feed_fusion_parking_slot(
      const std::shared_ptr<maf_perception_interface::FusionParkingSlotResult>
          fusion_parking_slots) = 0;
  MSD_API virtual void
  set_wm_parking_slot_callback(WorldModelParkingSlotCallback maf_psd) = 0;
  MSD_API virtual std::shared_ptr<mtaskflow::TasknapAgent>
  get_tasknap_agent() = 0;
  MSD_API virtual void
  set_simulation_sync_callback(NPPHeaderLoggerCallback callback) = 0;
};

} // namespace msquare

#endif
