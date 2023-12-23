#include "APAPlanningExec/executable.h"
#include "ap.hpp"
#include "pnc.h"

#include "yaml-cpp/yaml.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <map>
#include <thread>

#include "mjson/mjson.hpp"
#include "mlog_core/mlog.h"
#include "mlog_publisher/publisher.h"
#include "mtime_core/mtime.h"
#include "pnc/ldp_planning_engine_interface.h"
#include "timeline/timeline_mfr.h"

using namespace msd_planning;
using namespace msquare;

std::string read_file(const std::string &path) {
  std::ifstream fjson(path);
  std::string json_str((std::istreambuf_iterator<char>(fjson)),
                       std::istreambuf_iterator<char>());
  return json_str;
}

mlog::MLogLevel get_mlog_level(std::string level) {
  std::transform(level.begin(), level.end(), level.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (level == "debug") {
    return mlog::MLogLevel::MLOG_LEVEL_DEBUG;
  } else if (level == "info") {
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  } else if (level == "warn") {
    return mlog::MLogLevel::MLOG_LEVEL_WARN;
  } else if (level == "error") {
    return mlog::MLogLevel::MLOG_LEVEL_ERROR;
  } else if (level == "fatal") {
    return mlog::MLogLevel::MLOG_LEVEL_FATAL;
  } else {
    // use default level
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  }
}

int main(int argc, char **argv) {
  ap::APSpinner::init();

  ap::APSubscriber<
      ap_maf_mla_localization::mla_localization_service_interface::proxy::
          mla_localization_MLALocalizationServiceInterfaceProxy>
      sub_mla_egopose_by_localization(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubmla_localization_MLALocalization",
          "mla_localization_MLALocalization/mla_egopose/Localization");
  ap::APSubscriber<ap_maf_planning::mpc_trajectory_result_service_interface::
                       proxy::planning_MpcTrajectoryResultServiceInterfaceProxy>
      sub_msd_control_mpc_traj_by_control(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubplanning_MpcTrajectoryResult",
          "planning_MpcTrajectoryResult/msd_control_mpc_traj/Control");
  ap::APSubscriber<
      ap_maf_framework_status::module_status_service_interface::proxy::
          framework_status_ModuleStatusServiceInterfaceProxy>
      sub_msd_function_module_status_by_vehicle_service(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubframework_status_ModuleStatus",
          "framework_status_ModuleStatus/msd_function_module_status/"
          "VehicleService");
  ap::APSubscriber<
      ap_maf_system_manager::sys_planning_request_service_interface::proxy::
          system_manager_SysPlanningRequestServiceInterfaceProxy>
      sub_msd_planning_frequency_control_by_nppcpp_switch(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubsystem_manager_SysPlanningRequest",
          "system_manager_SysPlanningRequest/msd_planning_frequency_control/"
          "NPPCPPSwitch");
  ap::APSubscriber<ap_maf_worldmodel::prediction_result_service_interface::
                       proxy::worldmodel_PredictionResultServiceInterfaceProxy>
      sub_msd_prediction_prediction_result_by_ftp(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubworldmodel_PredictionResult",
          "worldmodel_PredictionResult/msd_prediction_prediction_result/FTP");
  ap::APSubscriber<
      ap_maf_sensor_interface::ultrasonic_upa_report_service_interface::proxy::
          sensor_interface_UltrasonicUpaReportServiceInterfaceProxy>
      sub_perception_fusion_distance_uss_by_parking_fusion(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubsensor_interface_UltrasonicUpaReport",
          "sensor_interface_UltrasonicUpaReport/perception_fusion_distance_uss/"
          "ParkingFusion");
  ap::APSubscriber<
      ap_maf_perception_interface::fusion_ground_line_result_service_interface::
          proxy::
              perception_interface_FusionGroundLineResultServiceInterfaceProxy>
      sub_perception_fusion_ground_line_by_parking_fusion(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_FusionGroundLineResult",
          "perception_interface_FusionGroundLineResult/"
          "perception_fusion_ground_line/ParkingFusion");
  ap::APSubscriber<
      ap_maf_perception_interface::fusion_ground_line_result_service_interface::
          proxy::
              perception_interface_FusionGroundLineResultServiceInterfaceProxy>
      sub_perception_fusion_ground_line_uss_by_parking_fusion(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_FusionGroundLineResult",
          "perception_interface_FusionGroundLineResult/"
          "perception_fusion_ground_line_uss/ParkingFusion");
  ap::APSubscriber<
      ap_maf_perception_interface::
          perception_fusion_object_result_service_interface::proxy::
              perception_interface_PerceptionFusionObjectResultServiceInterfaceProxy>
      sub_perception_fusion_object_by_ftp(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_PerceptionFusionObjectResult",
          "perception_interface_PerceptionFusionObjectResult/"
          "perception_fusion_object/FTP");
  ap::APSubscriber<
      ap_maf_perception_interface::
          perception_fusion_object_result_service_interface::proxy::
              perception_interface_PerceptionFusionObjectResultServiceInterfaceProxy>
      sub_perception_fusion_object_parking_environment_by_parking_fusion(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_PerceptionFusionObjectResult",
          "perception_interface_PerceptionFusionObjectResult/"
          "perception_fusion_object_parking_environment/ParkingFusion");
  ap::APSubscriber<
      ap_maf_perception_interface::
          fusion_parking_slot_result_service_interface::proxy::
              perception_interface_FusionParkingSlotResultServiceInterfaceProxy>
      sub_perception_fusion_parking_slot_by_parking_fusion(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_FusionParkingSlotResult",
          "perception_interface_FusionParkingSlotResult/"
          "perception_fusion_parking_slot/ParkingFusion");
  ap::APSubscriber<
      ap_maf_perception_interface::road_line_perception_service_interface::
          proxy::perception_interface_RoadLinePerceptionServiceInterfaceProxy>
      sub_perception_lidar_road_edge_by_lidar_detection(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_RoadLinePerception",
          "perception_interface_RoadLinePerception/perception_lidar_road_edge/"
          "LidarDetection");
  ap::APSubscriber<
      ap_maf_perception_interface::
          road_line_perception_service_interface_someip::proxy::
              perception_interface_RoadLinePerceptionServiceInterfaceSomeIpProxy>
      sub_perception_lidar_road_edge_someip_by_lidar_detection(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_RoadLinePerceptionSomeIp",
          "perception_interface_RoadLinePerception/"
          "perception_lidar_road_edge_someip/LidarDetection");
  ap::APSubscriber<
      ap_maf_perception_interface::road_line_perception_service_interface::
          proxy::perception_interface_RoadLinePerceptionServiceInterfaceProxy>
      sub_perception_fusion_lane_by_ftp(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_RoadLinePerception",
          "perception_interface_RoadLinePerception/perception_fusion_lane/FTP");
  ap::APSubscriber<
      ap_maf_perception_interface::radar_perception_result_service_interface::
          proxy::
              perception_interface_RadarPerceptionResultServiceInterfaceProxy>
      sub_perception_radar_result_front_by_sensor_service(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_RadarPerceptionResult",
          "perception_interface_RadarPerceptionResult/"
          "perception_radar_result_front/SensorService");
  ap::APSubscriber<
      ap_maf_perception_interface::road_line_perception_service_interface::
          proxy::perception_interface_RoadLinePerceptionServiceInterfaceProxy>
      sub_perception_vision_lane_by_camera_detection(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_RoadLinePerception",
          "perception_interface_RoadLinePerception/perception_vision_lane/"
          "CameraDetection");
  ap::APSubscriber<
      ap_maf_perception_interface::road_line_perception_service_interface::
          proxy::perception_interface_RoadLinePerceptionServiceInterfaceProxy>
      sub_perception_vision_landmark_by_camera_detection(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_RoadLinePerception",
          "perception_interface_RoadLinePerception/perception_vision_landmark/"
          "CameraDetection");
  ap::APSubscriber<ap_maf_gps_imu::mla_imu_service_interface::proxy::
                       gps_imu_MLAImuServiceInterfaceProxy>
      sub_sensor_imu_by_sensor_service(
          "APAPlanningExec/APAPlanningRoot/APAPlanningSubgps_imu_MLAImu",
          "gps_imu_MLAImu/sensor_imu/SensorService");
  ap::APSubscriber<ap_maf_gps_imu::mla_imu_service_interface::proxy::
                       gps_imu_MLAImuServiceInterfaceProxy>
      sub_sensor_imu_by_vehicle_service(
          "APAPlanningExec/APAPlanningRoot/APAPlanningSubgps_imu_MLAImu",
          "gps_imu_MLAImu/sensor_imu/VehicleService");
  ap::APSubscriber<ap_maf_std::header_service_interface::proxy::
                       std_HeaderServiceInterfaceProxy>
      sub_system_manager_info_parking_by_mff(
          "APAPlanningExec/APAPlanningRoot/APAPlanningSubstd_Header",
          "std_Header/system_manager_info_parking/MFF");
  ap::APSubscriber<
      ap_maf_system_manager::module_control_cmd_request_service_interface::
          proxy::system_manager_ModuleControlCmdRequestServiceInterfaceProxy>
      sub_system_manager_apa_planning_control_cmd_request_by_nppcpp_switch(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubsystem_manager_ModuleControlCmdRequest",
          "system_manager_ModuleControlCmdRequest/"
          "system_manager_apa_planning_control_cmd_request/NPPCPPSwitch");
  ap::APSubscriber<
      ap_maf_system_manager::sys_planning_request_service_interface::proxy::
          system_manager_SysPlanningRequestServiceInterfaceProxy>
      sub_system_manager_apa_planning_request_by_nppcpp_switch(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubsystem_manager_SysPlanningRequest",
          "system_manager_SysPlanningRequest/"
          "system_manager_apa_planning_request/NPPCPPSwitch");
  ap::APSubscriber<
      ap_maf_system_manager::module_control_cmd_request_service_interface::
          proxy::system_manager_ModuleControlCmdRequestServiceInterfaceProxy>
      sub_system_manager_worldmodel_control_cmd_request_highway_by_mff(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubsystem_manager_ModuleControlCmdRequest",
          "system_manager_ModuleControlCmdRequest/"
          "system_manager_worldmodel_control_cmd_request_highway/MFF");
  ap::APSubscriber<
      ap_maf_system_manager::module_control_cmd_request_service_interface::
          proxy::system_manager_ModuleControlCmdRequestServiceInterfaceProxy>
      sub_system_manager_worldmodel_control_cmd_request_parking_by_mff(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubsystem_manager_ModuleControlCmdRequest",
          "system_manager_ModuleControlCmdRequest/"
          "system_manager_worldmodel_control_cmd_request_parking/MFF");
  ap::APSubscriber<
      ap_maf_system_manager::sys_world_model_request_service_interface::proxy::
          system_manager_SysWorldModelRequestServiceInterfaceProxy>
      sub_system_manager_worldmodel_request_highway_by_mff(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubsystem_manager_SysWorldModelRequest",
          "system_manager_SysWorldModelRequest/"
          "system_manager_worldmodel_request_highway/MFF");
  ap::APSubscriber<ap_maf_endpoint::body_report_service_interface::proxy::
                       endpoint_BodyReportServiceInterfaceProxy>
      sub_vehicle_body_report_by_vehicle_service(
          "APAPlanningExec/APAPlanningRoot/APAPlanningSubendpoint_BodyReport",
          "endpoint_BodyReport/vehicle_body_report/VehicleService");
  ap::APSubscriber<ap_maf_endpoint::chassis_report_service_interface::proxy::
                       endpoint_ChassisReportServiceInterfaceProxy>
      sub_vehicle_chassis_report_by_vehicle_service(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubendpoint_ChassisReport",
          "endpoint_ChassisReport/vehicle_chassis_report/VehicleService");
  ap::APSubscriber<ap_maf_endpoint::wheel_report_service_interface::proxy::
                       endpoint_WheelReportServiceInterfaceProxy>
      sub_vehicle_wheel_report_by_vehicle_service(
          "APAPlanningExec/APAPlanningRoot/APAPlanningSubendpoint_WheelReport",
          "endpoint_WheelReport/vehicle_wheel_report/VehicleService");
  ap::APSubscriber<
      ap_maf_perception_interface::traffic_light_perception_service_interface::
          proxy::
              perception_interface_TrafficLightPerceptionServiceInterfaceProxy>
      sub_worldmodel_traffic_light_by_ftp(
          "APAPlanningExec/APAPlanningRoot/"
          "APAPlanningSubperception_interface_TrafficLightPerception",
          "perception_interface_TrafficLightPerception/"
          "worldmodel_traffic_light/FTP");
  ap::APSubscriber<ap_maf_std::header_service_interface::proxy::
                       std_HeaderServiceInterfaceProxy>
      sub_navigation_ehr_by_hmi_service(
          "APAPlanningExec/APAPlanningRoot/APAPlanningSubstd_Header",
          "std_Header/navigation_ehr/HmiService");
  ap::APSubscriber<ap_maf_std::header_service_interface::proxy::
                       std_HeaderServiceInterfaceProxy>
      sub_parking_fusion_to_cpplanning_parkingout_warm_prompt_by_parking_fusion(
          "APAPlanningExec/APAPlanningRoot/APAPlanningSubstd_Header",
          "std_Header/parking_fusion_to_cpplanning_parkingout_warm_prompt/"
          "ParkingFusion");

  ap::APPublisher<ap_maf_mlog::logs_service_interface::skeleton::
                      mlog_LogsServiceInterfaceSkeleton>
      pub_mlog_logs_apa_planning(
          "mlog_Logs/mlog_logs_apa_planning/APAPlanning");
  ap::APPublisher<ap_maf_planning::sbp_request_service_interface::skeleton::
                      planning_SBPRequestServiceInterfaceSkeleton>
      pub_msd_sbp_request("planning_SBPRequest/msd_sbp_request/APAPlanning");
  ap::APPublisher<ap_maf_std::header_service_interface::skeleton::
                      std_HeaderServiceInterfaceSkeleton>
      pub_msquare_cla_event_filter(
          "std_Header/msquare_cla_event_filter/APAPlanning");
  ap::APPublisher<
      ap_maf_framework_status::nodes_status_service_interface::skeleton::
          framework_status_NodesStatusServiceInterfaceSkeleton>
      pub_node_status_worldmodel_highway(
          "framework_status_NodesStatus/node_status_worldmodel_highway/"
          "APAPlanning");
  ap::APPublisher<
      ap_maf_framework_status::nodes_status_service_interface::skeleton::
          framework_status_NodesStatusServiceInterfaceSkeleton>
      pub_node_status_worldmodel_parking(
          "framework_status_NodesStatus/node_status_worldmodel_parking/"
          "APAPlanning");
  ap::APPublisher<ap_maf_std::header_service_interface::skeleton::
                      std_HeaderServiceInterfaceSkeleton>
      pub_apa_planning_info("std_Header/apa_planning_info/APAPlanning");
  ap::APPublisher<
      ap_maf_system_manager::sys_world_model_response_service_interface::
          skeleton::
              system_manager_SysWorldModelResponseServiceInterfaceSkeleton>
      pub_parking_system_manager_worldmodel_response(
          "system_manager_SysWorldModelResponse/"
          "parking_system_manager_worldmodel_response/APAPlanning");
  ap::APPublisher<
      ap_maf_system_manager::module_control_cmd_response_service_interface::
          skeleton::
              system_manager_ModuleControlCmdResponseServiceInterfaceSkeleton>
      pub_system_manager_apa_planning_control_cmd_response(
          "system_manager_ModuleControlCmdResponse/"
          "system_manager_apa_planning_control_cmd_response/APAPlanning");
  ap::APPublisher<
      ap_maf_system_manager::sys_planning_response_service_interface::skeleton::
          system_manager_SysPlanningResponseServiceInterfaceSkeleton>
      pub_system_manager_apa_planning_response(
          "system_manager_SysPlanningResponse/"
          "system_manager_apa_planning_response/APAPlanning");
  ap::APPublisher<
      ap_maf_system_manager::module_control_cmd_response_service_interface::
          skeleton::
              system_manager_ModuleControlCmdResponseServiceInterfaceSkeleton>
      pub_system_manager_worldmodel_control_cmd_response_highway(
          "system_manager_ModuleControlCmdResponse/"
          "system_manager_worldmodel_control_cmd_response_highway/APAPlanning");
  ap::APPublisher<
      ap_maf_system_manager::module_control_cmd_response_service_interface::
          skeleton::
              system_manager_ModuleControlCmdResponseServiceInterfaceSkeleton>
      pub_system_manager_worldmodel_control_cmd_response_parking(
          "system_manager_ModuleControlCmdResponse/"
          "system_manager_worldmodel_control_cmd_response_parking/APAPlanning");
  ap::APPublisher<ap_maf_std::header_service_interface::skeleton::
                      std_HeaderServiceInterfaceSkeleton>
      pub_system_manager_worldmodel_reset_response(
          "std_Header/system_manager_worldmodel_reset_response/APAPlanning");
  ap::APPublisher<
      ap_maf_system_manager::sys_world_model_response_service_interface::
          skeleton::
              system_manager_SysWorldModelResponseServiceInterfaceSkeleton>
      pub_system_manager_worldmodel_response_highway(
          "system_manager_SysWorldModelResponse/"
          "system_manager_worldmodel_response_highway/APAPlanning");
  ap::APPublisher<
      ap_maf_system_manager::sys_world_model_response_service_interface::
          skeleton::
              system_manager_SysWorldModelResponseServiceInterfaceSkeleton>
      pub_system_manager_worldmodel_response_parking(
          "system_manager_SysWorldModelResponse/"
          "system_manager_worldmodel_response_parking/APAPlanning");
  ap::APPublisher<
      ap_maf_worldmodel::objects_interface_service_interface::skeleton::
          worldmodel_ObjectsInterfaceServiceInterfaceSkeleton>
      pub_worldmodel_objects(
          "worldmodel_ObjectsInterface/worldmodel_objects/APAPlanning");
  ap::APPublisher<ap_maf_worldmodel::fusion_apa_service_interface::skeleton::
                      worldmodel_FusionAPAServiceInterfaceSkeleton>
      pub_worldmodel_parking_slot_info(
          "worldmodel_FusionAPA/worldmodel_parking_slot_info/APAPlanning");
  ap::APPublisher<ap_maf_worldmodel::processed_map_service_interface::skeleton::
                      worldmodel_ProcessedMapServiceInterfaceSkeleton>
      pub_worldmodel_processed_map(
          "worldmodel_ProcessedMap/worldmodel_processed_map/APAPlanning");
  ap::APPublisher<ap_maf_planning::planning_service_interface::skeleton::
                      planning_PlanningServiceInterfaceSkeleton>
      pub_msd_planning_apa("planning_Planning/msd_planning_apa/APAPlanning");
  ap::APPublisher<
      ap_maf_framework_status::nodes_status_service_interface::skeleton::
          framework_status_NodesStatusServiceInterfaceSkeleton>
      pub_node_status_apa_planning(
          "framework_status_NodesStatus/node_status_apa_planning/APAPlanning");


  // yaml file don't support env parse, so use relative path directly
  std::string config_file = std::string(std::getenv("EXEC_PATH")) +
                            "/../resource/config/msquare_planning.ap.mdc.json";
  std::string mtaskflow_config_file =
      std::string(std::getenv("EXEC_PATH")) +
      "/../resource/config/mtaskflow_config.json";
  std::string wm_config_file =
      std::string(std::getenv("EXEC_PATH")) +
      "/../resource/config/worldmodel_mfr.v1.ap.mdc.json";
  std::string wm_pec_config_file = std::string(std::getenv("EXEC_PATH")) +
                                   "/../resource/config/worldmodel_pec.json";
  std::string config_file_dir =
      std::string(std::getenv("EXEC_PATH")) + "/../resource/config/";
  std::string config_data = read_file(config_file).c_str();

  std::shared_ptr<PlanningEngineInterface> planning_engine;
  SceneType scene_type = SceneType::URBAN;
  double default_cruise_velocity = 120;
  uint32_t underclocking_enum = 10;
  bool enable_ilc = true;
  bool enable_alc = false;
  double ilc_limit_velocity = 0.0;
  double ilc_limit_time = 0.0;
  bool start_machine = false;
  bool is_acc_mode = false;
  bool is_ddmap = false;
  int driving_style = 1;
  bool is_paused = true;

  double v_target_0 = 0.0;
  double a_target_0 = 0.0;

  std::string use_mb_imu = "";

  // init_config_data
  char *log_level = std::getenv("MSQUARE_PNC_LOG_LEVEL");
  auto config_reader = mjson::Reader(config_data.c_str());
  config_reader.expand_environment_varialbe();
  if (log_level) {
    std::string log_level_lower(log_level);
    transform(log_level_lower.begin(), log_level_lower.end(),
              log_level_lower.begin(), tolower);
    auto logger_reader =
        mjson::Reader(config_reader.get<mjson::Json>("logger"));
    logger_reader.override_value("level", log_level_lower);
    config_reader.override_value("logger", logger_reader.raw());
    config_data = config_reader.raw().dump().c_str();
  }
  enable_ilc = config_reader.get<bool>("enable_ilc", false, true);
  enable_alc = config_reader.get<bool>("enable_alc", false, false);
  ilc_limit_velocity =
      config_reader.get<double>("interactive_lc_limit_velocity", false, 0.0);
  ilc_limit_time =
      config_reader.get<double>("interactive_lc_limit_time", false, 10.0);
  default_cruise_velocity =
      config_reader.get<double>("default_cruise_velocity", false, 120.0);
  underclocking_enum =
      config_reader.get<uint32_t>("underclocking_enum", false, 10);
  start_machine = config_reader.get<bool>("start_machine", false, false);
  is_acc_mode = config_reader.get<bool>("is_acc_mode", false, false);
  is_ddmap = config_reader.get<bool>("is_ddmap", false, false);
  driving_style = config_reader.get<int>("driving_style", false, 1);

  // init mlog and mtime
  mlog::PublisherSettings publisher_settings;
  void *node_handle = malloc(1);
  publisher_settings.handle = node_handle;
  publisher_settings.config_file = config_file.c_str();
  std::shared_ptr<mlog::MLogPublisher> publisher =
      mlog::MLogPublisher::make(publisher_settings);

  mlog::MLogEndpointCallback planning_callback =
      [publisher](const std::vector<mlog::MLogEntry> &entries) -> void {
    if (publisher != nullptr) {
      publisher->publish(entries);
    } else {
      fprintf(stderr, "create MLogPublisher failed.\n");
    }
  };

  MSDPlanning_set_log_config(config_data, node_handle);
  if (planning_callback) {
    MSDPlanning_set_log_callback(planning_callback);
  }
  MSDPlanning_init_log_manager();

  MSDPlanningTimeConfig planning_time_config{};
  planning_time_config.basic_config = config_data;
  mtime::MTimeConfig time_config{};
  time_config.timeline_type = mtime::MTIMELINE_TYPE_mtime_customize_timeline;
  planning_time_config.time_config = time_config;
  mtime::MTimeClockFeeder feeder;
  MSDPlanning_set_time_config(planning_time_config, feeder);

  // for wm
  msd_worldmodel::MSDWorldModelConfig worldmodel_config{};
  worldmodel_config.config_json = read_file(wm_config_file).c_str();

  mlog::PublisherSettings wm_publisher_settings;
  wm_publisher_settings.handle = nullptr;
  wm_publisher_settings.config_file = wm_config_file.c_str();
  auto mlog_publisher = mlog::MLogPublisher::make(publisher_settings);

  mlog::MLogEndpointCallback worldmodel_mlog_callback =
      [mlog_publisher](const std::vector<mlog::MLogEntry> &entries) {
        mlog_publisher->publish(entries);
      };

  // bool success = false;

  // init mlog
  // success =
  //     msd_worldmodel::MSDWorldModel_set_log_config(wm_config_file.c_str());
  // if (!success) {
  //   printf("ERROR: MSDWorldModel_set_log_config failed!");
  // }
  // msd_worldmodel::MSDWorldModel_set_log_callback(worldmodel_mlog_callback);
  // msd_worldmodel::MSDWorldModel_init_log_manager();

  // init PlanningEngine
  MSDPlanningConfig planning_config{};
  planning_config.scene_type = scene_type;
  planning_config.enable_timer_tick = true;
  planning_config.cruise_velocity = default_cruise_velocity;
  planning_config.underclocking_enum = underclocking_enum;
  planning_config.enable_ilc = enable_ilc;
  planning_config.enable_alc = enable_alc;
  planning_config.ilc_limit_velocity = ilc_limit_velocity;
  planning_config.ilc_limit_time = ilc_limit_time;
  planning_config.is_acc_mode = is_acc_mode;
  planning_config.is_ddmap = is_ddmap;
  planning_config.driving_style = driving_style;
  planning_config.config_file_dir = config_file_dir.c_str();
  planning_config.mtaskflow_config_file = mtaskflow_config_file.c_str();
  planning_config.wm_config_file = worldmodel_config.config_json.c_str();
  planning_config.wm_pec_config_file = read_file(wm_pec_config_file).c_str();
  planning_config.init_wm = true;
  planning_engine = PlanningEngineInterface::make(planning_config);

  planning_engine->init();

  if (start_machine) {
    is_paused = false;
    planning_engine->start();

    // feed start cmd
    maf_system_manager::ModuleControlCmdRequest request{};
    request.header.stamp = MTIME()->timestamp().ns();
    request.module_control_cmd.value =
        maf_system_manager::ModuleControlCmdEnum::RESUME;
    if (scene_type == SceneType::URBAN) {
      request.running_mode.value = maf_system_manager::RunningModeEnum::PILOT;
    } else {
      request.running_mode.value = maf_system_manager::RunningModeEnum::PARKING;
    }
    planning_engine->feed_planning_control_cmd_request(request);
  }

  auto env = std::getenv("SENSOR_IMU");
  if (env != nullptr) {
    use_mb_imu = std::string(env);
  }

  // set callback for sub topics
  sub_mla_egopose_by_localization.setCallback(
      [&](const ap_maf_mla_localization::MLALocalization *src) {
        if (is_paused) {
          return;
        }
        maf_mla_localization::MLALocalization dst{};
        FROM_AP(src, dst);
        auto dst_ptr = std::make_shared<maf_mla_localization::MLALocalization>(
            std::move(dst));
        planning_engine->feed_ego_pose(dst_ptr);
      });
  sub_msd_control_mpc_traj_by_control.setCallback(
      [&](const ap_maf_planning::MpcTrajectoryResult *src) {
        if (is_paused) {
          return;
        }
        maf_planning::MpcTrajectoryResult dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_planning::MpcTrajectoryResult>(std::move(dst));
        planning_engine->feed_mpc_trajectory(dst_ptr);
      });
  sub_msd_function_module_status_by_vehicle_service.setCallback(
      [&](const ap_maf_framework_status::ModuleStatus *src) {
        if (is_paused) {
          return;
        }
        maf_framework_status::ModuleStatus dst{};
        FROM_AP(src, dst);
        if (dst.module_type.value ==
            maf_framework_status::ModuleType::PLANNING) {
          return;
        }
        planning_engine->feed_module_status(dst);
      });
  sub_msd_planning_frequency_control_by_nppcpp_switch.setCallback(
      [&](const ap_maf_system_manager::SysPlanningRequest *src) {
        maf_system_manager::SysPlanningRequest dst{};
        FROM_AP(src, dst);
        planning_engine->feed_planning_request(dst);
      });
  sub_msd_prediction_prediction_result_by_ftp.setCallback(
      [&](const ap_maf_worldmodel::PredictionResult *src) {
        if (is_paused) {
          return;
        }
        maf_worldmodel::PredictionResult dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_worldmodel::PredictionResult>(std::move(dst));
        planning_engine->feed_prediction_info(dst_ptr);
      });
  sub_perception_fusion_distance_uss_by_parking_fusion.setCallback(
      [&](const ap_maf_sensor_interface::UltrasonicUpaReport *src) {
        if (is_paused) {
          return;
        }
        maf_sensor_interface::UltrasonicUpaReport dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_sensor_interface::UltrasonicUpaReport>(
                std::move(dst));
        planning_engine->feed_fusion_uss(dst_ptr);
      });
  sub_perception_fusion_ground_line_by_parking_fusion.setCallback(
      [&](const ap_maf_perception_interface::FusionGroundLineResult *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::FusionGroundLineResult dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::FusionGroundLineResult>(
                std::move(dst));
        planning_engine->feed_fusion_grond_lines(dst_ptr);
      });
  sub_perception_lidar_road_edge_by_lidar_detection.setCallback(
      [&](const ap_maf_perception_interface::RoadLinePerception *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::RoadLinePerception dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::RoadLinePerception>(
                std::move(dst));
        planning_engine->feed_perception_lidar_road_edge(dst_ptr);
      });
  sub_perception_fusion_ground_line_uss_by_parking_fusion.setCallback(
      [&](const ap_maf_perception_interface::FusionGroundLineResult *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::FusionGroundLineResult dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::FusionGroundLineResult>(
                std::move(dst));
        planning_engine->feed_fusion_uss_grond_lines(dst_ptr);
      });
  sub_perception_fusion_object_by_ftp.setCallback(
      [&](const ap_maf_perception_interface::PerceptionFusionObjectResult
              *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::PerceptionFusionObjectResult dst{};
        FROM_AP(src, dst);
        auto dst_ptr = std::make_shared<
            maf_perception_interface::PerceptionFusionObjectResult>(
            std::move(dst));
        planning_engine->feed_perception_fusion_result(dst_ptr);
      });
  sub_perception_fusion_object_parking_environment_by_parking_fusion
      .setCallback(
          [&](const ap_maf_perception_interface::PerceptionFusionObjectResult
                  *src) {
            if (is_paused) {
              return;
            }
            maf_perception_interface::PerceptionFusionObjectResult dst{};
            FROM_AP(src, dst);
            auto dst_ptr = std::make_shared<
                maf_perception_interface::PerceptionFusionObjectResult>(
                std::move(dst));
            planning_engine->feed_fusion_objects(dst_ptr);
          });
  sub_perception_fusion_parking_slot_by_parking_fusion.setCallback(
      [&](const ap_maf_perception_interface::FusionParkingSlotResult *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::FusionParkingSlotResult dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::FusionParkingSlotResult>(
                std::move(dst));
        planning_engine->feed_fusion_parking_slot(dst_ptr);
      });
  sub_perception_radar_result_front_by_sensor_service.setCallback(
      [&](const ap_maf_perception_interface::RadarPerceptionResult *src) {
        maf_perception_interface::RadarPerceptionResult dst{};
        if (is_paused) {
          return;
        }
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::RadarPerceptionResult>(
                std::move(dst));
        planning_engine->feed_perception_radar(dst_ptr);
      });
  sub_perception_vision_lane_by_camera_detection.setCallback(
      [&](const ap_maf_perception_interface::RoadLinePerception *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::RoadLinePerception dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::RoadLinePerception>(
                std::move(dst));
        planning_engine->feed_perception_vision_lane(dst_ptr);
      });
  sub_sensor_imu_by_sensor_service.setCallback(
      [&](const ap_maf_gps_imu::MLAImu *src) {
        if (is_paused) {
          return;
        }
        maf_gps_imu::MLAImu dst{};
        FROM_AP(src, dst);
        auto dst_ptr = std::make_shared<maf_gps_imu::MLAImu>(std::move(dst));
        if (use_mb_imu == "MB-Z") {
          planning_engine->feed_imu_report(dst_ptr);
        }
      });
  sub_sensor_imu_by_vehicle_service.setCallback(
      [&](const ap_maf_gps_imu::MLAImu *src) {
        if (is_paused) {
          return;
        }
        maf_gps_imu::MLAImu dst{};
        FROM_AP(src, dst);
        auto dst_ptr = std::make_shared<maf_gps_imu::MLAImu>(std::move(dst));
        if (use_mb_imu == "ORI") {
          planning_engine->feed_imu_report(dst_ptr);
        }
      });
  sub_system_manager_apa_planning_control_cmd_request_by_nppcpp_switch
      .setCallback(
          [&](const ap_maf_system_manager::ModuleControlCmdRequest *src) {
            maf_system_manager::ModuleControlCmdRequest dst{};
            FROM_AP(src, dst);
            if (dst.module_control_cmd.value ==
                maf_system_manager::ModuleControlCmdEnum::PAUSE) {
              is_paused = true;
              planning_engine->stop();
            } else if (dst.module_control_cmd.value ==
                       maf_system_manager::ModuleControlCmdEnum::RESUME) {
              is_paused = false;
              planning_engine->start();
            }
            planning_engine->feed_planning_control_cmd_request(dst);

            // send response
            maf_system_manager::ModuleControlCmdResponse
                planning_control_cmd_response{};
            planning_control_cmd_response.header.stamp =
                MTIME()->timestamp().ns();
            planning_control_cmd_response.request_seq = dst.header.seq;
            planning_control_cmd_response.success = true;

            auto to_ap_dst = pub_system_manager_apa_planning_control_cmd_response
                                 .allocateEmptyData();
            TO_AP(planning_control_cmd_response, to_ap_dst);
            pub_system_manager_apa_planning_control_cmd_response.publish(
                std::move(to_ap_dst));
          });
  sub_system_manager_apa_planning_request_by_nppcpp_switch.setCallback(
      [&](const ap_maf_system_manager::SysPlanningRequest *src) {
        maf_system_manager::SysPlanningRequest dst{};
        FROM_AP(src, dst);
        // ignore force stop when planning already started and return false
        bool ignore_force_stop = false;
        const double starting_target_speed = 0.2;
        const double starting_target_accel = 2;
        if (dst.cmd.value == maf_system_manager::SystemCmdTypeEnum::
                                 PLANNING_HIGHWAY_START_STOP) {
          if (dst.highway_info.start_stop_cmd.value ==
              maf_system_manager::StartStopCmdEnum::STOP) {
            if (v_target_0 > starting_target_speed ||
                a_target_0 > starting_target_accel) {
              ignore_force_stop = true;
            }
          }
        }

        bool success = true;
        if (!ignore_force_stop) {
          planning_engine->feed_planning_request(dst);
        } else {
          success = false;
        }

        // send response
        maf_system_manager::SysPlanningResponse planning_response{};
        planning_response.header.stamp = MTIME()->timestamp().ns();
        planning_response.request_seq = dst.header.seq;
        planning_response.success = success;

        auto to_ap_dst =
            pub_system_manager_apa_planning_response.allocateEmptyData();
        TO_AP(planning_response, to_ap_dst);
        pub_system_manager_apa_planning_response.publish(std::move(to_ap_dst));
      });
  sub_system_manager_info_parking_by_mff.setCallback(
      [&](const ap_maf_std::Header *src) {
        if (is_paused) {
          return;
        }
        maf_std::Header dst{};
        FROM_AP(src, dst);
        auto dst_ptr = std::make_shared<maf_std::Header>(std::move(dst));
        planning_engine->feed_mff_info(dst_ptr);
      });
  sub_system_manager_worldmodel_control_cmd_request_highway_by_mff.setCallback(
      [&](const ap_maf_system_manager::ModuleControlCmdRequest *src) {
        maf_system_manager::ModuleControlCmdRequest dst{};
        FROM_AP(src, dst);

        // send fake response
        maf_system_manager::ModuleControlCmdResponse response{};
        response.header.stamp = MTIME()->timestamp().ns();
        response.header.seq = dst.header.seq;
        response.success = true;

        auto to_ap_dst =
            pub_system_manager_worldmodel_control_cmd_response_highway
                .allocateEmptyData();
        TO_AP(response, to_ap_dst);
        pub_system_manager_worldmodel_control_cmd_response_highway.publish(
            std::move(to_ap_dst));
      });
  sub_system_manager_worldmodel_control_cmd_request_parking_by_mff.setCallback(
      [&](const ap_maf_system_manager::ModuleControlCmdRequest *src) {
        maf_system_manager::ModuleControlCmdRequest dst{};
        FROM_AP(src, dst);

        // send fake response
        maf_system_manager::ModuleControlCmdResponse parking_wm_response{};
        parking_wm_response.header.stamp = MTIME()->timestamp().ns();
        parking_wm_response.request_seq = dst.header.seq;
        parking_wm_response.success = true;

        auto to_ap_dst =
            pub_system_manager_worldmodel_control_cmd_response_parking
                .allocateEmptyData();
        TO_AP(parking_wm_response, to_ap_dst);
        pub_system_manager_worldmodel_control_cmd_response_parking.publish(
            std::move(to_ap_dst));
      });
  sub_system_manager_worldmodel_request_highway_by_mff.setCallback(
      [&](const ap_maf_system_manager::SysWorldModelRequest *src) {
        maf_system_manager::SysWorldModelRequest dst{};
        FROM_AP(src, dst);

        // send fake response
        maf_system_manager::SysWorldModelResponse maf_msg{};
        maf_msg.header.stamp = MTIME()->timestamp().ns();
        maf_msg.header.seq = dst.header.seq;
        maf_msg.success = true;

        auto to_ap_dst =
            pub_system_manager_worldmodel_response_highway.allocateEmptyData();
        TO_AP(maf_msg, to_ap_dst);
        pub_system_manager_worldmodel_response_highway.publish(
            std::move(to_ap_dst));
      });
  sub_vehicle_body_report_by_vehicle_service.setCallback(
      [&](const ap_maf_endpoint::BodyReport *src) {
        if (is_paused) {
          return;
        }
        maf_endpoint::BodyReport dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_endpoint::BodyReport>(std::move(dst));
        planning_engine->feed_body_report(dst_ptr);
      });
  sub_vehicle_chassis_report_by_vehicle_service.setCallback(
      [&](const ap_maf_endpoint::ChassisReport *src) {
        if (is_paused) {
          return;
        }
        maf_endpoint::ChassisReport dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_endpoint::ChassisReport>(std::move(dst));
        planning_engine->feed_chassis_report(dst_ptr);
      });
  sub_vehicle_wheel_report_by_vehicle_service.setCallback(
      [&](const ap_maf_endpoint::WheelReport *src) {
        if (is_paused) {
          return;
        }
        maf_endpoint::WheelReport dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_endpoint::WheelReport>(std::move(dst));
        planning_engine->feed_wheel_report(dst_ptr);
      });
  sub_worldmodel_traffic_light_by_ftp.setCallback(
      [&](const ap_maf_perception_interface::TrafficLightPerception *src) {
        if (is_paused) {
          return;
        }
        maf_perception_interface::TrafficLightPerception dst{};
        FROM_AP(src, dst);
        auto dst_ptr =
            std::make_shared<maf_perception_interface::TrafficLightPerception>(
                std::move(dst));
        planning_engine->feed_traffic_light(dst_ptr);
      });

  printf("start set cllback\n");

  // set callback for pub topics
  planning_engine->set_callback(
      [&pub_msd_planning_apa, &v_target_0,
       &a_target_0](const MSDPlanningOutputMeta &meta,
                    const maf_planning::Planning &planning_result,
                    const std::string &trigger_msg_id) {
        if (meta.succeed) {
          auto to_ap_dst = pub_msd_planning_apa.allocateEmptyData();
          TO_AP(planning_result, to_ap_dst);
          pub_msd_planning_apa.publish(std::move(to_ap_dst));

          if (planning_result.trajectory.velocity.vel_points.size() > 0) {
            v_target_0 = planning_result.trajectory.velocity.vel_points[0]
                             .target_velocity;
          }
          if (planning_result.trajectory.acceleration.acc_points.size() > 0) {
            a_target_0 =
                planning_result.trajectory.acceleration.acc_points[0].acc;
          }
        }
      });

  planning_engine->set_callback(
      [&pub_node_status_apa_planning, &pub_node_status_worldmodel_highway,
       &pub_node_status_worldmodel_parking](
          const maf_framework_status::NodeStatus &planning_node_status) {
        maf_framework_status::NodesStatus planning_nodes_status;
        planning_nodes_status.header.stamp = MTIME()->timestamp().ns();
        planning_nodes_status.nodes_status.push_back(planning_node_status);

        auto ap_planning_node_status =
            pub_node_status_apa_planning.allocateEmptyData();
        TO_AP(planning_nodes_status, ap_planning_node_status);
        pub_node_status_apa_planning.publish(std::move(ap_planning_node_status));

        // highway worldmodel
        maf_framework_status::NodesStatus highway_wm_nodes_status;
        highway_wm_nodes_status.nodes_status.resize(1);
        highway_wm_nodes_status.header.stamp = MTIME()->timestamp().ns();

        highway_wm_nodes_status.nodes_status[0].timestamp_us =
            MTIME()->timestamp().us();
        highway_wm_nodes_status.nodes_status[0].node_type =
            static_cast<uint16_t>(node_status::NodeType::MODULE_WORLDMODEL);
        highway_wm_nodes_status.nodes_status[0].reserved_info = "ddmap";
        highway_wm_nodes_status.nodes_status[0].message =
            "worldmodel is running NORMAL";
        highway_wm_nodes_status.nodes_status[0].running_mode.value =
            highway_wm_nodes_status.nodes_status[0].running_mode.PILOT;
        highway_wm_nodes_status.nodes_status[0].status =
            static_cast<uint16_t>(node_status::Status::RUNNING);

        auto ap_highway_wm_nodes_status =
            pub_node_status_worldmodel_highway.allocateEmptyData();
        TO_AP(highway_wm_nodes_status, ap_highway_wm_nodes_status);
        pub_node_status_worldmodel_highway.publish(
            std::move(ap_highway_wm_nodes_status));

        // parking worldmodel
        maf_framework_status::NodesStatus parking_wm_nodes_status;
        parking_wm_nodes_status.nodes_status.resize(1);
        parking_wm_nodes_status.header.stamp = MTIME()->timestamp().ns();

        parking_wm_nodes_status.nodes_status[0].timestamp_us =
            MTIME()->timestamp().us();
        parking_wm_nodes_status.nodes_status[0].node_type =
            static_cast<uint16_t>(node_status::NodeType::MODULE_WORLDMODEL);
        parking_wm_nodes_status.nodes_status[0].task_type =
            static_cast<uint16_t>(
                node_status::TaskType::NODE_WORLDMODEL_TASK_APA);
        parking_wm_nodes_status.nodes_status[0].detail_status =
            static_cast<uint16_t>(node_status::DetailStatus::WORLDMODEL_OK);
        parking_wm_nodes_status.nodes_status[0].running_mode.value =
            parking_wm_nodes_status.nodes_status[0].running_mode.PARKING;
        parking_wm_nodes_status.nodes_status[0].status =
            static_cast<uint16_t>(node_status::Status::RUNNING);

        auto ap_parking_wm_nodes_status =
            pub_node_status_worldmodel_parking.allocateEmptyData();
        TO_AP(parking_wm_nodes_status, ap_parking_wm_nodes_status);
        pub_node_status_worldmodel_parking.publish(
            std::move(ap_parking_wm_nodes_status));
      });

  planning_engine->set_callback(
      [&pub_apa_planning_info](const maf_std::Header &planning_info) {
        auto ap_planning_info = pub_apa_planning_info.allocateEmptyData();
        TO_AP(planning_info, ap_planning_info);
        pub_apa_planning_info.publish(std::move(ap_planning_info));
      });

  // for wm
  planning_engine->set_processed_map_callback(
      [&pub_worldmodel_processed_map,
       &planning_engine](ProcessedMapPtr processed_map) {
        auto ap_processed_map =
            pub_worldmodel_processed_map.allocateEmptyData();
        TO_AP(*processed_map, ap_processed_map);
        pub_worldmodel_processed_map.publish(std::move(ap_processed_map));

        planning_engine->feed_worldmodel_map(processed_map);
      });

  planning_engine->set_objects_interface_callback(
      [&pub_worldmodel_objects,
       &planning_engine](ObjectsInterfacePtr objects_interface) {
        auto ap_objects_interface = pub_worldmodel_objects.allocateEmptyData();
        TO_AP(*objects_interface, ap_objects_interface);
        pub_worldmodel_objects.publish(std::move(ap_objects_interface));

        planning_engine->feed_worldmodel_objects(objects_interface);
      });

  // parking worldmodel
  planning_engine->set_wm_parking_slot_callback(
      [&pub_worldmodel_parking_slot_info, &planning_engine](
          const std::shared_ptr<maf_worldmodel::FusionAPA> &parking_slot_info) {
        auto ap_parking_slot_info =
            pub_worldmodel_parking_slot_info.allocateEmptyData();
        TO_AP(*parking_slot_info, ap_parking_slot_info);
        pub_worldmodel_parking_slot_info.publish(
            std::move(ap_parking_slot_info));

        planning_engine->feed_world_model_parking_slots(parking_slot_info);
      });

  planning_engine->set_callback(
      [&pub_msd_sbp_request](const maf_planning::SBPRequest &sbp_request) {
        auto ap_sbp_request = pub_msd_sbp_request.allocateEmptyData();
        TO_AP(sbp_request, ap_sbp_request);
        pub_msd_sbp_request.publish(std::move(ap_sbp_request));
      });
  // Start algorithm handle if required

  printf("init done\n");

  ap::registerSignalHandler();
  ap::getSpinner().startPubSub();
  ap::getSpinner().spin();
  ap::APSpinner::exit();

  printf("exit\n");

  // Stop algorithm handle if required
}
