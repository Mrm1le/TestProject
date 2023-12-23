#include "common/planning_engine.h"
#include "common/version.h"
#include "planning/common/common.h"
#include <string>

namespace msquare {

PlanningEngine::PlanningEngine(const MSDPlanningConfig &planning_config)
    : enable_timer_tick_(planning_config.enable_timer_tick) {
  parking::PlanningContext::Instance()->set_config_file_dir(
      planning_config.config_file_dir);
  worldmodel_pec::SlotReleaseContext::Instance()->set_config_file_dir(
      planning_config.config_file_dir);

  std::string planning_version(PLANNING_PLATFORM);
  planning_version.append("#");
  planning_version.append(PLANNING_BRANCH);
  planning_version.append("#");
  planning_version.append(PLANNING_COMMIT);
  MSD_LOG(INFO, "planning_version: %s", planning_version.c_str());

  // parking version
  // parking::PlanningContext::Instance()->set_version("2022.0924.0-#f53d1df32df");
  parking::PlanningContext::Instance()->set_version(planning_version);

  planning_config_ = planning_config;

  // TODO 需要在构造函数中指明当前模块的NodeType
  status_manager_->set_node_type(static_cast<node_status::NodeType>(33)); // node_status::NodeType::MODULE_PLANNING
  // TODO
  // start_notifier会单独启动一个通知线程，每1000ms调用一次callback，如果模块为了节省线程，可以手动get_status()
  status_manager_->start_notifier(100);

  // monitor_ = ModuleMonitor::make_monitor(10);

  (void)machine_.init_config({planning_config.mtaskflow_config_file.c_str(),
                              "Planning", nullptr, nullptr,
                              PARAM_KEY_MFR_SNAPSHOT_NAME});

  if (machine_.tasknap_is_enable()) {
    taskagent_ptr_ =
        mtaskflow::TasknapAgent::GetInst(PARAM_KEY_MFR_SNAPSHOT_NAME);
  }

  // tick resource used for frame running
  auto tick_resource = machine_.create_resource<uint64_t>();

  mtaskflow::TaskConfig task_config{};

  if (enable_timer_tick_) {
    task_config.running_mode_config = {10,
                                       mtaskflow::RunningMode::FREQUENCY_MODE};
  } else {
    tick_publisher_ = tick_resource->create_publisher();
  }

  // planning task
  auto module_status_resource =
      machine_.create_resource<maf_framework_status::ModuleStatus>();
  module_status_publisher_ = module_status_resource->create_publisher();

  auto chassis_report_resource =
      machine_.create_resource<std::shared_ptr<maf_endpoint::ChassisReport>>();
  chassis_report_publisher_ = chassis_report_resource->create_publisher();

  auto wireless_charger_report_resource = machine_.create_resource<
      std::shared_ptr<maf_endpoint::WirelessChargerReport>>();
  wireless_charger_report_publisher_ =
      wireless_charger_report_resource->create_publisher();

  auto wheel_report_resource =
      machine_.create_resource<std::shared_ptr<maf_endpoint::WheelReport>>();
  wheel_report_publisher_ = wheel_report_resource->create_publisher();

  auto body_report_resource =
      machine_.create_resource<std::shared_ptr<maf_endpoint::BodyReport>>();
  body_report_publisher_ = body_report_resource->create_publisher();

  auto imu_report_resource =
      machine_.create_resource<std::shared_ptr<maf_gps_imu::MLAImu>>();
  imu_report_publisher_ = imu_report_resource->create_publisher();

  auto worldmodel_map_resource =
      machine_.create_resource<std::shared_ptr<maf_worldmodel::ProcessedMap>>();
  worldmodel_map_publisher_ = worldmodel_map_resource->create_publisher();

  auto worldmodel_objects_resource =
      machine_
          .create_resource<std::shared_ptr<maf_worldmodel::ObjectsInterface>>();
  worldmodel_objects_publisher_ =
      worldmodel_objects_resource->create_publisher();

  auto worldmodel_parking_slots_resource =
      machine_.create_resource<std::shared_ptr<maf_worldmodel::FusionAPA>>();
  worldmodel_parking_slots_publisher_ =
      worldmodel_parking_slots_resource->create_publisher();

  auto worldmodel_scene_objects_resource =
      machine_.create_resource<std::shared_ptr<maf_worldmodel::SceneObjects>>();
  worldmodel_scene_objects_publisher_ =
      worldmodel_scene_objects_resource->create_publisher();

  auto fusion_objects_resource = machine_.create_resource<std::shared_ptr<
      maf_perception_interface::PerceptionFusionObjectResult>>();
  fusion_objects_publisher_ = fusion_objects_resource->create_publisher();

  auto fusion_groundlines_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>();
  fusion_groundlines_publisher_ =
      fusion_groundlines_resource->create_publisher();

  auto fusion_uss_groundlines_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::FusionGroundLineResult>>();
  fusion_uss_groundlines_publisher_ =
      fusion_uss_groundlines_resource->create_publisher();

  auto fusion_uss_resource = machine_.create_resource<
      std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport>>();
  fusion_uss_publisher_ = fusion_uss_resource->create_publisher();

  auto traffic_light_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::TrafficLightPerception>>();
  traffic_light_publisher_ = traffic_light_resource->create_publisher();

  auto ego_pose_resource = machine_.create_resource<
      std::shared_ptr<maf_mla_localization::MLALocalization>>();
  ego_pose_publisher_ = ego_pose_resource->create_publisher();

  auto prediction_info_resource =
      machine_
          .create_resource<std::shared_ptr<maf_worldmodel::PredictionResult>>();
  prediction_info_publisher_ = prediction_info_resource->create_publisher();

  auto mpc_trajectory_resource = machine_.create_resource<
      std::shared_ptr<maf_planning::MpcTrajectoryResult>>();
  mpc_trajectory_publisher_ = mpc_trajectory_resource->create_publisher();

  auto planning_control_cmd_request_resource =
      machine_.create_resource<maf_system_manager::ModuleControlCmdRequest>();
  planning_control_cmd_request_publisher_ =
      planning_control_cmd_request_resource->create_publisher();

  auto planning_request_resource =
      machine_.create_resource<maf_system_manager::SysPlanningRequest>();
  planning_request_publisher_ = planning_request_resource->create_publisher();

  auto planning_reset_request_resource =
      machine_.create_resource<maf_std::Header>();
  planning_reset_request_publisher_ =
      planning_reset_request_resource->create_publisher();

  auto sequence_resource = machine_.create_resource<uint64_t>();

  auto sbp_result_resource = machine_.create_resource<SbpResult>();
  auto sbp_problem_resource =
      machine_.create_resource<parking::OpenspaceDeciderOutput>();

  auto perception_vision_lane_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>();
  perception_vision_lane_publisher_ =
      perception_vision_lane_resource->create_publisher();

  auto perception_lidar_road_edge_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>();
  perception_lidar_road_edge_publisher_ =
      perception_lidar_road_edge_resource->create_publisher();

  auto perception_radar_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::RadarPerceptionResult>>();
  perception_radar_publisher_ = perception_radar_resource->create_publisher();

  auto mff_info_resource =
      machine_.create_resource<std::shared_ptr<maf_std::Header>>();
  mff_info_publisher_ = mff_info_resource->create_publisher();

  auto fusion_parking_slot_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::FusionParkingSlotResult>>();
  fusion_parking_slot_publisher_ =
      fusion_parking_slot_resource->create_publisher();

  auto sbp_debug_resource = machine_.create_resource<std::string>();

  auto ddmap_processed_map_resource =
      machine_.create_resource<ProcessedMapPtr>();

  auto wm_obj_resource = machine_.create_resource<ObjectsInterfacePtr>();

  auto perception_fusion_resource =
      machine_.create_resource<PerceptionFusionObjectResultPtr>();
  perception_fusion_result_publisher_ =
      perception_fusion_resource->create_publisher();

  const auto rsize = mtaskflow::RECEIVER_DEFAULT_QUEUE_SIZE;
  bool snapshot = true;
  const char *sim_env = std::getenv("RealitySimulation");
  if (sim_env != nullptr && std::string(sim_env) == "simulation") {
    snapshot = false;
  }
  
  task_config.config_key = "ParkingPlanTask";
  // task_config.force_disable_snap = true;
  parking_planning_task_ = machine_.create_task<parking::PlanningTask>(
      task_config, planning_config, enable_timer_tick_, status_manager_,
      tick_resource->create_receiver(rsize, snapshot),
      module_status_resource->create_receiver(rsize, snapshot),
      chassis_report_resource->create_receiver(),
      wheel_report_resource->create_receiver(),
      body_report_resource->create_receiver(),
      imu_report_resource->create_receiver(rsize, snapshot),
      worldmodel_map_resource->create_receiver(),
      worldmodel_objects_resource->create_receiver(),
      worldmodel_parking_slots_resource->create_receiver(),
      worldmodel_scene_objects_resource->create_receiver(),
      fusion_objects_resource->create_receiver(),
      fusion_groundlines_resource->create_receiver(),
      fusion_uss_groundlines_resource->create_receiver(),
      fusion_uss_resource->create_receiver(),
      traffic_light_resource->create_receiver(),
      ego_pose_resource->create_receiver(),
      prediction_info_resource->create_receiver(),
      mpc_trajectory_resource->create_receiver(rsize, snapshot),
      planning_control_cmd_request_resource->create_receiver(32, false),
      planning_request_resource->create_receiver(rsize, snapshot),
      sbp_problem_resource->create_publisher(snapshot),
      sbp_result_resource->create_receiver(rsize, snapshot),
      sbp_debug_resource->create_receiver(rsize, snapshot),
      wireless_charger_report_resource->create_receiver(rsize, snapshot));

  task_config.config_key = "SbpPlanTask";
  task_config.running_mode_config.hz = 0.0;
  task_config.running_mode_config.mode = mtaskflow::RunningMode::RESOURCE_MODE;
  sbp_planning_task_ = machine_.create_task<parking::SBPlanningTask>(
      task_config, sbp_problem_resource->create_receiver(rsize, snapshot),
      sbp_result_resource->create_publisher(snapshot),
      sbp_debug_resource->create_publisher(snapshot));
  // task_config.force_disable_snap = false;

  // parking worldmodel
  if (planning_config.init_wm) {
    mtaskflow::TaskConfig pec_task_config;
    pec_task_config.config_key = "ParkingWorldModelTask";
    pec_task_config.running_mode_config.mode =
        mtaskflow::RunningMode::FREQUENCY_MODE;
    pec_task_config.running_mode_config.hz = 5;
    // pec_task_config.force_disable_snap = true;
    pec_task_ = machine_.create_task<::parking::SetPSDFusionTask>(
        pec_task_config,
        fusion_parking_slot_resource->create_receiver(),
        ego_pose_resource->create_receiver(),
        fusion_uss_groundlines_resource->create_receiver(),
        fusion_objects_resource->create_receiver(),
        planning_control_cmd_request_resource->create_receiver(32, false),
        wireless_charger_report_resource->create_receiver(rsize, snapshot),
        planning_request_resource->create_receiver(rsize, snapshot),
        planning_config.wm_pec_config_file);

    auto wm_reader = mjson::Reader(planning_config.wm_config_file);
    auto fusion_reader = mjson::Reader(
        wm_reader.get<mjson::Json>("worldmodel_config.task_config.FusionTask"));
    auto enable_fusion_filter =
        fusion_reader.get<bool>("enable_fusion_filter", false, false);
    bool enable_fusion_cone_filter =
        fusion_reader.get<bool>("enable_fusion_cone_filter", false, false);
    auto enable_force_filter =
        fusion_reader.get<bool>("enable_force_filter", false, false);
    auto force_filter_x_distance =
        fusion_reader.get<float>("force_filter_x_distance", false, 200);
    auto force_ddmap = wm_reader.get<bool>("worldmodel_config.force_ddmap");

    task_config.config_key = "WorldmodelPubTask";
    // task_config.force_disable_snap = true;
    wm_pub_task_ = machine_.create_task<msd_worldmodel::WorldmodelPubTask>(
        task_config,
        ddmap_processed_map_resource->create_receiver(),
        wm_obj_resource->create_receiver());
  }

  mtaskflow::MachineConfig machine_config{};
  machine_config.tasknap_config_key = "mfr_snapshot_planning";
  machine_config.timestamp_callback = []() -> double {
    return MTIME()->timestamp().ms();
  };
  machine_config.sleeper_callback = [](double duration_ms) {
    MTIME()->sleep(mtime::MTimeDuration::ms(duration_ms));
  };
  (void)machine_.init_config(machine_config);

  // machine_.run();
}

PlanningEngine::~PlanningEngine() {}

bool PlanningEngine::init() {
  // Init: from STARTING to STOP
  (void)machine_.init();
  // StatusManager会检查当前状态是否可以切换至目标状态，如果可以，则执行pre_transition_func后，再执行切换并返回true，否则返回false
  return status_manager_->try_change_status(node_status::Status::PENDING,
                                            [this]() {
                                              // TODO *** 这里进行算法初始化 ***
                                            });
}

bool PlanningEngine::start() {
  // start: from STOP to RUNNING
  if (!status_manager_->is_notifier_running()) {
    status_manager_->start_notifier(100);
  }
  return status_manager_->try_change_status(
      node_status::Status::RUNNING, [this]() {
        // TODO *** 这里启动算法 ***
        if (!is_machine_running_) {
          machine_.run();
          is_machine_running_ = true;
        }
        MSD_LOG(ERROR, "planning engine: machine start");
      });
}

bool PlanningEngine::stop() {
  // stop: from running to stop
  return status_manager_->try_change_status(
      node_status::Status::PENDING, [this]() {
        // TODO *** 这里停止算法 ***
        reset();
        if (is_machine_running_) {
          machine_.stop();
          is_machine_running_ = false;
        }
        MSD_LOG(ERROR, "planning engine: machine stop");
      });
}

bool PlanningEngine::reset() {
  // reset: from runningerror or stop to stop

  // TODO *** 这里重置算法 ***
  parking_planning_task_->reset();
  pec_task_->reset();
  MSD_LOG(ERROR, "planning engine: task reset");
  return true;
}

bool PlanningEngine::exit() {
  // reset: from any states to stop
  return status_manager_->try_change_status(node_status::Status::STOP,
                                            [this]() {
                                              // TODO *** 这里退出算法 ***
                                            });
}

void PlanningEngine::feed_tick(uint64_t tick) {
  if (!enable_timer_tick_ && nullptr != tick_publisher_) {
    tick_publisher_->publish(tick);
  }
}

void PlanningEngine::keep_run() {
  if (!is_machine_running_) {
    start();
  }
}

void PlanningEngine::keep_stop() {
  if (is_machine_running_) {
    stop();
  }
}

void PlanningEngine::stop_status_manager() {
  if (status_manager_->is_notifier_running()) {
    status_manager_->stop_notifier();
  }
}

void PlanningEngine::feed_module_status(
    const maf_framework_status::ModuleStatus &module_status) {
  if (nullptr != module_status_publisher_ &&
      ModuleType::ENDPOINT == module_status.module_type.value) {
    module_status_publisher_->publish(module_status);
  }
}

void PlanningEngine::feed_chassis_report(
    const std::shared_ptr<maf_endpoint::ChassisReport> chassis_report) {
  if (nullptr != chassis_report_publisher_) {
    chassis_report_publisher_->publish(chassis_report);
  }
}

void PlanningEngine::feed_wireless_charger_report(
    const std::shared_ptr<maf_endpoint::WirelessChargerReport>
        wireless_charger_report) {
  if (nullptr != wireless_charger_report_publisher_) {
    wireless_charger_report_publisher_->publish(wireless_charger_report);
  }
}

void PlanningEngine::feed_wheel_report(
    const std::shared_ptr<maf_endpoint::WheelReport> wheel_report) {
  if (nullptr != wheel_report_publisher_) {
    wheel_report_publisher_->publish(wheel_report);
  }
}

void PlanningEngine::feed_body_report(
    const std::shared_ptr<maf_endpoint::BodyReport> body_report) {
  if (nullptr != body_report_publisher_) {
    body_report_publisher_->publish(body_report);
  }
}

void PlanningEngine::feed_imu_report(
    const std::shared_ptr<maf_gps_imu::MLAImu> imu_report) {
  if (nullptr != imu_report_publisher_) {
    imu_report_publisher_->publish(imu_report);
  }
}

void PlanningEngine::feed_worldmodel_map(
    const std::shared_ptr<maf_worldmodel::ProcessedMap> processed_map) {
  if (nullptr != worldmodel_map_publisher_) {
    worldmodel_map_publisher_->publish(processed_map);
  }
}

void PlanningEngine::feed_worldmodel_objects(
    const std::shared_ptr<maf_worldmodel::ObjectsInterface> objects_interface) {
  if (nullptr != worldmodel_objects_publisher_) {
    worldmodel_objects_publisher_->publish(objects_interface);
  }
}

void PlanningEngine::feed_world_model_parking_slots(
    const std::shared_ptr<maf_worldmodel::FusionAPA> parking_slots) {
  if (nullptr != worldmodel_parking_slots_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PARKING) {
    worldmodel_parking_slots_publisher_->publish(parking_slots);
  }
}

void PlanningEngine::feed_world_model_scene_objects(
    const std::shared_ptr<maf_worldmodel::SceneObjects> scene_objects) {
  if (nullptr != worldmodel_scene_objects_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PARKING) {
    worldmodel_scene_objects_publisher_->publish(scene_objects);
  }
}
void PlanningEngine::feed_fusion_objects(
    const std::shared_ptr<
        maf_perception_interface::PerceptionFusionObjectResult>
        scene_objects) {
  if (nullptr != fusion_objects_publisher_) {
    fusion_objects_publisher_->publish(scene_objects);
  }
}
void PlanningEngine::feed_fusion_grond_lines(
    const std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
        ground_lines) {
  if (nullptr != fusion_groundlines_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PARKING) {
    fusion_groundlines_publisher_->publish(ground_lines);
  }
}

void PlanningEngine::feed_fusion_uss_grond_lines(
    const std::shared_ptr<maf_perception_interface::FusionGroundLineResult>
        ground_lines) {
  if (nullptr != fusion_uss_groundlines_publisher_) {
    fusion_uss_groundlines_publisher_->publish(ground_lines);
  }
}

void PlanningEngine::feed_fusion_uss(
    const std::shared_ptr<maf_sensor_interface::UltrasonicUpaReport> uss) {
  if (nullptr != fusion_uss_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PARKING) {
    fusion_uss_publisher_->publish(uss);
  }
}

void PlanningEngine::feed_traffic_light(
    const std::shared_ptr<maf_perception_interface::TrafficLightPerception>
        traffic_light_perception) {
  if (nullptr != traffic_light_publisher_) {
    traffic_light_publisher_->publish(traffic_light_perception);
  }
}

void PlanningEngine::feed_ego_pose(
    const std::shared_ptr<maf_mla_localization::MLALocalization> ego_pose) {
  if (nullptr != ego_pose_publisher_) {
    ego_pose_publisher_->publish(ego_pose);
  }
}

void PlanningEngine::feed_prediction_info(
    const std::shared_ptr<maf_worldmodel::PredictionResult>
        prediction_results) {
  if (nullptr != prediction_info_publisher_) {
    prediction_info_publisher_->publish(prediction_results);
  }
}

// void PlanningEngine::feed_control_command(
//       const std::shared_ptr<maf_endpoint::ControlCommand>
//       msd_control_command) {
//   if (nullptr != control_command_publisher_) {
//     control_command_publisher_->publish(msd_control_command);
//   }
// }

void PlanningEngine::feed_mpc_trajectory(
    const std::shared_ptr<maf_planning::MpcTrajectoryResult> mpc_trajectory) {
  if (nullptr != mpc_trajectory_publisher_) {
    mpc_trajectory_publisher_->publish(mpc_trajectory);
  }
}

void PlanningEngine::feed_tasknap_slice(uint8_t *ptr, uint64_t size) {
  if (taskagent_ptr_ == nullptr) {
    MSD_LOG(WARN, "feed tasknap fail for taskagent_ptr_ is nullptr!");
    return;
  }
  taskagent_ptr_->FeedbackTasknap(ptr, size);
}

void PlanningEngine::feed_planning_control_cmd_request(
    const maf_system_manager::ModuleControlCmdRequest &request) {
  running_mode_ = request.running_mode.value;
  if (nullptr != planning_control_cmd_request_publisher_) {
    planning_control_cmd_request_publisher_->publish(request);
  }
}

void PlanningEngine::feed_planning_request(
    const maf_system_manager::SysPlanningRequest &request) {
  if (nullptr != planning_request_publisher_) {
    planning_request_publisher_->publish(request);
  }
}

void PlanningEngine::feed_planning_reset_request(
    const maf_std::Header &request) {
  if (nullptr != planning_reset_request_publisher_ &&
      running_mode_ != maf_system_manager::RunningModeEnum::PARKING) {
    planning_reset_request_publisher_->publish(request);
  }

  try {
    mjson::Reader reset_json(request.frame_id);
    std::string reset = reset_json.get<std::string>("reset", false, "False");

    if (reset == "True") {
      parking_planning_task_->reset();
      pec_task_->reset();
    }
  } catch (mjson::Exception &e) {
  }
}

void PlanningEngine::feed_sbp_result(
    const maf_planning::SBPResult &sbp_result) {
  if (nullptr != sbp_result_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PARKING) {
    sbp_result_publisher_->publish(sbp_result);
  }
}

void PlanningEngine::feed_perception_vision_lane(
    const std::shared_ptr<maf_perception_interface::RoadLinePerception>
        perception_vision_lane) {
  if (nullptr != perception_vision_lane_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PILOT) {
    // cal center line index
    auto &lanes = perception_vision_lane->lane_perception.lanes;
    for (size_t i = 0; i < lanes.size(); i++) {
      auto &lane = lanes[i];
      if (lane.camera_source.value !=
              maf_perception_interface::CameraSourceEnum::
                  CAMERA_SOURCE_FRONT_MID ||
          lane.is_failed_3d) {
        continue;
      }

      if (lane.is_centerline) { // center_line
        lane.index = -100;
      }
    }

    std::vector<msd_worldmodel::worldmodel_v1::InterCenterLine> center_lines{};
    msd_worldmodel::worldmodel_v1::IntersectionPoint intersection_point;
    if (msd_worldmodel::worldmodel_v1::
            update_center_lane_and_find_current_index(
                perception_vision_lane->lane_perception, center_lines,
                intersection_point)) {
      for (int i = 0; i < center_lines.size(); i++) {
        lanes[center_lines[i].center_line_index].index =
            center_lines[i].relative_id;
      }
    }

    perception_vision_lane_publisher_->publish(perception_vision_lane);
  }
}

void PlanningEngine::feed_perception_lidar_road_edge(
    const std::shared_ptr<maf_perception_interface::RoadLinePerception>
        perception_lidar_road_edge) {
  if (nullptr != perception_lidar_road_edge_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PILOT) {
    perception_lidar_road_edge_publisher_->publish(perception_lidar_road_edge);
  }
}

void PlanningEngine::feed_perception_radar(
    const std::shared_ptr<maf_perception_interface::RadarPerceptionResult>
        perception_radar) {
  if (nullptr != perception_radar_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PILOT) {
    perception_radar_publisher_->publish(perception_radar);
  }
}

void PlanningEngine::feed_mff_info(
    const std::shared_ptr<maf_std::Header> mff_info) {
  if (nullptr != mff_info_publisher_ &&
      running_mode_ == maf_system_manager::RunningModeEnum::PILOT) {
    mff_info_publisher_->publish(mff_info);
  }
}

void PlanningEngine::set_callback(MSDPlanningOutputCallback callback) {
  parking_planning_task_->set_callback(callback);
}

void PlanningEngine::set_callback(MSDPlanningTriggerCallback callback) {
  parking_planning_task_->set_callback(callback);
}

// void PlanningEngine::set_callback(MSDPlanningModuleStatusCallback callback) {
//   monitor_->set_callback(callback);
// }

void PlanningEngine::set_callback(MSDPlanningNodeStatusCallback callback) {
  // monitor_->set_callback(callback);
  set_node_status_callback(callback);
}

// upload msd test callback
void PlanningEngine::set_callback(MSDScenarioRecognizeCallback callback) {
  // FIXME(hanzhiyuan): don't set callback to upload msg to mst-test,
  // in future, this will modify to upload data to sil platform
  // upload_msd_test_task_->set_callback(callback);
}

void PlanningEngine::set_callback(MSDPlanningTasknapCallback callback) {
  if (taskagent_ptr_ == nullptr) {
    MSD_LOG(WARN, "set tasknap callback failed for taskagent_ptr_ is null!");
    return;
  }

  taskagent_ptr_->set_snap_pub_callback(callback);
}

void PlanningEngine::set_callback(SBPRequestCallback callback) {
  parking_planning_task_->set_callback(callback);
}

void PlanningEngine::set_callback(MSDPlanningInfoCallback callback) {
  parking_planning_task_->set_callback(callback);
}

void PlanningEngine::set_callback(MSDMdebugCallback callback) {
}

// for wm
void PlanningEngine::feed_perception_fusion_result(
    PerceptionFusionObjectResultPtr perception_fusion_result) {
  if (perception_fusion_result_publisher_ != nullptr) {
    perception_fusion_result_publisher_->publish(perception_fusion_result);
  }
}

void PlanningEngine::feed_mode_switch_request(
    const maf_system_manager::SysWorldModelRequest &request) {}

void PlanningEngine::feed_system_reset_request(
    std::shared_ptr<maf_std::Header> system_reset_cmd) {}

void PlanningEngine::feed_system_control_request(
    std::shared_ptr<maf_system_manager::ModuleControlCmdRequest> &msg) {}

void PlanningEngine::set_processed_map_callback(MSDMapCallback callback) {
  if (wm_pub_task_ != nullptr) {
    wm_pub_task_->set_callback(callback);
  }
}

void PlanningEngine::set_wm_info_callback(WorldmodelInfoCallback callback) {}

void PlanningEngine::set_matched_info_callback(
    MatchedEgoPositionCallback callback) {}

void PlanningEngine::set_objects_interface_callback(
    MSDObjectsCallback callback) {
  if (wm_pub_task_ != nullptr) {
    wm_pub_task_->set_callback(callback);
  }
}

// parking worldmodel
void PlanningEngine::feed_fusion_parking_slot(
    const std::shared_ptr<maf_perception_interface::FusionParkingSlotResult>
        fusion_parking_slots) {
  if (nullptr != fusion_parking_slot_publisher_) {
    fusion_parking_slot_publisher_->publish(fusion_parking_slots);
  }
  auto &task_status =
      parking::PlanningContext::Instance()->planning_status().task_status;
  auto &id = parking::PlanningContext::Instance()
                 ->parking_behavior_planner_output()
                 .parking_slot_info.id;
  switch (task_status.task) {
  case parking::StatusType::APA:
    (void)pec_task_->setAPAStatus("apa_parking_in", id);
    break;
  case parking::StatusType::APOA:
    (void)pec_task_->setAPAStatus("apa_parking_out", id);
    break;
  case parking::StatusType::SEARCH:
  case parking::StatusType::AVP:
  case parking::StatusType::WAIT:
  default:
    (void)pec_task_->setAPAStatus("apa_wait", 0);
    break;
  }
}

void PlanningEngine::set_wm_parking_slot_callback(
    WorldModelParkingSlotCallback callback) {
  if (pec_task_) {
    pec_task_->setCallback(callback);
  } else {
    MSD_LOG(INFO, "************ pec task create failed!");
  }
}

std::shared_ptr<mtaskflow::TasknapAgent> PlanningEngine::get_tasknap_agent() {
  return taskagent_ptr_;
}

void PlanningEngine::set_simulation_sync_callback(
    NPPHeaderLoggerCallback callback) {
  if (parking_planning_task_ != nullptr) {
    parking_planning_task_->set_sync_callback(callback);
  }
}
} // namespace msquare
