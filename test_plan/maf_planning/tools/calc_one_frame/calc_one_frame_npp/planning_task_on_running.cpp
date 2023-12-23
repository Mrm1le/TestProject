#include "planning_task_on_running.h"
#include "mfr/mfr.h"
#include "mjson/mjson.hpp"

#include "maf_interface/maf_framework_status.h"
#include <maf_interface/maf_endpoint.h>
#include <maf_interface/maf_gps_imu.h>
#include <maf_interface/maf_perception_interface.h>
#include <maf_interface/maf_system_manager.h>

#include <chrono>
#include <thread>

// clang-format off
#include "common/utils/polyfit.h"
#include "planning/common/circle_object_buffer.h"
#include "common/planning_context.h"
#include "common/planning_task.hpp"
#include "common/apa_workflow/parking_planning_task.hpp"
// clang-format on 
#include "common/version.h"

#include "common.h"
#include "mtime_core/mtime.h"
#include "pnc/planning_engine_interface.h"
#include "time.h"

#include "common/planning_engine.h"

namespace msquare {

std::string read_file(const std::string &file_path) {
  std::ifstream t(file_path);
  std::stringstream buffer;
  buffer << t.rdbuf();
  return buffer.str();
}

class PlanningMTimeController {
public:
  PlanningMTimeController(const PlanningMTimeController &) = delete;
  void opterator(const PlanningMTimeController &) = delete;

  static void init() { set_current_time(0); }

  static void set_current_time(int64_t time_ns) {
    get_instance()->current_time_ns = time_ns;
  }

private:
  PlanningMTimeController() {
    auto planning_clock_callback = [this](const char *) -> mtime::MTimeClock {
      return mtime::MTimeClock{.nanoseconds =
                                   static_cast<uint64_t>(current_time_ns)};
    };

    mtime::MTimeConfig mtime_config;
    mtime_config.timeline_type = mtime::MTimelineType::MTIMELINE_TYPE_CALLBACK;
    mtime_config.clock_feeder = nullptr;
    mtime_config.clock_callback = planning_clock_callback;
    mtime_config.node_handle = nullptr;
    MTIME()->init(mtime_config);
  }
  static PlanningMTimeController *get_instance() {
    static PlanningMTimeController instance{};
    return &instance;
  }

  int64_t current_time_ns{};
};

void set_planning_context(const MSDPlanningConfig &planning_config) {
  PlanningContext::Instance()->set_config_file_dir(
      planning_config.config_file_dir);
  parking::PlanningContext::Instance()->set_config_file_dir(
      planning_config.config_file_dir);

  std::stringstream ss;
  ss << PLANNING_PLATFORM << "#" << PLANNING_BRANCH << "#" << PLANNING_COMMIT;
  const std::string version_string = ss.str();

  PlanningContext::Instance()->set_version(version_string);
  parking::PlanningContext::Instance()->set_version(version_string);
}

#define INIT_RESOURCE_AND_PUBLISHER(name)                                      \
  name##_resource_{                                                            \
      std::make_shared<decltype(name##_resource_)::element_type>()},           \
      name##_publisher_ {                                                      \
    name##_resource_->create_publisher()                                       \
  }

PlanningTaskOnRunning::PlanningTaskOnRunning(bool is_apa)
    : INIT_RESOURCE_AND_PUBLISHER(tick),
      INIT_RESOURCE_AND_PUBLISHER(module_status),
      INIT_RESOURCE_AND_PUBLISHER(chassis_report),
      INIT_RESOURCE_AND_PUBLISHER(wheel_report),
      INIT_RESOURCE_AND_PUBLISHER(body_report),
      INIT_RESOURCE_AND_PUBLISHER(imu_report),
      INIT_RESOURCE_AND_PUBLISHER(worldmodel_map),
      INIT_RESOURCE_AND_PUBLISHER(worldmodel_objects),
      INIT_RESOURCE_AND_PUBLISHER(worldmodel_parking_slots),
      INIT_RESOURCE_AND_PUBLISHER(worldmodel_scene_objects),
      INIT_RESOURCE_AND_PUBLISHER(fusion_objects),
      INIT_RESOURCE_AND_PUBLISHER(fusion_ground_lines),
      INIT_RESOURCE_AND_PUBLISHER(fusion_uss_ground_lines),
      INIT_RESOURCE_AND_PUBLISHER(fusion_uss),
      INIT_RESOURCE_AND_PUBLISHER(traffic_light),
      INIT_RESOURCE_AND_PUBLISHER(ego_pose),
      INIT_RESOURCE_AND_PUBLISHER(prediction_info),
      INIT_RESOURCE_AND_PUBLISHER(mpc_trajectory),
      INIT_RESOURCE_AND_PUBLISHER(planning_control_cmd_request),
      INIT_RESOURCE_AND_PUBLISHER(planning_request),
      INIT_RESOURCE_AND_PUBLISHER(planning_reset_request),
      INIT_RESOURCE_AND_PUBLISHER(sequence),
      INIT_RESOURCE_AND_PUBLISHER(perception_vision_lane),
      INIT_RESOURCE_AND_PUBLISHER(perception_lidar_road_edge),
      INIT_RESOURCE_AND_PUBLISHER(perception_radar),
      INIT_RESOURCE_AND_PUBLISHER(mff_info),
      INIT_RESOURCE_AND_PUBLISHER(perception_fusion_object),
      INIT_RESOURCE_AND_PUBLISHER(fusion_parking_slot),
      INIT_RESOURCE_AND_PUBLISHER(sbp_problem),
      INIT_RESOURCE_AND_PUBLISHER(sbp_result),
      INIT_RESOURCE_AND_PUBLISHER(sbp_debug),
      INIT_RESOURCE_AND_PUBLISHER(wireless_charger_report),
      is_apa_(is_apa)
#undef INIT_RESOURCE_AND_PUBLISHER
{
  LOG(INFO) << "PlanningTaskOnRunning()";
  set_planning_config();
  set_planning_context(planning_config_);

  worldmodel_map_receiver_ = worldmodel_map_resource_->create_receiver();
  worldmodel_objects_receiver_ =
      worldmodel_objects_resource_->create_receiver();
  worldmodel_parking_slot_info_receiver_ = worldmodel_parking_slots_resource_->create_receiver();

  PlanningMTimeController::init();
}

PlanningTaskOnRunning::~PlanningTaskOnRunning() {
  LOG(INFO) << "~PlanningTaskOnRunning";
}

template <typename T> decltype(auto) to_loggable(T &&v) { return v; }

std::string to_string(SceneType v) {
  switch (v) {
  case SceneType::URBAN:
    return std::string{"URBAN"};
  case SceneType::PARKING_SVP:
    return std::string{"PARKING_SVP"};
  case SceneType::HIGHWAY:
    return std::string{"HIGHWAY"};
  case SceneType::PARKING_LVP:
    return std::string{"PARKING_LVP"};
  case SceneType::PARKING_APA:
    return std::string{"PARKING_APA"};
  case SceneType::REMOTE_CONTROL:
    return std::string{"REMOTE_CONTROL"};
  case SceneType::NOT_DEFINED:
    return std::string{"NOT_DEFINED"};
  }
  return std::string{"ERROR"};
}

std::string to_loggable(SceneType v) { return to_string(v); }

void log_planning_config(const MSDPlanningConfig &planning_config) {
#define LOG_OBJECT_FIELD(object_name, field_name)                              \
  do {                                                                         \
    LOG(INFO) << #object_name << "." << #field_name << ": "                    \
              << to_loggable(object_name.field_name);                          \
  } while (0)
  LOG_OBJECT_FIELD(planning_config, scene_type);
  LOG_OBJECT_FIELD(planning_config, enable_timer_tick);
  LOG_OBJECT_FIELD(planning_config, config_file_dir);
  LOG_OBJECT_FIELD(planning_config, underclocking_enum);
  LOG_OBJECT_FIELD(planning_config, enable_ilc);
  LOG_OBJECT_FIELD(planning_config, enable_alc);
  LOG_OBJECT_FIELD(planning_config, ilc_limit_velocity);
  LOG_OBJECT_FIELD(planning_config, ilc_limit_time);
  LOG_OBJECT_FIELD(planning_config, cruise_velocity);
  LOG_OBJECT_FIELD(planning_config, is_acc_mode);
  LOG_OBJECT_FIELD(planning_config, is_ddmap);
  LOG_OBJECT_FIELD(planning_config, driving_style);
  LOG_OBJECT_FIELD(planning_config, mtaskflow_config_file);
  LOG_OBJECT_FIELD(planning_config, init_wm);
  LOG_OBJECT_FIELD(planning_config, wm_config_file);
  LOG_OBJECT_FIELD(planning_config, wm_pec_config_file);
#undef LOG_PLANNING_PLANNING_CONFIG_FIELD
}

void PlanningTaskOnRunning::set_planning_config() {
  const std::string config_file =
      get_planning_config_dir() + "msquare_planning.json";
  const std::string mtaskflow_config_file =
      get_planning_config_dir() + "mtaskflow_config.json";

  const auto config_reader = mjson::Reader(read_file(config_file));

  if(is_apa_){
    planning_config_.scene_type = SceneType::PARKING_APA;
  }else{
    planning_config_.scene_type = SceneType::URBAN;
  }
  planning_config_.enable_timer_tick = true;
  planning_config_.config_file_dir = get_planning_config_dir();
  planning_config_.underclocking_enum =
      config_reader.get<uint32_t>("underclocking_enum", false, 10);
  planning_config_.enable_ilc =
      config_reader.get<bool>("enable_ilc", false, true);
  planning_config_.enable_alc =
      config_reader.get<bool>("enable_alc", false, false);
  planning_config_.ilc_limit_velocity =
      config_reader.get<double>("interactive_lc_limit_velocity", false, 0.0);
  planning_config_.ilc_limit_time =
      config_reader.get<double>("interactive_lc_limit_time", false, 10.0);
  planning_config_.cruise_velocity =
      config_reader.get<double>("default_cruise_velocity", false, 120.0);
  planning_config_.is_acc_mode =
      config_reader.get<bool>("is_acc_mode", false, false);
  planning_config_.is_ddmap = config_reader.get<bool>("is_ddmap", false, false);
  planning_config_.driving_style =
      config_reader.get<int>("driving_style", false, 1);
  planning_config_.mtaskflow_config_file =
      mtaskflow_config_file; // this is a file path
  planning_config_.init_wm = true;
  const std::string wm_config_file =
      get_planning_config_dir() + "worldmodel_mfr.v1.json";
  planning_config_.wm_config_file = read_file(
      wm_config_file); // 这个叫做wm_config_file的成员变量其实是文件内容字符串
  const std::string wm_pec_config_file =
      get_planning_config_dir() + "worldmodel_pec.json";
  planning_config_.wm_pec_config_file = read_file(
      wm_pec_config_file);

  log_planning_config(planning_config_);
}

void PlanningTaskOnRunning::feed_scenario(const bool is_parking) {
  maf_system_manager::ModuleControlCmdRequest request{};
  request.header.stamp = MTIME()->timestamp().ns();
  request.module_control_cmd.value =
      maf_system_manager::ModuleControlCmdEnum::RESUME;
  request.running_mode.value =
      is_parking ? maf_system_manager::RunningModeEnum::PARKING
                 : maf_system_manager::RunningModeEnum::PILOT;
  planning_control_cmd_request_publisher_->publish(request);
}

void PlanningTaskOnRunning::set_current_frame_planning_start_time(
    int64_t start_time_ns) const {
  LOG(INFO) << "current_frame_planning_start_time: " << start_time_ns << "ns";
  PlanningMTimeController::set_current_time(start_time_ns);
}

void PlanningTaskOnRunning::init_planning_task() {
  LOG(INFO) << "init planning task";
  status_manager_ =
      std::make_shared<maf::StatusManager>();

  if(!is_apa_){
    MSD_LOG(INFO, "FPP: PlanningTaskOnRunning: init create PlanningTask ");
    // planning_task_ = std::make_shared<msquare::PlanningTask>(
    //     planning_config_, planning_config_.enable_timer_tick, status_manager_,
    //     tick_resource_->create_receiver(),
    //     module_status_resource_->create_receiver(),
    //     chassis_report_resource_->create_receiver(),
    //     wheel_report_resource_->create_receiver(),
    //     body_report_resource_->create_receiver(),
    //     imu_report_resource_->create_receiver(),
    //     worldmodel_map_resource_->create_receiver(),
    //     worldmodel_objects_resource_->create_receiver(),
    //     traffic_light_resource_->create_receiver(),
    //     ego_pose_resource_->create_receiver(),
    //     prediction_info_resource_->create_receiver(),
    //     mpc_trajectory_resource_->create_receiver(),
    //     planning_control_cmd_request_resource_->create_receiver(32, false),
    //     planning_request_resource_->create_receiver(),
    //     planning_reset_request_resource_->create_receiver(),
    //     perception_vision_lane_resource_->create_receiver(),
    //     perception_lidar_road_edge_resource_->create_receiver(),
    //     perception_radar_resource_->create_receiver(),
    //     mff_info_resource_->create_receiver(),
    //     perception_fusion_object_resource_->create_receiver()
    // );
    // planning_task_->on_init();
  } else {
 
    worldmodel_pec::SlotReleaseContext::Instance()->set_config_file_dir(
      get_planning_resource_dir());
    parking_planning_task_ = std::make_shared<msquare::parking::PlanningTask>(
        planning_config_, planning_config_.enable_timer_tick, status_manager_,
        tick_resource_->create_receiver(),
        module_status_resource_->create_receiver(),
        chassis_report_resource_->create_receiver(),
        wheel_report_resource_->create_receiver(),
        body_report_resource_->create_receiver(),
        imu_report_resource_->create_receiver(),
        worldmodel_map_resource_->create_receiver(),
        worldmodel_objects_resource_->create_receiver(),
        worldmodel_parking_slots_resource_->create_receiver(),
        worldmodel_scene_objects_resource_->create_receiver(),
        fusion_objects_resource_->create_receiver(),
        fusion_ground_lines_resource_->create_receiver(),
        fusion_uss_ground_lines_resource_->create_receiver(),
        fusion_uss_resource_->create_receiver(),
        traffic_light_resource_->create_receiver(),
        ego_pose_resource_->create_receiver(),
        prediction_info_resource_->create_receiver(),
        mpc_trajectory_resource_->create_receiver(),
        planning_control_cmd_request_resource_->create_receiver(32, false),
        planning_request_resource_->create_receiver(),
        sbp_problem_resource_->create_publisher(),
        sbp_result_resource_->create_receiver(),
        sbp_debug_resource_->create_receiver(),
        wireless_charger_report_resource_->create_receiver());
    parking_planning_task_->on_init();

    sbp_planning_task_ = std::make_shared<msquare::parking::SBPlanningTask>(
        sbp_problem_resource_->create_receiver(),
        sbp_result_resource_->create_publisher(),
        sbp_debug_resource_->create_publisher());
    sbp_planning_task_->on_init();

    pec_task_ = std::make_shared<::parking::SetPSDFusionTask>(
        fusion_parking_slot_resource_->create_receiver(),
        ego_pose_resource_->create_receiver(),
        fusion_uss_ground_lines_resource_->create_receiver(),
        fusion_objects_resource_->create_receiver(),
        planning_control_cmd_request_resource_->create_receiver(32, false),
        wireless_charger_report_resource_->create_receiver(),
        planning_request_resource_->create_receiver(),
        planning_config_.wm_pec_config_file);
    pec_task_->on_init();

    auto pec_cb =
        [this](const std::shared_ptr<maf_worldmodel::FusionAPA> &maf_msg) {
          worldmodel_parking_slots_publisher_->publish(maf_msg);
        };
    pec_task_->setCallback(pec_cb);
  }
}

void PlanningTaskOnRunning::init_ddmap_generator_task() {
  LOG(INFO) << "init ddmap generator task";
  mtaskflow::TaskConfig task_config{};
  task_config.config_key = "DdmapGeneratorTask";

  // ddmap_generator_task_ =
  //     msd_worldmodel::worldmodel_v1::DdmapGeneratorTask::make(
  //         perception_vision_lane_resource_->create_receiver(),
  //         ego_pose_resource_->create_receiver(),
  //         worldmodel_map_publisher_,
  //         planning_control_cmd_request_resource_->create_receiver(32, false));
  // ddmap_generator_task_->on_init();
}

void PlanningTaskOnRunning::init_fusion_object_task() {
  LOG(INFO) << "init fusion object task";
  bool enable_fusion_filter = true;
  bool enable_fusion_cone_filter = true;
  bool enable_force_filter = true;
  double force_filter_x_distance = 200.0;
  bool force_ddmap = true;

  mtaskflow::TaskConfig task_config{};
  task_config.config_key = "FusionTask";
  fusion_object_task_ = msd_worldmodel::worldmodel_v1::FusionTask::make(
      fusion_objects_resource_->create_receiver(),
      ego_pose_resource_->create_receiver(),
      worldmodel_map_resource_->create_receiver(),
      planning_control_cmd_request_resource_->create_receiver(32, false),
      worldmodel_objects_publisher_, enable_fusion_filter,
      enable_fusion_cone_filter, enable_force_filter, force_filter_x_distance,
      force_ddmap);
  fusion_object_task_->on_init();
}

void PlanningTaskOnRunning::init() {
  LOG(INFO) << "init";
  init_planning_task();

  if (planning_config_.init_wm) {
    if(!is_apa_){
      init_ddmap_generator_task();
    }
    init_fusion_object_task();
  }
}

} // namespace msquare
