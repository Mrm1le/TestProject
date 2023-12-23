#include "mfr/mfr.h"
#include "nlohmann/json.hpp"

#include "argparse.hpp"
#include "cp_pnc.h"
#include "mjson/mjson.hpp"
#include "mlog_msg_id/mlog_msg_id.hpp"
#include "mlog_publisher/publisher.h"
#include "mtime_core/mtime.h"
#include "pnc.h"
#include "timeline/timeline_mfr.h"

#include "endpoint_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "framework_status_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "gps_imu_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "mla_localization_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "perception_interface_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "planning_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "sensor_interface_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "std_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "worldmodel_mfrmsgs/mfr_cpp_struct_convert.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <map>
#include <tasknap/tasknap.hpp>
#include <thread>
#define LOG_PREFIX "Planning"
#include "log_util.hpp"

#if defined(FEAT_SWITCH_UTIL)
#include "switch_util.h"
#include <exception>
#endif // FEAT_SWITCH_UTIL

#if defined(FEAT_SHMCOM_MESSAGE)
#include "shmcom/shmcom_helper.hpp"
#endif // FEAT_SHMCOM_MESSAGE

#define MSG_MODE_MFR_SOCKET 0x01
#define MSG_MODE_INPCOM 0x02
#define MSG_MODE_PE_SHM 0x04
#define MSG_MODE_MFR_SHM 0x08

#define HAS_MSG_MODE(mask, mode) ((mask & (mode)) != 0)

using namespace mfr;
using namespace mmemory;

namespace {
mmemory::MFString read_file(const mmemory::MFString &path) {
  FILE *file = fopen(path.c_str(), "r");
  assert(file != nullptr);
  std::shared_ptr<FILE> fp(file, [](FILE *file) { fclose(file); });
  (void)fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  (void)fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  assert(read_bytes == content.size());
  (void)read_bytes;
  return mmemory::MFString(content.begin(), content.end());
}
} // namespace

static constexpr uint32_t kTimerFreqHz = 10;

class PlanningMFRNode : public MFRNode {
public:
  PlanningMFRNode() = default;

  bool on_init(MFRNodeHandle *node_handle) override {
    // get param
    auto param_reader = mjson::Reader(node_handle->node_param_json().c_str());

    // setup local log level
    SETUP_LOG_UTIL(mjson::Reader(
        param_reader.get<mjson::Json>("log_config", false, mjson::Json())));
    LOGI("on_init begin");
    const char *sim = std::getenv("RealitySimulation");
    if (sim != nullptr && std::strcmp(sim, "simulation") == 0) {
      is_in_simulation_ = true;
    } else {
      is_in_simulation_ = false;
    }
    if (std::getenv("USE_EGOPOSE_TIMESTAMP") != nullptr) {
      use_egopose_stamp_ = true;
    }
    MFString config_file = param_reader.get<std::string>("config_file").c_str();
    MFString mtaskflow_config_file =
        param_reader.get<std::string>("mtaskflow_config_file").c_str();
    MFString wm_config_file =
        param_reader.get<std::string>("wm_config_file").c_str();
    MFString wm_pec_config_file =
        param_reader.get<std::string>("wm_pec_config_file").c_str();
    MFString config_file_dir =
        param_reader.get<std::string>("config_file_dir").c_str();

    sub_imu_topic_ = param_reader.get<std::string>("sub_imu_topic").c_str();
    sub_mb_imu_topic_ =
        param_reader.get<std::string>("sub_mb_imu_topic").c_str();
    sub_prediction_topic_ =
        param_reader.get<std::string>("sub_prediction_topic").c_str();
    sub_worldmodel_map_topic_ =
        param_reader.get<std::string>("sub_worldmodel_map_topic").c_str();
    sub_worldmodel_parking_slots_topic_ =
        param_reader.get<std::string>("sub_worldmodel_parking_slots_topic")
            .c_str();
    sub_worldmodel_scene_object_topic_ =
        param_reader.get<std::string>("sub_worldmodel_scene_object_topic")
            .c_str();
    sub_worldmodel_objects_topic_ =
        param_reader.get<std::string>("sub_worldmodel_objects_topic").c_str();
    sub_traffic_light_topic_ =
        param_reader.get<std::string>("sub_traffic_light_topic").c_str();
    sub_fusion_objects_topic_ =
        param_reader.get<std::string>("sub_fusion_objects_topic").c_str();
    sub_fusion_groundlines_topic_ =
        param_reader.get<std::string>("sub_fusion_groundlines_topic").c_str();
    sub_fusion_uss_groundlines_topic_ =
        param_reader.get<std::string>("sub_fusion_uss_groundlines_topic")
            .c_str();
    sub_fusion_uss_topic_ =
        param_reader.get<std::string>("sub_fusion_uss_topic").c_str();
    sub_ego_pose_topic_ =
        param_reader.get<std::string>("sub_ego_pose_topic").c_str();
    sub_chassis_report_topic_ =
        param_reader.get<std::string>("sub_chassis_report_topic").c_str();
    sub_wireless_charger_report_topic_ =
        param_reader.get<std::string>("sub_wireless_charger_report_topic")
            .c_str();
    sub_wheel_report_topic_ =
        param_reader.get<std::string>("sub_wheel_report_topic").c_str();
    sub_body_report_topic_ =
        param_reader.get<std::string>("sub_body_report_topic").c_str();
    sub_mpc_trajecrory_topic_ =
        param_reader.get<std::string>("sub_mpc_trajecrory_topic").c_str();
    tasknap_slice_topic_ =
        param_reader.get<std::string>("tasknap_slice_topic").c_str();
    module_status_topic_ =
        param_reader.get<std::string>("module_status_topic").c_str();
    pub_nodes_status_topic_ =
        param_reader.get<std::string>("pub_nodes_status_topic").c_str();

    pub_planning_topic_ =
        param_reader.get<std::string>("pub_planning_topic").c_str();

    sub_sbp_result_topic_ =
        param_reader.get<std::string>("sub_sbp_result_topic").c_str();
    pub_sbp_request_topic_ =
        param_reader.get<std::string>("pub_sbp_request_topic").c_str();

    sub_planning_control_cmd_request_topic_ =
        param_reader.get<std::string>("sub_planning_control_cmd_request_topic")
            .c_str();
    pub_planning_control_cmd_response_topic_ =
        param_reader.get<std::string>("pub_planning_control_cmd_response_topic")
            .c_str();

    sub_planning_request_topic_ =
        param_reader.get<std::string>("sub_planning_request_topic").c_str();
    pub_planning_response_topic_ =
        param_reader.get<std::string>("pub_planning_response_topic").c_str();

    sub_planning_reset_request_topic_ =
        param_reader.get<std::string>("sub_planning_reset_request_topic")
            .c_str();
    pub_planning_reset_response_topic_ =
        param_reader.get<std::string>("pub_planning_reset_response_topic")
            .c_str();

    pub_planning_info_topic_ =
        param_reader.get<std::string>("pub_planning_info_topic").c_str();

    pub_mdebug_topic_ =
        param_reader.get<std::string>("pub_mdebug_topic").c_str();

    sub_perception_vision_lane_topic_ =
        param_reader.get<std::string>("sub_perception_vision_lane").c_str();

    sub_perception_vision_landmark_topic_ =
        param_reader.get<std::string>("sub_perception_vision_landmark").c_str();

    sub_perception_radar_topic_ =
        param_reader.get<std::string>("sub_perception_radar").c_str();

    sub_mff_info_topic_ = param_reader.get<std::string>("sub_mff_info").c_str();

    sub_planning_frequency_control_topic_ =
        param_reader.get<std::string>("sub_planning_frequency_control_topic")
            .c_str();

    sub_perception_lidar_road_edge_topic_ =
        param_reader.get<std::string>("sub_perception_lidar_road_edge_topic")
            .c_str();

    pub_planning_extra_data_topic_ =
        param_reader.get<std::string>("pub_planning_extra_data_topic").c_str();

    pub_planning_proto_debug_topic_ =
        param_reader.get<std::string>("pub_planning_proto_debug_topic").c_str();


    // for wm
    sub_perception_fusion_topic_ =
        param_reader.get<std::string>("sub_perception_fusion_topic").c_str();
    sub_mode_switch_cmd_topic_ =
        param_reader.get<std::string>("sub_mode_switch_cmd_topic").c_str();
    sub_system_control_topic_ =
        param_reader.get<std::string>("sub_system_control_topic").c_str();
    sub_system_reset_topic_ =
        param_reader.get<std::string>("sub_system_reset_topic").c_str();

    pub_map_topic_ = param_reader.get<std::string>("pub_map_topic").c_str();

    pub_object_topic_ =
        param_reader.get<std::string>("pub_object_topic").c_str();
    pub_wm_node_status_topic_ =
        param_reader.get<std::string>("pub_wm_node_status_topic").c_str();
    pub_mode_switch_cmd_topic_ =
        param_reader.get<std::string>("pub_mode_switch_cmd_topic").c_str();
    pub_system_control_response_topic_ =
        param_reader.get<std::string>("pub_system_control_response_topic")
            .c_str();
    pub_system_reset_response_topic_ =
        param_reader.get<std::string>("pub_system_reset_response_topic")
            .c_str();

    // parking worldmodel
    sub_fusion_parking_slot_topic_ =
        param_reader.get<std::string>("sub_fusion_parking_slot_topic").c_str();
    pub_worldmodel_parking_slot_topic_ =
        param_reader.get<std::string>("pub_worldmodel_parking_slot_topic")
            .c_str();

    sub_parking_wm_control_cmd_request_topic_ =
        param_reader
            .get<std::string>("sub_parking_wm_control_cmd_request_topic")
            .c_str();
    sub_parking_wm_request_topic_ =
        param_reader.get<std::string>("sub_parking_wm_request_topic").c_str();

    pub_parking_wm_control_cmd_response_topic_ =
        param_reader
            .get<std::string>("pub_parking_wm_control_cmd_response_topic")
            .c_str();
    pub_parking_wm_response_topic_ =
        param_reader.get<std::string>("pub_parking_wm_response_topic").c_str();
    pub_parking_wm_node_status_topic_ =
        param_reader.get<std::string>("pub_parking_wm_node_status_topic")
            .c_str();
    
    pub_node_status_apa_planning_topic_ =
        param_reader.get<std::string>("pub_node_status_apa_planning_topic")
            .c_str();

    // slp topics
    sub_planning_reload_request_topic_ =
        param_reader.get<std::string>("sub_planning_reload_request_topic")
            .c_str();
    pub_simulation_sync_topic_ =
        param_reader.get<std::string>("pub_simulation_sync_topic").c_str();
    pub_simulation_answer_topic_ =
        param_reader.get<std::string>("pub_simulation_answer_topic").c_str();
    sub_simulation_query_topic_ =
        param_reader.get<std::string>("sub_simulation_query_topic").c_str();

    scene_type_ = (msquare::SceneType)atoi(
        param_reader.get<std::string>("scene_type").c_str());
    LOGI("on_init load config data");

    std::string config_data = read_file(config_file).c_str();
    init_config_data(config_data);
    // init mlog and mtime
    mlog::PublisherSettings publisher_settings;
    publisher_settings.handle = node_handle;
    publisher_settings.config_file = config_file.c_str();
    auto publisher = mlog::MLogPublisher::make(publisher_settings);
    mlog::MLogEndpointCallback planning_callback = nullptr;
    if (publisher) {
      planning_callback =
          [publisher](const std::vector<mlog::MLogEntry> &entries) {
            publisher->publish(entries);
          };
    }

    msd_planning::MSDPlanning_set_log_config(config_data, node_handle);
    if (planning_callback) {
      msd_planning::MSDPlanning_set_log_callback(planning_callback);
    }
    msd_planning::MSDPlanning_init_log_manager();

    msd_planning::MSDPlanningTimeConfig planning_time_config{};
    planning_time_config.basic_config = config_data;
    mtime::MTimeConfig time_config{};
    time_config.node_handle = node_handle;
    planning_time_config.time_config = time_config;
    // init time
    msd_planning::MSDPlanning_set_time_config(planning_time_config, feeder_);

    // for wm
    cp_worldmodel::MSDWorldModelConfig worldmodel_config{};
    worldmodel_config.config_json = read_file(wm_config_file).c_str();

    mlog::PublisherSettings wm_publisher_settings;
    wm_publisher_settings.handle = node_handle;
    wm_publisher_settings.config_file = wm_config_file.c_str();
    auto mlog_publisher = mlog::MLogPublisher::make(wm_publisher_settings);

    mlog::MLogEndpointCallback worldmodel_mlog_callback =
        [mlog_publisher](const std::vector<mlog::MLogEntry> &entries) {
          mlog_publisher->publish(entries);
        };

    // init mlog
    bool success = false;
    success =
        msd_worldmodel::MSDWorldModel_set_log_config(wm_config_file.c_str());
    if (!success) {
      LOGE("ERROR: MSDWorldModel_set_log_config failed!");
    }
    msd_worldmodel::MSDWorldModel_set_log_callback(worldmodel_mlog_callback);
    msd_worldmodel::MSDWorldModel_init_log_manager();

    LOGI("on_init init engine");

    msquare::MSDPlanningConfig planning_config{};
    planning_config.scene_type = scene_type_;
    planning_config.enable_timer_tick = true;
    planning_config.underclocking_enum = underclocking_enum_;
    planning_config.enable_ilc = enable_ilc_;
    planning_config.enable_alc = enable_alc_;
    planning_config.ilc_limit_velocity = ilc_limit_velocity_;
    planning_config.ilc_limit_time = ilc_limit_time_;
    planning_config.cruise_velocity = default_cruise_velocity_;
    planning_config.is_acc_mode = is_acc_mode_;
    planning_config.is_ddmap = is_ddmap_;
    planning_config.driving_style = driving_style_;
    planning_config.config_file_dir = config_file_dir.c_str();
    planning_config.mtaskflow_config_file = mtaskflow_config_file.c_str();
    planning_config.wm_config_file = worldmodel_config.config_json.c_str();
    planning_config.wm_pec_config_file = read_file(wm_pec_config_file).c_str();
    planning_config.init_wm = init_wm_;
    planning_engine_ = msquare::PlanningEngineInterface::make(planning_config);

    (void)planning_engine_->init();

    if (start_machine_) {
      LOGI("on_init start engine");

      is_paused_.store(false);
      (void)planning_engine_->start();
      resume_subscribers();
      resume_publishers();
      // feed start cmd
      maf_system_manager::ModuleControlCmdRequest request{};
      request.header.stamp = MTIME()->timestamp().ns();
      request.module_control_cmd.value =
          maf_system_manager::ModuleControlCmdEnum::RESUME;
      running_mode_ == maf_system_manager::RunningModeEnum::PARKING;
      planning_engine_->feed_planning_control_cmd_request(request);
    }

    auto env = std::getenv("SENSOR_IMU");
    if (env != nullptr && std::strcmp(env, "MB") == 0) {
      use_mb_imu_ = true;
    } else {
      use_mb_imu_ = false;
    }

    setup_msg_modes();

    LOGI("on_init subscribe messages");

    // get subscriber
    auto &communication_manager = node_handle->communication_manager();
    {
      // MFRSubscriberConfig sub_config{};

      // imu
      if (HAS_MSG_MODE(sub_imu_mode_, MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_imu_topic_.c_str());
        subscribe_mfr_socket_imu(sub_imu_mode_, communication_manager);
      }
      if (HAS_MSG_MODE(sub_imu_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_imu_topic_.c_str());
        subscribe_shmcom_imu();
      }

      // mb imu
      if (HAS_MSG_MODE(sub_mb_imu_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_mb_imu_topic_.c_str());
        subscribe_mfr_socket_imu(sub_mb_imu_mode_, communication_manager);
      }

      // prediction
      if (HAS_MSG_MODE(sub_prediction_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_prediction_topic_.c_str());
        subscribe_mfr_socket_prediction(sub_prediction_mode_,
                                        communication_manager);
      }

      // worldmodel map
      if (HAS_MSG_MODE(sub_worldmodel_map_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_worldmodel_map_topic_.c_str());
        subscribe_mfr_socket_worldmodel_map(sub_worldmodel_map_mode_,
                                            communication_manager);
      }

      // worldmodel parking slots
      if (HAS_MSG_MODE(sub_worldmodel_parking_slots_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_worldmodel_parking_slots_topic_.c_str());
        subscribe_mfr_socket_worldmodel_parking_slots(
            sub_worldmodel_parking_slots_mode_, communication_manager);
      }

      // worldmodel scene object
      if (HAS_MSG_MODE(sub_worldmodel_scene_object_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_worldmodel_scene_object_topic_.c_str());
        subscribe_mfr_socket_worldmodel_scene_object(
            sub_worldmodel_scene_object_mode_, communication_manager);
      }

      // worldmodel objects
      if (HAS_MSG_MODE(sub_worldmodel_objects_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_worldmodel_objects_topic_.c_str());
        subscribe_mfr_socket_worldmodel_objects(sub_worldmodel_objects_mode_,
                                                communication_manager);
      }

      // traffic light
      if (HAS_MSG_MODE(sub_traffic_light_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_traffic_light_topic_.c_str());
        subscribe_mfr_socket_traffic_light(sub_traffic_light_mode_,
                                           communication_manager);
      }

      // fusion objects
      if (HAS_MSG_MODE(sub_fusion_objects_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_fusion_objects_topic_.c_str());
        subscribe_mfr_socket_fusion_objects(sub_fusion_objects_mode_,
                                            communication_manager);
      }

      // fusion groundlines
      if (HAS_MSG_MODE(sub_fusion_groundlines_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_fusion_groundlines_topic_.c_str());
        subscribe_mfr_socket_fusion_groundlines(sub_fusion_groundlines_mode_,
                                                communication_manager);
      }

      // uss groundlines
      if (HAS_MSG_MODE(sub_fusion_uss_groundlines_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_fusion_uss_groundlines_topic_.c_str());
        subscribe_mfr_socket_fusion_uss_groundlines(
            sub_fusion_uss_groundlines_mode_, communication_manager);
      }

      // fusion uss
      if (HAS_MSG_MODE(sub_fusion_uss_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_fusion_uss_topic_.c_str());
        subscribe_mfr_socket_fusion_uss(sub_fusion_uss_mode_,
                                        communication_manager);
      }

      // ego pose
      if (HAS_MSG_MODE(sub_ego_pose_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_ego_pose_topic_.c_str());
        subscribe_mfr_socket_ego_pose(sub_ego_pose_mode_,
                                      communication_manager);
      }
      if (HAS_MSG_MODE(sub_ego_pose_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_ego_pose_topic_.c_str());
        subscribe_shmcom_ego_pose();
      }

      // classis report
      if (HAS_MSG_MODE(sub_chassis_report_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_chassis_report_topic_.c_str());
        subscribe_mfr_socket_chassis_report(sub_chassis_report_mode_,
                                            communication_manager);
      }
      if (HAS_MSG_MODE(sub_chassis_report_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_chassis_report_topic_.c_str());
        subscribe_shmcom_chassis_report();
      }

      // wireless_charger_report
      if (HAS_MSG_MODE(sub_wireless_charger_report_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_wireless_charger_report_topic_.c_str());
        subscribe_mfr_socket_wireless_charger_report(
            sub_wireless_charger_report_mode_, communication_manager);
      }

      // wheel report
      if (HAS_MSG_MODE(sub_wheel_report_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_wheel_report_topic_.c_str());
        subscribe_mfr_socket_wheel_report(sub_wheel_report_mode_,
                                          communication_manager);
      }
      if (HAS_MSG_MODE(sub_wheel_report_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_wheel_report_topic_.c_str());
        subscribe_shmcom_wheel_report();
      }

      // body report
      if (HAS_MSG_MODE(sub_body_report_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_body_report_topic_.c_str());
        subscribe_mfr_socket_body_report(sub_body_report_mode_,
                                         communication_manager);
      }
      if (HAS_MSG_MODE(sub_body_report_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_body_report_topic_.c_str());
        subscribe_shmcom_body_report();
      }

      // module status
      if (HAS_MSG_MODE(sub_module_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", module_status_topic_.c_str());
        subscribe_mfr_socket_module_status(sub_module_status_mode_,
                                           communication_manager);
      }
      if (HAS_MSG_MODE(sub_module_status_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", module_status_topic_.c_str());
        subscribe_shmcom_module_status();
      }

      // tasknap slice for mfr snapshot
      subscribe_mfr_socket_tasknap_slice(communication_manager);

      // mpc trajectory
      if (HAS_MSG_MODE(sub_mpc_trajecrory_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_mpc_trajecrory_topic_.c_str());
        subscribe_mfr_socket_mpc_trajecrory(sub_mpc_trajecrory_mode_,
                                            communication_manager);
      }

      // sbp result
      if (HAS_MSG_MODE(sub_sbp_result_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_sbp_result_topic_.c_str());
        subscribe_mfr_socket_sbp_result(sub_sbp_result_mode_,
                                        communication_manager);
      }

      // planning control cmd request
      if (HAS_MSG_MODE(sub_planning_control_cmd_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s",
             sub_planning_control_cmd_request_topic_.c_str());
        subscribe_mfr_socket_planning_control_cmd_request(
            sub_planning_control_cmd_request_mode_, communication_manager);
      }

      // planning request
      if (HAS_MSG_MODE(sub_planning_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_planning_request_topic_.c_str());
        subscribe_mfr_socket_planning_request(sub_planning_request_mode_,
                                              communication_manager);
      }

      // planning reset request
      if (HAS_MSG_MODE(sub_planning_reset_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_planning_reset_request_topic_.c_str());
        subscribe_mfr_socket_planning_reset_request(
            sub_planning_reset_request_mode_, communication_manager);
      }

      // perception vision lane
      if (HAS_MSG_MODE(sub_perception_vision_lane_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_perception_vision_lane_topic_.c_str());
        subscribe_mfr_socket_perception_vision_lane(
            sub_perception_vision_lane_mode_, communication_manager);
      }

      // radar
      if (HAS_MSG_MODE(sub_perception_radar_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_perception_radar_topic_.c_str());
        subscribe_mfr_socket_perception_radar(sub_perception_radar_mode_,
                                              communication_manager);
      }
      if (HAS_MSG_MODE(sub_perception_radar_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_perception_radar_topic_.c_str());
        subscribe_shmcom_perception_radar();
      }

      // mff info
      if (HAS_MSG_MODE(sub_mff_info_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_mff_info_topic_.c_str());
        subscribe_mfr_socket_mff_info(sub_mff_info_mode_,
                                      communication_manager);
      }

      // mff info
      if (HAS_MSG_MODE(sub_planning_frequency_control_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_planning_frequency_control_topic_.c_str());
        subscribe_mfr_socket_planning_frequency_control(
            sub_planning_frequency_control_mode_, communication_manager);
      }

      // perception lidar road edge
      if (HAS_MSG_MODE(sub_perception_lidar_road_edge_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_perception_lidar_road_edge_topic_.c_str());
        subscribe_mfr_socket_perception_lidar_road_edge(
            sub_perception_lidar_road_edge_mode_, communication_manager);
      }

      // for wm
      MFRSubscriberConfig sub_config{};
      MFString wm_thread_name = "highway_wm";
      if (HAS_MSG_MODE(sub_perception_fusion_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_perception_fusion_topic_.c_str());
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            sub_perception_fusion_mode_ & MSG_MODE_MFR_SHM;
        sub_config.topic_name = sub_perception_fusion_topic_;
        sub_config.thread_tag = wm_thread_name;
        sub_config.pause_after_startup = !start_machine_;
        mfr_subscriber_map_[sub_perception_fusion_topic_] =
            communication_manager
                .subscribe<mmessage::perception_interface_mfrmsgs::
                               MFRMessagePerceptionFusionObjectResult>(
                    sub_config);
      }

      if (HAS_MSG_MODE(sub_system_control_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_system_control_topic_.c_str());
        sub_config.queue_size = 10;
        sub_config.use_shared_memory =
            sub_system_control_mode_ & MSG_MODE_MFR_SHM;
        sub_config.topic_name = sub_system_control_topic_;
        sub_config.thread_tag = wm_thread_name;
        sub_config.pause_after_startup = false;
        system_control_subscriber_ =
            communication_manager
                .subscribe<mmessage::system_manager_mfrmsgs::
                               MFRMessageModuleControlCmdRequest>(sub_config);
      }

      if (HAS_MSG_MODE(sub_system_reset_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_system_reset_topic_.c_str());
        sub_config.queue_size = 10;
        sub_config.use_shared_memory =
            sub_system_reset_mode_ & MSG_MODE_MFR_SHM;
        sub_config.topic_name = sub_system_reset_topic_;
        sub_config.thread_tag = wm_thread_name;
        sub_config.pause_after_startup = false;
        system_reset_subscriber_ =
            communication_manager
                .subscribe<mmessage::std_mfrmsgs::MFRMessageHeader>(sub_config);
      }

      if (HAS_MSG_MODE(sub_mode_switch_cmd_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_mode_switch_cmd_topic_.c_str());
        sub_config.queue_size = 10;
        sub_config.use_shared_memory =
            sub_mode_switch_cmd_mode_ & MSG_MODE_MFR_SHM;
        sub_config.topic_name = sub_mode_switch_cmd_topic_;
        sub_config.thread_tag = wm_thread_name;
        sub_config.pause_after_startup = false;
        mode_switch_subscriber_ = communication_manager.subscribe<
            mmessage::system_manager_mfrmsgs::MFRMessageSysWorldModelRequest>(
            sub_config);
      }

      // parking worldmodel
      MFString parking_wm_thread_name = "parking_wm";
      if (HAS_MSG_MODE(sub_fusion_parking_slot_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr socket %s", sub_fusion_parking_slot_topic_.c_str());
        sub_config.topic_name = sub_fusion_parking_slot_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            sub_fusion_parking_slot_mode_ & MSG_MODE_MFR_SHM;
        sub_config.thread_tag = parking_wm_thread_name;
        sub_config.pause_after_startup = !start_machine_;
        mfr_subscriber_map_[sub_fusion_parking_slot_topic_] =
            communication_manager
                .subscribe<mmessage::perception_interface_mfrmsgs::
                               MFRMessageFusionParkingSlotResult>(sub_config);
      }

      if (HAS_MSG_MODE(sub_planning_reload_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr socket %s",
             sub_planning_reload_request_topic_.c_str());
        subscribe_mfr_socket_planning_reload_request(
            sub_planning_reload_request_mode_, communication_manager);
      }

      if (HAS_MSG_MODE(sub_simulation_query_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr socket %s",
             sub_simulation_query_topic_.c_str());
        subscribe_mfr_socket_simulation_query(
            sub_simulation_query_mode_, communication_manager);
      }

      if (HAS_MSG_MODE(sub_parking_wm_control_cmd_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr socket %s",
             sub_parking_wm_control_cmd_request_topic_.c_str());
        sub_config.topic_name = sub_parking_wm_control_cmd_request_topic_;
        sub_config.queue_size = 10;
        sub_config.use_shared_memory =
            sub_parking_wm_control_cmd_request_mode_ & MSG_MODE_MFR_SHM;
        sub_config.thread_tag = parking_wm_thread_name;
        sub_config.pause_after_startup = false;
        mfr_subscriber_map_[sub_parking_wm_control_cmd_request_topic_] =
            communication_manager
                .subscribe<mmessage::system_manager_mfrmsgs::
                               MFRMessageModuleControlCmdRequest>(sub_config);
      }

      if (HAS_MSG_MODE(sub_parking_wm_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr socket %s", sub_parking_wm_request_topic_.c_str());
        sub_config.topic_name = sub_parking_wm_request_topic_;
        sub_config.queue_size = 10;
        sub_config.use_shared_memory =
            sub_parking_wm_request_mode_ & MSG_MODE_MFR_SHM;
        sub_config.thread_tag = parking_wm_thread_name;
        sub_config.pause_after_startup = false;
        mfr_subscriber_map_[sub_parking_wm_request_topic_] =
            communication_manager.subscribe<mmessage::system_manager_mfrmsgs::
                                                MFRMessageSysWorldModelRequest>(
                sub_config);
      }
    }

    LOGI("on_init advertise messages");

    // get publisher
    {
      MFRPublisherConfig pub_config{};
      pub_config.shared_memory_block_size = 3804 * 1024;
      // planning
      if (HAS_MSG_MODE(pub_planning_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_planning_topic_.c_str());
        publish_mfr_socket_planning(pub_planning_mode_, communication_manager);
      }
      if (HAS_MSG_MODE(pub_planning_mode_, MSG_MODE_PE_SHM)) {
        LOGI("advertise shmcom %s", pub_planning_topic_.c_str());
        advertise_shmcom_planning();
      }
      pub_config.shared_memory_block_size = 0;
      // pub_config.topic_name = module_status_topic_;
      // pub_config.queue_size = 1;
      // mfr_publisher_map_[module_status_topic_] =
      //     communication_manager.advertise<
      //         mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus>(
      //         pub_config);

      if (HAS_MSG_MODE(pub_nodes_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_nodes_status_topic_.c_str());
        publish_mfr_socket_nodes_status(pub_nodes_status_mode_,
                                        communication_manager);
      }

      if (HAS_MSG_MODE(pub_sbp_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_sbp_request_topic_.c_str());
        publish_mfr_socket_sbp_request_mode(pub_sbp_request_mode_,
                                            communication_manager);
      }

      if (HAS_MSG_MODE(pub_planning_control_cmd_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s",
             pub_planning_control_cmd_response_topic_.c_str());
        publish_mfr_planning_control_cmd_response(
            pub_planning_control_cmd_response_mode_, communication_manager);
      }

      if (HAS_MSG_MODE(pub_planning_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_planning_response_topic_.c_str());
        publish_mfr_planning_response(pub_planning_response_mode_,
                                      communication_manager);
      }

      // for mfr-snapshot realcar mode
      publish_mfr_tasknap_slice(communication_manager);

      if (HAS_MSG_MODE(pub_planning_reset_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_planning_reset_response_topic_.c_str());
        publish_mfr_planning_reset_response(pub_planning_reset_response_mode_,
                                            communication_manager);
      }

      if (HAS_MSG_MODE(pub_planning_info_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr socket %s", pub_planning_info_topic_.c_str());
        publish_mfr_socket_planning_info(pub_planning_info_mode_,
                                         communication_manager);
      }
      if (HAS_MSG_MODE(pub_planning_info_mode_, MSG_MODE_PE_SHM)) {
        LOGI("advertise shmcom %s", pub_planning_info_topic_.c_str());
        advertise_shmcom_planning_info();
      }

      if (HAS_MSG_MODE(pub_mdebug_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr socket %s", pub_mdebug_topic_.c_str());
        publish_mfr_socket_mdebug(pub_mdebug_mode_, communication_manager);
      }

      if (HAS_MSG_MODE(pub_simulation_sync_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr socket %s", pub_simulation_sync_topic_.c_str());
        pub_config.topic_name = pub_simulation_sync_topic_;
        pub_config.queue_size = 1;
        pub_config.use_shared_memory =
            pub_simulation_sync_mode_ & MSG_MODE_MFR_SHM;
        mfr_publisher_map_[pub_simulation_sync_topic_] =
            communication_manager
                .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
      }

      if (HAS_MSG_MODE(pub_simulation_answer_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr socket %s", pub_simulation_sync_topic_.c_str());
        pub_config.topic_name = pub_simulation_answer_topic_;
        pub_config.queue_size = 1;
        pub_config.use_shared_memory =
            pub_simulation_answer_mode_ & MSG_MODE_MFR_SHM;
        mfr_publisher_map_[pub_simulation_answer_topic_] =
            communication_manager
                .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
      }

      // for wm
      if (HAS_MSG_MODE(pub_map_mode_, MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_map_topic_.c_str());
        pub_config.queue_size = 10;
        pub_config.use_shared_memory = pub_map_mode_ & MSG_MODE_MFR_SHM;
        pub_config.topic_name = pub_map_topic_;
        pub_config.pause_after_startup = !start_machine_;
        mfr_publisher_map_[pub_map_topic_] = communication_manager.advertise<
            mmessage::worldmodel_mfrmsgs::MFRMessageProcessedMap>(pub_config);
      }

      if (HAS_MSG_MODE(pub_object_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_object_topic_.c_str());
        pub_config.queue_size = 10;
        pub_config.use_shared_memory = pub_object_mode_ & MSG_MODE_MFR_SHM;
        pub_config.topic_name = pub_object_topic_;
        pub_config.pause_after_startup = !start_machine_;
        mfr_publisher_map_[pub_object_topic_] = communication_manager.advertise<
            mmessage::worldmodel_mfrmsgs::MFRMessageObjectsInterface>(
            pub_config);
      }

      if (HAS_MSG_MODE(pub_wm_node_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_wm_node_status_topic_.c_str());
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_wm_node_status_mode_ & MSG_MODE_MFR_SHM;
        pub_config.topic_name = pub_wm_node_status_topic_;
        pub_config.pause_after_startup = false;
        wm_node_status_publisher_ = communication_manager.advertise<
            mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus>(
            pub_config);
      }

      if (HAS_MSG_MODE(pub_mode_switch_cmd_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_mode_switch_cmd_topic_.c_str());
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_mode_switch_cmd_mode_ & MSG_MODE_MFR_SHM;
        pub_config.topic_name = pub_mode_switch_cmd_topic_;
        pub_config.pause_after_startup = false;
        mode_switch_publisher_ = communication_manager.advertise<
            mmessage::system_manager_mfrmsgs::MFRMessageSysWorldModelResponse>(
            pub_config);
      }

      if (HAS_MSG_MODE(pub_system_control_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_system_control_response_topic_.c_str());
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_system_control_response_mode_ & MSG_MODE_MFR_SHM;
        pub_config.topic_name = pub_system_control_response_topic_;
        pub_config.pause_after_startup = false;
        system_control_publisher_ =
            communication_manager
                .advertise<mmessage::system_manager_mfrmsgs::
                               MFRMessageModuleControlCmdResponse>(pub_config);
      }

      if (HAS_MSG_MODE(pub_system_reset_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_system_reset_response_topic_.c_str());
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_system_reset_response_mode_ & MSG_MODE_MFR_SHM;
        pub_config.topic_name = pub_system_reset_response_topic_;
        pub_config.pause_after_startup = false;
        system_reset_publisher_ =
            communication_manager
                .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
      }

      // parking worldmodel
      if (HAS_MSG_MODE(pub_worldmodel_parking_slot_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_worldmodel_parking_slot_topic_.c_str());
        pub_config.topic_name = pub_worldmodel_parking_slot_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_worldmodel_parking_slot_mode_ & MSG_MODE_MFR_SHM;
        mfr_publisher_map_[pub_worldmodel_parking_slot_topic_] =
            communication_manager
                .advertise<mmessage::worldmodel_mfrmsgs::MFRMessageFusionAPA>(
                    pub_config);
      }

      if (HAS_MSG_MODE(pub_parking_wm_control_cmd_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s",
             pub_parking_wm_control_cmd_response_topic_.c_str());
        pub_config.topic_name = pub_parking_wm_control_cmd_response_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_parking_wm_control_cmd_response_mode_ & MSG_MODE_MFR_SHM;
        mfr_publisher_map_[pub_parking_wm_control_cmd_response_topic_] =
            communication_manager
                .advertise<mmessage::system_manager_mfrmsgs::
                               MFRMessageModuleControlCmdResponse>(pub_config);
      }

      if (HAS_MSG_MODE(pub_parking_wm_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_parking_wm_response_topic_.c_str());
        pub_config.topic_name = pub_parking_wm_response_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_parking_wm_response_mode_ & MSG_MODE_MFR_SHM;
        mfr_publisher_map_[pub_parking_wm_response_topic_] =
            communication_manager
                .advertise<mmessage::system_manager_mfrmsgs::
                               MFRMessageSysWorldModelResponse>(pub_config);
      }

      if (HAS_MSG_MODE(pub_parking_wm_node_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_parking_wm_node_status_topic_.c_str());
        pub_config.topic_name = pub_parking_wm_node_status_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_parking_wm_node_status_mode_ & MSG_MODE_MFR_SHM;
        mfr_publisher_map_[pub_parking_wm_node_status_topic_] =
            communication_manager.advertise<
                mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus>(
                pub_config);
      }

      if (HAS_MSG_MODE(pub_node_status_apa_planning_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_node_status_apa_planning_topic_.c_str());
        pub_config.topic_name = pub_node_status_apa_planning_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            pub_node_status_apa_planning_mode_ & MSG_MODE_MFR_SHM;
        mfr_publisher_map_[pub_node_status_apa_planning_topic_] =
            communication_manager.advertise<
                mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus>(
                pub_config);
      }
    }

    set_msquare_planning_engine_callback();

    if (internal_feed_) {
      (void)mfr_subscriber_map_[sub_worldmodel_map_topic_]->pause();
      (void)mfr_subscriber_map_[sub_worldmodel_objects_topic_]->pause();
      (void)mfr_subscriber_map_[sub_worldmodel_parking_slots_topic_]->pause();
    }

    (void)node_handle->trigger_manager().set_time_trigger(kTimerFreqHz);

    LOGI("on_init end");
    return true;
  }
  void on_finish() override {}
  void on_running(const MFRNodeRunningInfo &info) override {
    if (info.trigger == MFR_NODE_TRIGGER_MESSAGE) {
      if (info.topic_name == sub_prediction_topic_) {
        prediction_callback();
      } else if (info.topic_name == sub_worldmodel_map_topic_) {
        worldmodel_map_callback();
      } else if (info.topic_name == sub_imu_topic_) {
        imu_report_callback();
      } else if (info.topic_name == sub_mb_imu_topic_) {
        mb_imu_report_callback();
      } else if (info.topic_name == sub_worldmodel_objects_topic_) {
        worldmodel_object_callback();
      } else if (info.topic_name == sub_worldmodel_parking_slots_topic_) {
        worldmodel_parking_slots_callback();
      } else if (info.topic_name == sub_worldmodel_scene_object_topic_) {
        worldmodel_scene_objects_callback();
      } else if (info.topic_name == sub_traffic_light_topic_) {
        traffic_light_callback();
      } else if (info.topic_name == sub_fusion_objects_topic_) {
        fusion_object_callback();
      } else if (info.topic_name == sub_fusion_groundlines_topic_) {
        fusion_groundlines_callback();
      } else if (info.topic_name == sub_fusion_uss_groundlines_topic_) {
        fusion_uss_groundlines_callback();
      } else if (info.topic_name == sub_fusion_uss_topic_) {
        fusion_uss_callback();
      } else if (info.topic_name == sub_ego_pose_topic_) {
        ego_pose_callback();
      } else if (info.topic_name == sub_chassis_report_topic_) {
        chassis_report_callback();
      } else if (info.topic_name == sub_wireless_charger_report_topic_) {
        wireless_charger_report_callback();
      } else if (info.topic_name == sub_wheel_report_topic_) {
        wheel_report_callback();
      } else if (info.topic_name == sub_body_report_topic_) {
        body_report_callback();
      } else if (info.topic_name == sub_mpc_trajecrory_topic_) {
        mpc_trajectory_callback();
      } else if (info.topic_name == tasknap_slice_topic_) {
        tasknap_slice_callback();
      } else if (info.topic_name == module_status_topic_) {
        module_status_callback();
      } else if (info.topic_name == sub_sbp_result_topic_) {
        sbp_result_callback();
      } else if (info.topic_name == sub_planning_control_cmd_request_topic_) {
        planning_control_cmd_request_callback();
      } else if (info.topic_name == sub_planning_request_topic_) {
        planning_request_callback();
      } else if (info.topic_name == sub_planning_reset_request_topic_) {
        planning_reset_request_callback();
      } else if (info.topic_name == sub_perception_vision_lane_topic_) {
        perception_vision_lane_callback();
      } else if (info.topic_name == sub_perception_radar_topic_) {
        perception_radar_callback();
      } else if (info.topic_name == sub_mff_info_topic_) {
        mff_info_callback();
      } else if (info.topic_name == sub_planning_frequency_control_topic_) {
        planning_frequency_control_callback();
      } else if (info.topic_name == sub_perception_lidar_road_edge_topic_) {
        perception_lidar_road_edge_callback();
        // for wm
      } else if (info.topic_name == sub_perception_fusion_topic_) {
        perception_fusion_callback();
      } else if (info.topic_name == sub_planning_reload_request_topic_) {
        planning_reload_request_callback();
      } else if (info.topic_name == sub_simulation_query_topic_) {
        simulation_query_callback();
      } else if (info.topic_name == sub_system_control_topic_) {
        system_control_callback();
      } else if (info.topic_name == sub_system_reset_topic_) {
        system_reset_callback();
      } else if (info.topic_name == sub_mode_switch_cmd_topic_) {
        mode_switch_callback();
      } else if (info.topic_name == sub_fusion_parking_slot_topic_) {
        // parking worldmodel
        fusion_parking_slot_callback();
      } else if (info.topic_name ==
                  sub_parking_wm_control_cmd_request_topic_) {
        parking_wm_control_cmd_request_callback();
      } else if (info.topic_name == sub_parking_wm_request_topic_) {
        parking_wm_request_callback();
      }
    } else if (info.trigger == MFR_NODE_TRIGGER_TIME) {
      static int n_msg = 0;
      if ((++n_msg) % (kTimerFreqHz * 2) == 0) {
        float hz = 0;
        uint64_t delay = 0;
        for (auto item : mfr_publisher_map_) {
          (void)item.second->get_monitor_result(hz, delay);
          LOGD("monitor publish topic=%s hz=%.2f", item.first.c_str(), hz);
        }
        for (auto item : mfr_subscriber_map_) {
          (void)item.second->get_monitor_result(hz, delay);
          LOGD("monitor subscriber topic=%s hz=%.2f delay=%.2fms",
                item.first.c_str(), hz, delay / 1e6);
        }
      }
    }
  }

private:
  std::map<MFString, MFRSubscriber *> mfr_subscriber_map_;
  std::map<MFString, MFRPublisher *> mfr_publisher_map_;
  MFRSubscriber *module_control_cmd_request_subscriber_{};
  MFRPublisher *module_control_cmd_response_publisher_{};
  MFRPublisher *node_status_publisher_{};
  mtime::MTimeClockFeeder feeder_;

  uint8_t running_mode_ = maf_system_manager::RunningModeEnum::PILOT;
  std::shared_ptr<msquare::PlanningEngineInterface> planning_engine_;
  msquare::SceneType scene_type_ = msquare::SceneType::URBAN;
  uint32_t underclocking_enum_ = 10;
  bool enable_ilc_ = true;
  bool enable_alc_ = false;
  double ilc_limit_velocity_ = 0;
  double ilc_limit_time_ = 0;
  double default_cruise_velocity_ = 120;
  bool start_machine_ = false;
  bool is_acc_mode_ = false;
  bool is_ddmap_ = false;
  int driving_style_ = 1;
  std::atomic_bool is_paused_{true};
  double ego_vel_{0.0};
  bool is_in_simulation_ = false;
  bool use_egopose_stamp_ = false;
  bool use_mb_imu_ = false;
  uint32_t simulation_sync_seq_ = 0;

  MFString sub_union_prception_prediction_ego_pose_ =
      "prception_prediction_ego_pose";
  MFString sub_union_misc_ = "misc";
  MFString sub_request_ = "request";

  MFString sub_imu_topic_;
  MFString sub_mb_imu_topic_;
  MFString sub_prediction_topic_;
  MFString sub_worldmodel_map_topic_;
  MFString sub_worldmodel_objects_topic_;
  MFString sub_worldmodel_parking_slots_topic_;
  MFString sub_worldmodel_scene_object_topic_;
  MFString sub_traffic_light_topic_;
  MFString sub_fusion_objects_topic_;
  MFString sub_fusion_groundlines_topic_;
  MFString sub_fusion_uss_groundlines_topic_;
  MFString sub_fusion_uss_topic_;
  MFString sub_ego_pose_topic_;
  MFString sub_chassis_report_topic_;
  MFString sub_wireless_charger_report_topic_;
  MFString sub_wheel_report_topic_;
  MFString sub_body_report_topic_;
  MFString sub_mpc_trajecrory_topic_;

  MFString module_status_topic_;
  MFString pub_nodes_status_topic_;

  MFString tasknap_slice_topic_;
  mmessage::sensor_interface_mfrmsgs::MFRMessageCompressedVideo tasknap_cache_;
  MFString sub_planning_reload_request_topic_;
  MFString pub_simulation_sync_topic_;
  MFString sub_simulation_query_topic_;
  MFString pub_simulation_answer_topic_;
  MFString pub_planning_topic_;

  MFString sub_sbp_result_topic_;
  MFString pub_sbp_request_topic_;

  MFString sub_planning_control_cmd_request_topic_;
  MFString pub_planning_control_cmd_response_topic_;

  MFString sub_planning_request_topic_;
  MFString pub_planning_response_topic_;

  MFString sub_planning_reset_request_topic_;
  MFString pub_planning_reset_response_topic_;

  MFString pub_planning_info_topic_;
  MFString pub_mdebug_topic_;

  MFString sub_perception_vision_lane_topic_;
  MFString sub_perception_vision_landmark_topic_;
  MFString sub_perception_radar_topic_;

  MFString sub_mff_info_topic_;

  MFString sub_planning_frequency_control_topic_;
  MFString sub_perception_lidar_road_edge_topic_;

  MFString pub_planning_extra_data_topic_;
  MFString pub_planning_proto_debug_topic_;
  // for wm
  bool init_wm_ = true;
  bool internal_feed_ = true;
  MFRSubscriber *system_control_subscriber_{};
  MFRPublisher *system_control_publisher_{};

  MFRSubscriber *system_reset_subscriber_{};
  MFRPublisher *system_reset_publisher_{};

  MFRSubscriber *mode_switch_subscriber_{};
  MFRPublisher *mode_switch_publisher_{};

  MFRPublisher *wm_node_status_publisher_{};

  MFString sub_system_control_topic_;
  MFString sub_system_reset_topic_;
  MFString sub_perception_fusion_topic_;
  MFString sub_mode_switch_cmd_topic_;

  MFString pub_map_topic_;
  MFString pub_object_topic_;
  MFString pub_wm_node_status_topic_;
  MFString pub_mode_switch_cmd_topic_;
  MFString pub_system_control_response_topic_;
  MFString pub_system_reset_response_topic_;

  // parking worldmodel
  MFString sub_fusion_parking_slot_topic_;
  MFString pub_worldmodel_parking_slot_topic_;

  MFString sub_parking_wm_control_cmd_request_topic_;
  MFString sub_parking_wm_request_topic_;

  MFString pub_parking_wm_control_cmd_response_topic_;
  MFString pub_parking_wm_response_topic_;
  MFString pub_parking_wm_node_status_topic_;

  // messages' transfer mode
  int32_t sub_imu_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_mb_imu_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_prediction_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_worldmodel_map_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_worldmodel_objects_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_worldmodel_parking_slots_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_worldmodel_scene_object_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_traffic_light_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_fusion_objects_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_fusion_groundlines_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_fusion_uss_groundlines_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_fusion_uss_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_ego_pose_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_chassis_report_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_wireless_charger_report_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_wheel_report_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_body_report_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_mpc_trajecrory_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_module_status_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_sbp_result_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_planning_control_cmd_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_planning_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_planning_reset_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_vision_lane_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_vision_landmark_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_radar_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_mff_info_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_lidar_road_edge_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_planning_frequency_control_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_planning_reload_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_simulation_query_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_nodes_status_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_sbp_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_control_cmd_response_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_response_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_reset_response_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_info_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_mdebug_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_simulation_sync_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_extra_data_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_proto_debug_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_simulation_answer_mode_ = MSG_MODE_MFR_SOCKET;
  // for wm
  int32_t sub_system_control_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_system_reset_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_fusion_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_mode_switch_cmd_mode_ = MSG_MODE_MFR_SOCKET;

  int32_t pub_map_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_object_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_wm_node_status_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_mode_switch_cmd_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_system_control_response_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_system_reset_response_mode_ = MSG_MODE_MFR_SOCKET;

  // parking worldmodel
  int32_t sub_fusion_parking_slot_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_worldmodel_parking_slot_mode_ = MSG_MODE_MFR_SOCKET;

  int32_t sub_parking_wm_control_cmd_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_parking_wm_request_mode_ = MSG_MODE_MFR_SOCKET;

  int32_t pub_parking_wm_control_cmd_response_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_parking_wm_response_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_parking_wm_node_status_mode_ = MSG_MODE_MFR_SOCKET;

  // for shm
  MFString pub_node_status_apa_planning_topic_;
  int32_t last_apa_plan_trajectory_len_ = 0;
  int32_t pub_node_status_apa_planning_mode_ = MSG_MODE_MFR_SOCKET;

private:
  void init_config_data(std::string &config_data) {
    char *log_level = std::getenv("MSQUARE_PNC_LOG_LEVEL");
    auto config_reader = mjson::Reader(config_data.c_str());
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
    enable_ilc_ = config_reader.get<bool>("enable_ilc", false, true);
    enable_alc_ = config_reader.get<bool>("enable_alc", false, false);
    ilc_limit_velocity_ =
        config_reader.get<double>("interactive_lc_limit_velocity", false, 0.0);
    ilc_limit_time_ =
        config_reader.get<double>("interactive_lc_limit_time", false, 10.0);
    default_cruise_velocity_ =
        config_reader.get<double>("default_cruise_velocity", false, 120.0);
    underclocking_enum_ =
        config_reader.get<uint32_t>("underclocking_enum", false, 10);
    start_machine_ = config_reader.get<bool>("start_machine", false, false);
    init_wm_ = config_reader.get<bool>("init_wm", false, true);
    internal_feed_ = config_reader.get<bool>("internal_feed", false, true);
    is_acc_mode_ = config_reader.get<bool>("is_acc_mode", false, false);
    is_ddmap_ = config_reader.get<bool>("is_ddmap", false, false);
    driving_style_ = config_reader.get<int>("driving_style", false, 1);
  }

  void set_msquare_planning_engine_callback() {
    // set tasknap_slice callback
    auto &pub_tasknap = mfr_publisher_map_[tasknap_slice_topic_];
    planning_engine_->set_callback(
        [this, &pub_tasknap](uint8_t *ptr, uint64_t size) {
          tasknap_cache_.clear();
          tasknap_cache_.add_data(ptr, ptr + size);

          pub_tasknap->publish(tasknap_cache_);
        });

    LOGI("on_init set callbacks");

    auto &pub_planning = mfr_publisher_map_[pub_planning_topic_];
    planning_engine_->set_callback(
        [&pub_planning, this](const msquare::MSDPlanningOutputMeta &meta,
                              const maf_planning::Planning &planning_result,
                              const std::string &trigger_msg_id) {
          if (meta.succeed) {
            if (HAS_MSG_MODE(pub_planning_mode_,
                             MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
              LOGD("publish mfr %s", pub_planning_topic_.c_str());
              mmessage::planning_mfrmsgs::MFRMessagePlanning mfr_planning{};
              mfr_planning.get_internal_data()->set_original_data(
                  &planning_result);
              mfr_planning.get_internal_data()->set_serialize_preprocessor(
                  [&planning_result,  // parasoft-suppress AUTOSAR-A5_1_4
                   &mfr_planning]() { // parasoft-suppress AUTOSAR-A5_1_4
                    mfr_cpp_struct_convert::to_mfr(planning_result,
                                                   mfr_planning);
                  });

              msd_planning::MSDPlanning_record_relation(
                  mlog::MLOG_msg_id(planning_result.header.stamp,
                                    msd_planning::kPlanTag),
                  trigger_msg_id);
              pub_planning->publish(mfr_planning);
            }
            if (HAS_MSG_MODE(pub_planning_mode_, MSG_MODE_PE_SHM)) {
              LOGD("publish shmcom %s", pub_planning_topic_.c_str());
              publish_shmcom_planning(planning_result);
            }

            auto &pub_node_status_apa_planning =
                mfr_publisher_map_[pub_node_status_apa_planning_topic_];
            if (last_apa_plan_trajectory_len_ > 0 &&
                planning_result.trajectory.path.size() == 0) {
              CheckTrajectoryEmptyDuringAPA(pub_node_status_apa_planning,
                                            planning_result);
            }
            last_apa_plan_trajectory_len_ =
                planning_result.trajectory.path.size();
          }
        });

    auto &pub_nodes_status = node_status_publisher_;
    auto &pub_highway_worldmodel_node_status = wm_node_status_publisher_;
    auto &pub_parking_worldmodel_node_status =
        mfr_publisher_map_[pub_parking_wm_node_status_topic_];
    planning_engine_->set_callback([&pub_nodes_status,
                                    &pub_highway_worldmodel_node_status,
                                    &pub_parking_worldmodel_node_status,
                                    this](const maf_framework_status::NodeStatus
                                              &planning_node_status) {
      maf_framework_status::NodesStatus planning_nodes_status;
      planning_nodes_status.header.stamp = MTIME()->timestamp().ns();
      planning_nodes_status.nodes_status.push_back(planning_node_status);

      if (HAS_MSG_MODE(pub_nodes_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_nodes_status_topic_.c_str());
        mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus mfr_status{};
        mfr_status.get_internal_data()->set_original_data(
            &planning_nodes_status);
        mfr_status.get_internal_data()->set_serialize_preprocessor(
            [&planning_nodes_status, // parasoft-suppress AUTOSAR-A5_1_4
             &mfr_status]() {        // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(planning_nodes_status, mfr_status);
            });
        pub_nodes_status->publish(mfr_status);
      }

      if (HAS_MSG_MODE(pub_wm_node_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_wm_node_status_topic_.c_str());
        maf_framework_status::NodesStatus node_status;
        node_status.nodes_status.resize(1);
        node_status.header.stamp = MTIME()->timestamp().ns();

        node_status.nodes_status[0].timestamp_us = MTIME()->timestamp().us();
        node_status.nodes_status[0].node_type =
            static_cast<uint16_t>(node_status::NodeType::MODULE_WORLDMODEL);
        node_status.nodes_status[0].reserved_info = "ddmap";
        node_status.nodes_status[0].message = "worldmodel is running NORMAL";
        node_status.nodes_status[0].running_mode.value =
            node_status.nodes_status[0].running_mode.PILOT;
        node_status.nodes_status[0].status =
            static_cast<uint16_t>(node_status::Status::RUNNING);

        mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus
            mfr_node_status{};
        mfr_node_status.get_internal_data()->set_original_data(&node_status);
        mfr_node_status.get_internal_data()->set_serialize_preprocessor(
            [&node_status,         // parasoft-suppress AUTOSAR-A5_1_4
             &mfr_node_status]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(node_status, mfr_node_status);
            });
        pub_highway_worldmodel_node_status->publish(mfr_node_status);
      }

      // parking worldmodel
      if (HAS_MSG_MODE(pub_parking_wm_node_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_parking_wm_node_status_topic_.c_str());
        maf_framework_status::NodesStatus node_status;
        node_status.nodes_status.resize(1);
        node_status.header.stamp = MTIME()->timestamp().ns();

        node_status.nodes_status[0].timestamp_us = MTIME()->timestamp().us();
        node_status.nodes_status[0].node_type =
            static_cast<uint16_t>(node_status::NodeType::MODULE_WORLDMODEL);
        node_status.nodes_status[0].task_type = static_cast<uint16_t>(
            node_status::TaskType::NODE_WORLDMODEL_TASK_APA);
        node_status.nodes_status[0].detail_status =
            static_cast<uint16_t>(node_status::DetailStatus::WORLDMODEL_OK);
        node_status.nodes_status[0].running_mode.value =
            node_status.nodes_status[0].running_mode.PARKING;
        node_status.nodes_status[0].status =
            static_cast<uint16_t>(node_status::Status::RUNNING);

        mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus
            mfr_node_status{};

        mfr_node_status.get_internal_data()->set_original_data(&node_status);
        mfr_node_status.get_internal_data()->set_serialize_preprocessor(
            [&node_status,         // parasoft-suppress AUTOSAR-A5_1_4
             &mfr_node_status]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(node_status, mfr_node_status);
            });
        pub_parking_worldmodel_node_status->publish(mfr_node_status);
      }
    });

    auto &pub_sbp_request = mfr_publisher_map_[pub_sbp_request_topic_];
    planning_engine_->set_callback([&pub_sbp_request,
                                    this](const maf_planning::SBPRequest
                                              &sbp_request) {
      if (HAS_MSG_MODE(pub_sbp_request_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_sbp_request_topic_.c_str());
        mmessage::planning_mfrmsgs::MFRMessageSBPRequest sbp_request_mfrmsgs{};
        sbp_request_mfrmsgs.get_internal_data()->set_original_data(
            &sbp_request);
        sbp_request_mfrmsgs.get_internal_data()->set_serialize_preprocessor(
            [&sbp_request,             // parasoft-suppress AUTOSAR-A5_1_4
             &sbp_request_mfrmsgs]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(sbp_request, sbp_request_mfrmsgs);
            });
        pub_sbp_request->publish(sbp_request_mfrmsgs);
      }
    });

    auto &pub_planning_info = mfr_publisher_map_[pub_planning_info_topic_];

    planning_engine_->set_callback([&pub_planning_info, this](
                                       const maf_std::Header &planning_info) {
      if (HAS_MSG_MODE(pub_planning_info_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr socket %s", pub_planning_info_topic_.c_str());
        mmessage::std_mfrmsgs::MFRMessageHeader planning_info_mfrmsg{};
        planning_info_mfrmsg.get_internal_data()->set_original_data(
            &planning_info);
        planning_info_mfrmsg.get_internal_data()->set_serialize_preprocessor(
            [&planning_info,            // parasoft-suppress AUTOSAR-A5_1_4
             &planning_info_mfrmsg]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(planning_info,
                                             planning_info_mfrmsg);
            });

        pub_planning_info->publish(planning_info_mfrmsg);
      }
      if (HAS_MSG_MODE(pub_planning_info_mode_, MSG_MODE_PE_SHM)) {
        LOGD("publish shmcom %s", pub_planning_info_topic_.c_str());
        publish_shmcom_planning_info(planning_info);
      }
    });

    auto &pub_mdebug = mfr_publisher_map_[pub_mdebug_topic_];

    planning_engine_->set_callback([&pub_mdebug, this](std::string &&log) {
      if (HAS_MSG_MODE(pub_mdebug_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr socket %s", pub_mdebug_topic_.c_str());
        maf_std::Header mdebug;
        mdebug.frame_id.swap(log);
        mdebug.stamp = MTIME()->timestamp().ns();

        mmessage::std_mfrmsgs::MFRMessageHeader mdebug_mfrmsg{};
        mdebug_mfrmsg.get_internal_data()->set_original_data(&mdebug);
        mdebug_mfrmsg.get_internal_data()->set_serialize_preprocessor(
            [&mdebug, &mdebug_mfrmsg]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(mdebug, mdebug_mfrmsg);
            });

        pub_mdebug->publish(mdebug_mfrmsg);
      }
    });

    // for wm
    auto &pub_map = mfr_publisher_map_[pub_map_topic_];
    planning_engine_->set_processed_map_callback([&pub_map,
                                                  this](msquare::ProcessedMapPtr
                                                            processed_map) {
      if (HAS_MSG_MODE(pub_map_mode_, MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_map_topic_.c_str());
        mmessage::worldmodel_mfrmsgs::MFRMessageProcessedMap
            processed_map_msg{};

        processed_map_msg.get_internal_data()->set_original_data(processed_map);
        processed_map_msg.get_internal_data()->set_serialize_preprocessor(
            [&processed_map,         // parasoft-suppress AUTOSAR-A5_1_4
             &processed_map_msg]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(*processed_map, processed_map_msg);
            });
        pub_map->publish(processed_map_msg);

        if (internal_feed_) {
          planning_engine_->feed_worldmodel_map(processed_map);
        }
      }
    });

    auto &pub_object = mfr_publisher_map_[pub_object_topic_];
    planning_engine_->set_objects_interface_callback(
        [&pub_object, this](msquare::ObjectsInterfacePtr objects_interface) {
          if (HAS_MSG_MODE(pub_object_mode_,
                           MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
            LOGD("publish mfr %s", pub_object_topic_.c_str());
            mmessage::worldmodel_mfrmsgs::MFRMessageObjectsInterface
                objects_interface_msg{};
            objects_interface_msg.get_internal_data()->set_original_data(
                objects_interface);
            objects_interface_msg.get_internal_data()
                ->set_serialize_preprocessor(
                    [&objects_interface, // parasoft-suppress AUTOSAR-A5_1_4
                     &objects_interface_msg]() { // parasoft-suppress
                                                 // AUTOSAR-A5_1_4
                      mfr_cpp_struct_convert::to_mfr(*objects_interface,
                                                     objects_interface_msg);
                    });

            pub_object->publish(objects_interface_msg);

            if (internal_feed_) {
              planning_engine_->feed_worldmodel_objects(objects_interface);
            }
          }
        });

    // parking worldmodel
    auto &pub_worldmodel_parking_slot =
        mfr_publisher_map_[pub_worldmodel_parking_slot_topic_];
    planning_engine_->set_wm_parking_slot_callback(
        [&pub_worldmodel_parking_slot,
         this](const std::shared_ptr<maf_worldmodel::FusionAPA> &maf_msg) {
          if (HAS_MSG_MODE(pub_worldmodel_parking_slot_mode_,
                           MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
            LOGD("publish mfr %s", pub_worldmodel_parking_slot_topic_.c_str());
            mmessage::worldmodel_mfrmsgs::MFRMessageFusionAPA mfr_msg{};

            mfr_msg.get_internal_data()->set_original_data(maf_msg.get());
            mfr_msg.get_internal_data()->set_serialize_preprocessor(
                [&maf_msg, &mfr_msg]() { // parasoft-suppress AUTOSAR-A5_1_4
                  mfr_cpp_struct_convert::to_mfr(*maf_msg, mfr_msg);
                });

            pub_worldmodel_parking_slot->publish(mfr_msg);

            if (internal_feed_) {
              planning_engine_->feed_world_model_parking_slots(maf_msg);
            }
          }
        });

    planning_engine_->set_simulation_sync_callback([this](std::string &&log) {
      maf_std::Header header;
      header.seq = ++simulation_sync_seq_;
      header.frame_id.swap(log);
      header.stamp = MTIME()->timestamp().ns();
      mmessage::std_mfrmsgs::MFRMessageHeader simulation_sync_mfrmsg{};
      mfr_cpp_struct_convert::to_mfr(header, simulation_sync_mfrmsg);

      auto &pub_simulation_sync =
          mfr_publisher_map_[pub_simulation_sync_topic_];
      pub_simulation_sync->publish(simulation_sync_mfrmsg);
    });
  }

  void module_status_callback() {
    while (mfr_subscriber_map_[module_status_topic_]->empty() == false) {
      auto mfr_module_status =
          mfr_subscriber_map_[module_status_topic_]
              ->pop<
                  mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus>();

      auto maf_module_status =
          mfr_module_status.get_internal_data()
              ->get_original_data<maf_framework_status::ModuleStatus>();
      if (maf_module_status == nullptr) {
        maf_module_status =
            mmemory::MFMakeShared<maf_framework_status::ModuleStatus>();
        mfr_cpp_struct_convert::from_mfr(mfr_module_status, *maf_module_status);
      }

      (void)mfr_subscriber_map_[module_status_topic_]->record_delay_since(
          maf_module_status->header.stamp);
      if (maf_module_status->module_type.value ==
          maf_framework_status::ModuleType::PLANNING) {
        return;
      }

      LOGD("feed mfr %s", module_status_topic_.c_str());
      (void)feed_maf_module_status(maf_module_status);
    }
  }

  // for mfr-snapshot playback mode
  void tasknap_slice_callback() {
    while (mfr_subscriber_map_[tasknap_slice_topic_]->empty() == false) {
      auto mfr_mesg = mfr_subscriber_map_[tasknap_slice_topic_]
                          ->pop<mmessage::sensor_interface_mfrmsgs::
                                    MFRMessageCompressedVideo>();

      const auto &cdata = mfr_mesg.data();

      uint8_t *buf_ptr = (uint8_t *)cdata.data();
      uint64_t buf_size = cdata.size();

      planning_engine_->feed_tasknap_slice(buf_ptr, buf_size);
      _send_simulation_answer("snapshot:feed-slice");
    }
  }

  void _send_simulation_answer(const std::string &frame_id) {
    auto &pub_simulation_answer =
        mfr_publisher_map_[pub_simulation_answer_topic_];
    maf_std::Header simulation_answer{};
    simulation_answer.stamp = MTIME()->timestamp().ns();

    simulation_answer.frame_id = frame_id;

    // send answer
    mmessage::std_mfrmsgs::MFRMessageHeader mfr_simulation_answer{};
    mfr_simulation_answer.get_internal_data()->set_original_data(
        &simulation_answer);
    mfr_simulation_answer.get_internal_data()->set_serialize_preprocessor(
        [&simulation_answer, &mfr_simulation_answer]() {
          mfr_cpp_struct_convert::to_mfr(simulation_answer,
                                         mfr_simulation_answer);
        });

    pub_simulation_answer->publish(mfr_simulation_answer);
  }

  void planning_reload_request_callback() {
    while (!mfr_subscriber_map_[sub_planning_reload_request_topic_]->empty()) {
      mfr_subscriber_map_[sub_planning_reload_request_topic_]
          ->pop<mmessage::std_mfrmsgs::MFRMessageHeader>();
      if (planning_engine_->get_tasknap_agent()) {
        planning_engine_->get_tasknap_agent()->ReloadCache();
      }
    }
    _send_simulation_answer("snapshot:reload-ok");
  }

  void simulation_query_callback() {
    while (!mfr_subscriber_map_[sub_simulation_query_topic_]->empty()) {
      auto mfr_simulation_query =
          mfr_subscriber_map_[sub_simulation_query_topic_]
              ->pop<mmessage::std_mfrmsgs::MFRMessageHeader>();
      auto query = mfr_simulation_query.get_internal_data()
                         ->get_original_data<maf_std::Header>();
      if (query == nullptr) {
        query = mmemory::MFMakeShared<maf_std::Header>();
        mfr_cpp_struct_convert::from_mfr(mfr_simulation_query, *query);
      }

      if (query->frame_id == "mtime-setting") {
        if (use_egopose_stamp_) {
          feeder_(mtime::MTimeClock::ns(query->stamp));
        }
      } else if (query->frame_id == "version") {
        _send_simulation_answer("version:apa,answer:true");
      }
    }
  }

  void planning_control_cmd_request_callback() {
    while (module_control_cmd_request_subscriber_->empty() == false) {
      auto mfr_planning_control_cmd_request =
          module_control_cmd_request_subscriber_
              ->pop<mmessage::system_manager_mfrmsgs::
                        MFRMessageModuleControlCmdRequest>();

      auto request = mfr_planning_control_cmd_request.get_internal_data()
                         ->get_original_data<
                             maf_system_manager::ModuleControlCmdRequest>();
      if (request == nullptr) {
        request = mmemory::MFMakeShared<
            maf_system_manager::ModuleControlCmdRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_planning_control_cmd_request,
                                         *request);
      }

      running_mode_ = request->running_mode.value;
      if (request->module_control_cmd.value ==
          maf_system_manager::ModuleControlCmdEnum::PAUSE) {
        stop_planning_engine();
        pause_subscribers();
        pause_publishers();
      } else if (request->module_control_cmd.value ==
                  maf_system_manager::ModuleControlCmdEnum::RESUME) {
        run_planning_engine();
        resume_subscribers();
        resume_publishers();
      }
      planning_engine_->feed_planning_control_cmd_request(*request);
      LOGD("feed mfr %s", sub_planning_control_cmd_request_topic_.c_str());

      if (HAS_MSG_MODE(pub_planning_control_cmd_response_mode_,
                       MSG_MODE_MFR_SOCKET)) {
        auto &pub_planning_control_cmd_response =
            module_control_cmd_response_publisher_;
        maf_system_manager::ModuleControlCmdResponse
            planning_control_cmd_response{};
        planning_control_cmd_response.header.stamp = MTIME()->timestamp().ns();
        planning_control_cmd_response.request_seq = request->header.seq;
        planning_control_cmd_response.success = true;

        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdResponse
            mfr_planning_control_cmd_response{};
        mfr_planning_control_cmd_response.get_internal_data()
            ->set_original_data(&planning_control_cmd_response);
        mfr_planning_control_cmd_response.get_internal_data()
            ->set_serialize_preprocessor(
                [&planning_control_cmd_response,
                 &mfr_planning_control_cmd_response]() {
                  mfr_cpp_struct_convert::to_mfr(
                      planning_control_cmd_response,
                      mfr_planning_control_cmd_response);
                });

        pub_planning_control_cmd_response->publish(
            mfr_planning_control_cmd_response);
        LOGD("publish mfr %s",
             pub_planning_control_cmd_response_topic_.c_str());
      }
    }
  }

  void planning_request_callback() {
    while (mfr_subscriber_map_[sub_planning_request_topic_]->empty() == false) {
      auto mfr_planning_request =
          mfr_subscriber_map_[sub_planning_request_topic_]
              ->pop<mmessage::system_manager_mfrmsgs::
                        MFRMessageSysPlanningRequest>();

      auto request =
          mfr_planning_request.get_internal_data()
              ->get_original_data<maf_system_manager::SysPlanningRequest>();
      if (request == nullptr) {
        request =
            mmemory::MFMakeShared<maf_system_manager::SysPlanningRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_planning_request, *request);
      }

      // ignore force stop when ego vel > 0.3m/s
      bool ignore_force_stop = false;
      if (request->cmd.value ==
          maf_system_manager::SystemCmdTypeEnum::PLANNING_HIGHWAY_START_STOP) {
        if (request->highway_info.start_stop_cmd.value ==
            maf_system_manager::StartStopCmdEnum::STOP) {
          if (ego_vel_ > 0.3) {
            ignore_force_stop = true;
          }
        }
      }

      bool success = true;
      if (!ignore_force_stop) {
        planning_engine_->feed_planning_request(*request);
      } else {
        success = false;
      }

      auto &pub_planning_response =
          mfr_publisher_map_[pub_planning_response_topic_];
      maf_system_manager::SysPlanningResponse planning_response{};
      planning_response.header.stamp = MTIME()->timestamp().ns();
      planning_response.request_seq = request->header.seq;
      planning_response.success = success;

      mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningResponse
          mfr_planning_response{};
      mfr_planning_response.get_internal_data()->set_original_data(
          &planning_response);
      mfr_planning_response.get_internal_data()->set_serialize_preprocessor(
          [&planning_response, &mfr_planning_response]() {
            mfr_cpp_struct_convert::to_mfr(planning_response,
                                           mfr_planning_response);
          });

      (void)mfr_subscriber_map_[sub_planning_request_topic_]
          ->record_delay_since(request->header.stamp);
      LOGD("feed mfr %s", sub_planning_request_topic_.c_str());
      pub_planning_response->publish(mfr_planning_response);
    }
  }

  void planning_reset_request_callback() {
    while (mfr_subscriber_map_[sub_planning_reset_request_topic_]->empty() ==
           false) {
      auto mfr_planning_reset_request =
          mfr_subscriber_map_[sub_planning_reset_request_topic_]
              ->pop<mmessage::std_mfrmsgs::MFRMessageHeader>();
      simulation_sync_seq_ = 0;
      auto request = mfr_planning_reset_request.get_internal_data()
                         ->get_original_data<maf_std::Header>();
      if (request == nullptr) {
        request = mmemory::MFMakeShared<maf_std::Header>();
        mfr_cpp_struct_convert::from_mfr(mfr_planning_reset_request, *request);
      }
      planning_engine_->feed_planning_reset_request(*request);

      auto &pub_planning_reset_response =
          mfr_publisher_map_[pub_planning_reset_response_topic_];
      maf_std::Header planning_reset_response{};
      planning_reset_response.stamp = MTIME()->timestamp().ns();

      static mjson::Json::object response_json_object;
      response_json_object["request_seq"] = mjson::Json(request->seq);
      response_json_object["success"] = mjson::Json("True");
      planning_reset_response.frame_id =
          mjson::Json(response_json_object).dump();

      mmessage::std_mfrmsgs::MFRMessageHeader mfr_planning_reset_response{};
      mfr_planning_reset_response.get_internal_data()->set_original_data(
          &planning_reset_response);
      mfr_planning_reset_response.get_internal_data()
          ->set_serialize_preprocessor(
              [&planning_reset_response, &mfr_planning_reset_response]() {
                mfr_cpp_struct_convert::to_mfr(planning_reset_response,
                                               mfr_planning_reset_response);
              });

      (void)mfr_subscriber_map_[sub_planning_reset_request_topic_]
          ->record_delay_since(request->stamp);
      LOGD("feed mfr %s", sub_planning_reset_request_topic_.c_str());
      pub_planning_reset_response->publish(mfr_planning_reset_response);
    }
  }

  void imu_report_callback() {
    while (mfr_subscriber_map_[sub_imu_topic_]->empty() == false) {
      auto mfr_imu_report =
          mfr_subscriber_map_[sub_imu_topic_]
              ->pop<mmessage::gps_imu_mfrmsgs::MFRMessageMLAImu>();

      auto maf_imu_report = mfr_imu_report.get_internal_data()
                                ->get_original_data<maf_gps_imu::MLAImu>();
      if (maf_imu_report == nullptr) {
        maf_imu_report = mmemory::MFMakeShared<maf_gps_imu::MLAImu>();
        mfr_cpp_struct_convert::from_mfr(mfr_imu_report, *maf_imu_report);
      }

      (void)mfr_subscriber_map_[sub_imu_topic_]->record_delay_since(
          maf_imu_report->header.stamp);
      LOGD("feed mfr %s", sub_imu_topic_.c_str());
      if (!use_mb_imu_) {
        planning_engine_->feed_imu_report(maf_imu_report);
      }
    }
  }

  void mb_imu_report_callback() {
    while (mfr_subscriber_map_[sub_mb_imu_topic_]->empty() == false) {
      auto mfr_imu_report =
          mfr_subscriber_map_[sub_mb_imu_topic_]
              ->pop<mmessage::gps_imu_mfrmsgs::MFRMessageMLAImu>();

      auto maf_imu_report = mfr_imu_report.get_internal_data()
                                ->get_original_data<maf_gps_imu::MLAImu>();
      if (maf_imu_report == nullptr) {
        maf_imu_report = mmemory::MFMakeShared<maf_gps_imu::MLAImu>();
        mfr_cpp_struct_convert::from_mfr(mfr_imu_report, *maf_imu_report);
      }

      (void)mfr_subscriber_map_[sub_mb_imu_topic_]->record_delay_since(
          maf_imu_report->header.stamp);

      LOGD("feed mfr %s", sub_mb_imu_topic_.c_str());
      if (use_mb_imu_) {
        planning_engine_->feed_imu_report(maf_imu_report);
      }
    }
  }

  void prediction_callback() {
    while (mfr_subscriber_map_[sub_prediction_topic_]->empty() == false) {
      auto mfr_prediction_info =
          mfr_subscriber_map_[sub_prediction_topic_]
              ->pop<mmessage::worldmodel_mfrmsgs::MFRMessagePredictionResult>();

      auto maf_prediction =
          mfr_prediction_info.get_internal_data()
              ->get_original_data<maf_worldmodel::PredictionResult>();
      if (maf_prediction == nullptr) {
        maf_prediction =
            mmemory::MFMakeShared<maf_worldmodel::PredictionResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_prediction_info, *maf_prediction);
      }

      (void)mfr_subscriber_map_[sub_prediction_topic_]->record_delay_since(
          maf_prediction->header.stamp);
      LOGD("feed mfr %s", sub_prediction_topic_.c_str());
      planning_engine_->feed_prediction_info(maf_prediction);
    }
  }

  void chassis_report_callback() {
    while (mfr_subscriber_map_[sub_chassis_report_topic_]->empty() == false) {
      auto mfr_chassis_report =
          mfr_subscriber_map_[sub_chassis_report_topic_]
              ->pop<mmessage::endpoint_mfrmsgs::MFRMessageChassisReport>();

      auto maf_chassis_report =
          mfr_chassis_report.get_internal_data()
              ->get_original_data<maf_endpoint::ChassisReport>();
      if (maf_chassis_report == nullptr) {
        maf_chassis_report =
            mmemory::MFMakeShared<maf_endpoint::ChassisReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_chassis_report,
                                         *maf_chassis_report);
      }

      (void)mfr_subscriber_map_[sub_chassis_report_topic_]->record_delay_since(
          maf_chassis_report->header.stamp);
      LOGD("feed mfr %s", sub_chassis_report_topic_.c_str());
      planning_engine_->feed_chassis_report(maf_chassis_report);
    }
  }

  void wireless_charger_report_callback() {
    while (mfr_subscriber_map_[sub_wireless_charger_report_topic_]->empty() ==
           false) {
      auto mfr_wireless_charger_report =
          mfr_subscriber_map_[sub_wireless_charger_report_topic_]
              ->pop<mmessage::endpoint_mfrmsgs::
                        MFRMessageWirelessChargerReport>();

      auto maf_wireless_charger_report =
          mfr_wireless_charger_report.get_internal_data()
              ->get_original_data<maf_endpoint::WirelessChargerReport>();
      if (maf_wireless_charger_report == nullptr) {
        maf_wireless_charger_report =
            mmemory::MFMakeShared<maf_endpoint::WirelessChargerReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_wireless_charger_report,
                                         *maf_wireless_charger_report);
      }

      // TODO: no header at present.
      // MSDPlanning_record_timestamp(
      //     mlog::MLOG_msg_id((*maf_wireless_charger_report).header.stamp,
      //                       kWirelessChargerReportTag),
      //     MSDPlanning_msg_tag(kWirelessChargerReportTag), 0);

      // mfr_subscriber_map_[sub_wireless_charger_report_topic_]->record_delay_since(
      //     maf_wireless_charger_report->header.stamp);

      LOGD("feed mfr %s", sub_wireless_charger_report_topic_.c_str());
      planning_engine_->feed_wireless_charger_report(
          maf_wireless_charger_report);
    }
  }

  void wheel_report_callback() {
    while (mfr_subscriber_map_[sub_wheel_report_topic_]->empty() == false) {
      auto mfr_wheel_report =
          mfr_subscriber_map_[sub_wheel_report_topic_]
              ->pop<mmessage::endpoint_mfrmsgs::MFRMessageWheelReport>();

      auto maf_wheel_report =
          mfr_wheel_report.get_internal_data()
              ->get_original_data<maf_endpoint::WheelReport>();
      if (maf_wheel_report == nullptr) {
        maf_wheel_report = mmemory::MFMakeShared<maf_endpoint::WheelReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_wheel_report, *maf_wheel_report);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_wheel_report).header.stamp,
                            msd_planning::kWheelReportTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kWheelReportTag), 0);

      (void)mfr_subscriber_map_[sub_wheel_report_topic_]->record_delay_since(
          maf_wheel_report->header.stamp);
      LOGD("feed mfr %s", sub_wheel_report_topic_.c_str());
      planning_engine_->feed_wheel_report(maf_wheel_report);
    }
  }

  void body_report_callback() {
    while (mfr_subscriber_map_[sub_body_report_topic_]->empty() == false) {
      auto mfr_body_report =
          mfr_subscriber_map_[sub_body_report_topic_]
              ->pop<mmessage::endpoint_mfrmsgs::MFRMessageBodyReport>();

      auto maf_body_report =
          mfr_body_report.get_internal_data()
              ->get_original_data<maf_endpoint::BodyReport>();
      if (maf_body_report == nullptr) {
        maf_body_report = mmemory::MFMakeShared<maf_endpoint::BodyReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_body_report, *maf_body_report);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_body_report).header.stamp,
                            msd_planning::kBodyReportTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kBodyReportTag), 0);

      (void)mfr_subscriber_map_[sub_body_report_topic_]->record_delay_since(
          maf_body_report->header.stamp);
      LOGD("feed mfr %s", sub_body_report_topic_.c_str());
      planning_engine_->feed_body_report(maf_body_report);
    }
  }

  void worldmodel_map_callback() {
    while (mfr_subscriber_map_[sub_worldmodel_map_topic_]->empty() == false) {
      auto mfr_worldmodel_map =
          mfr_subscriber_map_[sub_worldmodel_map_topic_]
              ->pop<mmessage::worldmodel_mfrmsgs::MFRMessageProcessedMap>();

      auto maf_worldmodel_map =
          mfr_worldmodel_map.get_internal_data()
              ->get_original_data<maf_worldmodel::ProcessedMap>();
      if (maf_worldmodel_map == nullptr) {
        maf_worldmodel_map =
            mmemory::MFMakeShared<maf_worldmodel::ProcessedMap>();
        mfr_cpp_struct_convert::from_mfr(mfr_worldmodel_map,
                                         *maf_worldmodel_map);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_worldmodel_map).header.stamp,
                            msd_planning::kWorldModelMapTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kWorldModelMapTag),
          0);

      (void)mfr_subscriber_map_[sub_worldmodel_map_topic_]->record_delay_since(
          maf_worldmodel_map->header.stamp);
      LOGD("feed mfr %s", sub_worldmodel_map_topic_.c_str());

      if (!internal_feed_) {
        planning_engine_->feed_worldmodel_map(maf_worldmodel_map);
      }
    }
  }

  void worldmodel_object_callback() {
    while (mfr_subscriber_map_[sub_worldmodel_objects_topic_]->empty() ==
           false) {
      auto mfr_worldmodel_object =
          mfr_subscriber_map_[sub_worldmodel_objects_topic_]
              ->pop<mmessage::worldmodel_mfrmsgs::MFRMessageObjectsInterface>();

      auto maf_worldmodel_object =
          mfr_worldmodel_object.get_internal_data()
              ->get_original_data<maf_worldmodel::ObjectsInterface>();
      if (maf_worldmodel_object == nullptr) {
        maf_worldmodel_object =
            mmemory::MFMakeShared<maf_worldmodel::ObjectsInterface>();
        mfr_cpp_struct_convert::from_mfr(mfr_worldmodel_object,
                                         *maf_worldmodel_object);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_worldmodel_object).header.stamp,
                            msd_planning::kWorldModelObjectTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kWorldModelObjectTag),
          0);

      (void)mfr_subscriber_map_[sub_worldmodel_objects_topic_]
          ->record_delay_since(maf_worldmodel_object->header.stamp);
      LOGD("feed mfr %s", sub_worldmodel_objects_topic_.c_str());

      if (!internal_feed_) {
        planning_engine_->feed_worldmodel_objects(maf_worldmodel_object);
      }
    }
  }

  void worldmodel_parking_slots_callback() {
    while (mfr_subscriber_map_[sub_worldmodel_parking_slots_topic_]->empty() ==
           false) {
      auto mfr_worldmodel_parking_slots =
          mfr_subscriber_map_[sub_worldmodel_parking_slots_topic_]
              ->pop<mmessage::worldmodel_mfrmsgs::MFRMessageFusionAPA>();

      auto maf_worldmodel_parking_slots =
          mfr_worldmodel_parking_slots.get_internal_data()
              ->get_original_data<maf_worldmodel::FusionAPA>();
      if (maf_worldmodel_parking_slots == nullptr) {
        maf_worldmodel_parking_slots =
            mmemory::MFMakeShared<maf_worldmodel::FusionAPA>();
        mfr_cpp_struct_convert::from_mfr(mfr_worldmodel_parking_slots,
                                         *maf_worldmodel_parking_slots);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_worldmodel_parking_slots).header.stamp,
                            msd_planning::kWorldModelParkingSlotTag),
          msd_planning::MSDPlanning_msg_tag(
              msd_planning::kWorldModelParkingSlotTag),
          0);

      (void)mfr_subscriber_map_[sub_worldmodel_parking_slots_topic_]
          ->record_delay_since(maf_worldmodel_parking_slots->header.stamp);
      LOGD("feed mfr %s", sub_worldmodel_parking_slots_topic_.c_str());
      if (!internal_feed_) {
        planning_engine_->feed_world_model_parking_slots(
            maf_worldmodel_parking_slots);
      }
    }
  }

  void worldmodel_scene_objects_callback() {
    while (mfr_subscriber_map_[sub_worldmodel_scene_object_topic_]->empty() ==
           false) {
      auto mfr_worldmodel_scene_objects =
          mfr_subscriber_map_[sub_worldmodel_scene_object_topic_]
              ->pop<mmessage::worldmodel_mfrmsgs::MFRMessageSceneObjects>();

      auto maf_worldmodel_scene_objects =
          mfr_worldmodel_scene_objects.get_internal_data()
              ->get_original_data<maf_worldmodel::SceneObjects>();
      if (maf_worldmodel_scene_objects == nullptr) {
        maf_worldmodel_scene_objects =
            mmemory::MFMakeShared<maf_worldmodel::SceneObjects>();
        mfr_cpp_struct_convert::from_mfr(mfr_worldmodel_scene_objects,
                                         *maf_worldmodel_scene_objects);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_worldmodel_scene_objects).header.stamp,
                            msd_planning::kWorldModelSceneObjectTag),
          msd_planning::MSDPlanning_msg_tag(
              msd_planning::kWorldModelSceneObjectTag),
          0);

      (void)mfr_subscriber_map_[sub_worldmodel_scene_object_topic_]
          ->record_delay_since(maf_worldmodel_scene_objects->header.stamp);
      LOGD("feed mfr %s", sub_worldmodel_scene_object_topic_.c_str());
      planning_engine_->feed_world_model_scene_objects(
          maf_worldmodel_scene_objects);
    }
  }

  void traffic_light_callback() {
    while (mfr_subscriber_map_[sub_traffic_light_topic_]->empty() == false) {
      auto mfr_traffic_light =
          mfr_subscriber_map_[sub_traffic_light_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageTrafficLightPerception>();

      auto maf_traffic_light =
          mfr_traffic_light.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::TrafficLightPerception>();
      if (maf_traffic_light == nullptr) {
        maf_traffic_light = mmemory::MFMakeShared<
            maf_perception_interface::TrafficLightPerception>();
        mfr_cpp_struct_convert::from_mfr(mfr_traffic_light, *maf_traffic_light);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_traffic_light).header.stamp,
                            msd_planning::kTrafficLightTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kTrafficLightTag), 0);

      (void)mfr_subscriber_map_[sub_traffic_light_topic_]->record_delay_since(
          maf_traffic_light->header.stamp);
      LOGD("feed mfr %s", sub_traffic_light_topic_.c_str());
      planning_engine_->feed_traffic_light(maf_traffic_light);
    }
  }

  void fusion_object_callback() {
    while (mfr_subscriber_map_[sub_fusion_objects_topic_]->empty() == false) {
      auto mfr_fusion_objects =
          mfr_subscriber_map_[sub_fusion_objects_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessagePerceptionFusionObjectResult>();

      auto maf_fusion_objects =
          mfr_fusion_objects.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::PerceptionFusionObjectResult>();
      if (maf_fusion_objects == nullptr) {
        maf_fusion_objects = mmemory::MFMakeShared<
            maf_perception_interface::PerceptionFusionObjectResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_fusion_objects,
                                         *maf_fusion_objects);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_fusion_objects).header.stamp,
                            msd_planning::kFusionObjectTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kFusionObjectTag), 0);

      (void)mfr_subscriber_map_[sub_fusion_objects_topic_]->record_delay_since(
          maf_fusion_objects->header.stamp);
      LOGD("feed mfr %s", sub_fusion_objects_topic_.c_str());
      planning_engine_->feed_fusion_objects(maf_fusion_objects);
    }
  }

  void fusion_groundlines_callback() {
    while (mfr_subscriber_map_[sub_fusion_groundlines_topic_]->empty() ==
           false) {
      auto mfr_fusion_groundlines =
          mfr_subscriber_map_[sub_fusion_groundlines_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageFusionGroundLineResult>();

      auto maf_fusion_groundlines =
          mfr_fusion_groundlines.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::FusionGroundLineResult>();
      if (maf_fusion_groundlines == nullptr) {
        maf_fusion_groundlines = mmemory::MFMakeShared<
            maf_perception_interface::FusionGroundLineResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_fusion_groundlines,
                                         *maf_fusion_groundlines);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_fusion_groundlines).header.stamp,
                            msd_planning::kFusionGroundLineTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kFusionGroundLineTag),
          0);

      (void)mfr_subscriber_map_[sub_fusion_groundlines_topic_]
          ->record_delay_since(maf_fusion_groundlines->header.stamp);
      LOGD("feed mfr %s", sub_fusion_groundlines_topic_.c_str());
      planning_engine_->feed_fusion_grond_lines(maf_fusion_groundlines);
    }
  }

  void fusion_uss_groundlines_callback() {
    while (mfr_subscriber_map_[sub_fusion_uss_groundlines_topic_]->empty() ==
           false) {
      auto mfr_fusion_groundlines =
          mfr_subscriber_map_[sub_fusion_uss_groundlines_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageFusionGroundLineResult>();

      auto maf_fusion_groundlines =
          mfr_fusion_groundlines.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::FusionGroundLineResult>();
      if (maf_fusion_groundlines == nullptr) {
        maf_fusion_groundlines = mmemory::MFMakeShared<
            maf_perception_interface::FusionGroundLineResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_fusion_groundlines,
                                         *maf_fusion_groundlines);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_fusion_groundlines).header.stamp,
                            msd_planning::kFusionGroundLineTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kFusionGroundLineTag),
          0);

      LOGD("feed mfr %s", sub_fusion_uss_groundlines_topic_.c_str());
      planning_engine_->feed_fusion_uss_grond_lines(maf_fusion_groundlines);
    }
  }

  void fusion_uss_callback() {
    while (mfr_subscriber_map_[sub_fusion_uss_topic_]->empty() == false) {
      auto mfr_fusion_uss = mfr_subscriber_map_[sub_fusion_uss_topic_]
                                ->pop<mmessage::sensor_interface_mfrmsgs::
                                          MFRMessageUltrasonicUpaReport>();

      auto maf_fusion_uss =
          mfr_fusion_uss.get_internal_data()
              ->get_original_data<maf_sensor_interface::UltrasonicUpaReport>();
      if (maf_fusion_uss == nullptr) {
        maf_fusion_uss =
            mmemory::MFMakeShared<maf_sensor_interface::UltrasonicUpaReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_fusion_uss, *maf_fusion_uss);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_fusion_uss).header.stamp,
                            msd_planning::kFusionUssTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kFusionUssTag), 0);

      (void)mfr_subscriber_map_[sub_fusion_uss_topic_]->record_delay_since(
          maf_fusion_uss->header.stamp);
      LOGD("feed mfr %s", sub_fusion_uss_topic_.c_str());
      planning_engine_->feed_fusion_uss(maf_fusion_uss);
    }
  }

  void ego_pose_callback() {
    while (mfr_subscriber_map_[sub_ego_pose_topic_]->empty() == false) {
      auto mfr_ego_pose = mfr_subscriber_map_[sub_ego_pose_topic_]
                              ->pop<mmessage::mla_localization_mfrmsgs::
                                        MFRMessageMLALocalization>();

      auto maf_ego_pose =
          mfr_ego_pose.get_internal_data()
              ->get_original_data<maf_mla_localization::MLALocalization>();
      if (maf_ego_pose == nullptr) {
        maf_ego_pose =
            mmemory::MFMakeShared<maf_mla_localization::MLALocalization>();
        mfr_cpp_struct_convert::from_mfr(mfr_ego_pose, *maf_ego_pose);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_ego_pose).header.stamp,
                            msd_planning::kEgoPoseTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kEgoPoseTag), 0);

      (void)mfr_subscriber_map_[sub_ego_pose_topic_]->record_delay_since(
          maf_ego_pose->header.stamp);
      LOGD("feed mfr %s", sub_ego_pose_topic_.c_str());
      if (use_egopose_stamp_) {
        if (maf_ego_pose->header.frame_id.size() > 1 &&
            maf_ego_pose->header.frame_id[0] == 'T') {
          feeder_(mtime::MTimeClock::ns(maf_ego_pose->header.stamp));
        }
      }
      planning_engine_->feed_ego_pose(maf_ego_pose);

      if (maf_ego_pose->velocity.available &
          maf_mla_localization::MLAVelocity::MLA_VEL_LOCAL) {
        ego_vel_ = std::hypotf(maf_ego_pose->velocity.velocity_local.vx,
                               maf_ego_pose->velocity.velocity_local.vy);
      }
    }
  }

  void mpc_trajectory_callback() {
    while (mfr_subscriber_map_[sub_mpc_trajecrory_topic_]->empty() == false) {
      auto mfr_sub_mpc_trajectory =
          mfr_subscriber_map_[sub_mpc_trajecrory_topic_]
              ->pop<
                  mmessage::planning_mfrmsgs::MFRMessageMpcTrajectoryResult>();

      auto maf_mpc_trajectory =
          mfr_sub_mpc_trajectory.get_internal_data()
              ->get_original_data<maf_planning::MpcTrajectoryResult>();
      if (maf_mpc_trajectory == nullptr) {
        maf_mpc_trajectory =
            mmemory::MFMakeShared<maf_planning::MpcTrajectoryResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_sub_mpc_trajectory,
                                         *maf_mpc_trajectory);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_mpc_trajectory).header.stamp,
                            msd_planning::kMpcTrajectorydTag),
          msd_planning::MSDPlanning_msg_tag(msd_planning::kMpcTrajectorydTag),
          0);

      (void)mfr_subscriber_map_[sub_mpc_trajecrory_topic_]->record_delay_since(
          maf_mpc_trajectory->header.stamp);
      LOGD("feed mfr %s", sub_mpc_trajecrory_topic_.c_str());
      planning_engine_->feed_mpc_trajectory(maf_mpc_trajectory);
    }
  }

  void sbp_result_callback() {
    while (mfr_subscriber_map_[sub_sbp_result_topic_]->empty() == false) {
      auto mfr_sbp_result =
          mfr_subscriber_map_[sub_sbp_result_topic_]
              ->pop<mmessage::planning_mfrmsgs::MFRMessageSBPResult>();

      auto maf_sbp_result = mfr_sbp_result.get_internal_data()
                                ->get_original_data<maf_planning::SBPResult>();
      if (maf_sbp_result == nullptr) {
        maf_sbp_result = mmemory::MFMakeShared<maf_planning::SBPResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_sbp_result, *maf_sbp_result);
      }

      (void)mfr_subscriber_map_[sub_sbp_result_topic_]->record_delay_since(
          maf_sbp_result->header.stamp);
      LOGD("feed mfr %s", sub_sbp_result_topic_.c_str());
      planning_engine_->feed_sbp_result(*maf_sbp_result);
    }
  }

  void perception_vision_lane_callback() {
    while (mfr_subscriber_map_[sub_perception_vision_lane_topic_]->empty() ==
           false) {
      auto mfr_perception_vision_lane =
          mfr_subscriber_map_[sub_perception_vision_lane_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageRoadLinePerception>();

      auto maf_perception_vision_lane =
          mfr_perception_vision_lane.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::RoadLinePerception>();

      if (maf_perception_vision_lane == nullptr) {
        maf_perception_vision_lane = mmemory::MFMakeShared<
            maf_perception_interface::RoadLinePerception>();
        mfr_cpp_struct_convert::from_mfr(mfr_perception_vision_lane,
                                         *maf_perception_vision_lane);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_perception_vision_lane).header.stamp,
                            msd_planning::kPerceptionVisionLaneTag),
          msd_planning::MSDPlanning_msg_tag(
              msd_planning::kPerceptionVisionLaneTag),
          0);

      (void)mfr_subscriber_map_[sub_perception_vision_lane_topic_]
          ->record_delay_since(maf_perception_vision_lane->header.stamp);
      LOGD("feed mfr %s", sub_perception_vision_lane_topic_.c_str());
      planning_engine_->feed_perception_vision_lane(maf_perception_vision_lane);
    }
  }

  void perception_radar_callback() {
    while (mfr_subscriber_map_[sub_perception_radar_topic_]->empty() == false) {
      auto mfr_perception_radar =
          mfr_subscriber_map_[sub_perception_radar_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageRadarPerceptionResult>();

      auto maf_perception_radar =
          mfr_perception_radar.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::RadarPerceptionResult>();

      if (maf_perception_radar == nullptr) {
        maf_perception_radar = mmemory::MFMakeShared<
            maf_perception_interface::RadarPerceptionResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_perception_radar,
                                         *maf_perception_radar);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_perception_radar).header.stamp,
                            msd_planning::kPerceptionVisionLaneTag),
          msd_planning::MSDPlanning_msg_tag(
              msd_planning::kPerceptionVisionLaneTag),
          0);

      (void)mfr_subscriber_map_[sub_perception_radar_topic_]
          ->record_delay_since(maf_perception_radar->header.stamp);
      LOGD("feed mfr %s", sub_perception_radar_topic_.c_str());
      planning_engine_->feed_perception_radar(maf_perception_radar);
    }
  }

  void mff_info_callback() {
    while (mfr_subscriber_map_[sub_mff_info_topic_]->empty() == false) {
      auto mfr_mff_info = mfr_subscriber_map_[sub_mff_info_topic_]
                              ->pop<mmessage::std_mfrmsgs::MFRMessageHeader>();

      auto maf_mff_info = mfr_mff_info.get_internal_data()
                              ->get_original_data<maf_std::Header>();

      if (maf_mff_info == nullptr) {
        maf_mff_info = mmemory::MFMakeShared<maf_std::Header>();
        mfr_cpp_struct_convert::from_mfr(mfr_mff_info, *maf_mff_info);
      }

      LOGD("feed mfr %s", sub_mff_info_topic_.c_str());
      planning_engine_->feed_mff_info(maf_mff_info);
    }
  }

  void planning_frequency_control_callback() {
    while (
        mfr_subscriber_map_[sub_planning_frequency_control_topic_]->empty() ==
        false) {
      if (is_paused_ || is_in_simulation_) {
        return;
      }
      auto mfr_planning_frequency_control =
          mfr_subscriber_map_[sub_planning_frequency_control_topic_]
              ->pop<mmessage::system_manager_mfrmsgs::
                        MFRMessageSysPlanningRequest>();

      auto maf_planning_frequency_control =
          mfr_planning_frequency_control.get_internal_data()
              ->get_original_data<maf_system_manager::SysPlanningRequest>();

      if (maf_planning_frequency_control == nullptr) {
        maf_planning_frequency_control =
            mmemory::MFMakeShared<maf_system_manager::SysPlanningRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_planning_frequency_control,
                                         *maf_planning_frequency_control);
      }

      planning_engine_->feed_planning_request(*maf_planning_frequency_control);
    }
  }

  void perception_lidar_road_edge_callback() {
    while (
        mfr_subscriber_map_[sub_perception_lidar_road_edge_topic_]->empty() ==
        false) {

      auto mfr_perception_lidar_road_edge =
          mfr_subscriber_map_[sub_perception_lidar_road_edge_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageRoadLinePerception>();

      auto maf_perception_lidar_road_edge =
          mfr_perception_lidar_road_edge.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::RoadLinePerception>();

      if (maf_perception_lidar_road_edge == nullptr) {
        maf_perception_lidar_road_edge = mmemory::MFMakeShared<
            maf_perception_interface::RoadLinePerception>();
        mfr_cpp_struct_convert::from_mfr(mfr_perception_lidar_road_edge,
                                         *maf_perception_lidar_road_edge);
      }

      msd_planning::MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_perception_lidar_road_edge).header.stamp,
                            msd_planning::kPerceptionLidarRoadEdgeTag),
          msd_planning::MSDPlanning_msg_tag(
              msd_planning::kPerceptionLidarRoadEdgeTag),
          0);

      (void)mfr_subscriber_map_[sub_perception_lidar_road_edge_topic_]
          ->record_delay_since(maf_perception_lidar_road_edge->header.stamp);
      LOGD("feed mfr %s", sub_perception_lidar_road_edge_topic_.c_str());
      planning_engine_->feed_perception_lidar_road_edge(
          maf_perception_lidar_road_edge);
    }
  }

  // for wm
  void system_control_callback() {
    while (system_control_subscriber_->empty() == false) {
      auto mfr_module_control =
          system_control_subscriber_
              ->pop<mmessage::system_manager_mfrmsgs::
                        MFRMessageModuleControlCmdRequest>();

      auto maf_module_control =
          mfr_module_control.get_internal_data()
              ->get_original_data<
                  maf_system_manager::ModuleControlCmdRequest>();
      if (nullptr == maf_module_control) {
        maf_module_control = mmemory::MFMakeShared<
            maf_system_manager::ModuleControlCmdRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_module_control,
                                         *maf_module_control);
      }

      // send fake response
      auto &pub_system_control_response = system_control_publisher_;
      maf_system_manager::ModuleControlCmdResponse response{};
      response.header.stamp = MTIME()->timestamp().ns();
      response.header.seq = maf_module_control->header.seq;
      response.success = true;

      if (HAS_MSG_MODE(pub_system_control_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_system_control_response_topic_.c_str());
        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdResponse
            mfr_msg{};
        mfr_msg.get_internal_data()->set_original_data(&response);
        mfr_msg.get_internal_data()->set_serialize_preprocessor(
            [&response, &mfr_msg]() {
              mfr_cpp_struct_convert::to_mfr(response, mfr_msg);
            });

        pub_system_control_response->publish(mfr_msg);
      }
    }
  }

  void system_reset_callback() {
    while (system_reset_subscriber_->empty() == false) {
      auto mfr_system_reset =
          system_reset_subscriber_
              ->pop<mmessage::std_mfrmsgs::MFRMessageHeader>();

      auto maf_module_reset = mfr_system_reset.get_internal_data()
                                  ->get_original_data<maf_std::Header>();
      if (nullptr == maf_module_reset) {
        maf_module_reset = mmemory::MFMakeShared<maf_std::Header>();
        mfr_cpp_struct_convert::from_mfr(mfr_system_reset, *maf_module_reset);
      }

      // send fake response
      auto &pub_system_reset_response = system_reset_publisher_;
      maf_std::Header response{};
      response.stamp = MTIME()->timestamp().ns();
      response.seq = maf_module_reset->seq;

      if (HAS_MSG_MODE(pub_system_reset_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_system_reset_response_topic_.c_str());
        mmessage::std_mfrmsgs::MFRMessageHeader mfr_msg{};
        mfr_msg.get_internal_data()->set_original_data(&response);
        mfr_msg.get_internal_data()->set_serialize_preprocessor(
            [&response, &mfr_msg]() {
              mfr_cpp_struct_convert::to_mfr(response, mfr_msg);
            });
        pub_system_reset_response->publish(mfr_msg);
      }
    }
  }

  void perception_fusion_callback() {
    while (mfr_subscriber_map_[sub_perception_fusion_topic_]->empty() ==
           false) {
      auto mfr_perception_fusion =
          mfr_subscriber_map_[sub_perception_fusion_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessagePerceptionFusionObjectResult>();

      auto maf_perception_fusion =
          mfr_perception_fusion.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::PerceptionFusionObjectResult>();
      if (nullptr == maf_perception_fusion) {
        maf_perception_fusion = mmemory::MFMakeShared<
            maf_perception_interface::PerceptionFusionObjectResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_perception_fusion,
                                         *maf_perception_fusion);
      }

      LOGD("feed mfr %s", sub_perception_fusion_topic_.c_str());

      (void)mfr_subscriber_map_[sub_perception_fusion_topic_]
          ->record_delay_since((*maf_perception_fusion).header.stamp);
      planning_engine_->feed_perception_fusion_result(maf_perception_fusion);
    }
  }

  void mode_switch_callback() {
    while (mode_switch_subscriber_->empty() == false) {
      auto mfr_mode_switch = mode_switch_subscriber_->pop<
          mmessage::system_manager_mfrmsgs::MFRMessageSysWorldModelRequest>();

      auto maf_mode_switch =
          mfr_mode_switch.get_internal_data()
              ->get_original_data<maf_system_manager::SysWorldModelRequest>();
      if (nullptr == maf_mode_switch) {
        maf_mode_switch =
            mmemory::MFMakeShared<maf_system_manager::SysWorldModelRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_mode_switch, *maf_mode_switch);
      }

      // send fake response
      auto &pub_mode_switch_cmd_response = mode_switch_publisher_;
      if (HAS_MSG_MODE(pub_mode_switch_cmd_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        maf_system_manager::SysWorldModelResponse maf_msg{};
        maf_msg.header.stamp = MTIME()->timestamp().ns();
        maf_msg.header.seq = maf_mode_switch->header.seq;
        maf_msg.success = true;

        mmessage::system_manager_mfrmsgs::MFRMessageSysWorldModelResponse
            mfr_msg{};
        mfr_msg.get_internal_data()->set_original_data(&maf_msg);
        mfr_msg.get_internal_data()->set_serialize_preprocessor(
            [&maf_msg, &mfr_msg]() {
              mfr_cpp_struct_convert::to_mfr(maf_msg, mfr_msg);
            });

        pub_mode_switch_cmd_response->publish(mfr_msg);
      }
    }
  }

  // parking worldmodel
  void fusion_parking_slot_callback() {
    while (mfr_subscriber_map_[sub_fusion_parking_slot_topic_]->empty() ==
           false) {
      auto mfr_fusion_parking_slot =
          mfr_subscriber_map_[sub_fusion_parking_slot_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageFusionParkingSlotResult>();

      auto maf_fusion_parking_slot =
          mfr_fusion_parking_slot.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::FusionParkingSlotResult>();
      if (maf_fusion_parking_slot == nullptr) {
        maf_fusion_parking_slot = mmemory::MFMakeShared<
            maf_perception_interface::FusionParkingSlotResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_fusion_parking_slot,
                                         *maf_fusion_parking_slot);
      }

      (void)mfr_subscriber_map_[sub_fusion_parking_slot_topic_]
          ->record_delay_since(maf_fusion_parking_slot->header.stamp);
      LOGD("feed mfr %s", sub_fusion_parking_slot_topic_.c_str());

      planning_engine_->feed_fusion_parking_slot(maf_fusion_parking_slot);
    }
  }

  void parking_wm_control_cmd_request_callback() {
    while (mfr_subscriber_map_[sub_parking_wm_control_cmd_request_topic_]
               ->empty() == false) {
      auto mfr_parking_wm_request =
          mfr_subscriber_map_[sub_parking_wm_control_cmd_request_topic_]
              ->pop<mmessage::system_manager_mfrmsgs::
                        MFRMessageModuleControlCmdRequest>();

      auto request = mfr_parking_wm_request.get_internal_data()
                         ->get_original_data<
                             maf_system_manager::ModuleControlCmdRequest>();
      if (request == nullptr) {
        request = mmemory::MFMakeShared<
            maf_system_manager::ModuleControlCmdRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_parking_wm_request, *request);
      }

      auto &pub_parking_wm_response =
          mfr_publisher_map_[pub_parking_wm_control_cmd_response_topic_];
      maf_system_manager::ModuleControlCmdResponse parking_wm_response{};
      parking_wm_response.header.stamp = MTIME()->timestamp().ns();
      parking_wm_response.request_seq = request->header.seq;
      parking_wm_response.success = true;

      mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdResponse
          mfr_parking_wm_response{};
      mfr_parking_wm_response.get_internal_data()->set_original_data(
          &parking_wm_response);
      mfr_parking_wm_response.get_internal_data()->set_serialize_preprocessor(
          [&parking_wm_response, &mfr_parking_wm_response]() {
            mfr_cpp_struct_convert::to_mfr(parking_wm_response,
                                           mfr_parking_wm_response);
          });

      (void)mfr_subscriber_map_[sub_parking_wm_control_cmd_request_topic_]
          ->record_delay_since(request->header.stamp);
      LOGD("feed mfr %s", sub_parking_wm_control_cmd_request_topic_.c_str());
      if (init_wm_) {
        pub_parking_wm_response->publish(mfr_parking_wm_response);
      }
    }
  }

  void parking_wm_request_callback() {
    while (mfr_subscriber_map_[sub_parking_wm_request_topic_]->empty() ==
           false) {
      auto mfr_parking_wm_request =
          mfr_subscriber_map_[sub_parking_wm_request_topic_]
              ->pop<mmessage::system_manager_mfrmsgs::
                        MFRMessageSysWorldModelRequest>();

      auto request =
          mfr_parking_wm_request.get_internal_data()
              ->get_original_data<maf_system_manager::SysWorldModelRequest>();
      if (request == nullptr) {
        request =
            mmemory::MFMakeShared<maf_system_manager::SysWorldModelRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_parking_wm_request, *request);
      }

      auto &pub_parking_wm_response =
          mfr_publisher_map_[pub_parking_wm_response_topic_];
      maf_system_manager::SysWorldModelResponse parking_wm_response{};
      parking_wm_response.header.stamp = MTIME()->timestamp().ns();
      parking_wm_response.request_seq = request->header.seq;
      parking_wm_response.success = true;

      mmessage::system_manager_mfrmsgs::MFRMessageSysWorldModelResponse
          mfr_parking_wm_response{};
      mfr_parking_wm_response.get_internal_data()->set_original_data(
          &parking_wm_response);
      mfr_parking_wm_response.get_internal_data()->set_serialize_preprocessor(
          [&parking_wm_response, &mfr_parking_wm_response]() {
            mfr_cpp_struct_convert::to_mfr(parking_wm_response,
                                           mfr_parking_wm_response);
          });

      (void)mfr_subscriber_map_[sub_parking_wm_request_topic_]
          ->record_delay_since(request->header.stamp);
      LOGD("feed mfr %s", sub_parking_wm_request_topic_.c_str());
      if (init_wm_) {
        pub_parking_wm_response->publish(mfr_parking_wm_response);
      }
    }
  }

  void pause_subscribers() {
    for (auto &item : mfr_subscriber_map_) {
      if (!item.second->is_paused()) {
        (void)item.second->pause();
      }
    }
  }

  void pause_publishers() {
    for (auto &item : mfr_publisher_map_) {
      if (!item.second->is_paused()) {
        (void)item.second->pause();
      }
    }
  }

  void resume_subscribers() {
    for (auto &item : mfr_subscriber_map_) {
      if (internal_feed_) {
        if (item.first == sub_worldmodel_map_topic_ ||
            item.first == sub_worldmodel_objects_topic_ ||
            item.first == sub_worldmodel_parking_slots_topic_) {
          continue;
        }
      }
      if (item.second->is_paused()) {
        (void)item.second->resume();
      }
    }
  }

  void resume_publishers() {
    for (auto &item : mfr_publisher_map_) {
      if (item.second->is_paused()) {
        (void)item.second->resume();
      }
    }
  }

  void
  CheckTrajectoryEmptyDuringAPA(MFRPublisher *publisher,
                                const maf_planning::Planning &planning_result) {
    const static std::string TASK_APA = "\"task\" : \"APA\"";
    const static std::string STATUS_RUNNING = "\"status\" : \"RUNNING\"";

    if ((planning_result.extra.available & planning_result.extra.JSON) &&
        (planning_result.extra.json.find(TASK_APA) != std::string::npos) &&
        (planning_result.extra.json.find(STATUS_RUNNING) !=
         std::string::npos)) {
      maf_framework_status::NodesStatus node_status;
      node_status.nodes_status.resize(1);
      node_status.header.stamp = MTIME()->timestamp().ns();
      node_status.nodes_status[0].timestamp_us = MTIME()->timestamp().us();
      static uint16_t APA_PLANNING_NO_TRAJECTORY = 0xB301;
      node_status.nodes_status[0].detail_status =
          static_cast<uint16_t>(APA_PLANNING_NO_TRAJECTORY);
      node_status.nodes_status[0].status =
          static_cast<uint16_t>(node_status::Status::RUNNING_ERROR);

      mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus
          mfr_node_status{};

      mfr_node_status.get_internal_data()->set_original_data(&node_status);
      mfr_node_status.get_internal_data()->set_serialize_preprocessor(
          [&node_status,         // parasoft-suppress AUTOSAR-A5_1_4
           &mfr_node_status]() { // parasoft-suppress AUTOSAR-A5_1_4
            mfr_cpp_struct_convert::to_mfr(node_status, mfr_node_status);
          });
      publisher->publish(mfr_node_status);
    }
  }

private:
  bool feed_maf_module_status(
      const std::shared_ptr<maf_framework_status::ModuleStatus>
          maf_module_status) {
    if (maf_module_status->module_type.value ==
        maf_framework_status::ModuleType::PLANNING) {
      return false;
    }

    msd_planning::MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id((*maf_module_status).header.stamp,
                          msd_planning::kModuleStatusTag),
        msd_planning::MSDPlanning_msg_tag(msd_planning::kModuleStatusTag), 0);

    planning_engine_->feed_module_status(*maf_module_status);

    return true;
  }

#if defined(FEAT_SWITCH_UTIL)

private:
  const std::string KEY_MODULE_NAME = "Planning";
  const std::string KEY_SUB = "sub";
  const std::string KEY_PUB = "pub";
  const std::string KEY_METHOD_PE_SHM = "PE_shm";
  const std::string KEY_METHOD_INPCOM = "inpcom";
  const std::string KEY_METHOD_MFR_SOCKET = "mfr_socket";
  const std::string KEY_METHOD_MFR_SHM = "mfr_shm";

  void setup_msg_modes() {
    Switch sw;

    sub_imu_mode_ = get_msg_sub_mode(sw, sub_imu_topic_.c_str());
    sub_mb_imu_mode_ = get_msg_sub_mode(sw, sub_mb_imu_topic_.c_str());
    sub_prediction_mode_ = get_msg_sub_mode(sw, sub_prediction_topic_.c_str());
    sub_worldmodel_map_mode_ =
        get_msg_sub_mode(sw, sub_worldmodel_map_topic_.c_str());
    sub_worldmodel_objects_mode_ =
        get_msg_sub_mode(sw, sub_worldmodel_objects_topic_.c_str());
    sub_worldmodel_parking_slots_mode_ =
        get_msg_sub_mode(sw, sub_worldmodel_parking_slots_topic_.c_str());
    sub_worldmodel_scene_object_mode_ =
        get_msg_sub_mode(sw, sub_worldmodel_scene_object_topic_.c_str());
    sub_traffic_light_mode_ =
        get_msg_sub_mode(sw, sub_traffic_light_topic_.c_str());
    sub_fusion_objects_mode_ =
        get_msg_sub_mode(sw, sub_fusion_objects_topic_.c_str());
    sub_fusion_groundlines_mode_ =
        get_msg_sub_mode(sw, sub_fusion_groundlines_topic_.c_str());
    sub_fusion_uss_groundlines_mode_ =
        get_msg_sub_mode(sw, sub_fusion_uss_groundlines_topic_.c_str());
    sub_fusion_uss_mode_ = get_msg_sub_mode(sw, sub_fusion_uss_topic_.c_str());
    sub_ego_pose_mode_ = get_msg_sub_mode(sw, sub_ego_pose_topic_.c_str());
    sub_chassis_report_mode_ =
        get_msg_sub_mode(sw, sub_chassis_report_topic_.c_str());
    sub_wireless_charger_report_mode_ =
        get_msg_sub_mode(sw, sub_wireless_charger_report_topic_.c_str());
    sub_wheel_report_mode_ =
        get_msg_sub_mode(sw, sub_wheel_report_topic_.c_str());
    sub_body_report_mode_ =
        get_msg_sub_mode(sw, sub_body_report_topic_.c_str());
    sub_mpc_trajecrory_mode_ =
        get_msg_sub_mode(sw, sub_mpc_trajecrory_topic_.c_str());
    sub_sbp_result_mode_ = get_msg_sub_mode(sw, sub_sbp_result_topic_.c_str());
    sub_planning_control_cmd_request_mode_ =
        get_msg_sub_mode(sw, sub_planning_control_cmd_request_topic_.c_str());
    sub_planning_request_mode_ =
        get_msg_sub_mode(sw, sub_planning_request_topic_.c_str());
    sub_planning_reset_request_mode_ =
        get_msg_sub_mode(sw, sub_planning_reset_request_topic_.c_str());
    sub_module_status_mode_ =
        get_msg_sub_mode(sw, module_status_topic_.c_str());
    sub_perception_vision_lane_mode_ =
        get_msg_sub_mode(sw, sub_perception_vision_lane_topic_.c_str());
    sub_perception_vision_landmark_mode_ =
        get_msg_sub_mode(sw, sub_perception_vision_landmark_topic_.c_str());
    sub_perception_radar_mode_ =
        get_msg_sub_mode(sw, sub_perception_radar_topic_.c_str());
    sub_mff_info_mode_ = get_msg_sub_mode(sw, sub_mff_info_topic_.c_str());
    sub_perception_lidar_road_edge_mode_ =
        get_msg_sub_mode(sw, sub_perception_lidar_road_edge_topic_.c_str());
    sub_planning_frequency_control_mode_ =
        get_msg_sub_mode(sw, sub_planning_frequency_control_topic_.c_str());
    pub_nodes_status_mode_ =
        get_msg_pub_mode(sw, pub_nodes_status_topic_.c_str());
    pub_planning_mode_ = get_msg_pub_mode(sw, pub_planning_topic_.c_str());
    pub_sbp_request_mode_ =
        get_msg_pub_mode(sw, pub_sbp_request_topic_.c_str());
    pub_planning_control_cmd_response_mode_ =
        get_msg_pub_mode(sw, pub_planning_control_cmd_response_topic_.c_str());
    pub_planning_response_mode_ =
        get_msg_pub_mode(sw, pub_planning_response_topic_.c_str());
    pub_planning_reset_response_mode_ =
        get_msg_pub_mode(sw, pub_planning_reset_response_topic_.c_str());
    pub_planning_info_mode_ =
        get_msg_pub_mode(sw, pub_planning_info_topic_.c_str());
    pub_mdebug_mode_ = get_msg_pub_mode(sw, pub_mdebug_topic_.c_str());
    pub_planning_extra_data_mode_ =
        get_msg_sub_mode(sw, pub_planning_extra_data_topic_.c_str());
    pub_planning_proto_debug_mode_ =
        get_msg_sub_mode(sw, pub_planning_proto_debug_topic_.c_str());
    // for wm
    sub_system_control_mode_ =
        get_msg_sub_mode(sw, sub_system_control_topic_.c_str());
    sub_system_reset_mode_ =
        get_msg_sub_mode(sw, sub_system_reset_topic_.c_str());
    sub_perception_fusion_mode_ =
        get_msg_sub_mode(sw, sub_perception_fusion_topic_.c_str());
    sub_mode_switch_cmd_mode_ =
        get_msg_sub_mode(sw, sub_mode_switch_cmd_topic_.c_str());

    pub_map_mode_ = get_msg_pub_mode(sw, pub_map_topic_.c_str());
    pub_object_mode_ = get_msg_pub_mode(sw, pub_object_topic_.c_str());
    pub_wm_node_status_mode_ =
        get_msg_pub_mode(sw, pub_wm_node_status_topic_.c_str());
    pub_mode_switch_cmd_mode_ =
        get_msg_pub_mode(sw, pub_mode_switch_cmd_topic_.c_str());
    pub_system_control_response_mode_ =
        get_msg_pub_mode(sw, pub_system_control_response_topic_.c_str());
    pub_system_reset_response_mode_ =
        get_msg_pub_mode(sw, pub_system_reset_response_topic_.c_str());

    // parking worldmodel
    sub_fusion_parking_slot_mode_ =
        get_msg_sub_mode(sw, sub_fusion_parking_slot_topic_.c_str());
    pub_worldmodel_parking_slot_mode_ =
        get_msg_pub_mode(sw, pub_worldmodel_parking_slot_topic_.c_str());

    sub_parking_wm_control_cmd_request_mode_ =
        get_msg_sub_mode(sw, sub_parking_wm_control_cmd_request_topic_.c_str());
    sub_parking_wm_request_mode_ =
        get_msg_sub_mode(sw, sub_parking_wm_request_topic_.c_str());

    pub_parking_wm_control_cmd_response_mode_ = get_msg_sub_mode(
        sw, pub_parking_wm_control_cmd_response_topic_.c_str());
    pub_parking_wm_response_mode_ =
        get_msg_sub_mode(sw, pub_parking_wm_response_topic_.c_str());
    pub_parking_wm_node_status_mode_ =
        get_msg_sub_mode(sw, pub_parking_wm_node_status_topic_.c_str());
  }

  int32_t get_msg_sub_mode(Switch &sw, const std::string &topic) {
#if defined(FEAT_SHMCOM_MESSAGE)
    try {
      if (sw.get_switcher_value(KEY_MODULE_NAME, KEY_SUB, topic,
                                KEY_METHOD_PE_SHM)) {
        return MSG_MODE_PE_SHM;
      }
    } catch (std::exception &e) {
    }
#endif // FEAT_SHMCOM_MESSAGE
    try {
      if (sw.get_switcher_value(KEY_MODULE_NAME, KEY_SUB, topic,
                                KEY_METHOD_MFR_SOCKET)) {
        return MSG_MODE_MFR_SOCKET;
      }
    } catch (std::exception &e) {
    }
    try {
      if (sw.get_switcher_value(KEY_MODULE_NAME, KEY_SUB, topic,
                                KEY_METHOD_MFR_SHM)) {
        return MSG_MODE_MFR_SHM;
      }
    } catch (std::exception &e) {
    }
    return MSG_MODE_MFR_SOCKET;
  }

  int32_t get_msg_pub_mode(Switch &sw, const std::string &topic) {
    int32_t mode = 0;
#if defined(FEAT_SHMCOM_MESSAGE)
    try {
      if (sw.get_switcher_value(KEY_MODULE_NAME, KEY_PUB, topic,
                                KEY_METHOD_PE_SHM)) {
        mode |= MSG_MODE_PE_SHM;
      }
    } catch (std::exception &e) {
    }
#endif // FEAT_SHMCOM_MESSAGE
    try {
      if (sw.get_switcher_value(KEY_MODULE_NAME, KEY_PUB, topic,
                                KEY_METHOD_MFR_SOCKET)) {
        mode |= MSG_MODE_MFR_SOCKET;
      }
    } catch (std::exception &e) {
    }
    try {
      if (sw.get_switcher_value(KEY_MODULE_NAME, KEY_PUB, topic,
                                KEY_METHOD_MFR_SHM)) {
        mode |= MSG_MODE_MFR_SHM;
        mode &= ~MSG_MODE_MFR_SOCKET;
      }
    } catch (std::exception &e) {
    }
    return (mode != 0 ? mode : MSG_MODE_MFR_SOCKET);
  }

#else

private:
  void setup_msg_modes() {}

#endif // FEAT_SWITCH_UTIL

#if defined(FEAT_SHMCOM_MESSAGE)

private:
  template <typename T>
  using ShmcomHandler = std::shared_ptr<shmcom::SimpleShmcomHelper<T>>;
  template <typename T>
  ShmcomHandler<T> make_shmcom_handler(const std::string &name, uint32_t hz) {
    return std::make_shared<shmcom::SimpleShmcomHelper<T>>(name, "cpplan",
                                                           1000U / hz);
  }

  ShmcomHandler<shmcom_gps_imu::PODMLAImu> sub_shmcom_imu_;
  ShmcomHandler<shmcom_endpoint::PODChassisReport> sub_shmcom_chassis_report_;
  ShmcomHandler<shmcom_endpoint::PODWheelReport> sub_shmcom_wheel_report_;
  ShmcomHandler<shmcom_endpoint::PODBodyReport> sub_shmcom_body_report_;
  ShmcomHandler<shmcom_framework_status::PODModuleStatus>
      sub_shmcom_module_status_;
  ShmcomHandler<shmcom_mla_localization::PODMLALocalization>
      sub_shmcom_ego_pose_;
  ShmcomHandler<shmcom_perception_interface::PODRadarPerceptionResult>
      sub_shmcom_perception_radar_;

  ShmcomHandler<shmcom_planning::PODPlanning> pub_shmcom_planning_;
  shmcom_planning::PODPlanning *ptr_shmcom_planning_ = nullptr;
  ShmcomHandler<shmcom_std::PODPlanningInfo> pub_shmcom_planning_info_;
  shmcom_std::PODPlanningInfo *ptr_shmcom_planning_info_ = nullptr;

  void subscribe_shmcom_imu() {
    if (sub_shmcom_imu_) {
      return;
    }
    sub_shmcom_imu_ = make_shmcom_handler<shmcom_gps_imu::PODMLAImu>(
        shmcom_gps_imu::PODMLAImu_shm_name, 50 * 2);
    if (!sub_shmcom_imu_->TryAttach()) {
      LOGE("subscribe shmcom %s failed", sub_imu_topic_.c_str());
      sub_shmcom_imu_.reset();
      std::abort();
      return;
    }
    sub_shmcom_imu_->Subscribe(
        [this](const shmcom_gps_imu::PODMLAImu &pod_obj) {
          if (planning_engine_ && !is_paused_.load()) {
            LOGD("feed shmcom %s", sub_imu_topic_.c_str());
            auto maf_ptr = std::make_shared<maf_gps_imu::MLAImu>();
            shmcom_gps_imu::pod2maf(pod_obj, *maf_ptr);
            planning_engine_->feed_imu_report(maf_ptr);
          }
        });
  }

  void subscribe_shmcom_chassis_report() {
    if (sub_shmcom_chassis_report_) {
      return;
    }
    sub_shmcom_chassis_report_ =
        make_shmcom_handler<shmcom_endpoint::PODChassisReport>(
            shmcom_endpoint::PODChassisReport_shm_name, 50 * 2);
    if (!sub_shmcom_chassis_report_->TryAttach()) {
      LOGE("subscribe shmcom %s failed", sub_chassis_report_topic_.c_str());
      sub_shmcom_chassis_report_.reset();
      std::abort();
      return;
    }
    sub_shmcom_chassis_report_->Subscribe(
        [this](const shmcom_endpoint::PODChassisReport &pod_obj) {
          if (planning_engine_ && !is_paused_.load()) {
            LOGD("feed shmcom %s", sub_chassis_report_topic_.c_str());
            auto maf_ptr = std::make_shared<maf_endpoint::ChassisReport>();
            shmcom_endpoint::pod2maf(pod_obj, *maf_ptr);
            planning_engine_->feed_chassis_report(maf_ptr);
          }
        });
  }

  void subscribe_shmcom_wheel_report() {
    if (sub_shmcom_wheel_report_) {
      return;
    }
    sub_shmcom_wheel_report_ =
        make_shmcom_handler<shmcom_endpoint::PODWheelReport>(
            shmcom_endpoint::PODWheelReport_shm_name, 50 * 2);
    if (!sub_shmcom_wheel_report_->TryAttach()) {
      LOGE("subscribe shmcom %s failed", sub_wheel_report_topic_.c_str());
      sub_shmcom_wheel_report_.reset();
      std::abort();
      return;
    }
    sub_shmcom_wheel_report_->Subscribe(
        [this](const shmcom_endpoint::PODWheelReport &pod_obj) {
          if (planning_engine_ && !is_paused_.load()) {
            LOGD("feed shmcom %s", sub_wheel_report_topic_.c_str());
            auto maf_ptr = std::make_shared<maf_endpoint::WheelReport>();
            shmcom_endpoint::pod2maf(pod_obj, *maf_ptr);
            planning_engine_->feed_wheel_report(maf_ptr);
          }
        });
  }

  void subscribe_shmcom_body_report() {
    if (sub_shmcom_body_report_) {
      return;
    }
    sub_shmcom_body_report_ =
        make_shmcom_handler<shmcom_endpoint::PODBodyReport>(
            shmcom_endpoint::PODBodyReport_shm_name, 10 * 2);
    if (!sub_shmcom_body_report_->TryAttach()) {
      LOGE("subscribe shmcom %s failed", sub_body_report_topic_.c_str());
      sub_shmcom_body_report_.reset();
      std::abort();
      return;
    }
    sub_shmcom_body_report_->Subscribe(
        [this](const shmcom_endpoint::PODBodyReport &pod_obj) {
          if (planning_engine_ && !is_paused_.load()) {
            LOGD("feed shmcom %s", sub_body_report_topic_.c_str());
            auto maf_ptr = std::make_shared<maf_endpoint::BodyReport>();
            shmcom_endpoint::pod2maf(pod_obj, *maf_ptr);
            planning_engine_->feed_body_report(maf_ptr);
          }
        });
  }

  void subscribe_shmcom_module_status() {
    if (sub_shmcom_module_status_) {
      return;
    }
    sub_shmcom_module_status_ =
        make_shmcom_handler<shmcom_framework_status::PODModuleStatus>(
            shmcom_framework_status::PODModuleStatus_shm_name, 10 * 2);
    if (!sub_shmcom_module_status_->TryAttach()) {
      LOGE("subscribe shmcom %s failed", module_status_topic_.c_str());
      sub_shmcom_module_status_.reset();
      std::abort();
      return;
    }
    sub_shmcom_module_status_->Subscribe(
        [this](const shmcom_framework_status::PODModuleStatus &pod_obj) {
          if (planning_engine_ && !is_paused_.load()) {
            auto maf_obj =
                std::make_shared<maf_framework_status::ModuleStatus>();
            shmcom_framework_status::pod2maf(pod_obj, *maf_obj);
            LOGD("feed shmcom %s", module_status_topic_.c_str());
            feed_maf_module_status(maf_obj);
          }
        });
  }

  void subscribe_shmcom_ego_pose() {
    if (sub_shmcom_ego_pose_) {
      return;
    }
    sub_shmcom_ego_pose_ =
        make_shmcom_handler<shmcom_mla_localization::PODMLALocalization>(
            shmcom_mla_localization::PODMLALocalization_shm_name, 50 * 2);
    if (!sub_shmcom_ego_pose_->TryAttach()) {
      LOGE("subscribe shmcom %s failed", sub_ego_pose_topic_.c_str());
      sub_shmcom_ego_pose_.reset();
      std::abort();
      return;
    }
    sub_shmcom_ego_pose_->Subscribe(
        [this](const shmcom_mla_localization::PODMLALocalization &pod_obj) {
          if (planning_engine_ && !is_paused_.load()) {
            LOGD("feed shmcom %s", sub_ego_pose_topic_.c_str());
            auto maf_ptr =
                std::make_shared<maf_mla_localization::MLALocalization>();
            shmcom_mla_localization::pod2maf(pod_obj, *maf_ptr);
            planning_engine_->feed_ego_pose(maf_ptr);
          }
        });
  }

  void subscribe_shmcom_perception_radar() {
    if (sub_shmcom_perception_radar_) {
      return;
    }
    sub_shmcom_perception_radar_ = make_shmcom_handler<
        shmcom_perception_interface::PODRadarPerceptionResult>(
        shmcom_perception_interface::PODRadarPerceptionResultFront_shm_name,
        10 * 2);
    if (!sub_shmcom_perception_radar_->TryAttach()) {
      LOGE("subscribe shmcom %s failed", sub_perception_radar_topic_.c_str());
      sub_shmcom_perception_radar_.reset();
      std::abort();
      return;
    }
    sub_shmcom_perception_radar_->Subscribe(
        [this](const shmcom_perception_interface::PODRadarPerceptionResult
                   &pod_obj) {
          if (planning_engine_ && !is_paused_.load()) {
            LOGD("feed shmcom %s", sub_perception_radar_topic_.c_str());
            auto maf_ptr = std::make_shared<
                maf_perception_interface::RadarPerceptionResult>();
            shmcom_perception_interface::pod2maf(pod_obj, *maf_ptr);
            planning_engine_->feed_perception_radar(maf_ptr);
          }
        });
  }

  void publish_shmcom_planning(const maf_planning::Planning &maf_obj) {
    if (pub_shmcom_planning_ && ptr_shmcom_planning_) {
      LOGD("publish shmcom %s", pub_planning_topic_.c_str());
      shmcom_planning::maf2pod(maf_obj, *ptr_shmcom_planning_);
      pub_shmcom_planning_->WriteNewMessage(ptr_shmcom_planning_);
    }
  }

  void advertise_shmcom_planning() {
    if (ptr_shmcom_planning_) {
      return;
    }
    pub_shmcom_planning_ = make_shmcom_handler<shmcom_planning::PODPlanning>(
        shmcom_planning::PODPlanning_shm_name, 10 * 2);
    if (!(ptr_shmcom_planning_ = pub_shmcom_planning_->TryAttach())) {
      LOGE("advertise shmcom %s failed", pub_planning_topic_.c_str());
      pub_shmcom_planning_.reset();
      std::abort();
    }
  }

  void publish_shmcom_planning_info(const maf_std::Header &maf_obj) {
    if (pub_shmcom_planning_info_ && ptr_shmcom_planning_info_) {
      LOGD("publish shmcom %s", pub_planning_info_topic_.c_str());
      shmcom_std::maf2pod(maf_obj, *ptr_shmcom_planning_info_);
      pub_shmcom_planning_info_->WriteNewMessage(ptr_shmcom_planning_info_);
    }
  }

  void advertise_shmcom_planning_info() {
    if (ptr_shmcom_planning_info_) {
      return;
    }
    pub_shmcom_planning_info_ =
        make_shmcom_handler<shmcom_std::PODPlanningInfo>(
            shmcom_std::PODPlanningInfo_shm_name, 10 * 2);
    if (!(ptr_shmcom_planning_info_ = pub_shmcom_planning_info_->TryAttach())) {
      LOGE("advertise shmcom %s failed", pub_planning_info_topic_.c_str());
      pub_shmcom_planning_info_.reset();
      std::abort();
    }
  }

#else

  void subscribe_shmcom_imu() {}
  void subscribe_shmcom_chassis_report() {}
  void subscribe_shmcom_wheel_report() {}
  void subscribe_shmcom_body_report() {}
  void subscribe_shmcom_module_status() {}
  void subscribe_shmcom_ego_pose() {}
  void subscribe_shmcom_perception_radar() {}

  void publish_shmcom_planning(const maf_planning::Planning &maf_obj) {}
  void advertise_shmcom_planning() {}
  void publish_shmcom_planning_info(const maf_std::Header &maf_obj) {}
  void advertise_shmcom_planning_info() {}

#endif // FEAT_SHMCOM_MESSAGE

  void
  subscribe_mfr_socket_imu(int32_t mode,
                           MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_imu_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_imu_topic_] =
        communication_manager
            .subscribe<mmessage::gps_imu_mfrmsgs::MFRMessageMLAImu>(sub_config);
  }

  void
  subscribe_mfr_socket_mb_imu(int32_t mode,
                              MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_mb_imu_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_mb_imu_topic_] =
        communication_manager
            .subscribe<mmessage::gps_imu_mfrmsgs::MFRMessageMLAImu>(sub_config);
  }

  void subscribe_mfr_socket_prediction(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_prediction_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_prediction_topic_] =
        communication_manager.subscribe<
            mmessage::worldmodel_mfrmsgs::MFRMessagePredictionResult>(
            sub_config);
  }

  void subscribe_mfr_socket_worldmodel_map(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_worldmodel_map_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_worldmodel_map_topic_] =
        communication_manager
            .subscribe<mmessage::worldmodel_mfrmsgs::MFRMessageProcessedMap>(
                sub_config);
  }

  void subscribe_mfr_socket_worldmodel_parking_slots(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_worldmodel_parking_slots_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_worldmodel_parking_slots_topic_] =
        communication_manager
            .subscribe<mmessage::worldmodel_mfrmsgs::MFRMessageFusionAPA>(
                sub_config);
  }

  void subscribe_mfr_socket_worldmodel_scene_object(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_worldmodel_scene_object_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_worldmodel_scene_object_topic_] =
        communication_manager
            .subscribe<mmessage::worldmodel_mfrmsgs::MFRMessageSceneObjects>(
                sub_config);
  }

  void subscribe_mfr_socket_worldmodel_objects(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_worldmodel_objects_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_worldmodel_objects_topic_] =
        communication_manager.subscribe<
            mmessage::worldmodel_mfrmsgs::MFRMessageObjectsInterface>(
            sub_config);
  }

  void subscribe_mfr_socket_traffic_light(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_traffic_light_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_traffic_light_topic_] =
        communication_manager.subscribe<mmessage::perception_interface_mfrmsgs::
                                            MFRMessageTrafficLightPerception>(
            sub_config);
  }

  void subscribe_mfr_socket_fusion_objects(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_fusion_objects_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_fusion_objects_topic_] =
        communication_manager
            .subscribe<mmessage::perception_interface_mfrmsgs::
                           MFRMessagePerceptionFusionObjectResult>(sub_config);
  }

  void subscribe_mfr_socket_fusion_groundlines(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_fusion_groundlines_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_fusion_groundlines_topic_] =
        communication_manager.subscribe<mmessage::perception_interface_mfrmsgs::
                                            MFRMessageFusionGroundLineResult>(
            sub_config);
  }

  void subscribe_mfr_socket_fusion_uss_groundlines(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_fusion_uss_groundlines_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_fusion_uss_groundlines_topic_] =
        communication_manager.subscribe<mmessage::perception_interface_mfrmsgs::
                                            MFRMessageFusionGroundLineResult>(
            sub_config);
  }

  void subscribe_mfr_socket_fusion_uss(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_fusion_uss_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_fusion_uss_topic_] =
        communication_manager.subscribe<
            mmessage::sensor_interface_mfrmsgs::MFRMessageUltrasonicUpaReport>(
            sub_config);
  }

  void subscribe_mfr_socket_ego_pose(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_ego_pose_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_ego_pose_topic_] = communication_manager.subscribe<
        mmessage::mla_localization_mfrmsgs::MFRMessageMLALocalization>(
        sub_config);
  }

  void subscribe_mfr_socket_chassis_report(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_chassis_report_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_chassis_report_topic_] =
        communication_manager
            .subscribe<mmessage::endpoint_mfrmsgs::MFRMessageChassisReport>(
                sub_config);
  }

  void subscribe_mfr_socket_wireless_charger_report(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_wireless_charger_report_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_wireless_charger_report_topic_] =
        communication_manager.subscribe<
            mmessage::endpoint_mfrmsgs::MFRMessageWirelessChargerReport>(
            sub_config);
  }

  void subscribe_mfr_socket_wheel_report(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_wheel_report_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_wheel_report_topic_] =
        communication_manager
            .subscribe<mmessage::endpoint_mfrmsgs::MFRMessageWheelReport>(
                sub_config);
  }

  void subscribe_mfr_socket_body_report(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_body_report_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_body_report_topic_] =
        communication_manager
            .subscribe<mmessage::endpoint_mfrmsgs::MFRMessageBodyReport>(
                sub_config);
  }

  void subscribe_mfr_socket_module_status(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = module_status_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[module_status_topic_] = communication_manager.subscribe<
        mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus>(sub_config);
  }

  void subscribe_mfr_socket_tasknap_slice(
      MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = tasknap_slice_topic_;
    sub_config.queue_size = 32;
    sub_config.use_shared_memory = false;
    sub_config.thread_tag = sub_union_misc_;
    // sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[tasknap_slice_topic_] = communication_manager.subscribe<
        mmessage::sensor_interface_mfrmsgs::MFRMessageCompressedVideo>(
        sub_config);
  }

  void subscribe_mfr_socket_mpc_trajecrory(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_mpc_trajecrory_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_mpc_trajecrory_topic_] =
        communication_manager.subscribe<
            mmessage::planning_mfrmsgs::MFRMessageMpcTrajectoryResult>(
            sub_config);
  }

  void subscribe_mfr_socket_sbp_result(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_sbp_result_topic_;
    sub_config.queue_size = 2;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_misc_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_sbp_result_topic_] =
        communication_manager
            .subscribe<mmessage::planning_mfrmsgs::MFRMessageSBPResult>(
                sub_config);
  }

  void subscribe_mfr_socket_planning_control_cmd_request(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_planning_control_cmd_request_topic_;
    sub_config.queue_size = 10;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_request_;
    sub_config.pause_after_startup = false;
    module_control_cmd_request_subscriber_ = communication_manager.subscribe<
        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdRequest>(
        sub_config);
  }

  void subscribe_mfr_socket_planning_request(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_planning_request_topic_;
    sub_config.queue_size = 10;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_request_;
    sub_config.pause_after_startup = !start_machine_;
    mfr_subscriber_map_[sub_planning_request_topic_] =
        communication_manager.subscribe<
            mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningRequest>(
            sub_config);
  }

  void subscribe_mfr_socket_planning_reset_request(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_planning_reset_request_topic_;
    sub_config.queue_size = 10;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_request_;
    sub_config.pause_after_startup = false;
    mfr_subscriber_map_[sub_planning_reset_request_topic_] =
        communication_manager
            .subscribe<mmessage::std_mfrmsgs::MFRMessageHeader>(sub_config);
  }

  void subscribe_mfr_socket_perception_vision_lane(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_perception_vision_lane_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = true;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    mfr_subscriber_map_[sub_perception_vision_lane_topic_] =
        communication_manager.subscribe<mmessage::perception_interface_mfrmsgs::
                                            MFRMessageRoadLinePerception>(
            sub_config);
  }

  void subscribe_mfr_socket_perception_vision_landmark(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_perception_vision_landmark_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = true;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    mfr_subscriber_map_[sub_perception_vision_landmark_topic_] =
        communication_manager.subscribe<mmessage::perception_interface_mfrmsgs::
                                            MFRMessageRoadLinePerception>(
            sub_config);
  }

  void subscribe_mfr_socket_perception_lidar_road_edge(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_perception_lidar_road_edge_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    mfr_subscriber_map_[sub_perception_lidar_road_edge_topic_] =
        communication_manager.subscribe<mmessage::perception_interface_mfrmsgs::
                                            MFRMessageRoadLinePerception>(
            sub_config);
  }

  void subscribe_mfr_socket_perception_radar(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_perception_radar_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    mfr_subscriber_map_[sub_perception_radar_topic_] =
        communication_manager.subscribe<mmessage::perception_interface_mfrmsgs::
                                            MFRMessageRadarPerceptionResult>(
            sub_config);
  }

  void subscribe_mfr_socket_mff_info(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_mff_info_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_union_prception_prediction_ego_pose_;
    mfr_subscriber_map_[sub_mff_info_topic_] =
        communication_manager
            .subscribe<mmessage::std_mfrmsgs::MFRMessageHeader>(sub_config);
  }

  void subscribe_mfr_socket_planning_frequency_control(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_planning_frequency_control_topic_;
    sub_config.queue_size = 5;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_request_;
    mfr_subscriber_map_[sub_planning_frequency_control_topic_] =
        communication_manager.subscribe<
            mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningRequest>(
            sub_config);
  }

  void subscribe_mfr_socket_planning_reload_request(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_planning_reload_request_topic_;
    sub_config.queue_size = 2;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_request_;
    mfr_subscriber_map_[sub_planning_reload_request_topic_] =
        communication_manager
            .subscribe<mmessage::std_mfrmsgs::MFRMessageHeader>(sub_config);
  }

  void subscribe_mfr_socket_simulation_query(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRSubscriberConfig sub_config{};
    sub_config.topic_name = sub_simulation_query_topic_;
    sub_config.queue_size = 2;
    sub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    sub_config.thread_tag = sub_request_;
    mfr_subscriber_map_[sub_simulation_query_topic_] =
        communication_manager
            .subscribe<mmessage::std_mfrmsgs::MFRMessageHeader>(sub_config);
  }

  void
  publish_mfr_socket_planning(int32_t mode,
                              MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_planning_topic_;
    pub_config.queue_size = 10;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    pub_config.pause_after_startup = !start_machine_;
    mfr_publisher_map_[pub_planning_topic_] =
        communication_manager
            .advertise<mmessage::planning_mfrmsgs::MFRMessagePlanning>(
                pub_config);
  }

  void publish_mfr_socket_planning_info(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_planning_info_topic_;
    pub_config.queue_size = 10;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    mfr_publisher_map_[pub_planning_info_topic_] =
        communication_manager
            .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
  }

  void
  publish_mfr_socket_mdebug(int32_t mode,
                            MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_mdebug_topic_;
    pub_config.queue_size = 10;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    mfr_publisher_map_[pub_mdebug_topic_] =
        communication_manager
            .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
  }

  void publish_mfr_socket_nodes_status(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_nodes_status_topic_;
    pub_config.queue_size = 10;
    pub_config.pause_after_startup = false;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    node_status_publisher_ = communication_manager.advertise<
        mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus>(pub_config);
  }

  void publish_mfr_socket_sbp_request_mode(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_sbp_request_topic_;
    pub_config.queue_size = 10;
    pub_config.pause_after_startup = !start_machine_;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    mfr_publisher_map_[pub_sbp_request_topic_] =
        communication_manager
            .advertise<mmessage::planning_mfrmsgs::MFRMessageSBPRequest>(
                pub_config);
  }

  void publish_mfr_planning_control_cmd_response(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_planning_control_cmd_response_topic_;
    pub_config.queue_size = 10;
    pub_config.pause_after_startup = false;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    module_control_cmd_response_publisher_ = communication_manager.advertise<
        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdResponse>(
        pub_config);
  }

  void publish_mfr_planning_response(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_planning_response_topic_;
    pub_config.queue_size = 10;
    pub_config.pause_after_startup = !start_machine_;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    mfr_publisher_map_[pub_planning_response_topic_] =
        communication_manager.advertise<
            mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningResponse>(
            pub_config);
  }

  void
  publish_mfr_tasknap_slice(MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = tasknap_slice_topic_;
    pub_config.queue_size = 32;
    pub_config.pause_after_startup = !start_machine_;
    pub_config.use_shared_memory = false;
    mfr_publisher_map_[tasknap_slice_topic_] = communication_manager.advertise<
        mmessage::sensor_interface_mfrmsgs::MFRMessageCompressedVideo>(
        pub_config);
  }

  void publish_mfr_planning_reset_response(
      int32_t mode, MFRCommunicationManager &communication_manager) {
    MFRPublisherConfig pub_config{};
    pub_config.topic_name = pub_planning_reset_response_topic_;
    pub_config.queue_size = 10;
    pub_config.pause_after_startup = !start_machine_;
    pub_config.use_shared_memory = mode & MSG_MODE_MFR_SHM;
    mfr_publisher_map_[pub_planning_reset_response_topic_] =
        communication_manager
            .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
  }

  void run_planning_engine() {
    if (is_paused_.load()) {
      is_paused_.store(false);
    }
    planning_engine_->keep_run();
  }

  void stop_planning_engine() {
    if (!is_paused_.load()) {
      is_paused_.store(true);
    }
    planning_engine_->keep_stop();
  }
};
// MFR_REGISTER_NODE(PlanningMFRNode, "planning_mfr_node_type");
MFR_REGISTER_NODE_WITH_VERSION(PlanningMFRNode, "apa_planning_mfr_node_type",
                               "2022.0126.0");
