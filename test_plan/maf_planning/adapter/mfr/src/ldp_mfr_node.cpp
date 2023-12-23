#include "mfr/mfr.h"
#include "nlohmann/json.hpp"
#include "pnc.h"

#include "argparse.hpp"
#include "mjson/mjson.hpp"
#include "mlog_msg_id/mlog_msg_id.hpp"
#include "mlog_publisher/publisher.h"
#include "mtime_core/mtime.h"
#include "timeline/timeline_mfr.h"

#include "endpoint_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "framework_status_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "perception_interface_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "planning_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "pnc/ldp_planning_engine_interface.h"
#include "std_mfrmsgs/mfr_cpp_struct_convert.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <map>
#include <thread>

#define LOG_PREFIX "LDPPlanning"
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
using namespace msd_planning;
using namespace msquare;

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

class LdpMFRNode : public MFRNode {
public:
  bool on_init(MFRNodeHandle *node_handle) override {
    // get param
    auto param_reader = mjson::Reader(node_handle->node_param_json().c_str());

    // setup local log level
    SETUP_LOG_UTIL(mjson::Reader(
        param_reader.get<mjson::Json>("log_config", false, mjson::Json())));
    LOGI("on_init begin");

    MFString config_file = param_reader.get<std::string>("config_file").c_str();
    MFString mtaskflow_config_file =
        param_reader.get<std::string>("mtaskflow_config_file").c_str();
    MFString config_file_dir =
        param_reader.get<std::string>("config_file_dir").c_str();

    sub_chassis_report_topic_ =
        param_reader.get<std::string>("sub_chassis_report_topic").c_str();
    sub_wheel_report_topic_ =
        param_reader.get<std::string>("sub_wheel_report_topic").c_str();
    sub_body_report_topic_ =
        param_reader.get<std::string>("sub_body_report_topic").c_str();
    sub_perception_vision_lane_topic_ =
        param_reader.get<std::string>("sub_perception_vision_lane").c_str();
    sub_perception_fusion_topic_ =
        param_reader.get<std::string>("sub_perception_fusion_topic").c_str();
    sub_perception_fusion_aeb_topic_ =
        param_reader.get<std::string>("sub_perception_fusion_aeb_topic")
            .c_str();

    module_status_topic_ =
        param_reader.get<std::string>("module_status_topic").c_str();
    pub_nodes_status_topic_ =
        param_reader.get<std::string>("pub_nodes_status_topic").c_str();

    pub_planning_topic_ =
        param_reader.get<std::string>("pub_planning_topic").c_str();
    pub_planning_ldp_topic_ =
        param_reader.get<std::string>("pub_planning_ldp_topic").c_str();

    pub_ess_planning_topic_ =
        param_reader.get<std::string>("pub_ess_planning_topic").c_str();
    pub_planning_ess_topic_ =
        param_reader.get<std::string>("pub_planning_ess_topic").c_str();

    sub_mff_planning_request_topic_ =
        param_reader.get<std::string>("sub_mff_planning_request_topic").c_str();
    pub_mff_planning_response_topic_ =
        param_reader.get<std::string>("pub_mff_planning_response_topic")
            .c_str();

    sub_mff_planning_ess_request_topic_ =
        param_reader.get<std::string>("sub_mff_planning_ess_request_topic")
            .c_str();
    pub_mff_planning_ess_response_topic_ =
        param_reader.get<std::string>("pub_mff_planning_ess_response_topic")
            .c_str();

    sub_mpc_trajecrory_topic_ =
        param_reader.get<std::string>("sub_mpc_trajecrory_topic").c_str();

    LOGI("on_init load config data");

    std::cout << param_reader.raw().dump() << std::endl;

    // init mlog and mtime
    mlog::PublisherSettings publisher_settings;
    publisher_settings.handle = node_handle;
    publisher_settings.config_file = config_file.c_str();
    auto publisher = mlog::MLogPublisher::make(publisher_settings);

    std::string config_data = read_file(config_file).c_str();
    init_config_data(config_data);

    mlog::MLogEndpointCallback planning_callback = nullptr;
    if (publisher) {
      planning_callback =
          [publisher](const std::vector<mlog::MLogEntry> &entries) {
            publisher->publish(entries);
          };
    }

    MSDPlanning_set_log_config(config_data, node_handle);
    if (planning_callback) {
      MSDPlanning_set_log_callback(planning_callback);
    }
    MSDPlanning_init_log_manager();

    MSDPlanningTimeConfig planning_time_config{};
    planning_time_config.basic_config = config_data;
    mtime::MTimeConfig time_config{};
    time_config.node_handle = node_handle;
    planning_time_config.time_config = time_config;
    // init time
    MSDPlanning_set_time_config(planning_time_config, feeder_);

    LOGI("on_init init engine");

    MSDLdpPlanningConfig planning_config{};
    planning_config.enable_timer_tick = true;
    planning_config.config_file_dir = config_file_dir.c_str();
    planning_config.mtaskflow_config_file = mtaskflow_config_file.c_str();
    planning_engine_ = LdpPlanningEngineInterface::make(planning_config);

    (void)planning_engine_->init();

    if (start_machine_) {
      LOGI("on_init start engine");

      is_paused_.store(false);
      is_engine_paused_.store(false);
      (void)planning_engine_->start();
    }

    setup_msg_modes();

    LOGI("on_init subscribe messages");

    // get subscriber
    auto &communication_manager = node_handle->communication_manager();
    {
      MFRSubscriberConfig sub_config{};
      if (HAS_MSG_MODE(sub_chassis_report_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_chassis_report_topic_.c_str());
        sub_config.topic_name = sub_chassis_report_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            (sub_chassis_report_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_chassis_report_topic_] =
            communication_manager
                .subscribe<mmessage::endpoint_mfrmsgs::MFRMessageChassisReport>(
                    sub_config);
      }
      if (HAS_MSG_MODE(sub_chassis_report_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_chassis_report_topic_.c_str());
        subscribe_shmcom_chassis_report();
      }

      if (HAS_MSG_MODE(sub_wheel_report_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_wheel_report_topic_.c_str());
        sub_config.topic_name = sub_wheel_report_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            (sub_wheel_report_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_wheel_report_topic_] =
            communication_manager
                .subscribe<mmessage::endpoint_mfrmsgs::MFRMessageWheelReport>(
                    sub_config);
      }
      if (HAS_MSG_MODE(sub_wheel_report_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_wheel_report_topic_.c_str());
        subscribe_shmcom_wheel_report();
      }

      if (HAS_MSG_MODE(sub_body_report_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_body_report_topic_.c_str());
        sub_config.topic_name = sub_body_report_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            (sub_body_report_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_body_report_topic_] =
            communication_manager
                .subscribe<mmessage::endpoint_mfrmsgs::MFRMessageBodyReport>(
                    sub_config);
      }
      if (HAS_MSG_MODE(sub_body_report_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", sub_body_report_topic_.c_str());
        subscribe_shmcom_body_report();
      }

      if (HAS_MSG_MODE(sub_perception_vision_lane_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_perception_vision_lane_topic_.c_str());
        sub_config.topic_name = sub_perception_vision_lane_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            (sub_perception_vision_lane_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_perception_vision_lane_topic_] =
            communication_manager
                .subscribe<mmessage::perception_interface_mfrmsgs::
                               MFRMessageRoadLinePerception>(sub_config);
      }

      if (HAS_MSG_MODE(sub_perception_fusion_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_perception_fusion_topic_.c_str());
        sub_config.topic_name = sub_perception_fusion_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            (sub_perception_fusion_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_perception_fusion_topic_] =
            communication_manager
                .subscribe<mmessage::perception_interface_mfrmsgs::
                               MFRMessagePerceptionFusionObjectResult>(
                    sub_config);
      }

      if (HAS_MSG_MODE(sub_perception_fusion_aeb_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_perception_fusion_aeb_topic_.c_str());
        sub_config.topic_name = sub_perception_fusion_aeb_topic_;
        sub_config.queue_size = 5;
#if defined(__QNX__)
        sub_config.use_shared_memory = true;
#endif
        mfr_subscriber_map_[sub_perception_fusion_aeb_topic_] =
            communication_manager
                .subscribe<mmessage::perception_interface_mfrmsgs::
                               MFRMessagePerceptionFusionAEBResult>(sub_config);
      }

      if (HAS_MSG_MODE(sub_module_status_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", module_status_topic_.c_str());
        sub_config.topic_name = module_status_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            (sub_module_status_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[module_status_topic_] =
            communication_manager.subscribe<
                mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus>(
                sub_config);
      }
      if (HAS_MSG_MODE(sub_module_status_mode_, MSG_MODE_PE_SHM)) {
        LOGI("subscribe shmcom %s", module_status_topic_.c_str());
        subscribe_shmcom_module_status();
      }

      if (HAS_MSG_MODE(sub_mff_planning_request_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_mff_planning_request_topic_.c_str());
        sub_config.topic_name = sub_mff_planning_request_topic_;
        sub_config.queue_size = 10;
        sub_config.use_shared_memory =
            (sub_mff_planning_request_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_mff_planning_request_topic_] =
            communication_manager
                .subscribe<mmessage::std_mfrmsgs::MFRMessageHeader>(sub_config);
      }

      if (HAS_MSG_MODE(sub_mff_planning_ess_request_mode_,
                       (MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM))) {
        LOGI("subscribe mfr %s", sub_mff_planning_ess_request_topic_.c_str());
        sub_config.topic_name = sub_mff_planning_ess_request_topic_;
        sub_config.queue_size = 10;
        sub_config.use_shared_memory =
            (sub_mff_planning_ess_request_mode_ & MSG_MODE_MFR_SHM);
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_mff_planning_ess_request_topic_] =
            communication_manager
                .subscribe<mmessage::std_mfrmsgs::MFRMessageHeader>(sub_config);
      }

      // mpc trajectory
      if (HAS_MSG_MODE(sub_mpc_trajecrory_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("subscribe mfr %s", sub_mpc_trajecrory_topic_.c_str());
        sub_config.topic_name = sub_mpc_trajecrory_topic_;
        sub_config.queue_size = 5;
        sub_config.use_shared_memory =
            sub_mpc_trajecrory_mode_ & MSG_MODE_MFR_SHM;
        sub_config.thread_tag = sub_union_topic_;
        mfr_subscriber_map_[sub_mpc_trajecrory_topic_] =
            communication_manager.subscribe<
                mmessage::planning_mfrmsgs::MFRMessageMpcTrajectoryResult>(
                sub_config);
      }
    }

    LOGI("on_init advertise messages");

    // get publisher
    {
      MFRPublisherConfig pub_config{};
      if (HAS_MSG_MODE(pub_planning_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_planning_topic_.c_str());
        pub_config.topic_name = pub_planning_topic_;
        pub_config.queue_size = 10;
        pub_config.shared_memory_block_size = 980 * 1024;
        pub_config.use_shared_memory = (pub_planning_mode_ & MSG_MODE_MFR_SHM);
        mfr_publisher_map_[pub_planning_topic_] =
            communication_manager
                .advertise<mmessage::planning_mfrmsgs::MFRMessagePlanning>(
                    pub_config);
      }
      if (HAS_MSG_MODE(pub_planning_mode_, MSG_MODE_PE_SHM)) {
        LOGI("advertise shmcom %s", pub_planning_topic_.c_str());
        advertise_shmcom_planning();
      }
      pub_config.shared_memory_block_size = 0;

      if (HAS_MSG_MODE(pub_planning_ldp_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_planning_ldp_topic_.c_str());
        pub_config.topic_name = pub_planning_ldp_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            (pub_planning_ldp_mode_ & MSG_MODE_MFR_SHM);
        mfr_publisher_map_[pub_planning_ldp_topic_] =
            communication_manager
                .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
      }

      if (HAS_MSG_MODE(pub_ess_planning_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_ess_planning_topic_.c_str());
        pub_config.topic_name = pub_ess_planning_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            (pub_ess_planning_mode_ & MSG_MODE_MFR_SHM);
        mfr_publisher_map_[pub_ess_planning_topic_] =
            communication_manager
                .advertise<mmessage::planning_mfrmsgs::MFRMessagePlanning>(
                    pub_config);
      }

      if (HAS_MSG_MODE(pub_planning_ess_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_planning_ess_topic_.c_str());
        pub_config.topic_name = pub_planning_ess_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            (pub_planning_ess_mode_ & MSG_MODE_MFR_SHM);
        mfr_publisher_map_[pub_planning_ess_topic_] =
            communication_manager
                .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
      }

      if (HAS_MSG_MODE(pub_nodes_status_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_nodes_status_topic_.c_str());
        pub_config.topic_name = pub_nodes_status_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            (pub_nodes_status_mode_ & MSG_MODE_MFR_SHM);
        mfr_publisher_map_[pub_nodes_status_topic_] =
            communication_manager.advertise<
                mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus>(
                pub_config);
      }

      if (HAS_MSG_MODE(pub_mff_planning_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_mff_planning_response_topic_.c_str());
        pub_config.topic_name = pub_mff_planning_response_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            (pub_mff_planning_response_mode_ & MSG_MODE_MFR_SHM);
        mfr_publisher_map_[pub_mff_planning_response_topic_] =
            communication_manager
                .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
      }

      if (HAS_MSG_MODE(pub_mff_planning_ess_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGI("advertise mfr %s", pub_mff_planning_ess_response_topic_.c_str());
        pub_config.topic_name = pub_mff_planning_ess_response_topic_;
        pub_config.queue_size = 10;
        pub_config.use_shared_memory =
            (pub_mff_planning_ess_response_mode_ & MSG_MODE_MFR_SHM);
        mfr_publisher_map_[pub_mff_planning_ess_response_topic_] =
            communication_manager
                .advertise<mmessage::std_mfrmsgs::MFRMessageHeader>(pub_config);
      }
    }

    LOGI("on_init set callbacks");

    auto &pub_planning = mfr_publisher_map_[pub_planning_topic_];
    planning_engine_->set_callback(
        [&pub_planning, this](const MSDPlanningOutputMeta &meta,
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

              MSDPlanning_record_relation(
                  mlog::MLOG_msg_id(planning_result.header.stamp, kLdpPlanTag),
                  trigger_msg_id);

              pub_planning->publish(mfr_planning);
            }
            if (HAS_MSG_MODE(pub_planning_mode_, MSG_MODE_PE_SHM)) {
              LOGD("publish shmcom %s", pub_planning_topic_.c_str());
              publish_shmcom_planning(planning_result);
            }
          }
        });

    auto &pub_ldp_status = mfr_publisher_map_[pub_planning_ldp_topic_];
    planning_engine_->set_callback([&pub_ldp_status,
                                    this](const std::string &ldp_output) {
      maf_std::Header maf_ldp_status{};
      maf_ldp_status.stamp = MTIME()->timestamp().ns();
      maf_ldp_status.frame_id = ldp_output;
      if (HAS_MSG_MODE(pub_planning_ldp_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_planning_ldp_topic_.c_str());
        mmessage::std_mfrmsgs::MFRMessageHeader ldp_status_mfrmsg{};
        ldp_status_mfrmsg.get_internal_data()->set_original_data(
            &maf_ldp_status);
        ldp_status_mfrmsg.get_internal_data()->set_serialize_preprocessor(
            [&maf_ldp_status,        // parasoft-suppress AUTOSAR-A5_1_4
             &ldp_status_mfrmsg]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(maf_ldp_status, ldp_status_mfrmsg);
            });
        pub_ldp_status->publish(ldp_status_mfrmsg);
      }
    });

    auto &pub_ess_planning = mfr_publisher_map_[pub_ess_planning_topic_];
    planning_engine_->set_ess_callback(
        [&pub_ess_planning, this](const MSDPlanningOutputMeta &meta,
                                  const maf_planning::Planning &planning_result,
                                  const std::string &trigger_msg_id) {
          if (meta.succeed) {
            if (HAS_MSG_MODE(pub_ess_planning_mode_,
                             MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
              LOGD("publish mfr %s", pub_ess_planning_topic_.c_str());
              mmessage::planning_mfrmsgs::MFRMessagePlanning mfr_planning{};
              mfr_planning.get_internal_data()->set_original_data(
                  &planning_result);
              mfr_planning.get_internal_data()->set_serialize_preprocessor(
                  [&planning_result,  // parasoft-suppress AUTOSAR-A5_1_4
                   &mfr_planning]() { // parasoft-suppress AUTOSAR-A5_1_4
                    mfr_cpp_struct_convert::to_mfr(planning_result,
                                                   mfr_planning);
                  });

              MSDPlanning_record_relation(
                  mlog::MLOG_msg_id(planning_result.header.stamp, kLdpPlanTag),
                  trigger_msg_id);

              pub_ess_planning->publish(mfr_planning);
            }
            if (HAS_MSG_MODE(pub_ess_planning_mode_, MSG_MODE_PE_SHM)) {
              LOGD("publish shmcom %s", pub_ess_planning_topic_.c_str());
              publish_shmcom_planning(planning_result);
            }
          }
        });

    auto &pub_ess_status = mfr_publisher_map_[pub_planning_ess_topic_];
    planning_engine_->set_ess_callback([&pub_ess_status,
                                        this](const std::string &ess_output) {
      maf_std::Header maf_ess_status{};
      maf_ess_status.stamp = MTIME()->timestamp().ns();
      maf_ess_status.frame_id = ess_output;
      if (HAS_MSG_MODE(pub_planning_ess_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        LOGD("publish mfr %s", pub_planning_ess_topic_.c_str());
        mmessage::std_mfrmsgs::MFRMessageHeader ess_status_mfrmsg{};
        ess_status_mfrmsg.get_internal_data()->set_original_data(
            &maf_ess_status);
        ess_status_mfrmsg.get_internal_data()->set_serialize_preprocessor(
            [&maf_ess_status,        // parasoft-suppress AUTOSAR-A5_1_4
             &ess_status_mfrmsg]() { // parasoft-suppress AUTOSAR-A5_1_4
              mfr_cpp_struct_convert::to_mfr(maf_ess_status, ess_status_mfrmsg);
            });
        pub_ess_status->publish(ess_status_mfrmsg);
      }
    });

    auto &pub_nodes_status = mfr_publisher_map_[pub_nodes_status_topic_];
    planning_engine_->set_callback([&pub_nodes_status,
                                    this](const maf_framework_status::NodeStatus
                                              &planning_node_status) {
      maf_framework_status::NodesStatus planning_nodes_status{};
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
    });

    return true;
  }
  void on_finish() override {}
  void on_running(const MFRNodeRunningInfo &info) override {
    if (info.trigger == MFR_NODE_TRIGGER_MESSAGE) {
      if (info.topic_name == sub_chassis_report_topic_) {
        chassis_report_callback();
      } else if (info.topic_name == sub_wheel_report_topic_) {
        wheel_report_callback();
      } else if (info.topic_name == sub_body_report_topic_) {
        body_report_callback();
      } else if (info.topic_name == sub_perception_vision_lane_topic_) {
        perception_vision_lane_callback();
      } else if (info.topic_name == sub_perception_fusion_topic_) {
        perception_fusion_callback();
      } else if (info.topic_name == sub_perception_fusion_aeb_topic_) {
        perception_fusion_aeb_callback();
      } else if (info.topic_name == module_status_topic_) {
        module_status_callback();
      } else if (info.topic_name == sub_mff_planning_request_topic_) {
        mff_planning_request_callback();
      } else if (info.topic_name == sub_mff_planning_ess_request_topic_) {
        mff_planning_ess_request_callback();
      } else if (info.topic_name == sub_mpc_trajecrory_topic_) {
        mpc_trajectory_callback();
      }

      // check stop delay
      double current_time = MTIME()->timestamp().sec();
      if (last_stop_time_ < 0) {
        last_stop_time_ = current_time;
      }
      if (is_paused_.load()) {
        if (current_time - last_stop_time_ > stop_delay_time_ &&
            !is_engine_paused_.load()) {
          (void)planning_engine_->stop();
          is_engine_paused_.store(true);
        }
      } else {
        last_stop_time_ = current_time;
      }
    }
  }

private:
  std::map<MFString, MFRSubscriber *> mfr_subscriber_map_;
  std::map<MFString, MFRPublisher *> mfr_publisher_map_;

  std::shared_ptr<LdpPlanningEngineInterface> planning_engine_;

  bool start_machine_ = false;
  std::atomic_bool is_paused_{true};
  std::atomic_bool is_engine_paused_{true};
  const double stop_delay_time_ = 1.0; // 1.0s delay for stop
  double last_stop_time_ = -1.0;

  MFString sub_union_topic_ = "union_topic";

  MFString sub_chassis_report_topic_;
  MFString sub_wheel_report_topic_;
  MFString sub_body_report_topic_;
  MFString sub_perception_vision_lane_topic_;
  MFString sub_perception_fusion_topic_;
  MFString sub_perception_fusion_aeb_topic_;

  MFString module_status_topic_;
  MFString pub_nodes_status_topic_;

  MFString pub_planning_topic_;
  MFString pub_planning_ldp_topic_;

  MFString pub_ess_planning_topic_;
  MFString pub_planning_ess_topic_;

  MFString sub_mff_planning_request_topic_;
  MFString pub_mff_planning_response_topic_;

  MFString sub_mff_planning_ess_request_topic_;
  MFString pub_mff_planning_ess_response_topic_;

  MFString sub_mpc_trajecrory_topic_;

  // messages' transfer mode
  int32_t sub_chassis_report_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_wheel_report_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_body_report_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_vision_lane_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_fusion_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_perception_fusion_aeb_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_module_status_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_mff_planning_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_mff_planning_ess_request_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t sub_mpc_trajecrory_mode_ = MSG_MODE_MFR_SOCKET;

  // int32_t pub_module_status_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_nodes_status_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_ldp_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_ess_planning_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_planning_ess_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_mff_planning_response_mode_ = MSG_MODE_MFR_SOCKET;
  int32_t pub_mff_planning_ess_response_mode_ = MSG_MODE_MFR_SOCKET;

  mtime::MTimeClockFeeder feeder_;

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

    start_machine_ = config_reader.get<bool>("start_machine", false, false);
  }

  void module_status_callback() {
    while (mfr_subscriber_map_[module_status_topic_]->empty() == false) {
      auto mfr_module_status =
          mfr_subscriber_map_[module_status_topic_]
              ->pop<
                  mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus>();
      if (is_engine_paused_.load()) {
        continue;
      }

      auto maf_module_status =
          mfr_module_status.get_internal_data()
              ->get_original_data<maf_framework_status::ModuleStatus>();
      if (maf_module_status == nullptr) {
        maf_module_status =
            mmemory::MFMakeShared<maf_framework_status::ModuleStatus>();
        mfr_cpp_struct_convert::from_mfr(mfr_module_status, *maf_module_status);
      }

      LOGD("feed mfr %s", module_status_topic_.c_str());
      (void)feed_maf_module_status(maf_module_status);
    }
  }

  void mff_planning_request_callback() {
    while (mfr_subscriber_map_[sub_mff_planning_request_topic_]->empty() ==
           false) {
      auto mfr_mff_planning_request =
          mfr_subscriber_map_[sub_mff_planning_request_topic_]
              ->pop<mmessage::std_mfrmsgs::MFRMessageHeader>();

      auto request = mfr_mff_planning_request.get_internal_data()
                         ->get_original_data<maf_std::Header>();
      if (request == nullptr) {
        request = mmemory::MFMakeShared<maf_std::Header>();
        mfr_cpp_struct_convert::from_mfr(mfr_mff_planning_request, *request);
      }

      // send response
      maf_std::Header maf_response{};
      maf_response.stamp = MTIME()->timestamp().ns();

      auto response_json = mjson::Json(mjson::Json::object());
      response_json["request_seq"] = mjson::Json((*request).seq);
      response_json["success"] = mjson::Json(true);
      maf_response.frame_id = response_json.dump();

      if (HAS_MSG_MODE(pub_mff_planning_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        mmessage::std_mfrmsgs::MFRMessageHeader response_mfrmsg{};
        response_mfrmsg.get_internal_data()->set_original_data(&maf_response);
        response_mfrmsg.get_internal_data()->set_serialize_preprocessor(
            [&maf_response, &response_mfrmsg]() {
              mfr_cpp_struct_convert::to_mfr(maf_response, response_mfrmsg);
            });
        LOGD("publish mfr %s", pub_mff_planning_response_topic_.c_str());
        mfr_publisher_map_[pub_mff_planning_response_topic_]->publish(
            response_mfrmsg);
      }

      (void)feed_maf_mff_planning_request(request);
      if (is_engine_paused_.load()) {
        continue;
      }

      planning_engine_->feed_mff_planning_request((*request).frame_id);

      LOGD("feed mfr %s", sub_mff_planning_request_topic_.c_str());
    }
  }

  void mff_planning_ess_request_callback() {
    while (mfr_subscriber_map_[sub_mff_planning_ess_request_topic_]->empty() ==
           false) {
      auto mfr_mff_planning_request =
          mfr_subscriber_map_[sub_mff_planning_ess_request_topic_]
              ->pop<mmessage::std_mfrmsgs::MFRMessageHeader>();

      auto request = mfr_mff_planning_request.get_internal_data()
                         ->get_original_data<maf_std::Header>();
      if (request == nullptr) {
        request = mmemory::MFMakeShared<maf_std::Header>();
        mfr_cpp_struct_convert::from_mfr(mfr_mff_planning_request, *request);
      }

      // send response
      maf_std::Header maf_response{};
      maf_response.stamp = MTIME()->timestamp().ns();

      auto response_json = mjson::Json(mjson::Json::object());
      response_json["request_seq"] = mjson::Json((*request).seq);
      response_json["success"] = mjson::Json(true);
      maf_response.frame_id = response_json.dump();

      if (HAS_MSG_MODE(pub_mff_planning_response_mode_,
                       MSG_MODE_MFR_SOCKET | MSG_MODE_MFR_SHM)) {
        mmessage::std_mfrmsgs::MFRMessageHeader response_mfrmsg{};
        response_mfrmsg.get_internal_data()->set_original_data(&maf_response);
        response_mfrmsg.get_internal_data()->set_serialize_preprocessor(
            [&maf_response, &response_mfrmsg]() {
              mfr_cpp_struct_convert::to_mfr(maf_response, response_mfrmsg);
            });
        LOGD("publish mfr %s", pub_mff_planning_ess_response_topic_.c_str());
        mfr_publisher_map_[pub_mff_planning_ess_response_topic_]->publish(
            response_mfrmsg);
      }

      (void)feed_maf_mff_planning_request(request);
      if (is_engine_paused_.load()) {
        continue;
      }

      planning_engine_->feed_mff_planning_request((*request).frame_id);

      LOGD("feed mfr %s", sub_mff_planning_ess_request_topic_.c_str());
    }
  }

  void mpc_trajectory_callback() {
    while (mfr_subscriber_map_[sub_mpc_trajecrory_topic_]->empty() == false) {
      auto mfr_sub_mpc_trajectory =
          mfr_subscriber_map_[sub_mpc_trajecrory_topic_]
              ->pop<
                  mmessage::planning_mfrmsgs::MFRMessageMpcTrajectoryResult>();
      if (is_engine_paused_.load()) {
        continue;
      }

      auto maf_mpc_trajectory =
          mfr_sub_mpc_trajectory.get_internal_data()
              ->get_original_data<maf_planning::MpcTrajectoryResult>();
      if (maf_mpc_trajectory == nullptr) {
        maf_mpc_trajectory =
            mmemory::MFMakeShared<maf_planning::MpcTrajectoryResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_sub_mpc_trajectory,
                                         *maf_mpc_trajectory);
      }

      MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_mpc_trajectory).header.stamp,
                            kMpcTrajectorydTag),
          MSDPlanning_msg_tag(kMpcTrajectorydTag), 0);

      (void)mfr_subscriber_map_[sub_mpc_trajecrory_topic_]->record_delay_since(
          maf_mpc_trajectory->header.stamp);
      LOGD("feed mfr %s", sub_mpc_trajecrory_topic_.c_str());
      planning_engine_->feed_mpc_trajectory(maf_mpc_trajectory);
    }
  }

  void chassis_report_callback() {
    while (mfr_subscriber_map_[sub_chassis_report_topic_]->empty() == false) {
      auto mfr_chassis_report =
          mfr_subscriber_map_[sub_chassis_report_topic_]
              ->pop<mmessage::endpoint_mfrmsgs::MFRMessageChassisReport>();
      if (is_engine_paused_.load()) {
        continue;
      }

      auto maf_chassis_report =
          mfr_chassis_report.get_internal_data()
              ->get_original_data<maf_endpoint::ChassisReport>();
      if (maf_chassis_report == nullptr) {
        maf_chassis_report =
            mmemory::MFMakeShared<maf_endpoint::ChassisReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_chassis_report,
                                         *maf_chassis_report);
      }

      MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_chassis_report).header.stamp,
                            kChassisReportTag),
          MSDPlanning_msg_tag(kChassisReportTag), 0);

      LOGD("feed mfr %s", sub_chassis_report_topic_.c_str());
      planning_engine_->feed_chassis_report(maf_chassis_report);
    }
  }

  void wheel_report_callback() {
    while (mfr_subscriber_map_[sub_wheel_report_topic_]->empty() == false) {
      auto mfr_wheel_report =
          mfr_subscriber_map_[sub_wheel_report_topic_]
              ->pop<mmessage::endpoint_mfrmsgs::MFRMessageWheelReport>();
      if (is_engine_paused_.load()) {
        continue;
      }

      auto maf_wheel_report =
          mfr_wheel_report.get_internal_data()
              ->get_original_data<maf_endpoint::WheelReport>();
      if (maf_wheel_report == nullptr) {
        maf_wheel_report = mmemory::MFMakeShared<maf_endpoint::WheelReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_wheel_report, *maf_wheel_report);
      }

      MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_wheel_report).header.stamp, kWheelReportTag),
          MSDPlanning_msg_tag(kWheelReportTag), 0);

      LOGD("feed mfr %s", sub_wheel_report_topic_.c_str());
      planning_engine_->feed_wheel_report(maf_wheel_report);
    }
  }

  void body_report_callback() {
    while (mfr_subscriber_map_[sub_body_report_topic_]->empty() == false) {
      auto mfr_body_report =
          mfr_subscriber_map_[sub_body_report_topic_]
              ->pop<mmessage::endpoint_mfrmsgs::MFRMessageBodyReport>();
      if (is_engine_paused_.load()) {
        continue;
      }

      auto maf_body_report =
          mfr_body_report.get_internal_data()
              ->get_original_data<maf_endpoint::BodyReport>();
      if (maf_body_report == nullptr) {
        maf_body_report = mmemory::MFMakeShared<maf_endpoint::BodyReport>();
        mfr_cpp_struct_convert::from_mfr(mfr_body_report, *maf_body_report);
      }

      MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_body_report).header.stamp, kBodyReportTag),
          MSDPlanning_msg_tag(kBodyReportTag), 0);

      LOGD("feed mfr %s", sub_body_report_topic_.c_str());
      planning_engine_->feed_body_report(maf_body_report);
    }
  }

  void perception_vision_lane_callback() {
    while (mfr_subscriber_map_[sub_perception_vision_lane_topic_]->empty() ==
           false) {
      auto mfr_perception_vision_lane =
          mfr_subscriber_map_[sub_perception_vision_lane_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessageRoadLinePerception>();
      if (is_engine_paused_.load()) {
        continue;
      }

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

      MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_perception_vision_lane).header.stamp,
                            kPerceptionVisionLaneTag),
          MSDPlanning_msg_tag(kPerceptionVisionLaneTag), 0);

      LOGD("feed mfr %s", sub_perception_vision_lane_topic_.c_str());
      planning_engine_->feed_perception_vision_lane(maf_perception_vision_lane);
    }
  }

  void perception_fusion_callback() {
    while (mfr_subscriber_map_[sub_perception_fusion_topic_]->empty() ==
           false) {
      auto mfr_perception_fusion =
          mfr_subscriber_map_[sub_perception_fusion_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessagePerceptionFusionObjectResult>();
      if (is_engine_paused_.load()) {
        continue;
      }

      auto maf_perception_fusion =
          mfr_perception_fusion.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::PerceptionFusionObjectResult>();
      if (maf_perception_fusion == nullptr) {
        maf_perception_fusion = mmemory::MFMakeShared<
            maf_perception_interface::PerceptionFusionObjectResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_perception_fusion,
                                         *maf_perception_fusion);
      }

      MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_perception_fusion).header.stamp,
                            kPerceptionFusionTag),
          MSDPlanning_msg_tag(kPerceptionFusionTag), 0);

      LOGD("feed mfr %s", sub_perception_fusion_topic_.c_str());
      planning_engine_->feed_perception_fusion_result(maf_perception_fusion);
    }
  }

  void perception_fusion_aeb_callback() {
    while (mfr_subscriber_map_[sub_perception_fusion_aeb_topic_]->empty() ==
           false) {
      auto mfr_perception_fusion_aeb =
          mfr_subscriber_map_[sub_perception_fusion_aeb_topic_]
              ->pop<mmessage::perception_interface_mfrmsgs::
                        MFRMessagePerceptionFusionAEBResult>();
      if (is_engine_paused_.load()) {
        continue;
      }

      auto maf_perception_fusion_aeb =
          mfr_perception_fusion_aeb.get_internal_data()
              ->get_original_data<
                  maf_perception_interface::PerceptionFusionAEBResult>();
      if (maf_perception_fusion_aeb == nullptr) {
        maf_perception_fusion_aeb = mmemory::MFMakeShared<
            maf_perception_interface::PerceptionFusionAEBResult>();
        mfr_cpp_struct_convert::from_mfr(mfr_perception_fusion_aeb,
                                         *maf_perception_fusion_aeb);
      }

      MSDPlanning_record_timestamp(
          mlog::MLOG_msg_id((*maf_perception_fusion_aeb).header.stamp,
                            kPerceptionFusionTag),
          MSDPlanning_msg_tag(kPerceptionFusionTag), 0);

      LOGD("feed mfr %s", sub_perception_fusion_aeb_topic_.c_str());
      planning_engine_->feed_perception_fusion_aeb_result(
          maf_perception_fusion_aeb);
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

    MSDPlanning_record_timestamp(
        mlog::MLOG_msg_id((*maf_module_status).header.stamp, kModuleStatusTag),
        MSDPlanning_msg_tag(kModuleStatusTag), 0);

    planning_engine_->feed_module_status(*maf_module_status);

    return true;
  }

  bool feed_maf_mff_planning_request(std::shared_ptr<maf_std::Header> request) {
    // check ldp_w_status
    if ((*request).frame_id != "") {
      auto request_reader = mjson::Reader((*request).frame_id);

      static bool ldp_enable = false;
      auto ldp_string =
          request_reader.get<std::string>("ldp_w_status", false, "");
      if (ldp_string != "") {
        auto ldp_reader = mjson::Reader(ldp_string);
        int status = ldp_reader.get<int>("status", false, 0);
        // 0: off 4: error
        if (status == 0 || status == 4) {
          ldp_enable = false;
        } else {
          ldp_enable = true;
        }
      }

      static bool elk_enable = false;
      auto elk_string =
          request_reader.get<std::string>("elk_status", false, "");
      if (elk_string != "") {
        auto elk_reader = mjson::Reader(elk_string);
        int re_status = elk_reader.get<int>("re_status", false, 0);
        int ot_status = elk_reader.get<int>("ot_status", false, 0);
        int oc_status = elk_reader.get<int>("oc_status", false, 0);
        // 0: disable 1: enable
        if (re_status || ot_status || oc_status) {
          elk_enable = true;
        } else {
          elk_enable = false;
        }
      }

      static bool absm_enable = false;
      auto absm_string =
          request_reader.get<std::string>("absm_status", false, "");
      if (absm_string != "") {
        auto absm_reader = mjson::Reader(absm_string);
        int status = absm_reader.get<int>("status", false, 0);
        // 0: disable 1: enable
        if (status) {
          absm_enable = true;
        } else {
          absm_enable = false;
        }
      }

      static bool ess_enable = false;
      auto ess_string =
          request_reader.get<std::string>("ess_status", false, "");
      if (ess_string != "") {
        auto ess_reader = mjson::Reader(ess_string);
        int status = ess_reader.get<int>("status", false, 0);
        // 0: disable 1: enable
        if (status) {
          ess_enable = true;
        } else {
          ess_enable = false;
        }
      }

      if (ldp_enable || elk_enable || absm_enable || ess_enable) {
        is_paused_.store(false);
        is_engine_paused_.store(false);
        (void)planning_engine_->start();
      } else {
        is_paused_.store(true);
        // planning_engine_->stop();
      }
    }

    return true;
  }

#if defined(FEAT_SWITCH_UTIL)

private:
  const std::string KEY_MODULE_NAME = "LDPPlanning";
  const std::string KEY_SUB = "sub";
  const std::string KEY_PUB = "pub";
  const std::string KEY_METHOD_PE_SHM = "PE_shm";
  const std::string KEY_METHOD_INPCOM = "inpcom";
  const std::string KEY_METHOD_MFR_SOCKET = "mfr_socket";
  const std::string KEY_METHOD_MFR_SHM = "mfr_shm";

  void setup_msg_modes() {
    Switch sw;

    sub_chassis_report_mode_ =
        get_msg_sub_mode(sw, sub_chassis_report_topic_.c_str());
    sub_wheel_report_mode_ =
        get_msg_sub_mode(sw, sub_wheel_report_topic_.c_str());
    sub_body_report_mode_ =
        get_msg_sub_mode(sw, sub_body_report_topic_.c_str());
    sub_perception_vision_lane_mode_ =
        get_msg_sub_mode(sw, sub_perception_vision_lane_topic_.c_str());
    sub_perception_fusion_mode_ =
        get_msg_sub_mode(sw, sub_perception_fusion_topic_.c_str());
    sub_perception_fusion_aeb_mode_ =
        get_msg_sub_mode(sw, sub_perception_fusion_topic_.c_str());
    sub_module_status_mode_ =
        get_msg_sub_mode(sw, module_status_topic_.c_str());
    sub_mff_planning_request_mode_ =
        get_msg_sub_mode(sw, sub_mff_planning_request_topic_.c_str());
    sub_mff_planning_ess_request_mode_ =
        get_msg_sub_mode(sw, sub_mff_planning_ess_request_topic_.c_str());
    sub_mpc_trajecrory_mode_ =
        get_msg_sub_mode(sw, sub_mpc_trajecrory_topic_.c_str());

    // unused variable
    // pub_module_status_mode_ =
    //     get_msg_pub_mode(sw, module_status_topic_.c_str());
    pub_nodes_status_mode_ =
        get_msg_pub_mode(sw, pub_nodes_status_topic_.c_str());
    pub_planning_mode_ = get_msg_pub_mode(sw, pub_planning_topic_.c_str());
    pub_planning_ldp_mode_ =
        get_msg_pub_mode(sw, pub_planning_ldp_topic_.c_str());
    pub_ess_planning_mode_ =
        get_msg_pub_mode(sw, pub_ess_planning_topic_.c_str());
    pub_planning_ess_mode_ =
        get_msg_pub_mode(sw, pub_planning_ess_topic_.c_str());
    pub_mff_planning_response_mode_ =
        get_msg_pub_mode(sw, pub_mff_planning_response_topic_.c_str());
    pub_mff_planning_ess_response_mode_ =
        get_msg_pub_mode(sw, pub_mff_planning_ess_response_topic_.c_str());
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
    return std::make_shared<shmcom::SimpleShmcomHelper<T>>(name, "ldp",
                                                           1000U / hz);
  }

  ShmcomHandler<shmcom_endpoint::PODChassisReport> sub_shmcom_chassis_report_;
  ShmcomHandler<shmcom_endpoint::PODWheelReport> sub_shmcom_wheel_report_;
  ShmcomHandler<shmcom_endpoint::PODBodyReport> sub_shmcom_body_report_;
  ShmcomHandler<shmcom_framework_status::PODModuleStatus>
      sub_shmcom_module_status_;

  ShmcomHandler<shmcom_planning::PODPlanning> pub_shmcom_planning_;
  shmcom_planning::PODPlanning *ptr_shmcom_planning_ = nullptr;

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
          if (planning_engine_ && !is_engine_paused_.load()) {
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
          if (planning_engine_ && !is_engine_paused_.load()) {
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
          if (planning_engine_ && !is_engine_paused_.load()) {
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
          if (planning_engine_ && !is_engine_paused_.load()) {
            auto maf_obj =
                std::make_shared<maf_framework_status::ModuleStatus>();
            shmcom_framework_status::pod2maf(pod_obj, *maf_obj);
            LOGD("feed shmcom %s", module_status_topic_.c_str());
            feed_maf_module_status(maf_obj);
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
        shmcom_planning::PODPlanningLDP_shm_name, 10 * 2);
    if (!(ptr_shmcom_planning_ = pub_shmcom_planning_->TryAttach())) {
      LOGE("advertise shmcom %s failed", pub_planning_topic_.c_str());
      pub_shmcom_planning_.reset();
    }
  }

#else

  void subscribe_shmcom_chassis_report() {}
  void subscribe_shmcom_wheel_report() {}
  void subscribe_shmcom_body_report() {}
  void subscribe_shmcom_module_status() {}

  void publish_shmcom_planning(const maf_planning::Planning &maf_obj) {}
  void advertise_shmcom_planning() {}

#endif // FEAT_SHMCOM_MESSAGE
};

MFR_REGISTER_NODE(LdpMFRNode, "ldp_mfr_node_type");
