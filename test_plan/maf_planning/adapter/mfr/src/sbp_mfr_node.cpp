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
#include "planning_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "pnc/search_based_planning_engine_interface.h"
#include "std_mfrmsgs/mfr_cpp_struct_convert.hpp"
#include "worldmodel_mfrmsgs/mfr_cpp_struct_convert.hpp"

#include <chrono>
#include <csignal>
#include <map>
#include <thread>

using namespace mfr;
using namespace mmemory;
using namespace msd_planning;
using namespace msquare;

// namespace {
// mmemory::MFString read_file(const mmemory::MFString &path) {
//   FILE *file = fopen(path.c_str(), "r");
//   assert(file != nullptr);
//   std::shared_ptr<FILE> fp(file, [](FILE *file) { fclose(file); });
//   fseek(fp.get(), 0, SEEK_END);
//   std::vector<char> content(ftell(fp.get()));
//   fseek(fp.get(), 0, SEEK_SET);
//   auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
//   assert(read_bytes == content.size());
//   (void)read_bytes;
//   return mmemory::MFString(content.begin(), content.end());
// }
// } // namespace

class SearchBasedPlanningMFRNode : public MFRNode {
public:
  bool on_init(MFRNodeHandle *node_handle) override {

    // get param
    auto param_reader = mjson::Reader(node_handle->node_param_json().c_str());
    MFString mtaskflow_config_file =
        param_reader.get<std::string>("mtaskflow_config_file").c_str();
    MFString vehicle_calib_file =
        param_reader.get<std::string>("vehicle_calib_file").c_str();
    MFString config_file_dir =
        param_reader.get<std::string>("config_file_dir").c_str();
    sub_sbp_request_topic_ =
        param_reader.get<std::string>("sub_sbp_request_topic").c_str();
    pub_sbp_result_topic_ =
        param_reader.get<std::string>("pub_sbp_result_topic").c_str();
    bool enable_dump_file = param_reader.get<bool>("enable_dump_file");
    std::cout << param_reader.raw().dump() << std::endl;

    // get subscriber
    auto &communication_manager = node_handle->communication_manager();
    {
      MFRSubscriberConfig sub_config{};
      sub_config.topic_name = sub_sbp_request_topic_;
      sub_config.queue_size = 2;
      sub_config.thread_tag = sub_sbp_request_topic_;
      mfr_subscriber_map_[sub_sbp_request_topic_] =
          communication_manager
              .subscribe<mmessage::planning_mfrmsgs::MFRMessageSBPRequest>(
                  sub_config);
    }

    // get publisher
    {
      MFRPublisherConfig pub_config{};
      pub_config.topic_name = pub_sbp_result_topic_;
      pub_config.queue_size = 2;
      mfr_publisher_map_[pub_sbp_result_topic_] =
          communication_manager
              .advertise<mmessage::planning_mfrmsgs::MFRMessageSBPResult>(
                  pub_config);
    }

    // std::string config_data = read_file(config_file).c_str();
    // init_config_data(config_data);
    // MSDPlanning_set_log_config(config_data, node_handle);
    // MSDPlanning_init_log_manager();
    // MSDPlanning_set_time_config({config_data}, node_handle);

    // MSDPlanningConfig planning_config{scene_type_, true,
    // default_cruise_velocity_, config_file_dir.c_str()};
    // planning_config.cruise_velocity = default_cruise_velocity_;
    sbp_engine_ = SearchBasedPlanningEngineInterface::make(
        vehicle_calib_file.c_str(), mtaskflow_config_file.c_str(),
        config_file_dir.c_str());
    sbp_engine_->setEnableDumpFile(enable_dump_file);
    sbp_engine_->setCallback(
        std::bind(&SearchBasedPlanningMFRNode::onCallbackSbpResult, this,
                  std::placeholders::_1));
    return true;
  }

  void on_finish() override {}

  void on_running(const MFRNodeRunningInfo &info) override {
    if (info.trigger == MFR_NODE_TRIGGER_MESSAGE) {
      if (info.thread_tag == sub_sbp_request_topic_) {
        onCallbackSBPRequest();
      }
    }
  }

private:
  std::map<MFString, MFRSubscriber *> mfr_subscriber_map_;
  std::map<MFString, MFRPublisher *> mfr_publisher_map_;

  std::shared_ptr<SearchBasedPlanningEngineInterface> sbp_engine_;

  MFString sub_sbp_request_topic_;
  MFString pub_sbp_result_topic_;

private:
  void onCallbackSBPRequest() {
    while (mfr_subscriber_map_[sub_sbp_request_topic_]->empty() == false) {
      auto mfr_sub_request_command =
          mfr_subscriber_map_[sub_sbp_request_topic_]
              ->pop<mmessage::planning_mfrmsgs::MFRMessageSBPRequest>();

      auto request = mfr_sub_request_command.get_internal_data()
                         ->get_original_data<maf_planning::SBPRequest>();
      if (request == nullptr) {
        request = mmemory::MFMakeShared<maf_planning::SBPRequest>();
        mfr_cpp_struct_convert::from_mfr(mfr_sub_request_command, *request);
      }

      sbp_engine_->feedSBPRequest(*request);
    }
  }
  void onCallbackSbpResult(const maf_planning::SBPResult &sbp_result) {
    mmessage::planning_mfrmsgs::MFRMessageSBPResult msg{};
    msg.get_internal_data()->set_original_data(&sbp_result);
    msg.get_internal_data()->set_serialize_preprocessor([&sbp_result, &msg]() {
      mfr_cpp_struct_convert::to_mfr(sbp_result, msg);
    });
    mfr_publisher_map_[pub_sbp_result_topic_]->publish(msg);
  }
};

MFR_REGISTER_NODE(SearchBasedPlanningMFRNode, "sbp_mfr_node_type");
