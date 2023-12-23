#include "argparse.hpp"
#include "common.h"
#include "nlohmann/json.hpp"
#include "pnc.h"
#include <chrono>
#include <csignal>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>

#define REGISTER_TOPIC(_topic_name, _queue_size, _ros_msg_type, _mfr_msg_type) \
  {                                                                            \
    ros_publisher_map_[_topic_name] =                                          \
        nh_ptr->advertise<_ros_msg_type>(_topic_name, _queue_size);            \
                                                                               \
    mfr::MFRSubscriberConfig sub_config{};                                     \
    sub_config.topic_name = _topic_name;                                       \
    sub_config.queue_size = _queue_size;                                       \
    sub_config.thread_tag = _topic_name;                                       \
    mfr_subscriber_map_[_topic_name] =                                         \
        node_handle->communication_manager().subscribe<_mfr_msg_type>(         \
            sub_config);                                                       \
                                                                               \
    if (ros_publish_func_map_.find(_topic_name) ==                             \
        ros_publish_func_map_.end())                                           \
      ros_publish_func_map_[_topic_name] =                                     \
          std::bind(&MFR2ROSAdaptor::mfr_subscriber_callback<_mfr_msg_type,    \
                                                             _ros_msg_type>,   \
                    this, std::placeholders::_1);                              \
  }

#define REGISTER_TOPIC_WITH_CALLBACK_FUNC(                                     \
    _topic_name, _queue_size, _ros_msg_type, _mfr_msg_type, _callback_func)    \
  {                                                                            \
    ros_publish_func_map_[_topic_name] =                                       \
        std::bind(_callback_func<_mfr_msg_type, _ros_msg_type>, this,          \
                  std::placeholders::_1);                                      \
    REGISTER_TOPIC(_topic_name, _queue_size, _ros_msg_type, _mfr_msg_type)     \
  }

ros::NodeHandle *nh_ptr;

class MFR2ROSAdaptor : public mfr::MFRNode {
public:
  bool on_init(mfr::MFRNodeHandle *node_handle) override {
    REGISTER_TOPIC("/node_status/planning", 10,
                   framework_status_msgs::NodesStatus,
                   mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus);

    REGISTER_TOPIC("/node_status/ldp_planning", 10,
                   framework_status_msgs::NodesStatus,
                   mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus);

    REGISTER_TOPIC("/msquare/cla_event_filter", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC("/msd/planning/plan", 10, planning_msgs::Planning,
                   mmessage::planning_mfrmsgs::MFRMessagePlanning);

    REGISTER_TOPIC("/msd/planning/ldp_plan", 10, planning_msgs::Planning,
                   mmessage::planning_mfrmsgs::MFRMessagePlanning);

    REGISTER_TOPIC("/mlog/logs/planning", 10, mlog_msgs::Logs,
                   mmessage::mlog_mfrmsgs::MFRMessageLogs);

    REGISTER_TOPIC(
        "/system_manager/planning/control_cmd_response", 10,
        system_manager_msgs::ModuleControlCmdResponse,
        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdResponse);

    REGISTER_TOPIC(
        "/system_manager/planning/response", 10,
        system_manager_msgs::SysPlanningResponse,
        mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningResponse);

    REGISTER_TOPIC("/system_manager/planning/reset_response", 10,
                   std_msgs::Header, mmessage::std_mfrmsgs::MFRMessageHeader)

    REGISTER_TOPIC("/planning/info", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC("/cp_mdebug", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC("/mlog/logs/ldp_planning", 10, mlog_msgs::Logs,
                   mmessage::mlog_mfrmsgs::MFRMessageLogs)

    REGISTER_TOPIC("/planning/ldp_status", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader)

    REGISTER_TOPIC("/mff/ldp_planning/response", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader)

    REGISTER_TOPIC("/msd/sbp_request", 10, planning_msgs::SBPRequest,
                   mmessage::planning_mfrmsgs::MFRMessageSBPRequest);

    // from wm
    REGISTER_TOPIC("/mlog/logs/worldmodel", 10, mlog_msgs::Logs,
                   mmessage::mlog_mfrmsgs::MFRMessageLogs);

    // REGISTER_TOPIC("/worldmodel/processed_map", 10,
    //                worldmodel_msgs::ProcessedMap,
    //                mmessage::worldmodel_mfrmsgs::MFRMessageProcessedMap);

    REGISTER_TOPIC("/worldmodel/info", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC("/system_manager/worldmodel/reset_response", 10,
                   std_msgs::Header, mmessage::std_mfrmsgs::MFRMessageHeader)

    // REGISTER_TOPIC("/worldmodel/objects", 10,
    // worldmodel_msgs::ObjectsInterface,
    //                mmessage::worldmodel_mfrmsgs::MFRMessageObjectsInterface);

    REGISTER_TOPIC("/hdmap/horizon_request", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC("/node_status/worldmodel/highway", 10,
                   framework_status_msgs::NodesStatus,
                   mmessage::framework_status_mfrmsgs::MFRMessageNodesStatus);

    REGISTER_TOPIC(
        "/system_manager/worldmodel/response/highway", 10,
        system_manager_msgs::SysWorldModelResponse,
        mmessage::system_manager_mfrmsgs::MFRMessageSysWorldModelResponse);

    REGISTER_TOPIC(
        "/system_manager/worldmodel/control_cmd_response/highway", 10,
        system_manager_msgs::ModuleControlCmdResponse,
        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdResponse);

    REGISTER_TOPIC("/worldmodel/map_matched_info", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC("/worldmodel/parking_slot_info", 10,
                   worldmodel_msgs::FusionAPA,
                   mmessage::worldmodel_mfrmsgs::MFRMessageFusionAPA);

    return true;
  }

  void on_finish() override {}
  void on_running(const mfr::MFRNodeRunningInfo &info) override {
    if (info.trigger == mfr::MFR_NODE_TRIGGER_MESSAGE) {
      auto topic_name = std::string(info.thread_tag.c_str());
      if (mfr_subscriber_map_.find(topic_name) != mfr_subscriber_map_.end()) {
        while (mfr_subscriber_map_[topic_name]->empty() == false) {
          ros_publish_func_map_[topic_name](topic_name);
        }
      } else {
        ROS_ERROR("[MFR2ROSAdaptor] Can not find topic name: %s",
                  topic_name.c_str());
      }
    }
  }

private:
  template <typename MFR_MSG, typename ROS_MSG>
  void mfr_subscriber_callback(const std::string &topic_name) {
    auto mfr_msg = mfr_subscriber_map_[topic_name]->pop<MFR_MSG>();
    ROS_MSG ros_msg{};
    ros_mfr_convert::from_mfr(mfr_msg, ros_msg);
    ros_publisher_map_[topic_name].publish(ros_msg);
  }

private:
  std::map<std::string, ros::Publisher> ros_publisher_map_;
  std::map<std::string, mfr::MFRSubscriber *> mfr_subscriber_map_;
  std::map<std::string, std::function<void(const std::string &)>>
      ros_publish_func_map_;
};

MFR_REGISTER_NODE(MFR2ROSAdaptor, "planning_mfr2ros_adaptor_node_type");

int main(int argc, char **argv) {
  ros::init(argc, argv, "planning_mfr2ros_adaptor");
  ros::NodeHandle n("~");
  nh_ptr = &n;

  argparse::ArgumentParser argument_parser{};
  argument_parser.long_name("--mfrrpc")
      .default_value("mfrrpc://127.0.0.1:11300")
      .help("Connect MFRMachine rpc url")
      .done();

  auto args = argument_parser.parse_args_any_type(argc, argv);
  mmemory::MFString mfrrpc = args.get_value<std::string>("mfrrpc").c_str();

  register_node("planning_mfr2ros_adaptor_node");
  init_machine(mfrrpc, "planning_mfr2ros_adaptor_machine");
  run_machine();

  ros::spin();
  stop_machine();

  return 0;
}
