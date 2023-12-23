#include "argparse.hpp"
#include "common.h"
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

    //  REGISTER_TOPIC_WITH_CALLBACK_FUNC(
    //     "/msd/function_module_status", 1,
    //     framework_status_msgs::ModuleStatus,
    //     mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus,
    //     &MFR2ROSAdaptor::mfr_module_status_callback);

    REGISTER_TOPIC("/msd/sbp_request", 1, planning_msgs::SBPRequest,
                   mmessage::planning_mfrmsgs::MFRMessageSBPRequest);

    REGISTER_TOPIC("/msd/sbp_result", 1, planning_msgs::SBPResult,
                   mmessage::planning_mfrmsgs::MFRMessageSBPResult);
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

  // template <typename MFR_MSG, typename ROS_MSG>
  // void mfr_module_status_callback(const std::string &topic_name) {
  //   auto mfr_msg = mfr_subscriber_map_[topic_name]->pop<MFR_MSG>();
  //   if (mfr_msg.module_type().value() !=
  //   maf_framework_status::ModuleType::PLANNING) {
  //     return;
  //   }
  //   ROS_MSG ros_msg{};
  //   ros_mfr_convert::from_mfr(mfr_msg, ros_msg);
  //   ros_publisher_map_[topic_name].publish(ros_msg);
  // }

private:
  std::map<std::string, ros::Publisher> ros_publisher_map_;
  std::map<std::string, mfr::MFRSubscriber *> mfr_subscriber_map_;
  std::map<std::string, std::function<void(const std::string &)>>
      ros_publish_func_map_;
};

MFR_REGISTER_NODE(MFR2ROSAdaptor, "sbp_mfr2ros_adaptor_node_type");

int main(int argc, char **argv) {
  ros::init(argc, argv, "sbp_mfr2ros_adaptor");
  ros::NodeHandle n("~");
  nh_ptr = &n;

  argparse::ArgumentParser argument_parser{};
  argument_parser.long_name("--mfrrpc")
      .default_value("mfrrpc://127.0.0.1:11300")
      .help("Connect MFRMachine rpc url")
      .done();

  auto args = argument_parser.parse_args_any_type(argc, argv);
  mmemory::MFString mfrrpc = args.get_value<std::string>("mfrrpc").c_str();

  register_node("sbp_mfr2ros_adaptor_node");
  init_machine(mfrrpc, "sbp_mfr2ros_adaptor_machine");
  run_machine();

  ros::spin();
  stop_machine();

  return 0;
}
