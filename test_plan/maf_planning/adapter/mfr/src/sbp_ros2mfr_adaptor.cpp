#include "argparse.hpp"
#include "common.h"
#include <can_msgs/Frame.h>
#include <chrono>
#include <csignal>
#include <cstring>
#include <functional>
#include <mutex>
#include <thread>

#define REGISTER_TOPIC(_topic_name, _queue_size, _ros_msg_type, _mfr_msg_type) \
  {                                                                            \
    if (ros_subscriber_map_.find(_topic_name) == ros_subscriber_map_.end()) {  \
      ros_subscriber_map_[_topic_name] = nh_ptr->subscribe<_ros_msg_type>(     \
          _topic_name, _queue_size,                                            \
          std::bind(                                                           \
              &ROS2MFRAdaptor::ros_subscribe_callback<_ros_msg_type::ConstPtr, \
                                                      _mfr_msg_type>,          \
              this, std::placeholders::_1, _topic_name),                       \
          ros::VoidConstPtr(), no_delay);                                      \
    }                                                                          \
    mfr::MFRPublisherConfig pub_config{};                                      \
    pub_config.topic_name = _topic_name;                                       \
    pub_config.queue_size = _queue_size;                                       \
    mfr_publisher_map_[_topic_name] =                                          \
        node_handle->communication_manager().advertise<_mfr_msg_type>(         \
            pub_config);                                                       \
  }

#define REGISTER_TOPIC_WITH_CALLBACK_FUNC(                                     \
    _topic_name, _queue_size, _ros_msg_type, _mfr_msg_type, _callback_func)    \
  {                                                                            \
    ros_subscriber_map_[_topic_name] = nh_ptr->subscribe<_ros_msg_type>(       \
        _topic_name, _queue_size,                                              \
        std::bind(_callback_func<_ros_msg_type::ConstPtr, _mfr_msg_type>,      \
                  this, std::placeholders::_1, _topic_name),                   \
        ros::VoidConstPtr(), no_delay);                                        \
    REGISTER_TOPIC(_topic_name, _queue_size, _ros_msg_type, _mfr_msg_type)     \
  }

ros::NodeHandle *nh_ptr;

class ROS2MFRAdaptor : public mfr::MFRNode {
public:
  bool on_init(mfr::MFRNodeHandle *node_handle) override {
    auto no_delay = ros::TransportHints().tcpNoDelay(true);

    // REGISTER_TOPIC_WITH_CALLBACK_FUNC("/msd/function_module_status", 1,
    //     framework_status_msgs::ModuleStatus,
    //     mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus,
    //     &ROS2MFRAdaptor::module_status_callback);

    REGISTER_TOPIC("/msd/sbp_request", 1, planning_msgs::SBPRequest,
                   mmessage::planning_mfrmsgs::MFRMessageSBPRequest);

    REGISTER_TOPIC("/msd/sbp_result", 1, planning_msgs::SBPResult,
                   mmessage::planning_mfrmsgs::MFRMessageSBPResult);
    return true;
  }

  void on_finish() override {}

private:
  template <typename ROS_MSG_CONSTPTR, typename MFR_MSG>
  void ros_subscribe_callback(const ROS_MSG_CONSTPTR &msg,
                              const std::string &topic_name) {
    MFR_MSG mfr_msg{};
    ros_mfr_convert::to_mfr(*msg, mfr_msg);
    if (mfr_publisher_map_.find(topic_name) != mfr_publisher_map_.end()) {
      mfr_publisher_map_[topic_name]->publish(mfr_msg);
    } else {
      ROS_ERROR("[ROS2MFRAdaptor] Can not find topic name: %s",
                topic_name.c_str());
    }
  }

  //   template <typename ROS_MSG_CONSTPTR, typename MFR_MSG>
  //   void module_status_callback(const ROS_MSG_CONSTPTR &msg,
  //                               const std::string &topic_name) {

  //     if (msg->module_type.value !=
  //     framework_status_msgs::ModuleType::ENDPOINT) {
  //       return;
  //     }
  //     ROS2MFRAdaptor::ros_subscribe_callback<ROS_MSG_CONSTPTR, MFR_MSG>(
  //       msg, topic_name);
  //   }

private:
  std::map<std::string, ros::Subscriber> ros_subscriber_map_;
  std::map<std::string, mfr::MFRPublisher *> mfr_publisher_map_;
};

MFR_REGISTER_NODE(ROS2MFRAdaptor, "sbp_ros2mfr_adaptor_node_type");

int main(int argc, char **argv) {
  ros::init(argc, argv, "sbp_ros2mfr_adaptor");
  ros::NodeHandle n("~");
  nh_ptr = &n;

  argparse::ArgumentParser argument_parser{};
  argument_parser.long_name("--mfrrpc")
      .default_value("mfrrpc://127.0.0.1:11300")
      .help("Connect MFRMachine rpc url")
      .done();

  auto args = argument_parser.parse_args_any_type(argc, argv);
  mmemory::MFString mfrrpc = args.get_value<std::string>("mfrrpc").c_str();

  register_node("sbp_ros2mfr_adaptor_node");
  puts("ros2mfr adapter mfr node finish register");
  init_machine(mfrrpc, "sbp_ros2mfr_adaptor_machine");
  run_machine();
  puts("ros2mfr adapter machine has run");

  ros::spin();
  stop_machine();

  return 0;
}
