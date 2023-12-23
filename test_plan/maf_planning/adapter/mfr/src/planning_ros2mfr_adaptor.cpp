#include "argparse.hpp"
#include "nlohmann/json.hpp"

#include "common.h"
#include "mtime_mfrmsgs.h"
#include "rosgraph_msgs/Clock.h"
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

namespace ros_mfr_convert {
inline void to_mfr(const rosgraph_msgs::Clock &src,
                   mmessage::mtime_mfrmsgs::MFRMessageClock &dst) {
  dst.set_clock(src.clock.toNSec());
}
} // namespace ros_mfr_convert

ros::NodeHandle *nh_ptr;

class ROS2MFRAdaptor : public mfr::MFRNode {
public:
  bool on_init(mfr::MFRNodeHandle *node_handle) override {
    auto no_delay = ros::TransportHints().tcpNoDelay(true);

    REGISTER_TOPIC("/clock", 10, rosgraph_msgs::Clock,
                   mmessage::mtime_mfrmsgs::MFRMessageClock);
    REGISTER_TOPIC_WITH_CALLBACK_FUNC(
        "/msd/function_module_status", 10, framework_status_msgs::ModuleStatus,
        mmessage::framework_status_mfrmsgs::MFRMessageModuleStatus,
        &ROS2MFRAdaptor::module_status_callback);

    REGISTER_TOPIC("/sensor/imu", 10, gps_imu_msgs::MLAImu,
                   mmessage::gps_imu_mfrmsgs::MFRMessageMLAImu);

    REGISTER_TOPIC("/msd/prediction/prediction_result", 10,
                   worldmodel_msgs::PredictionResult,
                   mmessage::worldmodel_mfrmsgs::MFRMessagePredictionResult);

    REGISTER_TOPIC("/worldmodel/processed_map", 10,
                   worldmodel_msgs::ProcessedMap,
                   mmessage::worldmodel_mfrmsgs::MFRMessageProcessedMap);

    REGISTER_TOPIC("/worldmodel/objects", 10, worldmodel_msgs::ObjectsInterface,
                   mmessage::worldmodel_mfrmsgs::MFRMessageObjectsInterface);

    // REGISTER_TOPIC("/worldmodel/parking_slot_info", 10,
    //                worldmodel_msgs::FusionAPA,
    //                mmessage::worldmodel_mfrmsgs::MFRMessageFusionAPA);

    REGISTER_TOPIC("/worldmodel/scene_object", 10,
                   worldmodel_msgs::SceneObjects,
                   mmessage::worldmodel_mfrmsgs::MFRMessageSceneObjects);

    REGISTER_TOPIC("/worldmodel/traffic_light", 10,
                   perception_interface_msgs::TrafficLightPerception,
                   mmessage::perception_interface_mfrmsgs::
                       MFRMessageTrafficLightPerception);

    REGISTER_TOPIC("/perception/fusion/object_parking_environment", 10,
                   perception_interface_msgs::PerceptionFusionObjectResult,
                   mmessage::perception_interface_mfrmsgs::
                       MFRMessagePerceptionFusionObjectResult);

    REGISTER_TOPIC("/perception/fusion/ground_line", 10,
                   perception_interface_msgs::FusionGroundLineResult,
                   mmessage::perception_interface_mfrmsgs::
                       MFRMessageFusionGroundLineResult);

    REGISTER_TOPIC("/perception/fusion/ground_line_uss", 10,
                   perception_interface_msgs::FusionGroundLineResult,
                   mmessage::perception_interface_mfrmsgs::
                       MFRMessageFusionGroundLineResult);

    REGISTER_TOPIC(
        "/perception/fusion/distance_uss", 10,
        sensor_interface_msgs::UltrasonicUpaReport,
        mmessage::sensor_interface_mfrmsgs::MFRMessageUltrasonicUpaReport);

    REGISTER_TOPIC(
        "/mla/egopose", 10, mla_localization_msgs::MLALocalization,
        mmessage::mla_localization_mfrmsgs::MFRMessageMLALocalization);

    REGISTER_TOPIC("/vehicle/chassis_report", 10, endpoint_msgs::ChassisReport,
                   mmessage::endpoint_mfrmsgs::MFRMessageChassisReport);
    REGISTER_TOPIC("/vehicle/wireless_charger_report", 10,
                   endpoint_msgs::WirelessChargerReport,
                   mmessage::endpoint_mfrmsgs::MFRMessageWirelessChargerReport);

    REGISTER_TOPIC("/vehicle/wheel_report", 10, endpoint_msgs::WheelReport,
                   mmessage::endpoint_mfrmsgs::MFRMessageWheelReport);

    REGISTER_TOPIC("/vehicle/body_report", 10, endpoint_msgs::BodyReport,
                   mmessage::endpoint_mfrmsgs::MFRMessageBodyReport);

    REGISTER_TOPIC("/msd/control/mpc_traj", 10,
                   planning_msgs::MpcTrajectoryResult,
                   mmessage::planning_mfrmsgs::MFRMessageMpcTrajectoryResult);

    REGISTER_TOPIC(
        "/system_manager/planning/control_cmd_request", 10,
        system_manager_msgs::ModuleControlCmdRequest,
        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdRequest);

    REGISTER_TOPIC(
        "/system_manager/planning/request", 10,
        system_manager_msgs::SysPlanningRequest,
        mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningRequest);

    REGISTER_TOPIC(
        "/msd/planning_frequency_control", 10,
        system_manager_msgs::SysPlanningRequest,
        mmessage::system_manager_mfrmsgs::MFRMessageSysPlanningRequest);

    REGISTER_TOPIC("/system_manager/planning/reset_request", 10,
                   std_msgs::Header, mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC(
        "/perception/vision/lane", 10,
        perception_interface_msgs::RoadLinePerception,
        mmessage::perception_interface_mfrmsgs::MFRMessageRoadLinePerception);

    REGISTER_TOPIC(
        "/perception/vision/landmark", 10,
        perception_interface_msgs::RoadLinePerception,
        mmessage::perception_interface_mfrmsgs::MFRMessageRoadLinePerception);

    REGISTER_TOPIC(
        "/perception/lidar/road_edge", 10,
        perception_interface_msgs::RoadLinePerception,
        mmessage::perception_interface_mfrmsgs::MFRMessageRoadLinePerception);

    REGISTER_TOPIC("/mff/ldp_planning/request", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC("/perception/radar/result_front", 10,
                   perception_interface_msgs::RadarPerceptionResult,
                   mmessage::perception_interface_mfrmsgs::
                       MFRMessageRadarPerceptionResult);

    REGISTER_TOPIC("/system_manager/info/parking", 10, std_msgs::Header,
                   mmessage::std_mfrmsgs::MFRMessageHeader);

    // for wm
    REGISTER_TOPIC("/system_manager/worldmodel/reset_request", 10,
                   std_msgs::Header, mmessage::std_mfrmsgs::MFRMessageHeader);

    REGISTER_TOPIC(
        "/system_manager/worldmodel/control_cmd_request/highway", 10,
        system_manager_msgs::ModuleControlCmdRequest,
        mmessage::system_manager_mfrmsgs::MFRMessageModuleControlCmdRequest);

    REGISTER_TOPIC("/perception/fusion/object", 10,
                   perception_interface_msgs::PerceptionFusionObjectResult,
                   mmessage::perception_interface_mfrmsgs::
                       MFRMessagePerceptionFusionObjectResult);

    REGISTER_TOPIC(
        "/system_manager/worldmodel/request/highway", 10,
        system_manager_msgs::SysWorldModelRequest,
        mmessage::system_manager_mfrmsgs::MFRMessageSysWorldModelRequest);

    REGISTER_TOPIC("/perception/fusion/parking_slot", 10,
                   perception_interface_msgs::FusionParkingSlotResult,
                   mmessage::perception_interface_mfrmsgs::
                       MFRMessageFusionParkingSlotResult);

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

  template <typename ROS_MSG_CONSTPTR, typename MFR_MSG>
  void module_status_callback(const ROS_MSG_CONSTPTR &msg,
                              const std::string &topic_name) {

    if (msg->module_type.value != framework_status_msgs::ModuleType::ENDPOINT) {
      return;
    }
    ROS2MFRAdaptor::ros_subscribe_callback<ROS_MSG_CONSTPTR, MFR_MSG>(
        msg, topic_name);
  }

private:
  std::map<std::string, ros::Subscriber> ros_subscriber_map_;
  std::map<std::string, mfr::MFRPublisher *> mfr_publisher_map_;
};

MFR_REGISTER_NODE(ROS2MFRAdaptor, "planning_ros2mfr_adaptor_node_type");

int main(int argc, char **argv) {
  ros::init(argc, argv, "plannning_ros2mfr_adaptor");
  ros::NodeHandle n("~");
  nh_ptr = &n;

  argparse::ArgumentParser argument_parser{};
  argument_parser.long_name("--mfrrpc")
      .default_value("mfrrpc://127.0.0.1:11300")
      .help("Connect MFRMachine rpc url")
      .done();

  auto args = argument_parser.parse_args_any_type(argc, argv);
  mmemory::MFString mfrrpc = args.get_value<std::string>("mfrrpc").c_str();

  register_node("planning_ros2mfr_adaptor_node");
  puts("ros2mfr adapter mfr node finish register");
  init_machine(mfrrpc, "planning_ros2mfr_adaptor_machine");
  run_machine();
  puts("ros2mfr adapter machine has run");

  ros::spin();
  stop_machine();

  return 0;
}
