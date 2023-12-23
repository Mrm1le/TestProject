#include "maf_message_cache.h"

#include "include/msg_cache_defines.h"
#include "ros/time.h"
#include "ros_convert.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <maf_interface/maf_planning.h>

namespace msc {

template <typename TBag, typename MafMsg> struct MapMafMsgType;

template <>
struct MapMafMsgType<rosbag::Bag, maf_system_manager::SysPlanningRequest> {
  typedef system_manager_msgs::SysPlanningRequest ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_framework_status::ModuleStatus> {
  typedef framework_status_msgs::ModuleStatus ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_endpoint::ChassisReport> {
  typedef endpoint_msgs::ChassisReport ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_endpoint::WheelReport> {
  typedef endpoint_msgs::WheelReport ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_endpoint::BodyReport> {
  typedef endpoint_msgs::BodyReport ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_hdmap::LocalMap> {
  typedef hdmap_msgs::LocalMap ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_std::Header> {
  typedef std_msgs::Header ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag,
                     maf_perception_interface::TrafficLightPerception> {
  typedef perception_interface_msgs::TrafficLightPerception ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_mla_localization::MLALocalization> {
  typedef mla_localization_msgs::MLALocalization ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag,
                     maf_perception_interface::PerceptionFusionObjectResult> {
  typedef perception_interface_msgs::PerceptionFusionObjectResult ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag,
                     maf_perception_interface::FusionGroundLineResult> {
  typedef perception_interface_msgs::FusionGroundLineResult ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_worldmodel::PredictionResult> {
  typedef worldmodel_msgs::PredictionResult ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_planning::Planning> {
  typedef planning_msgs::Planning ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_endpoint::ControlCommand> {
  typedef endpoint_msgs::ControlCommand ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag,
                     maf_perception_interface::RoadLinePerception> {
  typedef perception_interface_msgs::RoadLinePerception ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_system_manager::SysMapManagerRequest> {
  typedef system_manager_msgs::SysMapManagerRequest ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_planning::MpcTrajectoryResult> {
  typedef planning_msgs::MpcTrajectoryResult ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_gps_imu::MLAImu> {
  typedef gps_imu_msgs::MLAImu ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_worldmodel::ProcessedMap> {
  typedef worldmodel_msgs::ProcessedMap ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_worldmodel::ObjectsInterface> {
  typedef worldmodel_msgs::ObjectsInterface ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag,
                     maf_perception_interface::RadarPerceptionResult> {
  typedef perception_interface_msgs::RadarPerceptionResult ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_worldmodel::SceneObjects> {
  typedef worldmodel_msgs::SceneObjects ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_sensor_interface::UltrasonicUpaReport> {
  typedef sensor_interface_msgs::UltrasonicUpaReport ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_planning::SBPResult> {
  typedef planning_msgs::SBPResult ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_planning::SBPRequest> {
  typedef planning_msgs::SBPRequest ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_system_manager::ModuleControlCmdRequest> {
  typedef system_manager_msgs::ModuleControlCmdRequest ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag,
                     maf_perception_interface::FusionParkingSlotResult> {
  typedef perception_interface_msgs::FusionParkingSlotResult ToMsgType;
};

template <> struct MapMafMsgType<rosbag::Bag, maf_worldmodel::FusionAPA> {
  typedef worldmodel_msgs::FusionAPA ToMsgType;
};

template <>
struct MapMafMsgType<rosbag::Bag, maf_framework_status::NodesStatus> {
  typedef framework_status_msgs::NodesStatus ToMsgType;
};

#define USE_HEADER_TIMESTAMP 1

template <typename MsgFrames, typename Topics>
static bool
load_topic(const rosbag::Bag &bag, Topics &&topics, MsgFrames &msgs,
           std::function<bool(const typename MsgFrames::MafMessageType &)>
               message_filter) {
  typedef typename MsgFrames::MafMessageType MafMsg;
  typedef typename MapMafMsgType<rosbag::Bag, MafMsg>::ToMsgType RosMsg;
  rosbag::View view(bag, rosbag::TopicQuery(std::forward<Topics>(topics)));
  msgs.mutable_messages()->reserve(view.size());
  for (const auto &m : view) {
    auto ros_msg = m.instantiate<RosMsg>();
    if (ros_msg == nullptr) {
      LOG(ERROR) << "wrong message type: " << typeid(MafMsg).name();
      return false;
    }
    msgs.mutable_messages()->emplace_back();
    auto *maf_msg = msgs.mutable_messages()->back().mutable_msg();
    ros_cpp_struct_convert::from_ros(*ros_msg, *maf_msg);
    if (message_filter != nullptr && !message_filter(*maf_msg)) {
      msgs.mutable_messages()->pop_back();
      continue;
    }

#if USE_HEADER_TIMESTAMP
    uint64_t timestamp = get_header_timestamp(*maf_msg);
    if (timestamp == 0) {
      timestamp = m.getTime().toNSec();
      set_header_timestamp(timestamp, maf_msg);
    }
    msgs.mutable_messages()->back().set_timestamp(timestamp);
#else
    msgs.mutable_messages()->back().set_timestamp(m.getTime().toNSec());
#endif
  }

  std::sort(msgs.mutable_messages()->begin(), msgs.mutable_messages()->end(),
            [](const auto &a, const auto &b) {
              return a.timestamp() < b.timestamp();
            });
  return true;
}

template <typename TBag>
static bool load_topics(const TBag &bag, MafMessageCache *msg_cache) {
  if (!load_topic(bag, TOPIC_CP_PLANNING_REQUEST,
                  *msg_cache->mutable_origin_sys_planning_request_msgs(),
                  nullptr)) {
    return false;
  }
  if (msg_cache->origin_sys_planning_request_msgs().messages().empty()) {
    if (!load_topic(bag, TOPIC_PLANNING_REQUEST,
                    *msg_cache->mutable_origin_sys_planning_request_msgs(),
                    nullptr)) {
      return false;
    }
  }

  if (!load_topic(bag, TOPIC_MODULE_STATUS,
                  *msg_cache->mutable_origin_module_status_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_CHASSIS_REPORT,
                  *msg_cache->mutable_origin_chassis_report_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_WHEEL_REPORT,
                  *msg_cache->mutable_origin_wheel_report_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_BODY_REPORT,
                  *msg_cache->mutable_origin_body_report_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_TRAFFIC_LIGHT,
                  *msg_cache->mutable_origin_traffic_light_perception_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_EGO_POSE,
                  *msg_cache->mutable_origin_mla_localization_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(
          bag, TOPIC_FUSION_OBJECTS,
          *msg_cache->mutable_origin_perception_fusion_object_result_msgs(),
          nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_FUSION_GROUNDLINE,
                  *msg_cache->mutable_origin_fusion_ground_line_result_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_PREDICTION_INFO,
                  *msg_cache->mutable_origin_prediction_result_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_MPC_TRAJECTORY,
                  *msg_cache->mutable_origin_mpc_trajectory_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_PLANNING_OUTPUT,
                  *msg_cache->mutable_origin_planning_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_PLANNING_INFO,
                  *msg_cache->mutable_origin_planning_info_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_PERCEPTION_VISION_LANE,
                  *msg_cache->mutable_origin_perception_vision_lane_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_PERCEPTION_VISION_LANDMARK,
                  *msg_cache->mutable_origin_perception_vision_landmark_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_PERCEPTION_LIDAR_ROAD_EDGE,
                  *msg_cache->mutable_origin_perception_lidar_road_edge_msgs(),
                  nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_PLANNING_FREQ_CONTROL,
                  *msg_cache->mutable_origin_planning_frequency_control_msgs(),
                  nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_SENSOR_IMU,
                  *msg_cache->mutable_origin_mla_imu_msgs(), nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_WORLDMODEL_PROCESSED_MAP,
                  *msg_cache->mutable_origin_processed_map_msgs(), nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_WORLDMODEL_OBJECTS,
                  *msg_cache->mutable_origin_objects_interface_msgs(),
                  nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_PERCEPTION_RADAR_FRONT,
                  *msg_cache->mutable_origin_radar_perception_result_msgs(),
                  nullptr)) {
    return false;
  }

  if (!load_topic(
          bag, TOPIC_FUSION_GROUNDLINE_USS,
          *msg_cache->mutable_origin_fusion_uss_ground_line_result_msgs(),
          nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_FUSION_DISTANCE_USS,
                  *msg_cache->mutable_origin_ultrasonic_upa_report_msgs(),
                  nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_FUSION_DISTANCE_USS_RAW,
                  *msg_cache->mutable_origin_ultrasonic_upa_report_raw_msgs(),
                  nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_SBP_RESULT,
                  *msg_cache->mutable_origin_sbp_result_msgs(), nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_PLANNING_RESET_REQUEST,
                  *msg_cache->mutable_origin_planning_reset_request_msgs(),
                  nullptr)) {
    return false;
  }

  if (!load_topic(bag, TOPIC_CP_PLANNING_CONTROL_CMD_REQUEST,
                  *msg_cache->mutable_origin_module_control_cmd_request_msgs(),
                  nullptr)) {
    return false;
  }
  if (msg_cache->origin_module_control_cmd_request_msgs().messages().empty()) {
    if (!load_topic(
            bag, TOPIC_PLANNING_CONTROL_CMD_REQUEST,
            *msg_cache->mutable_origin_module_control_cmd_request_msgs(),
            nullptr)) {
      return false;
    }
  }
  if (!load_topic(bag, TOPIC_WORLDMODEL_SCENE_OBJECT,
                  *msg_cache->mutable_origin_world_model_scene_objects_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_PERCEPTION_FUSION_PARKING_SLOT,
                  *msg_cache->mutable_origin_fusion_parking_slot_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_WORLDMODEL_PARKING_SLOT,
                  *msg_cache->mutable_origin_worldmodel_fusion_apa_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(
          bag, TOPIC_PERCEPTION_FUSION_OBJ_PARKING_ENV,
          *msg_cache->mutable_origin_perception_fusion_obj_parking_env_msgs(),
          nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_SBP_REQUEST,
                  *msg_cache->mutable_origin_sbp_request_msgs(), nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_NODE_STATUS_PLANNING,
                  *msg_cache->mutable_origin_node_status_planning_msgs(),
                  nullptr)) {
    return false;
  }
  if (!load_topic(bag, TOPIC_NODE_STATUS_APA_PLANNING,
                  *msg_cache->mutable_origin_node_status_apa_planning_msgs(),
                  nullptr)) {
    return false;
  }

  return true;
}

bool MafMessageCache::load_bag(const std::string &bag_path,
                               const bool is_mfbag) {
  if (is_mfbag) {
    return false;
  } else {
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    bool res = load_topics(bag, this);
    bag.close();
    return res;
  }
}

template <typename MsgFrames>
static void write_bag(const MsgFrames &msgs, const std::string &topic,
                      rosbag::Bag *bag) {
  try {
    typedef typename MsgFrames::MafMessageType MafMsg;
    typedef typename MapMafMsgType<rosbag::Bag, MafMsg>::ToMsgType RosMsg;
    for (const auto &msg : msgs.messages()) {
      RosMsg ros_msg;
      ros_cpp_struct_convert::to_ros(msg.msg(), ros_msg);
      bag->write(topic, ros::Time().fromNSec(msg.timestamp()), ros_msg);
    }
  } catch (const std::exception &ex) {
    LOG(ERROR) << "write topic: " << topic << ", exception: " << ex.what();
  }
}

template <typename Bag>
static void write_bag(const MafMessageCache &msg_cache,
                      const bool is_closed_loop, Bag *bag) {
  write_bag(msg_cache.new_planning_msgs(), TOPIC_PLANNING_OUTPUT, bag);
  write_bag(msg_cache.new_planning_info_msgs(), TOPIC_PLANNING_INFO, bag);
  write_bag(msg_cache.new_mdebug_msgs(), TOPIC_MDEBUG, bag);
  write_bag(msg_cache.new_planning_extra_data_msgs(), TOPIC_PLANNING_EXTRA_DATA,
            bag);
  write_bag(msg_cache.origin_traffic_light_perception_msgs(),
            TOPIC_TRAFFIC_LIGHT, bag);
  write_bag(msg_cache.origin_prediction_result_msgs(), TOPIC_PREDICTION_INFO,
            bag);
  write_bag(msg_cache.origin_mpc_trajectory_msgs(), TOPIC_MPC_TRAJECTORY, bag);
  write_bag(msg_cache.new_control_command_msgs(), TOPIC_CONTROL_COMMAND, bag);

  write_bag(msg_cache.new_processed_map_msgs(), TOPIC_WORLDMODEL_PROCESSED_MAP,
            bag);
  write_bag(msg_cache.new_objects_interface_msgs(), TOPIC_WORLDMODEL_OBJECTS,
            bag);
  write_bag(msg_cache.origin_perception_lidar_road_edge_msgs(),
            TOPIC_PERCEPTION_LIDAR_ROAD_EDGE, bag);
  write_bag(msg_cache.origin_fusion_uss_ground_line_result_msgs(),
            TOPIC_FUSION_GROUNDLINE_USS, bag);
  write_bag(msg_cache.origin_ultrasonic_upa_report_msgs(),
            TOPIC_FUSION_DISTANCE_USS, bag);
  write_bag(msg_cache.origin_ultrasonic_upa_report_raw_msgs(),
            TOPIC_FUSION_DISTANCE_USS_RAW, bag);
  write_bag(msg_cache.origin_fusion_ground_line_result_msgs(),
            TOPIC_FUSION_GROUNDLINE, bag);
  write_bag(msg_cache.origin_perception_fusion_obj_parking_env_msgs(),
            TOPIC_PERCEPTION_FUSION_OBJ_PARKING_ENV, bag);

  if (is_closed_loop) {
    write_bag(msg_cache.origin_planning_msgs(), MSIM_TOPIC_PLANNING_OUTPUT,
              bag);
    write_bag(msg_cache.new_module_status_msgs(), TOPIC_MODULE_STATUS, bag);
    write_bag(msg_cache.origin_module_status_msgs(), MSIM_TOPIC_MODULE_STATUS,
              bag);
    write_bag(msg_cache.new_chassis_report_msgs(), TOPIC_CHASSIS_REPORT, bag);
    write_bag(msg_cache.origin_chassis_report_msgs(), MSIM_TOPIC_CHASSIS_REPORT,
              bag);
    write_bag(msg_cache.new_wheel_report_msgs(), TOPIC_WHEEL_REPORT, bag);
    write_bag(msg_cache.new_body_report_msgs(), TOPIC_BODY_REPORT, bag);
    write_bag(msg_cache.new_perception_fusion_object_result_msgs(),
              TOPIC_FUSION_OBJECTS, bag);
    write_bag(msg_cache.origin_planning_info_msgs(), MSIM_TOPIC_PLANNING_INFO,
              bag);
    write_bag(msg_cache.new_perception_vision_lane_msgs(),
              TOPIC_PERCEPTION_VISION_LANE, bag);
    write_bag(msg_cache.new_perception_vision_landmark_msgs(),
              TOPIC_PERCEPTION_VISION_LANDMARK, bag);
    write_bag(msg_cache.origin_mla_localization_msgs(), MSIM_TOPIC_EGO_POSE,
              bag);
    write_bag(msg_cache.origin_mla_localization_msgs(), MVIZ_TOPIC_EGO_CAR_GT,
              bag);
    write_bag(msg_cache.new_mla_localization_msgs(), TOPIC_EGO_POSE, bag);
    write_bag(msg_cache.origin_fusion_parking_slot_msgs(),
              TOPIC_PERCEPTION_FUSION_PARKING_SLOT, bag);
    write_bag(msg_cache.origin_worldmodel_fusion_apa_msgs(),
              MSIM_TOPIC_WORLDMODEL_PARKING_SLOT, bag);
    write_bag(msg_cache.new_worldmodel_fusion_apa_msgs(),
              TOPIC_WORLDMODEL_PARKING_SLOT, bag);
    write_bag(msg_cache.origin_world_model_scene_objects_msgs(),
              MSIM_TOPIC_WORLDMODEL_SCENE_OBJECT, bag);
    write_bag(msg_cache.new_world_model_scene_objects_msgs(),
              TOPIC_WORLDMODEL_SCENE_OBJECT, bag);
    write_bag(msg_cache.origin_sbp_request_msgs(), MSIM_TOPIC_SBP_REQUEST, bag);
    write_bag(msg_cache.new_sbp_request_msgs(), TOPIC_SBP_REQUEST, bag);
    write_bag(msg_cache.origin_node_status_planning_msgs(),
              MSIM_TOPIC_NODE_STATUS_PLANNING, bag);
    write_bag(msg_cache.new_node_status_planning_msgs(),
              TOPIC_NODE_STATUS_PLANNING, bag);
    write_bag(msg_cache.origin_node_status_apa_planning_msgs(),
              MSIM_TOPIC_NODE_STATUS_APA_PLANNING, bag);
    write_bag(msg_cache.new_node_status_apa_planning_msgs(),
              TOPIC_NODE_STATUS_APA_PLANNING, bag);
    write_bag(msg_cache.origin_sys_planning_request_msgs(),
              TOPIC_CP_PLANNING_REQUEST, bag);
    write_bag(msg_cache.origin_sys_planning_request_msgs(),
              MSIM_TOPIC_CP_PLANNING_REQUEST, bag);
    write_bag(msg_cache.origin_sys_planning_request_msgs(),
              TOPIC_PLANNING_REQUEST, bag);
    write_bag(msg_cache.origin_node_status_planning_msgs(),
              TOPIC_NODE_STATUS_PLANNING, bag);
  } else {
    write_bag(msg_cache.origin_module_status_msgs(), TOPIC_MODULE_STATUS, bag);
    write_bag(msg_cache.origin_chassis_report_msgs(), TOPIC_CHASSIS_REPORT,
              bag);
    write_bag(msg_cache.origin_wheel_report_msgs(), TOPIC_WHEEL_REPORT, bag);
    write_bag(msg_cache.origin_body_report_msgs(), TOPIC_BODY_REPORT, bag);
    write_bag(msg_cache.origin_perception_fusion_object_result_msgs(),
              TOPIC_FUSION_OBJECTS, bag);
    write_bag(msg_cache.origin_perception_vision_lane_msgs(),
              TOPIC_PERCEPTION_VISION_LANE, bag);
    write_bag(msg_cache.origin_perception_vision_landmark_msgs(),
              TOPIC_PERCEPTION_VISION_LANDMARK, bag);
    write_bag(msg_cache.origin_mla_localization_msgs(), TOPIC_EGO_POSE, bag);
    write_bag(msg_cache.origin_sys_planning_request_msgs(),
              TOPIC_CP_PLANNING_REQUEST, bag);
    write_bag(msg_cache.origin_fusion_parking_slot_msgs(),
              TOPIC_PERCEPTION_FUSION_PARKING_SLOT, bag);
    write_bag(msg_cache.origin_worldmodel_fusion_apa_msgs(),
              TOPIC_WORLDMODEL_PARKING_SLOT, bag);
  }
}

template <typename RosMessageType>
uint64_t get_ros_header_timestamp(const RosMessageType &m) {
  return m.header.stamp.toNSec();
}

template <> uint64_t get_ros_header_timestamp(const std_msgs::Header &m) {
  return m.stamp.toNSec();
}

static const std::unordered_map<
    std::string, std::function<uint64_t(const rosbag::MessageInstance &)>>
    supported_pass_through_topics{
        {"/perception/radar/result_front",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<
                   perception_interface_msgs::RadarPerceptionResult>()));
         }},
        {"/perception/radar/result_lc",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<
                   perception_interface_msgs::RadarPerceptionResult>()));
         }},
        {"/perception/radar/result_lr",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<
                   perception_interface_msgs::RadarPerceptionResult>()));
         }},
        {"/perception/radar/result_rc",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<
                   perception_interface_msgs::RadarPerceptionResult>()));
         }},
        {"/perception/radar/result_rr",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<
                   perception_interface_msgs::RadarPerceptionResult>()));
         }},
        {"/perception/vision/object",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<perception_interface_msgs::ObjectPerception>()));
         }},
        {"/perception/vision/object/detection_raw",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<perception_interface_msgs::ObjectPerception>()));
         }},
        {"/perception/lidar/object/detection",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<
                   perception_interface_msgs::LidarPerceptionResult>()));
         }},
        {"/perception/lidar/object/detection_raw",
         [](const rosbag::MessageInstance &m) -> uint64_t {
           return get_ros_header_timestamp(
               *(m.instantiate<
                   perception_interface_msgs::LidarPerceptionResult>()));
         }}};

static bool is_in_microseconds(const uint64_t timestamp) {
  return timestamp < 1e18 && timestamp > 1e15;
}

static void
pass_through_origin_msg(rosbag::Bag &bag, const std::string &ori_bag_name,
                        const std::vector<std::string> &pass_through_topics) {
  rosbag::Bag ori_bag;
  ori_bag.open(ori_bag_name, rosbag::bagmode::Read);

  rosbag::View view(ori_bag, rosbag::TopicQuery(pass_through_topics));
  for (const auto &m : view) {
    const auto &topic_info = supported_pass_through_topics.find(m.getTopic());
    if (topic_info == supported_pass_through_topics.end()) {
      LOG(ERROR) << "not supported pass throw topic: " << m.getTopic();
      continue;
    }
    auto func_get_timestamp = topic_info->second;
    uint64_t timestamp = func_get_timestamp(m);
    if (is_in_microseconds(
            timestamp)) { // fix bug of timestamp unit error for some topic
      timestamp *= MICRO_NS;
    }
    const auto &topic = m.getTopic();
    ros::Time ros_time;
    ros_time.fromNSec(timestamp);
    bag.write(topic, ros_time, m);
  }
  ori_bag.close();
}

void write_rosbag(const std::string &bag_path, const MafMessageCache &msg_cache,
                  const bool is_closed_loop, const std::string &ori_bag_path,
                  const std::vector<std::string> &pass_through_topics) {
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Write);
  bag.setCompression(rosbag::CompressionType::LZ4);
  pass_through_origin_msg(bag, ori_bag_path, pass_through_topics);
  write_bag(msg_cache, is_closed_loop, &bag);
  bag.close();
}

} // namespace msc
