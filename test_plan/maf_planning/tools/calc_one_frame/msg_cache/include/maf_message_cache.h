#pragma once

#include <glog/logging.h>
#include <maf_interface/maf_endpoint.h>
#include <maf_interface/maf_framework_status.h>
#include <maf_interface/maf_gps_imu.h>
#include <maf_interface/maf_hdmap.h>
#include <maf_interface/maf_planning.h>
#include <maf_interface/maf_sensor_interface.h>
#include <maf_interface/maf_std.h>
#include <maf_interface/maf_system_manager.h>
#include <maf_interface/maf_worldmodel.h>
#include <stdint.h>

#include <ctime>
#include <limits>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "msg_cache_defines.h"

namespace msc {

template <typename Msg> uint64_t get_header_timestamp(const Msg &msg) {
  return msg.header.stamp;
}

template <typename Msg>
void set_header_timestamp(const uint64_t timestamp, Msg *msg) {
  msg->header.stamp = timestamp;
}

template <> inline uint64_t get_header_timestamp(const maf_std::Header &msg) {
  return msg.stamp;
}

template <>
inline void set_header_timestamp(const uint64_t timestamp,
                                 maf_std::Header *msg) {
  msg->stamp = timestamp;
}

template <typename T> struct is_shared_ptr : std::false_type {};

template <typename T>
struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <typename MsgWrap>
std::enable_if_t<is_shared_ptr<MsgWrap>::value, MsgWrap> create_msg() {
  return std::make_shared<typename MsgWrap::element_type>();
}

template <typename MsgWrap>
std::enable_if_t<!is_shared_ptr<MsgWrap>::value, MsgWrap> create_msg() {
  return MsgWrap();
}

template <typename MsgWrap,
          std::enable_if_t<is_shared_ptr<std::remove_cv_t<
                               std::remove_reference_t<MsgWrap>>>::value,
                           bool>
              _ = true>
decltype(auto) dewrap_msg(MsgWrap &&msg) {
  return *(msg.get());
}

template <typename MsgWrap,
          std::enable_if_t<!is_shared_ptr<std::remove_cv_t<
                               std::remove_reference_t<MsgWrap>>>::value,
                           bool>
              _ = true>
decltype(auto) dewrap_msg(MsgWrap &&msg) {
  return msg;
}

template <typename Msg, bool wrap_msg_in_shared_ptr> class MessageWrapper {
public:
  typedef typename std::conditional<wrap_msg_in_shared_ptr,
                                    std::shared_ptr<Msg>, Msg>::type MsgWrap;

  MessageWrapper() : msg_{create_msg<MsgWrap>()} {}

  Msg *mutable_msg() {
    static_assert(std::is_same<Msg &, decltype(dewrap_msg(msg_))>::value, "");
    return &dewrap_msg(msg_);
  }
  const Msg &msg() const {
    static_assert(
        std::is_same<Msg &, decltype(dewrap_msg(msg_))>::value ||
            std::is_same<const Msg &, decltype(dewrap_msg(msg_))>::value,
        "");
    return dewrap_msg(msg_);
  }

  MsgWrap *mutable_msg_wrap() { return &msg_; }
  const MsgWrap &msg_wrap() const { return msg_; }
  MsgWrap msg_wrap_copy() const {
    MsgWrap tmp{create_msg<MsgWrap>()};
    dewrap_msg(tmp) = dewrap_msg(msg_);
    return tmp;
  }
  uint64_t timestamp() const { return timestamp_; }
  int frame_id() const { return frame_id_; }
  void set_timestamp(uint64_t value) { timestamp_ = value; }
  void set_frame_id(int value) { frame_id_ = value; }

private:
  MsgWrap msg_;
  uint64_t timestamp_ = 0;
  int frame_id_ = 0;
};

template <typename MessageWrapperType, bool is_const> class MafMessageFrame {
public:
  typedef typename std::conditional<
      is_const, typename std::vector<MessageWrapperType>::const_iterator,
      typename std::vector<MessageWrapperType>::iterator>::type Iterator;

  typedef MessageWrapperType MessageWrapper;
  MafMessageFrame(Iterator begin, Iterator end) : begin_{begin}, end_{end} {}

  Iterator begin() const { return begin_; }
  Iterator end() const { return end_; }
  size_t size() const { return end_ - begin_; }
  bool empty() const { return end_ == begin_; }

private:
  Iterator begin_;
  Iterator end_;
};

template <typename MessageWrapperType>
using ConstMafMessageFrame = MafMessageFrame<MessageWrapperType, true>;

template <typename MessageWrapperType>
using MutableMafMessageFrame = MafMessageFrame<MessageWrapperType, false>;

class FrameInterval {
public:
  FrameInterval(int begin, int end) : begin_{begin}, end_{end} {}
  FrameInterval() : begin_{0}, end_{0} {}
  int begin() const { return begin_; }
  int end() const { return end_; }
  void set_begin(int begin) { begin_ = begin; }
  void set_end(int end) { end_ = end; }

private:
  int begin_;
  int end_;
};

class MafMessageCache;

template <typename Msg, bool wrap_msg_in_shared_ptr = true>
class MafMessagesInFrames {
public:
  typedef MessageWrapper<Msg, wrap_msg_in_shared_ptr> MessageWrapperType;
  typedef Msg MafMessageType;

  MafMessagesInFrames() = default;
  MafMessagesInFrames(const std::string &msg_name) : msg_name_{msg_name} {}

  auto *mutable_messages() { return &messages_; }
  const auto &messages() const { return messages_; }
  size_t message_count() const { return messages_.size(); }
  size_t frame_count() const { return frames_.size(); }

  ConstMafMessageFrame<MessageWrapperType> frame(uint32_t frame_id) const {
    if (frame_id >= frames_.size()) {
      return ConstMafMessageFrame<MessageWrapperType>{messages_.cbegin(),
                                                      messages_.cbegin()};
    }
    return ConstMafMessageFrame<MessageWrapperType>{
        messages_.cbegin() + frames_[frame_id].begin(),
        messages_.cbegin() + frames_[frame_id].end()};
  }

  MutableMafMessageFrame<MessageWrapperType> mutable_frame(uint32_t frame_id) {
    if (frame_id >= frames_.size()) {
      return MutableMafMessageFrame<MessageWrapperType>{messages_.begin(),
                                                        messages_.begin()};
    }
    return MutableMafMessageFrame<MessageWrapperType>{
        messages_.begin() + frames_[frame_id].begin(),
        messages_.begin() + frames_[frame_id].end()};
  }

  // 向后追加空frame, 使frames_.size()等于frame_id+1
  void add_frame(const uint32_t frame_id) {
    if (frame_id < frames_.size()) {
      // 只能追加frame, 不能从中间插入
      LOG(ERROR) << "frame aleady exit, frame_id: " << frame_id
                 << ", msg_name: " << msg_name_;
      std::abort();
    }
    for (uint32_t i = frames_.size(); i < frame_id; i++) {
      // 补全空缺的frame
      LOG(WARNING) << "indirectly add frame, id: " << i
                   << ", msg_name: " << msg_name_;
      frames_.emplace_back(messages_.size(), messages_.size());
    }
    frames_.emplace_back(messages_.size(), messages_.size());
  }

  auto *add_message_to_last_frame() {
    if (frames_.empty()) {
      LOG(ERROR) << "frames_ is empty";
      std::abort();
    }
    messages_.emplace_back();
    frames_.back().set_end(messages_.size());
    messages_.back().set_frame_id(frames_.size() - 1);
    return &messages_.back();
  }

private:
  void reset_message_cursor() { message_cursor_ = 0; }
  void reserve_frames(uint32_t approximate_frame_count) {
    frames_.reserve(approximate_frame_count);
  }

  void set_one_frame(int frame_id, uint64_t max_timestamp) {
    int frame_begin = message_cursor_;
    while (message_cursor_ < messages_.size() &&
           messages_[message_cursor_].timestamp() < max_timestamp) {
      messages_[message_cursor_].set_frame_id(frame_id);
      message_cursor_++;
    }
    int frame_end = message_cursor_;
    frames_.emplace_back(frame_begin, frame_end);
  }

  friend class MafMessageCache;

  std::string msg_name_;
  std::vector<MessageWrapperType> messages_;
  std::vector<FrameInterval> frames_;
  uint32_t message_cursor_ = 0;
};

class FrameTimeSpan {
public:
  FrameTimeSpan(uint64_t start_time_ns, uint64_t end_time_ns)
      : start_time_ns_{start_time_ns}, end_time_ns_{end_time_ns} {}

  uint64_t start_time_ns() const { return start_time_ns_; }
  uint64_t end_time_ns() const { return end_time_ns_; }

private:
  uint64_t start_time_ns_;
  uint64_t end_time_ns_;
};

class MafMessageCache {
public:
  MafMessageCache(uint64_t time_slice_ns) : time_slice_ns_(time_slice_ns) {}

  ~MafMessageCache() = default;

  bool load_bag(const std::string &bag_path, const bool is_mfbag);

  bool segement_msgs() {
    constexpr int kEgoPoseMessageHz = 50;
    if (origin_mla_localization_msgs_.message_count() < kEgoPoseMessageHz) {
      LOG(ERROR) << "egopose messages count < " << kEgoPoseMessageHz;
      return false;
    }
    reset_message_cursor();
    reset_message_duration(
        origin_mla_localization_msgs_.messages().front().timestamp(),
        origin_mla_localization_msgs_.messages().back().timestamp());

    uint32_t approximate_frame_count = static_cast<uint32_t>(
        ((origin_mla_localization_msgs_.messages().back().timestamp() +
          time_slice_ns_ -
          origin_mla_localization_msgs_.messages().front().timestamp()) /
         time_slice_ns_) +
        1);
    reserve_frames(approximate_frame_count);

    const uint64_t begin_timestamp =
        origin_mla_localization_msgs_.messages().front().timestamp() +
        time_slice_ns_;
    const uint64_t end_timestamp =
        origin_mla_localization_msgs_.messages().back().timestamp() -
        time_slice_ns_;

    uint32_t frame_id = 0;
    for (uint64_t timestamp = begin_timestamp; timestamp < end_timestamp;
         timestamp += time_slice_ns_) {
      set_one_frame(frame_id, timestamp);
      frame_id++;
    }
    set_one_frame(frame_id, std::numeric_limits<uint64_t>::max());
    origin_frame_count_ = frame_id + 1;
    reset_message_cursor();
    return true;
  }

  uint32_t origin_frame_count() const { return origin_frame_count_; }
  uint64_t start_time_ns() {
    return origin_mla_localization_msgs_.messages().front().timestamp();
  }
  FrameTimeSpan get_frame_time_span(uint32_t frame_id) {
    return {start_time_ns() + (frame_id * time_slice_ns_),
            start_time_ns() + ((frame_id + 1) * time_slice_ns_)};
  }

private:
  template <typename Func> void for_each_origin_msgs(Func callback) {
    callback(origin_sys_planning_request_msgs_);
    callback(origin_planning_frequency_control_msgs_);
    callback(origin_module_status_msgs_);
    callback(origin_chassis_report_msgs_);
    callback(origin_wheel_report_msgs_);
    callback(origin_body_report_msgs_);
    callback(origin_traffic_light_perception_msgs_);
    callback(origin_mla_localization_msgs_);
    callback(origin_perception_fusion_object_result_msgs_);
    callback(origin_fusion_ground_line_result_msgs_);
    callback(origin_fusion_uss_ground_line_result_msgs_);
    callback(origin_prediction_result_msgs_);
    callback(origin_mpc_trajectory_msgs_);
    callback(origin_planning_msgs_);
    callback(origin_planning_info_msgs_);
    callback(origin_perception_vision_lane_msgs_);
    callback(origin_perception_vision_landmark_msgs_);
    callback(origin_perception_lidar_road_edge_msgs_);
    callback(origin_mla_imu_msgs_);
    callback(origin_processed_map_msgs_);
    callback(origin_objects_interface_msgs_);
    callback(origin_radar_perception_result_msgs_);
    callback(origin_ultrasonic_upa_report_msgs_);
    callback(origin_ultrasonic_upa_report_raw_msgs_);
    callback(origin_sbp_result_msgs_);
    callback(origin_sbp_request_msgs_);
    callback(origin_planning_reset_request_msgs_);
    callback(origin_module_control_cmd_request_msgs_);
    callback(origin_world_model_scene_objects_msgs_);
    callback(origin_fusion_parking_slot_msgs_);
    callback(origin_worldmodel_fusion_apa_msgs_);
    callback(origin_perception_fusion_obj_parking_env_msgs_);
    callback(origin_node_status_planning_msgs_);
    callback(origin_node_status_apa_planning_msgs_);
  }

  template <typename Func> void for_each_new_msgs(Func callback) {
    callback(new_processed_map_msgs_);
    callback(new_objects_interface_msgs_);
    callback(new_control_command_msgs_);
    callback(new_module_status_msgs_);
    callback(new_mla_localization_msgs_);
    callback(new_chassis_report_msgs_);
    callback(new_wheel_report_msgs_);
    callback(new_body_report_msgs_);
    callback(new_perception_fusion_object_result_msgs_);
    callback(new_perception_vision_lane_msgs_);
    callback(new_perception_vision_landmark_msgs_);
    callback(new_planning_info_msgs_);
    callback(new_planning_msgs_);
    callback(new_planning_extra_data_msgs_);
    callback(new_world_model_scene_objects_msgs_);
    callback(new_worldmodel_fusion_apa_msgs_);
    callback(new_sbp_request_msgs_);
    callback(new_node_status_planning_msgs_);
    callback(new_node_status_apa_planning_msgs_);
    callback(new_sys_planning_request_msgs_);
  }

  void reset_message_cursor() {
    for_each_origin_msgs([](auto &msgs) { msgs.reset_message_cursor(); });
  }

  void reserve_frames(uint32_t approximate_frame_count) {
    for_each_origin_msgs([approximate_frame_count](auto &msgs) {
      msgs.reserve_frames(approximate_frame_count);
    });
    for_each_new_msgs([approximate_frame_count](auto &msgs) {
      msgs.reserve_frames(approximate_frame_count);
    });
  }

  void reset_message_duration(const uint64_t begin_timestamp,
                              const uint64_t end_timestamp) {
    for_each_origin_msgs([begin_timestamp, end_timestamp](auto &msgs) {
      for (auto &msg : *(msgs.mutable_messages())) {
        if (msg.timestamp() < begin_timestamp) {
          msg.set_timestamp(begin_timestamp);
          set_header_timestamp(begin_timestamp, msg.mutable_msg());
        } else if (msg.timestamp() > end_timestamp) {
          msg.set_timestamp(end_timestamp);
          set_header_timestamp(end_timestamp, msg.mutable_msg());
        }
      }
    });
  }

  void set_one_frame(int frame_id, uint64_t max_timestamp) {
    for_each_origin_msgs([frame_id, max_timestamp](auto &msgs) {
      msgs.set_one_frame(frame_id, max_timestamp);
    });
  }

#define DECLARE_MSG(type, wrapped_in_shared_ptr, name)                         \
public:                                                                        \
  const auto &name() const { return name##_; }                                 \
  auto *mutable_##name() { return &name##_; }                                  \
                                                                               \
private:                                                                       \
  MafMessagesInFrames<type, wrapped_in_shared_ptr> name##_{#name};

  DECLARE_MSG(maf_system_manager::SysPlanningRequest, false,
              origin_sys_planning_request_msgs)
  DECLARE_MSG(maf_system_manager::SysPlanningRequest, false,
              origin_planning_frequency_control_msgs)
  DECLARE_MSG(maf_framework_status::ModuleStatus, false,
              origin_module_status_msgs)
  DECLARE_MSG(maf_endpoint::ChassisReport, true, origin_chassis_report_msgs)
  DECLARE_MSG(maf_endpoint::WheelReport, true, origin_wheel_report_msgs)
  DECLARE_MSG(maf_endpoint::BodyReport, true, origin_body_report_msgs)
  DECLARE_MSG(maf_perception_interface::TrafficLightPerception, true,
              origin_traffic_light_perception_msgs)
  DECLARE_MSG(maf_mla_localization::MLALocalization, true,
              origin_mla_localization_msgs)
  DECLARE_MSG(maf_perception_interface::PerceptionFusionObjectResult, true,
              origin_perception_fusion_object_result_msgs)
  DECLARE_MSG(maf_perception_interface::FusionGroundLineResult, true,
              origin_fusion_ground_line_result_msgs)
  DECLARE_MSG(maf_perception_interface::FusionGroundLineResult, true,
              origin_fusion_uss_ground_line_result_msgs)
  DECLARE_MSG(maf_worldmodel::PredictionResult, true,
              origin_prediction_result_msgs)
  DECLARE_MSG(maf_planning::MpcTrajectoryResult, true,
              origin_mpc_trajectory_msgs)
  DECLARE_MSG(maf_planning::Planning, true, origin_planning_msgs)
  DECLARE_MSG(maf_std::Header, false, origin_planning_info_msgs)
  DECLARE_MSG(maf_perception_interface::RoadLinePerception, true,
              origin_perception_vision_lane_msgs)
  DECLARE_MSG(maf_perception_interface::RoadLinePerception, true,
              origin_perception_vision_landmark_msgs)
  DECLARE_MSG(maf_perception_interface::RoadLinePerception, true,
              origin_perception_lidar_road_edge_msgs)
  DECLARE_MSG(maf_gps_imu::MLAImu, true, origin_mla_imu_msgs)
  DECLARE_MSG(maf_worldmodel::ProcessedMap, true, origin_processed_map_msgs)
  DECLARE_MSG(maf_worldmodel::ObjectsInterface, true,
              origin_objects_interface_msgs)
  DECLARE_MSG(maf_perception_interface::RadarPerceptionResult, true,
              origin_radar_perception_result_msgs)
  DECLARE_MSG(maf_sensor_interface::UltrasonicUpaReport, true,
              origin_ultrasonic_upa_report_msgs)
  DECLARE_MSG(maf_sensor_interface::UltrasonicUpaReport, true,
              origin_ultrasonic_upa_report_raw_msgs)
  DECLARE_MSG(maf_planning::SBPResult, true, origin_sbp_result_msgs)
  DECLARE_MSG(maf_planning::SBPRequest, true, origin_sbp_request_msgs)
  DECLARE_MSG(maf_planning::SBPRequest, true, new_sbp_request_msgs)
  DECLARE_MSG(maf_std::Header, false, origin_planning_reset_request_msgs)
  DECLARE_MSG(maf_system_manager::ModuleControlCmdRequest, false,
              origin_module_control_cmd_request_msgs)
  DECLARE_MSG(maf_worldmodel::SceneObjects, true,
              origin_world_model_scene_objects_msgs)
  DECLARE_MSG(maf_worldmodel::SceneObjects, true,
              new_world_model_scene_objects_msgs)

  DECLARE_MSG(maf_worldmodel::ProcessedMap, true, new_processed_map_msgs)
  DECLARE_MSG(maf_worldmodel::ObjectsInterface, true,
              new_objects_interface_msgs)
  DECLARE_MSG(maf_endpoint::ControlCommand, true, new_control_command_msgs)
  DECLARE_MSG(maf_framework_status::ModuleStatus, false,
              new_module_status_msgs);
  DECLARE_MSG(maf_mla_localization::MLALocalization, true,
              new_mla_localization_msgs);
  DECLARE_MSG(maf_endpoint::ChassisReport, true, new_chassis_report_msgs);
  DECLARE_MSG(maf_endpoint::WheelReport, true, new_wheel_report_msgs)
  DECLARE_MSG(maf_endpoint::BodyReport, true, new_body_report_msgs);
  DECLARE_MSG(maf_perception_interface::PerceptionFusionObjectResult, true,
              new_perception_fusion_object_result_msgs);
  DECLARE_MSG(maf_perception_interface::RoadLinePerception, true,
              new_perception_vision_lane_msgs);
  DECLARE_MSG(maf_perception_interface::RoadLinePerception, true,
              new_perception_vision_landmark_msgs);
  DECLARE_MSG(maf_std::Header, true, new_planning_info_msgs);
  DECLARE_MSG(maf_planning::Planning, true, new_planning_msgs);
  DECLARE_MSG(maf_std::Header, false, new_mdebug_msgs);
  DECLARE_MSG(maf_std::Header, true, new_planning_extra_data_msgs);

  // APA
  DECLARE_MSG(maf_perception_interface::FusionParkingSlotResult, true,
              origin_fusion_parking_slot_msgs);
  DECLARE_MSG(maf_worldmodel::FusionAPA, true,
              origin_worldmodel_fusion_apa_msgs);
  DECLARE_MSG(maf_perception_interface::PerceptionFusionObjectResult, true,
              origin_perception_fusion_obj_parking_env_msgs);
  DECLARE_MSG(maf_worldmodel::FusionAPA, true, new_worldmodel_fusion_apa_msgs);
  DECLARE_MSG(maf_framework_status::NodesStatus, true,
              origin_node_status_planning_msgs);
  DECLARE_MSG(maf_framework_status::NodesStatus, true,
              new_node_status_planning_msgs);
  DECLARE_MSG(maf_framework_status::NodesStatus, true,
              origin_node_status_apa_planning_msgs);
  DECLARE_MSG(maf_framework_status::NodesStatus, true,
              new_node_status_apa_planning_msgs);
  DECLARE_MSG(maf_system_manager::SysPlanningRequest, false,
              new_sys_planning_request_msgs);

#undef DECLARE_MSG

  uint32_t origin_frame_count_ = 0;
  const uint64_t time_slice_ns_;
};

inline void write_mfbag(const std::string &bag_path,
                        const MafMessageCache &msg_cache,
                        const bool is_closed_loop) {
  std::abort();
}

void write_rosbag(const std::string &bag_path, const MafMessageCache &msg_cache,
                  const bool is_closed_loop, const std::string &ori_bag_path,
                  const std::vector<std::string> &pass_through_topics);
} // namespace msc
