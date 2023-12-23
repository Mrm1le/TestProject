#include "worldmodel/common.h"
#include "worldmodel/fusion_filter_interface.hpp"
#include "worldmodel/fusion_task.hpp"

namespace msd_worldmodel {
namespace worldmodel_v1 {

class FusionTaskImpl : public FusionTask {
public:
  explicit FusionTaskImpl(
      mtaskflow::FlowReceiver<PerceptionFusionObjectResultPtr>
          fusion_result_receiver,
      mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver,
      mtaskflow::FlowReceiver<ProcessedMapPtr> map_result_receiver,
      mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
          planning_control_cmd_request_receiver,
      mtaskflow::FlowPublisher<ObjectsInterfacePtr> wm_obj_publisher,
      bool enable_fusion_filter, bool enable_fusion_cone_filter,
      bool enable_force_filter, float force_filter_x_distance, bool is_ddmap)
      : fusion_result_receiver_(std::move(fusion_result_receiver)),
        localization_receiver_(std::move(localization_receiver)),
        map_result_receiver_(std::move(map_result_receiver)),
        planning_control_cmd_request_receiver_(
            std::move(planning_control_cmd_request_receiver)),
        wm_obj_publisher_(std::move(wm_obj_publisher)),
        enable_fusion_filter_(enable_fusion_filter),
        enable_fusion_cone_filter_(enable_fusion_cone_filter),
        enable_force_filter_(enable_force_filter),
        force_filter_x_distance_(force_filter_x_distance) {
    force_ddmap_.store(is_ddmap);
    fusion_filter_ = FusionFilter::make();
  };

  void change_ddmap(bool is_ddmap) override { force_ddmap_.store(is_ddmap); }

  void on_init() override {}

  void on_running() override {
    if (!planning_control_cmd_request_receiver_->empty()) {
      maf_system_manager::ModuleControlCmdRequest module_control_cmd_request{};
      auto ret = planning_control_cmd_request_receiver_->fetch_newest_and_clear(
          module_control_cmd_request);
      if (ret) {
        if (module_control_cmd_request.running_mode.value !=
                maf_system_manager::RunningModeEnum::PILOT ||
            module_control_cmd_request.module_control_cmd.value ==
                maf_system_manager::ModuleControlCmdEnum::PAUSE) {
          clear_all_recievers();
          return;
        }
      }
    }

    while (!localization_receiver_->empty()) {
      MLALocalizationPtr localization_ptr{};
      auto ret = localization_receiver_->pop_oldest(localization_ptr);
      if (ret) {
        localization_buffer_.emplace_back(localization_ptr);
        if (localization_buffer_.size() > MAX_LOCLIZATION_BUFFER_SIZE) {
          localization_buffer_.pop_front();
        }
      }
    }

    while (!map_result_receiver_->empty()) {
      ProcessedMapPtr cur_map_ptr{};
      auto ret = map_result_receiver_->pop_oldest(cur_map_ptr);
      if (ret) {
        cur_map_ = cur_map_ptr;
      }
    }

    while (!fusion_result_receiver_->empty()) {
      MTIME_WATCHER_UNIT_CALLBACK(
          FusionTaskImpl, [](const std::string &func_name,
                             const mtime::MTimeDuration &duration) {
            MSD_LOG(DEBUG, "[worldmodel_v1][fusion_task] Time cost: %f ms",
                    duration.ms());
          });

      PerceptionFusionObjectResultPtr fusion_result_ptr{};
      auto ret = fusion_result_receiver_->pop_oldest(fusion_result_ptr);
      if (!ret) {
        continue;
      }
      auto fusion_result = *(fusion_result_ptr);

      auto msg = std::make_shared<maf_worldmodel::ObjectsInterface>();
      msg->object_interface.reserve(
          fusion_result.perception_fusion_objects_data.size());
      msg->meta.sensor_timestamp_us = fusion_result.meta.sensor_timestamp_us;

      for (size_t idx = 0;
           idx < fusion_result.perception_fusion_objects_data.size(); idx++) {
        auto &perception_fusion_obj_data =
            fusion_result.perception_fusion_objects_data[idx];

        bool is_cone = false;
        if (enable_fusion_cone_filter_) {
          if (perception_fusion_obj_data.type_info.type.value ==
                  maf_perception_interface::ObjectTypeEnum::
                      OBJECT_TYPE_TRAFFIC_BARRIER &&
              perception_fusion_obj_data.type_info.traffic_barrier_type.value ==
                  maf_perception_interface::TrafficBarrierTypeEnum::
                      TRAFFIC_BARRIER_TYPE_CONE) {
            is_cone = true;
            MSD_LOG(DEBUG, "hack one cone..");
          }
        }
        if (!is_cone) {
          maf_worldmodel::ObjectInterface obj{};
          obj.object_fusion_data = std::move(perception_fusion_obj_data);
          msg->object_interface.push_back(std::move(obj));
        }
      }

      if (enable_fusion_filter_) {

        if (enable_force_filter_) {
          for (auto &fusion_ob : msg->object_interface) {
            if (fusion_ob.object_fusion_data.relative_position.x >
                force_filter_x_distance_) {
              fusion_ob.is_ignore = true;
            }
          }
        }

        if (cur_map_) {

          auto &cur_map_ts = cur_map_->meta.egopose_timestamp_us;
          auto &cur_obj_ts = msg->meta.sensor_timestamp_us;
          auto ts_diff = (cur_map_ts > cur_obj_ts) ? (cur_map_ts - cur_obj_ts)
                                                   : (cur_obj_ts - cur_map_ts);

          // the map is valid
          if (ts_diff < THE_VALID_TIME_DIFF) {
            MLALocalizationPtr matched_egopose{};
            bool find_egopose = false;
            for (auto &egopose : localization_buffer_) {
              auto &egopose_ts = egopose->meta.timestamp_us;

              if (egopose_ts < cur_map_ts) {
                localization_buffer_.pop_front();
                continue;
              }
              if (egopose_ts == cur_map_ts) {
                matched_egopose = localization_buffer_.front();
                find_egopose = true;
              }
              break;
            }
            if (find_egopose) {
              bool is_ddmap = force_ddmap_.load();
              fusion_filter_->filter(msg->object_interface, cur_map_,
                                     matched_egopose, is_ddmap);
            }
          }
        }
      }

      msg->header.stamp = MTIME()->timestamp().ns();
      msg->header.frame_id = get_frame_id();
      msg->header.seq = wm_obj_seq_++;
      wm_obj_publisher_->publish(msg);
    }
  }

  void clear_all_recievers() {
    fusion_result_receiver_->clear();
    localization_receiver_->clear();
    map_result_receiver_->clear();
  }

private:
  mtaskflow::FlowReceiver<PerceptionFusionObjectResultPtr>
      fusion_result_receiver_{};
  mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver_{};
  mtaskflow::FlowReceiver<ProcessedMapPtr> map_result_receiver_{};
  mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
      planning_control_cmd_request_receiver_;

  mtaskflow::FlowPublisher<ObjectsInterfacePtr> wm_obj_publisher_;
  std::deque<MLALocalizationPtr> localization_buffer_{};
  ProcessedMapPtr cur_map_{};

  std::shared_ptr<FusionFilter> fusion_filter_;

  MSDObjectsCallback callback_;
  uint32_t wm_obj_seq_ = 1;
  std::atomic<bool> force_ddmap_;

  bool enable_fusion_filter_;
  bool enable_fusion_cone_filter_;
  bool enable_force_filter_{false};
  float force_filter_x_distance_{200};

  // uint64_t timestamp_us_ = 0;
  static constexpr uint64_t MAX_LOCLIZATION_BUFFER_SIZE = 50;
  static constexpr uint64_t THE_VALID_TIME_DIFF = 1000000; // 1s
};

std::shared_ptr<FusionTask> FusionTask::make(
    mtaskflow::FlowReceiver<PerceptionFusionObjectResultPtr>
        fusion_result_receiver,
    mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver,
    mtaskflow::FlowReceiver<ProcessedMapPtr> map_result_receiver,
    mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
        planning_control_cmd_request_receiver,
    mtaskflow::FlowPublisher<ObjectsInterfacePtr> wm_obj_publisher,
    bool enable_fusion_filter, bool enable_fusion_cone_filter,
    bool enable_force_filter, float force_filter_x_distance, bool is_ddmap) {
  return std::make_shared<FusionTaskImpl>(
      std::move(fusion_result_receiver), std::move(localization_receiver),
      std::move(map_result_receiver),
      std::move(planning_control_cmd_request_receiver),
      std::move(wm_obj_publisher), enable_fusion_filter,
      enable_fusion_cone_filter, enable_force_filter, force_filter_x_distance,
      is_ddmap);
}

} // namespace worldmodel_v1
} // namespace msd_worldmodel
