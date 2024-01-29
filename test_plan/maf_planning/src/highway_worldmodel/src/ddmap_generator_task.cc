#include "worldmodel/ddmap_generator_task.hpp"
#include "worldmodel/common.h"
#include "worldmodel/ddmap_generator_interface.hpp"
#include "worldmodel/utility.hpp"
#include <deque>

namespace msd_worldmodel {
namespace worldmodel_v1 {

class DdmapGeneratorTaskImpl : public DdmapGeneratorTask {
public:
  explicit DdmapGeneratorTaskImpl(
      mtaskflow::FlowReceiver<PerceptionLanePtr> perception_lane_receiver,
      mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver,
      mtaskflow::FlowPublisher<ProcessedMapPtr> ddmap_result_publisher,
      mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
          planning_control_cmd_request_receiver)
      : perception_lane_receiver_(std::move(perception_lane_receiver)),
        localization_receiver_(std::move(localization_receiver)),
        ddmap_result_publisher_(std::move(ddmap_result_publisher)),
        planning_control_cmd_request_receiver_(
            std::move(planning_control_cmd_request_receiver)) {
    const char *sim_env = std::getenv("RealitySimulation");
    if (sim_env != nullptr && std::string(sim_env) == "simulation") {
      is_in_simulation_ = true;
    }
  }

  void on_init() override { ddmap_generator_ = DdmapGenerator::make(); }

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

    // sync the perception lane and egopose
    while (!localization_receiver_->empty()) {
      MLALocalizationPtr localization_ptr{};
      auto ret = localization_receiver_->pop_oldest(localization_ptr);
      if (!ret) {
        continue;
      }
      ddmap_generator_->updateLocation(localization_ptr);
    }

    while (!perception_lane_receiver_->empty()) {
      PerceptionLanePtr perception_lane_ptr{};
      auto ret = perception_lane_receiver_->pop_oldest(perception_lane_ptr);
      if (!ret) {
        continue;
      }
      ProcessedMapPtr processed_map =
          std::make_shared<maf_worldmodel::ProcessedMap>();
      if (ddmap_generator_->processLanePtr(perception_lane_ptr,
                                           processed_map)) {
        ddmap_generator_->convertTimeStamp(processed_map);
        ddmap_generator_->convertSelfPosition(processed_map);
        ddmap_generator_->convertExtraInfo(processed_map);
        ddmap_generator_->convertOther(processed_map);
        processed_map->header.seq = ddmap_out_seq_++;
        processed_map->header.stamp = MTIME()->timestamp().ns();
        processed_map->header.frame_id = get_frame_id();
        ddmap_result_publisher_->publish(processed_map);
      } else {
        if (is_in_simulation_) {
          processed_map->header.seq = ddmap_out_seq_++;
          processed_map->header.stamp = MTIME()->timestamp().ns();
          processed_map->header.frame_id = "";
          ddmap_result_publisher_->publish(processed_map);
        }
      }
    }
  }

  void reset() override { ddmap_generator_->reset(); }

  void clear_all_recievers() override {
    perception_lane_receiver_->clear();
    localization_receiver_->clear();
  }

private:
  uint64_t ddmap_out_seq_ = 1;
  bool is_in_simulation_ = false;

  std::shared_ptr<DdmapGenerator> ddmap_generator_;

  mtaskflow::FlowReceiver<PerceptionLanePtr> perception_lane_receiver_;
  mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver_;
  mtaskflow::FlowPublisher<ProcessedMapPtr> ddmap_result_publisher_;
  mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
      planning_control_cmd_request_receiver_;
};

std::shared_ptr<DdmapGeneratorTask> DdmapGeneratorTask::make(
    mtaskflow::FlowReceiver<PerceptionLanePtr> perception_lane_receiver,
    mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver,
    mtaskflow::FlowPublisher<ProcessedMapPtr> ddmap_result_publisher,
    mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
        planning_control_cmd_request_receiver) {
  return std::make_shared<DdmapGeneratorTaskImpl>(
      std::move(perception_lane_receiver), std::move(localization_receiver),
      std::move(ddmap_result_publisher),
      std::move(planning_control_cmd_request_receiver));
}

} // namespace worldmodel_v1
} // namespace msd_worldmodel
