#pragma once

#include "maf_interface/maf_perception_interface.h"
#include "maf_interface/maf_worldmodel.h"
#include "msd/worldmodel/worldmodel_generator.h"
#include "mtaskflow/mtaskflow.hpp"

#include <atomic>
#include <memory>

namespace msd_worldmodel {
namespace worldmodel_v1 {

class FusionTask : public mtaskflow::FlowTask {
public:
  virtual void change_ddmap(bool is_ddmap) = 0;

  static std::shared_ptr<FusionTask>
  make(mtaskflow::FlowReceiver<PerceptionFusionObjectResultPtr>
           fusion_result_receiver,
       mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver,
       mtaskflow::FlowReceiver<ProcessedMapPtr> map_result_receiver,
       mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
           planning_control_cmd_request_receiver,
       mtaskflow::FlowPublisher<ObjectsInterfacePtr> wm_obj_publisher,
       bool enable_fusion_filter, bool enable_fusion_cone_filter,
       bool enable_force_filter, float force_filter_x_distance, bool is_ddmap);
};

} // namespace worldmodel_v1
} // namespace msd_worldmodel
