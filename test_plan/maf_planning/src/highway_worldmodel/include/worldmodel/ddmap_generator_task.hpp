
#pragma once

#include <memory>

#include "msd/worldmodel/worldmodel_generator.h"
#include "mtaskflow/mtaskflow.hpp"

namespace msd_worldmodel {
namespace worldmodel_v1 {

class DdmapGeneratorTask : public mtaskflow::FlowTask {
public:
  virtual void reset() = 0;
  virtual void clear_all_recievers() = 0;
  static std::shared_ptr<DdmapGeneratorTask>
  make(mtaskflow::FlowReceiver<PerceptionLanePtr> perception_lane_receiver,
       mtaskflow::FlowReceiver<MLALocalizationPtr> localization_receiver,
       mtaskflow::FlowPublisher<ProcessedMapPtr> ddmap_result_publisher,
       mtaskflow::FlowReceiver<maf_system_manager::ModuleControlCmdRequest>
           planning_control_cmd_request_receiver);
};

} // namespace worldmodel_v1
} // namespace msd_worldmodel
