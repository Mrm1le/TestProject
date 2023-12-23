#pragma once

#include <cstdint>
#include <memory>

#include "common/common.h"
#include "worldmodel/worldmodel.h"
#include <vector>

namespace cp_worldmodel {
struct MSDWorldModelConfig {
  std::string config_json;
  bool use_sync_signal{false};
};

using MLALocalizationPtr =
    std::shared_ptr<maf_mla_localization::MLALocalization>;
using PerceptionLanePtr =
    std::shared_ptr<maf_perception_interface::RoadLinePerception>;
using PerceptionFusionObjectResultPtr =
    std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>;
using ObjectsInterfacePtr = std::shared_ptr<maf_worldmodel::ObjectsInterface>;
using ProcessedMapPtr = std::shared_ptr<maf_worldmodel::ProcessedMap>;
using NodesStatusPtr = std::shared_ptr<maf_framework_status::NodesStatus>;

using MSDMapCallback = std::function<void(ProcessedMapPtr processed_map)>;
using MSDObjectsCallback =
    std::function<void(ObjectsInterfacePtr object_interface)>;

/// @brief The callback function for worldmodel status result.
///
/// @param[in] status The result of worldmodel module status.
using MSDNodesStatusCallback = std::function<void(NodesStatusPtr status)>;

using CalcFinishCallback = std::function<void()>;

using HrzResponseCallback =
    std::function<void(const maf_std::Header &response)>;

using ModeSwitchCmdCallback = std::function<void(
    const maf_system_manager::SysWorldModelResponse &response)>;

using WorldmodelInfoCallback =
    std::function<void(const maf_std::Header &response)>;

using MatchedEgoPositionCallback =
    std::function<void(const maf_std::Header &response)>;

using SystemResetCallback =
    std::function<void(const maf_std::Header &response)>;

} // namespace cp_worldmodel
