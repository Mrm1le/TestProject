#pragma once

#include <functional>
#include <memory>
#include <string>

#include "maf_interface.h"

namespace parking {

using MLALocalizationPtr =
    std::shared_ptr<maf_mla_localization::MLALocalization>;
using FusionAPAPtr = std::shared_ptr<maf_worldmodel::FusionAPA>;
using ProcessedMapPtr = std::shared_ptr<maf_worldmodel::ProcessedMap>;
using SceneObjectsPtr = std::shared_ptr<maf_worldmodel::SceneObjects>;
using FusionParkingSlotResultPtr =
    std::shared_ptr<maf_perception_interface::FusionParkingSlotResult>;
using PerceptionFusionObjectResultPtr =
    std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>;
using FusionGroundLineResultPtr =
    std::shared_ptr<maf_perception_interface::FusionGroundLineResult>;
using NodesStatusPtr = std::shared_ptr<maf_framework_status::NodesStatus>;
using SysWorldModelResponsePtr =
    std::shared_ptr<maf_system_manager::SysWorldModelResponse>;
using ModuleControlCmdResponsePtr =
    std::shared_ptr<maf_system_manager::ModuleControlCmdResponse>;
using SystemManagerInfoPtr = std::shared_ptr<maf_std::Header>;

using FusionAPACallback = std::function<void(const FusionAPAPtr &)>;
using ProcessedMapCallback = std::function<void(const ProcessedMapPtr &)>;
using SceneObjectsCallback = std::function<void(const SceneObjectsPtr &)>;
using MapPSDCallback = std::function<void(const FusionAPAPtr &)>;
using LocalizationCallback = std::function<void(const MLALocalizationPtr &)>;
using NodesStatusCallback = std::function<void(const NodesStatusPtr &)>;
using SysWorldModelResponseCallback =
    std::function<void(const SysWorldModelResponsePtr &)>;
using ModuleControlCmdResponseCallback =
    std::function<void(const ModuleControlCmdResponsePtr &)>;

} // namespace parking