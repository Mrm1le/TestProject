#pragma once
#include "maf_interface/maf_perception_interface.h"
#include "msd/worldmodel/worldmodel/worldmodel.h"
#include "msd/worldmodel/worldmodel_generator.h"
// #include "worldmodel/worldmodel_v1/mdk_wrapper.hpp"

namespace msd_worldmodel {
namespace worldmodel_v1 {
class FusionFilter {
public:
  virtual void
  filter(std::vector<maf_worldmodel::ObjectInterface> &fusion_objects,
         const ProcessedMapPtr &processed_map_ptr,
         const MLALocalizationPtr &localization_ptr, bool is_ddmap) = 0;

  static std::shared_ptr<FusionFilter> make();
  virtual ~FusionFilter() = default;
};

} // namespace worldmodel_v1
} // namespace msd_worldmodel
