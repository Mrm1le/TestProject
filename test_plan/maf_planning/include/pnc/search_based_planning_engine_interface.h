#pragma once

#include "pnc.h"
#include <memory>

namespace msquare {

using SBPResultCallback =
    std::function<void(const maf_planning::SBPResult &sbp_result)>;

class SearchBasedPlanningEngineInterface {
public:
  MSD_API virtual ~SearchBasedPlanningEngineInterface() = default;

  MSD_API static std::shared_ptr<SearchBasedPlanningEngineInterface>
  make(const std::string &vehicle_calib_file,
       const std::string &mtaskflow_config_file,
       const std::string &config_file_dir);
  MSD_API virtual void
  feedSBPRequest(const maf_planning::SBPRequest &sbp_request) = 0;
  MSD_API virtual void setCallback(SBPResultCallback sbp_result_cb) = 0;
  MSD_API virtual void setEnableDumpFile(bool enable_dump_file) = 0;
};

} // namespace msquare
