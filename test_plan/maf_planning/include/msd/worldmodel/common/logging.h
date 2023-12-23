#pragma once
#include "macro.h"
#include "mlog_core/mlog.h"
#include <string>
#include <vector>

namespace msd_worldmodel {

MSD_API bool MSDWorldModel_set_log_config(const std::string &config);
MSD_API void
MSDWorldModel_set_log_callback(mlog::MLogEndpointCallback callback);
MSD_API void MSDWorldModel_init_log_manager();

} // namespace msd_worldmodel
