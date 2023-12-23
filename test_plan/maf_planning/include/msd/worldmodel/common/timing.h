#pragma once

#include "mtime_core/mtime.h"
#include <chrono>
#include <sys/time.h>

namespace msd_worldmodel {

struct MSDWorldModelTimeConfig {
  std::string basic_config;
  mtime::MTimeConfig time_config;
};

MSD_API bool
MSDWorldModel_set_time_config(const MSDWorldModelTimeConfig &config);

} // namespace msd_worldmodel
