#pragma once
#include "macro.h"
#include "mph_assert.h"
#include "mtime_core/mtime.h"
#include <string>

namespace cp {

struct MSDPlanningTimeConfig {
  std::string basic_config;
  mtime::MTimeConfig time_config;
};

MSD_API void MSDPlanning_set_time_config(const MSDPlanningTimeConfig &config, mtime::MTimeClockFeeder &feeder);
} // namespace cp
