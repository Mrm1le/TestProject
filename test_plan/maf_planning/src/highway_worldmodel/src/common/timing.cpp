#include "mtime_core/mtime.h"
#include "worldmodel/common.h"
#include <chrono>
#include <thread>

namespace msd_worldmodel {

bool MSDWorldModel_set_time_config(const MSDWorldModelTimeConfig &config) {

  constexpr auto PARAM_KEY_TIMER = "timer";
  constexpr auto PARAM_KEY_TIMELINE_TYPE = "timeline_type";
  constexpr auto PARAM_KEY_SLEEP_DURATION = "sleep_duration";

  auto reader = mjson::Reader(config.basic_config);
  reader.expand_environment_varialbe();
  auto timer_reader = mjson::Reader(reader.get<mjson::Json>(PARAM_KEY_TIMER));

  mtime::MTimeConfig time_config{};

  auto timeline_type_str =
      timer_reader.get<std::string>(PARAM_KEY_TIMELINE_TYPE);
  if (timeline_type_str == "system") {
    time_config.timeline_type = mtime::MTIMELINE_TYPE_SYSTEM;
  } else if (timeline_type_str == "mfr") {
    time_config = config.time_config;
    if (time_config.timeline_type !=
        mtime::MTIMELINE_TYPE_mtime_customize_timeline) {
      MSD_LOG(ERROR, "No support timeline type: %s\n",
              timeline_type_str.c_str());
      return false;
    }
  } else {
    MSD_LOG(ERROR, "No support timeline type: %s\n", timeline_type_str.c_str());
    return false;
  }

  auto sleep_duration = timer_reader.get<int>(PARAM_KEY_SLEEP_DURATION);
  time_config.sleep_duration = std::uint32_t(sleep_duration);

  bool res = MTIME()->init(time_config);
  if (res) {
    MSD_LOG(INFO, "(%s)mtime init succeed, timeline type: %d.\n", __FUNCTION__,
            (int)MTIME()->timeline_type());
  } else {
    MSD_LOG(ERROR, "(%s)mtime init failed, timeline type: %d.\n", __FUNCTION__,
            (int)MTIME()->timeline_type());
    return false;
  }
  return true;
}
} // namespace msd_worldmodel
