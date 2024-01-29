#include "msd/planning/timing.h"
#include "mjson/mjson.hpp"
#include "mtime_core/mtime.h"
#include "planning/common/logging.h"
#include "timeline/timeline_mfr.h"

namespace msd_planning {

void MSDPlanning_set_time_config(const MSDPlanningTimeConfig &config,
                                 mtime::MTimeClockFeeder &feeder) {

  constexpr auto PARAM_KEY_TIMER = "timer";
  constexpr auto PARAM_KEY_TIMELINE_TYPE = "timeline_type";

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
    time_config.timeline_type = mtime::MTIMELINE_TYPE_mtime_customize_timeline;
    std::shared_ptr<mtime::Timeline> timeline =
        std::make_shared<mtime::TimelineMFR>(time_config.node_handle);
    time_config.timeline = timeline;
    if (time_config.timeline_type !=
        mtime::MTIMELINE_TYPE_mtime_customize_timeline) {
      MSD_LOG(ERROR, "No support timeline type: %s\n",
              timeline_type_str.c_str());
      mph_assert(0);
    }
  } else if (timeline_type_str == "feeder") {
    time_config.timeline_type = mtime::MTIMELINE_TYPE_FEEDER;
    time_config.clock_feeder = &feeder;
  } else {
    MSD_LOG(ERROR, "No support timeline type: %s\n", timeline_type_str.c_str());
    mph_assert(0);
  }

  bool res = MTIME()->init(time_config);
  if (res) {
    MSD_LOG(ERROR, "(%s)mtime init succeed, timeline type: %d.\n", __FUNCTION__,
            (int)MTIME()->timeline_type());
  } else {
    MSD_LOG(ERROR, "(%s)mtime init failed, timeline type: %d.\n", __FUNCTION__,
            (int)MTIME()->timeline_type());
    mph_assert(0);
  }
}
} // namespace msd_planning
