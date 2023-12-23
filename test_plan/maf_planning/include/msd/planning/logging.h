#pragma once
#include "macro.h"
#include "mlog_core/mlog.h"
#include <string>
#include <vector>

namespace msd_planning {

enum MSDPlanningLogDataType {
  MSD_PLANNING_LOG_DATA_TYPE_NORMAL,        ///< Record normal log.
  MSD_PLANNING_LOG_DATA_TYPE_MSG_TIMESTAMP, ///< Record time stamp of key
                                            ///< points.
  MSD_PLANNING_LOG_DATA_TYPE_MSG_RELATION,  ///< Record relationship between
                                            ///< record data.
};

struct MSDPlanningLogEntry {
  MSDPlanningLogDataType type;
  std::string data;
  uint64_t timestamp_ns;
};

typedef void (*MSDPlanningLogCallback)(
    const std::vector<MSDPlanningLogEntry> &data);

MSD_API void MSDPlanning_set_log_config(const std::string &config,
                                        void *node_handle = nullptr);
MSD_API void MSDPlanning_set_log_callback(mlog::MLogEndpointCallback callback);
MSD_API void MSDPlanning_init_log_manager();

MSD_API void MSDPlanning_record_timestamp(const std::string &msg_id,
                                          const std::string &tag,
                                          uint64_t timestamp_ns);
MSD_API void MSDPlanning_record_relation(const std::string &current_msg_id,
                                         const std::string &relevant_msg_id);

constexpr auto kModuleStatusTag = "module-status";
constexpr auto kImuReportTag = "imu-report";
constexpr auto kChassisReportTag = "chassis-report";
constexpr auto kWirelessChargerReportTag = "wireless-charger-report";
constexpr auto kWheelReportTag = "wheel-report";
constexpr auto kBodyReportTag = "body-report";
constexpr auto kWorldModelMapTag = "world-model-map";
constexpr auto kWorldModelObjectTag = "world-model-object";
constexpr auto kWorldModelParkingSlotTag = "world-model-parking-slot";
constexpr auto kWorldModelSceneObjectTag = "world-model-scene-object";
constexpr auto kTrafficLightTag = "traffic-light";
constexpr auto kFusionObjectTag = "fusion-object";
constexpr auto kFusionGroundLineTag = "fusion-Ground-Line";
constexpr auto kFusionUssTag = "fusion-uss";
constexpr auto kEgoPoseTag = "ego-pose";
constexpr auto kPredictionTag = "prediction";
constexpr auto kControlCommandTag = "control-command";
constexpr auto kMpcTrajectorydTag = "mpc-trajectory";
constexpr auto kMapPlanningTag = "map-planning";
constexpr auto kPlanTag = "plan";
constexpr auto kLdpPlanTag = "ldp_plan";
constexpr auto kTriggerTag = "pnc-plan-TRIGGER";
constexpr auto kPerceptionVisionLaneTag = "perception-vision-lane";
constexpr auto kPerceptionVisionLandmarkTag = "perception-vision-landmark";
constexpr auto kPerceptionFusionTag = "perception-fusion";
constexpr auto kPerceptionRararTag = "perception-radar";
constexpr auto kPerceptionLidarRoadEdgeTag = "perception-lidar-road-edge";

inline std::string MSDPlanning_msg_tag(const char *tag) {
  return std::string("pnc_") + tag + "_recv";
}

} // namespace msd_planning
