#ifndef MSQUARE_LDP_PLANNING_COMMON_PLANNING_ENGINE_INTERFACE_H_
#define MSQUARE_LDP_PLANNING_COMMON_PLANNING_ENGINE_INTERFACE_H_

#include <memory>

#include "pnc.h"

namespace msquare {

struct MSDLdpPlanningConfig {
  bool enable_timer_tick{true};
  std::string mtaskflow_config_file{""};
  std::string config_file_dir{""};
};

class LdpPlanningEngineInterface : public maf::CoreBase {
public:
  MSD_API virtual ~LdpPlanningEngineInterface() = default;

  MSD_API static std::shared_ptr<LdpPlanningEngineInterface>
  make(const MSDLdpPlanningConfig &planning_config);

  MSD_API virtual void feed_tick(uint64_t tick) = 0;

  MSD_API virtual void feed_module_status(
      const maf_framework_status::ModuleStatus &module_status) = 0;
  MSD_API virtual void feed_chassis_report(
      const std::shared_ptr<maf_endpoint::ChassisReport> chassis_report) = 0;
  MSD_API virtual void feed_wheel_report(
      const std::shared_ptr<maf_endpoint::WheelReport> wheel_report) = 0;
  MSD_API virtual void feed_body_report(
      const std::shared_ptr<maf_endpoint::BodyReport> body_report) = 0;
  MSD_API virtual void feed_perception_vision_lane(
      const std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_vision_lane) = 0;
  MSD_API virtual void feed_perception_fusion_result(
      const std::shared_ptr<
          maf_perception_interface::PerceptionFusionObjectResult>
          perception_fusion_result) = 0;
  MSD_API virtual void feed_perception_fusion_aeb_result(
      const std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>
          perception_fusion_aeb_result) = 0;
  MSD_API virtual void
  feed_mff_planning_request(const std::string &request) = 0;
  MSD_API virtual void
  feed_mpc_trajectory(const std::shared_ptr<maf_planning::MpcTrajectoryResult>
                          mpc_trajectory) = 0;

  MSD_API virtual void set_callback(MSDPlanningOutputCallback callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningLdpOutputCallback callback) = 0;
  MSD_API virtual void set_ess_callback(MSDPlanningOutputCallback callback) = 0;
  MSD_API virtual void
  set_ess_callback(MSDPlanningLdpOutputCallback callback) = 0;
  MSD_API virtual void set_callback(MSDPlanningNodeStatusCallback callback) = 0;
};

} // namespace msquare

#endif
