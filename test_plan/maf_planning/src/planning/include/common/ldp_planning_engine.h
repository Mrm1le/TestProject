#ifndef MSQUARE_LDP_PLANNING_COMMON_PLANNING_ENGINE_H_
#define MSQUARE_LDP_PLANNING_COMMON_PLANNING_ENGINE_H_

#include <math.h> /* asin */
#include <mutex>

#include "common/ess_planning_task.hpp"
#include "common/ldp_planning_task.hpp"
#include "common/timer_task.hpp"
#include "mtaskflow/mtaskflow.hpp"
#include "planning_task_interface.h"
#include "pnc/ldp_planning_engine_interface.h"

using namespace msd_planning;

namespace msquare {

class LdpPlanningEngine : public LdpPlanningEngineInterface {
public:
  LdpPlanningEngine(const MSDLdpPlanningConfig &planning_config);
  ~LdpPlanningEngine();

  virtual bool init() override;
  virtual bool start() override;
  virtual bool stop() override;
  virtual bool reset() override;
  virtual bool exit() override;

  virtual void feed_tick(uint64_t tick) override;

  virtual void feed_module_status(
      const maf_framework_status::ModuleStatus &module_status) override;

  virtual void feed_chassis_report(
      const std::shared_ptr<maf_endpoint::ChassisReport> chassis_report)
      override;
  virtual void feed_wheel_report(
      const std::shared_ptr<maf_endpoint::WheelReport> wheel_report) override;
  virtual void feed_body_report(
      const std::shared_ptr<maf_endpoint::BodyReport> body_report) override;
  virtual void feed_perception_vision_lane(
      const std::shared_ptr<maf_perception_interface::RoadLinePerception>
          perception_vision_lane) override;
  virtual void feed_perception_fusion_result(
      const std::shared_ptr<
          maf_perception_interface::PerceptionFusionObjectResult>
          perception_fusion_result) override;
  virtual void feed_perception_fusion_aeb_result(
      const std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>
          perception_fusion_aeb_result) override;

  virtual void feed_mff_planning_request(const std::string &request) override;
  virtual void feed_mpc_trajectory(
      const std::shared_ptr<maf_planning::MpcTrajectoryResult> mpc_trajectory);

  virtual void set_callback(MSDPlanningOutputCallback callback) override;
  virtual void set_callback(MSDPlanningLdpOutputCallback callback) override;
  virtual void set_ess_callback(MSDPlanningOutputCallback callback) override;
  virtual void set_ess_callback(MSDPlanningLdpOutputCallback callback) override;
  virtual void set_callback(MSDPlanningNodeStatusCallback callback) override;

private:
  //   std::shared_ptr<ModuleMonitor> monitor_;
  mtaskflow::FlowMachine machine_{};
  bool is_machine_running_ = false;

  MSDLdpPlanningConfig planning_config_{};

  bool enable_timer_tick_{true};
  mtaskflow::FlowPublisher<uint64_t> tick_publisher_{};

  mtaskflow::FlowPublisher<maf_framework_status::ModuleStatus>
      module_status_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_endpoint::ChassisReport>>
      chassis_report_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_endpoint::WheelReport>>
      wheel_report_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_endpoint::BodyReport>>
      body_report_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>
      perception_vision_lane_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::PerceptionFusionObjectResult>>
      perception_fusion_result_publisher_{};
  mtaskflow::FlowPublisher<
      std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>>
      perception_fusion_aeb_result_publisher_{};

  mtaskflow::FlowPublisher<std::string> system_control_cmd_request_publisher_{};
  mtaskflow::FlowPublisher<std::string> mff_planning_request_publisher_{};
  mtaskflow::FlowPublisher<std::shared_ptr<maf_planning::MpcTrajectoryResult>>
      mpc_trajectory_publisher_{};

  std::shared_ptr<PlanningTaskInterface> planning_task_{};
  std::shared_ptr<PlanningTaskInterface> ess_planning_task_{};
};

} // namespace msquare

#endif
