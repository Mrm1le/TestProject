#include "common/ldp_planning_engine.h"
#include "planning/common/common.h"
#include <string>

namespace msquare {

LdpPlanningEngine::LdpPlanningEngine(
    const MSDLdpPlanningConfig &planning_config)
    : enable_timer_tick_(planning_config.enable_timer_tick) {
  PlanningContext::Instance()->set_config_file_dir(
      planning_config.config_file_dir);

  planning_config_ = planning_config;

  // TODO 需要在构造函数中指明当前模块的NodeType
  status_manager_->set_node_type(node_status::NodeType::MODULE_LDP_PLANNING);
  // TODO
  // start_notifier会单独启动一个通知线程，每1000ms调用一次callback，如果模块为了节省线程，可以手动get_status()
  status_manager_->start_notifier(100);

  // monitor_ = ModuleMonitor::make_monitor(10);

  (void)machine_.init_config(
      {planning_config.mtaskflow_config_file.c_str(), "LdpPlanning"});

  // tick resource used for frame running
  auto tick_resource = machine_.create_resource<uint64_t>();

  mtaskflow::TaskConfig task_config{};
  task_config.running_mode_config.hz = 0.0;
  task_config.running_mode_config.mode = mtaskflow::RunningMode::RESOURCE_MODE;

  // planning task
  auto module_status_resource =
      machine_.create_resource<maf_framework_status::ModuleStatus>();
  module_status_publisher_ = module_status_resource->create_publisher();

  auto chassis_report_resource =
      machine_.create_resource<std::shared_ptr<maf_endpoint::ChassisReport>>();
  chassis_report_publisher_ = chassis_report_resource->create_publisher();

  auto wheel_report_resource =
      machine_.create_resource<std::shared_ptr<maf_endpoint::WheelReport>>();
  wheel_report_publisher_ = wheel_report_resource->create_publisher();

  auto body_report_resource =
      machine_.create_resource<std::shared_ptr<maf_endpoint::BodyReport>>();
  body_report_publisher_ = body_report_resource->create_publisher();

  auto perception_vision_lane_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::RoadLinePerception>>();
  perception_vision_lane_publisher_ =
      perception_vision_lane_resource->create_publisher();

  auto perception_fusion_resource = machine_.create_resource<std::shared_ptr<
      maf_perception_interface::PerceptionFusionObjectResult>>();
  perception_fusion_result_publisher_ =
      perception_fusion_resource->create_publisher();

  auto perception_fusion_aeb_resource = machine_.create_resource<
      std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>>();
  perception_fusion_aeb_result_publisher_ =
      perception_fusion_aeb_resource->create_publisher();

  auto mff_planning_request_resource = machine_.create_resource<std::string>();
  mff_planning_request_publisher_ =
      mff_planning_request_resource->create_publisher();

  auto mpc_trajectory_resource = machine_.create_resource<
      std::shared_ptr<maf_planning::MpcTrajectoryResult>>();
  mpc_trajectory_publisher_ = mpc_trajectory_resource->create_publisher();

  auto planning_ess_status_resource = machine_.create_resource<std::string>();

  const auto rsize = mtaskflow::RECEIVER_DEFAULT_QUEUE_SIZE;
  bool snapshot = true;
  const char *env = std::getenv("RealitySimulation");
  if (env != nullptr && std::string(env) == "simulation") {
    snapshot = false;
  }
  task_config.config_key = "LdpPlanTask";
  // planning_task_ = machine_.create_task<LdpPlanningTask>(
  //     task_config, planning_config, enable_timer_tick_, status_manager_,
  //     tick_resource->create_receiver(rsize, snapshot),
  //     module_status_resource->create_receiver(rsize, snapshot),
  //     chassis_report_resource->create_receiver(),
  //     wheel_report_resource->create_receiver(),
  //     body_report_resource->create_receiver(),
  //     perception_vision_lane_resource->create_receiver(),
  //     perception_fusion_resource->create_receiver(),
  //     perception_fusion_aeb_resource->create_receiver(),
  //     mff_planning_request_resource->create_receiver(),
  //     planning_ess_status_resource->create_receiver());

  task_config.config_key = "EssPlanTask";
  task_config.running_mode_config.hz = 50.0;
  task_config.running_mode_config.mode = mtaskflow::RunningMode::FREQUENCY_MODE;
  ess_planning_task_ = machine_.create_task<EssPlanningTask>(
      task_config, planning_config, enable_timer_tick_,
      tick_resource->create_receiver(),
      module_status_resource->create_receiver(),
      chassis_report_resource->create_receiver(),
      wheel_report_resource->create_receiver(),
      body_report_resource->create_receiver(),
      perception_vision_lane_resource->create_receiver(),
      perception_fusion_resource->create_receiver(),
      perception_fusion_aeb_resource->create_receiver(),
      mff_planning_request_resource->create_receiver(),
      mpc_trajectory_resource->create_receiver(),
      planning_ess_status_resource->create_publisher());

  // machine_.run();
}

LdpPlanningEngine::~LdpPlanningEngine() {}

bool LdpPlanningEngine::init() {
  // Init: from STARTING to STOP
  (void)machine_.init();
  // StatusManager会检查当前状态是否可以切换至目标状态，如果可以，则执行pre_transition_func后，再执行切换并返回true，否则返回false
  return status_manager_->try_change_status(node_status::Status::PENDING,
                                            [this]() {
                                              // TODO *** 这里进行算法初始化 ***
                                            });
}

bool LdpPlanningEngine::start() {
  // start: from STOP to RUNNING
  return status_manager_->try_change_status(
      node_status::Status::RUNNING, [this]() {
        // TODO *** 这里启动算法 ***
        if (!is_machine_running_) {
          machine_.run();
          is_machine_running_ = true;
        }
        MSD_LOG(ERROR, "planning engine: machine start");
      });
}

bool LdpPlanningEngine::stop() {
  // stop: from running to stop
  return status_manager_->try_change_status(
      node_status::Status::PENDING, [this]() {
        // TODO *** 这里停止算法 ***
        reset();
        if (is_machine_running_) {
          machine_.stop();
          is_machine_running_ = false;
        }
        MSD_LOG(ERROR, "planning engine: machine stop");
      });
}

bool LdpPlanningEngine::reset() {
  // reset: from runningerror or stop to stop

  // TODO *** 这里重置算法 ***
  if (planning_task_ != nullptr)
    planning_task_->reset();
  ess_planning_task_->reset();
  MSD_LOG(ERROR, "planning engine: task reset");

  return true;
}

bool LdpPlanningEngine::exit() {
  // reset: from any states to stop
  return status_manager_->try_change_status(node_status::Status::STOP,
                                            [this]() {
                                              // TODO *** 这里退出算法 ***
                                            });
}

void LdpPlanningEngine::feed_tick(uint64_t tick) {
  if (!enable_timer_tick_ && nullptr != tick_publisher_) {
    tick_publisher_->publish(tick);
  }
}

void LdpPlanningEngine::feed_module_status(
    const maf_framework_status::ModuleStatus &module_status) {
  if (nullptr != module_status_publisher_ &&
      ModuleType::ENDPOINT == module_status.module_type.value) {
    module_status_publisher_->publish(module_status);
  }
}

void LdpPlanningEngine::feed_chassis_report(
    const std::shared_ptr<maf_endpoint::ChassisReport> chassis_report) {
  if (nullptr != chassis_report_publisher_) {
    chassis_report_publisher_->publish(chassis_report);
  }
}

void LdpPlanningEngine::feed_wheel_report(
    const std::shared_ptr<maf_endpoint::WheelReport> wheel_report) {
  if (nullptr != wheel_report_publisher_) {
    wheel_report_publisher_->publish(wheel_report);
  }
}

void LdpPlanningEngine::feed_body_report(
    const std::shared_ptr<maf_endpoint::BodyReport> body_report) {
  if (nullptr != body_report_publisher_) {
    body_report_publisher_->publish(body_report);
  }
}

void LdpPlanningEngine::feed_perception_vision_lane(
    const std::shared_ptr<maf_perception_interface::RoadLinePerception>
        perception_vision_lane) {
  if (nullptr != perception_vision_lane_publisher_) {
    perception_vision_lane_publisher_->publish(perception_vision_lane);
  }
}

void LdpPlanningEngine::feed_perception_fusion_result(
    const std::shared_ptr<
        maf_perception_interface::PerceptionFusionObjectResult>
        perception_fusion_result) {
  if (perception_fusion_result_publisher_ != nullptr) {
    perception_fusion_result_publisher_->publish(perception_fusion_result);
  }
}

void LdpPlanningEngine::feed_perception_fusion_aeb_result(
    const std::shared_ptr<maf_perception_interface::PerceptionFusionAEBResult>
        perception_fusion_aeb_result) {
  if (perception_fusion_aeb_result_publisher_ != nullptr) {
    perception_fusion_aeb_result_publisher_->publish(
        perception_fusion_aeb_result);
  }
}

void LdpPlanningEngine::feed_mff_planning_request(const std::string &request) {
  if (nullptr != mff_planning_request_publisher_) {
    mff_planning_request_publisher_->publish(request);
  }
}

void LdpPlanningEngine::feed_mpc_trajectory(
    const std::shared_ptr<maf_planning::MpcTrajectoryResult> mpc_trajectory) {
  if (nullptr != mpc_trajectory_publisher_) {
    mpc_trajectory_publisher_->publish(mpc_trajectory);
  }
}

void LdpPlanningEngine::set_callback(MSDPlanningOutputCallback callback) {
  if (planning_task_ != nullptr)
    planning_task_->set_callback(callback);
}

void LdpPlanningEngine::set_callback(MSDPlanningLdpOutputCallback callback) {
  if (planning_task_ != nullptr)
    planning_task_->set_callback(callback);
}

void LdpPlanningEngine::set_ess_callback(MSDPlanningOutputCallback callback) {
  ess_planning_task_->set_callback(callback);
}

void LdpPlanningEngine::set_ess_callback(
    MSDPlanningLdpOutputCallback callback) {
  ess_planning_task_->set_callback(callback);
}

void LdpPlanningEngine::set_callback(MSDPlanningNodeStatusCallback callback) {
  set_node_status_callback(callback);
}

} // namespace msquare
