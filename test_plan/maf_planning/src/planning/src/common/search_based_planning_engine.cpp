#include "common/search_based_planning_engine.h"
#include "common/planning_context.h"
#include "common/search_based_planning_utils.h"
#include "nlohmann/json.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"

namespace msquare {
namespace parking {

SearchBasedPlanningEngine::SearchBasedPlanningEngine(
    const std::string &vehicle_calib_file,
    const std::string &mtaskflow_config_file,
    const std::string &config_file_dir) {
  msquare::PlanningContext::Instance()->set_config_file_dir(config_file_dir);
  msquare::parking::PlanningContext::Instance()->set_config_file_dir(
      config_file_dir);

  (void)machine_.init_config({mtaskflow_config_file.c_str(), "SbpPlanning"});

  auto problem_resource = machine_.create_resource<OpenspaceDeciderOutput>();
  auto result_resource = machine_.create_resource<SbpResult>();

  sbp_problem_publisher_ = problem_resource->create_publisher();

  mtaskflow::TaskConfig task_config{};

  task_config.config_key = "SbpPlanTask";
  sbp_task_ = machine_.create_task<SBPlanningTask>(
      task_config, problem_resource->create_receiver(),
      result_resource->create_publisher());

  // in case of multiple sbp tasks, SBPTasksResultAggregateTask can cope with
  // them.
  task_config.config_key = "SbpResultAggregateTask";
  sbp_tasks_result_aggrerate_task_ =
      machine_.create_task<SBPTasksResultAggregateTask>(
          task_config, result_resource->create_receiver());
  sbp_tasks_result_aggrerate_task_->setCallback(
      std::bind(&SearchBasedPlanningEngine::onCallbackSbpTasks, this,
                std::placeholders::_1));

  // std::string vehicle_calib_file =
  // "/home/ros/catkin_ws/src/maf_planning/src/planning/include/common/config/scenario_configs_json/parking/vehicle_param.yaml";
  // std::string config_file =
  // "/home/ros/catkin_ws/src/maf_planning/src/planning/include/common/config/scenario_configs_json/parking/apa.yaml";
  if (!CarParams::GetInstance()->loadFile(vehicle_calib_file)) {
    // MSD_LOG(ERROR, "openspace strategy params config file load failed!\n");
  }
  if (!VehicleParam::Instance()->loadFile(vehicle_calib_file)) {
    // MSD_LOG(ERROR, "openspace strategy params config file load failed!\n");
  }
  // CarParams::GetInstance()->loadFile4Plan(config_file);
  // HybridAstarConfig::GetInstance()->loadFile(config_file);

  (void)machine_.run();
}

SearchBasedPlanningEngine::~SearchBasedPlanningEngine() {
  (void)machine_.stop();
}

void SearchBasedPlanningEngine::feedSBPRequest(
    const maf_planning::SBPRequest &sbp_request) {
  request_timestamp_us_ = sbp_request.meta.timestamp_us;
  nlohmann::json param_json_obj = nlohmann::json::parse(
      sbp_request.task_config.problem_config.params_string);
  request_goal_id_ = sbp_request.task_info.goal_id;
  deserialization(param_json_obj["HybridAstarConfig"],
                  HybridAstarConfig::GetInstance());
  deserialization(param_json_obj["VehicleParam"], VehicleParam::Instance());
  deserialization(param_json_obj["CarParams"], CarParams::GetInstance());
  deserialization(param_json_obj["StrategyParams"],
                  StrategyParams::GetInstance());
  deserialization(param_json_obj["TrajectoryOptimizerConfig"],
                  TrajectoryOptimizerConfig::GetInstance());
  OpenspaceDeciderOutput problem = param_json_obj["OpenspaceDeciderOutput"];

  sbp_problem_publisher_->publish(problem);
}

void SearchBasedPlanningEngine::onCallbackSbpTasks(
    const SbpResult &sbp_tasks_result) {
  maf_planning::SBPResult sbp_result_pub{};
  auto timestamp = MTIME()->timestamp("publish");
  sbp_result_pub.header.stamp = timestamp.ns();
  sbp_result_pub.header.frame_id = "map";
  sbp_result_pub.meta.timestamp_us = request_timestamp_us_;
  maf_planning::TaskStatus task_status{};
  task_status.available |= maf_planning::TaskStatus::RESULT;
  task_status.goal_id = request_goal_id_;
  convert_to_maf(sbp_tasks_result, task_status.result);

  // auto_fill_accumulated_s
  SbpResult sbp_result_tmp(sbp_tasks_result);
  if (sbp_result_tmp.x.empty())
    sbp_result_tmp.accumulated_s.clear();
  else {
    sbp_result_tmp.accumulated_s.resize(sbp_result_tmp.x.size());
    sbp_result_tmp.accumulated_s[0] = 0;
    for (std::size_t i = 1; i < sbp_result_tmp.x.size(); i++) {
      double ds = std::hypot(sbp_result_tmp.x[i] - sbp_result_tmp.x[i - 1],
                             sbp_result_tmp.y[i] - sbp_result_tmp.y[i - 1]);
      sbp_result_tmp.accumulated_s[i] =
          sbp_result_tmp.accumulated_s[i - 1] + ds;
    }
  }
  // add extra json
  nlohmann::json extra_json_obj;
  extra_json_obj["accumulated_s"] = sbp_result_tmp.accumulated_s;
  std::vector<double> accumulated_t(sbp_result_tmp.accumulated_s.size(), 0);
  std::transform(sbp_result_tmp.accumulated_s.begin(),
                 sbp_result_tmp.accumulated_s.end(), accumulated_t.begin(),
                 [](double s) -> double { return s / 0.4; });
  extra_json_obj["accumulated_t"] = accumulated_t;
  // clear accumulated_s
  sbp_result_tmp.accumulated_s.clear();

  sbp_result_pub.extra.available = 1;
  sbp_result_pub.extra.json = extra_json_obj.dump();
  sbp_result_pub.task_status.push_back(task_status);

  sbp_result_cb_(sbp_result_pub);
}

} // namespace parking

} // namespace msquare
