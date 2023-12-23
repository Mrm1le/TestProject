#include "common/search_based_planning_task.hpp"
#include "common/planning_config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"
#include "planning/common/common.h"
// #include "common/utils/yaml_utils.h"
#include "common/sbp_strategy.h"

using namespace msquare;
using namespace msquare::parking;
using namespace msquare::planning_math;
using msquare::hybrid_a_star_2::HybridAstar2;

SBPlanningTask::SBPlanningTask(
    mtaskflow::FlowReceiver<OpenspaceDeciderOutput> sbp_problem_receiver,
    mtaskflow::FlowPublisher<SbpResult> sbp_result_publisher,
    mtaskflow::FlowPublisher<std::string> sbp_debug_publisher)
    : sbp_problem_receriver_(sbp_problem_receiver),
      sbp_result_publisher_(sbp_result_publisher),
      sbp_debug_publisher_(sbp_debug_publisher) {
  hybrid_a_star_solver_ = std::make_shared<HybridAstar2>(
      PlanningContext::Instance()->get_config_file_dir() +
      "/scenario_configs_json/parking/");
}

SBPlanningTask::~SBPlanningTask() {}

void SBPlanningTask::on_running() {
  std::string debug_info{};
  double timestamp = MTIME()->timestamp().sec();
  debug_info = "sbp_task on_runnning() " + std::to_string(timestamp);
  if (sbp_debug_publisher_)
    sbp_debug_publisher_->publish(debug_info);
  while (!sbp_problem_receriver_->empty()) {
    timestamp = MTIME()->timestamp().sec();
    debug_info = "sbp_task pop_oldest() " + std::to_string(timestamp);
    if (sbp_debug_publisher_)
      sbp_debug_publisher_->publish(debug_info);

    OpenspaceDeciderOutput problem{};
    auto ret = sbp_problem_receriver_->pop_oldest(problem);
    if (!ret) {
      continue;
    }
    
    reschedule("mode_running");
    timestamp = MTIME()->timestamp().sec();
    debug_info = "sbp_task construct() " + std::to_string(timestamp);
    if (sbp_debug_publisher_)
      sbp_debug_publisher_->publish(debug_info);

    if (HybridAstar2::planningCoreSupported(
            HybridAstarConfig::GetInstance()->planning_core)) {
      hybrid_a_star_solver_->setSlotTypeByPlanningCore(
          HybridAstarConfig::GetInstance()->planning_core);
      hybrid_a_star_solver_->Update(problem.map_boundary);
      solver_ = hybrid_a_star_solver_;
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 2) {
      solver_ = std::make_shared<msquare::PatternPlanner>(problem.pattern_path);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 1) {
      solver_ = std::make_shared<msquare::PerpendicularRulePlanner>(problem);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 3) {
      solver_ = std::make_shared<msquare::ParallelRulePlanner>(problem);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 4) {
      solver_ = std::make_shared<msquare::PerpendicularOutRulePlanner>(problem);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 5) {
      solver_ = std::make_shared<msquare::ParallelOutRulePlanner>(problem);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 6) {
      solver_ = std::make_shared<msquare::ObliqueRulePlanner>(problem);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 7) {
      solver_ = std::make_shared<msquare::RpaStraightPlanner>(problem);
    } else {
      // throw std::logic_error("unsupported planner!");
    }
    SbpResult result;

    timestamp = MTIME()->timestamp().sec();
    debug_info = "sbp_task plan() " + std::to_string(timestamp);
    if (sbp_debug_publisher_)
      sbp_debug_publisher_->publish(debug_info);

    auto start_timestamp = MTIME()->timestamp();
    if (std::abs(problem.map_boundary.length()) > 1e-6 &&
        std::abs(problem.map_boundary.width()) > 1e-6) {
      (void)genOpenspacePath(solver_, problem, result);
    }
    if (result.x.empty()) {
      // std::cout << "Sbp method fails!!!!!!!!" << std::endl;
    } else {
      // std::cout << "Sbp method succeeds!!!!!!!!!" << std::endl;
    }
    auto end_timestamp = MTIME()->timestamp();
    nlohmann::json duration_json;
    duration_json["calc_duration"] = (end_timestamp - start_timestamp).ms();
    result.debug_string = duration_json.dump();

    timestamp = MTIME()->timestamp().sec();
    debug_info = "sbp_task publish() " + std::to_string(timestamp);
    if (sbp_debug_publisher_)
      sbp_debug_publisher_->publish(debug_info);

    sbp_result_publisher_->publish(result);

    timestamp = MTIME()->timestamp().sec();
    debug_info = "sbp_task finish() " + std::to_string(timestamp);
    if (sbp_debug_publisher_)
      sbp_debug_publisher_->publish(debug_info);
  }
}
