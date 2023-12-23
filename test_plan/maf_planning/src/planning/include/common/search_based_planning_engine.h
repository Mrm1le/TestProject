#pragma once
#include "common/search_based_planning_task.hpp"
#include "maf_interface/maf_planning.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "pnc/search_based_planning_engine_interface.h"

namespace msquare {
namespace parking {

class SBPTasksResultAggregateTask : public mtaskflow::FlowTask {
private:
  mtaskflow::FlowReceiver<SbpResult>
      sbp_result_rcv_{}; // multimple tasks is to be supported by changing
                         // SbpResult to std::vector<SbpResult>
  std::function<void(const SbpResult &sbp_result)> sbp_tasks_result_callback_;

public:
  SBPTasksResultAggregateTask(mtaskflow::FlowReceiver<SbpResult> sbp_result_rcv)
      : sbp_result_rcv_(sbp_result_rcv) {}
  ~SBPTasksResultAggregateTask() {}

  void
  setCallback(std::function<void(const SbpResult &sbp_result)> sbp_result_cb) {
    sbp_tasks_result_callback_ = sbp_result_cb;
  }

  void on_running() {
    while (!sbp_result_rcv_->empty()) {
      SbpResult sbp_result{};
      auto ret = sbp_result_rcv_->pop_oldest(sbp_result);
      if (!ret) {
        continue;
      }
      sbp_tasks_result_callback_(sbp_result);
    }
  }
};

class SearchBasedPlanningEngine : public SearchBasedPlanningEngineInterface {
private:
  void onCallbackSbpTasks(const SbpResult &sbp_tasks_result);
  mtaskflow::FlowMachine machine_{};
  std::shared_ptr<SBPlanningTask> sbp_task_; // multimple tasks is to be
                                             // supported by changing
                                             // SBPlanningTask to
                                             // std::vector<SBPlanningTask>
  std::shared_ptr<SBPTasksResultAggregateTask> sbp_tasks_result_aggrerate_task_;
  mtaskflow::FlowPublisher<OpenspaceDeciderOutput> sbp_problem_publisher_{};
  SBPResultCallback sbp_result_cb_;

  void setEnableDumpFile(bool enable_setting) {
    enable_dump_file_ = enable_setting;
  }
  bool enable_dump_file_ = false;

public:
  SearchBasedPlanningEngine(const std::string &vehicle_calib_file,
                            const std::string &mtaskflow_config_file,
                            const std::string &config_file_dir);
  ~SearchBasedPlanningEngine();

  virtual void feedSBPRequest(const maf_planning::SBPRequest &sbp_request);
  virtual void setCallback(SBPResultCallback sbp_result_cb) {
    sbp_result_cb_ = sbp_result_cb;
  }

  double request_timestamp_us_ = 0.0;
  maf_actionlib::GoalID request_goal_id_;
};

} // namespace parking

} // namespace msquare
