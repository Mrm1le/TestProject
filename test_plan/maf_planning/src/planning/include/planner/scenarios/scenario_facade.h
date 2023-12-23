#ifndef MSQUARE_DECISION_PLANNING_SCENARIO_FACADE_H_
#define MSQUARE_DECISION_PLANNING_SCENARIO_FACADE_H_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/baseline_info.h"
#include "common/config/scenario_facade_config.h"
#include "common/obstacle_decision_manager.h"
#include "common/planning_context.h"
#include "common/scenario_facade_context.h"
#include "common/world_model.h"
#include "planner/tasks/task.h"

namespace msquare {

// struct ScenarioFacadeContext {};

class ScenarioFacade : public std::enable_shared_from_this<ScenarioFacade> {
public:
  enum ScenarioFacadeStatus {
    STATUS_UNKNOWN = 0,
    STATUS_PROCESSING = 1,
    STATUS_DONE = 2,
  };

  ScenarioFacade(const ScenarioFacadeConfig &config);

  virtual ~ScenarioFacade() = default;

  ScenarioFacadeConfig::ScenarioFacadeType scenario_facade_type() const {
    return config_.scenario_facade_type_;
  }

  virtual bool process(std::shared_ptr<WorldModel> world_model);

  // const ScenarioFacadeStatus& get_status() const { return
  // scenario_facade_status_; }

  virtual void init();

  virtual void reset(const ScenarioFacadeConfig &config);

  virtual void unset();

  virtual void post_process();

  const std::string &name() const { return name_; }
  const std::string &get_msg() const { return msg_; }
  const std::vector<Task *> &task_list() const { return task_list_; }
  Task *FindTask(TaskConfig::TaskType task_type) const;

  const std::shared_ptr<BaseLineInfo> get_baseline_info() const {
    return baseline_info_;
  }

  const std::shared_ptr<WorldModel> get_world_model() const {
    return world_model_;
  }

  std::shared_ptr<ScenarioFacadeContext> get_scenario_context() {
    return scenario_facade_context_;
  }

  template <typename T> T *get_context_as() {
    return static_cast<T *>(scenario_facade_context_);
  }

  void output_planning_context();

  static void output_planning_context(const ScenarioFacadeContext &context);

protected:
  ScenarioFacadeStatus scenario_facade_status_ = STATUS_UNKNOWN;
  ScenarioFacadeConfig config_;
  std::shared_ptr<ScenarioFacadeContext> scenario_facade_context_ = nullptr;
  std::map<TaskConfig::TaskType, std::shared_ptr<Task>> tasks_;
  std::vector<Task *> task_list_;
  std::string name_;
  std::string msg_; // debug msg
  std::shared_ptr<WorldModel> world_model_;
  std::shared_ptr<BaseLineInfo> baseline_info_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_SCENARIO_FACADE_H_
