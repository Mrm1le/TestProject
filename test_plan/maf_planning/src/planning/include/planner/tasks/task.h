#ifndef MSQUARE_DECISION_PLANNING_PLANNER_TASK_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_TASK_H_

#include <memory>

#include "common/baseline_info.h"
#include "common/config/task_config.h"
#include "common/planning_context.h"
#include "common/scenario_facade_context.h"
#include "common/world_model.h"

namespace msquare {

// class ScenarioFacade;
typedef enum {
  STATUS_SUCCESS = 0,
  STATUS_FAILED = 1,
  STATUS_FORK_LOOP = 2,
  STATUS_SUCCESS_BREAK = 3,
} TaskStatus;

class Task {
public:
  Task(const TaskConfig &config);

  virtual ~Task() = default;

  virtual void init(std::shared_ptr<WorldModel> world_model);

  // virtual bool execute(std::shared_ptr<WorldModel> world_model,
  //                      std::shared_ptr<BaseLineInfo> baseline_info);

  virtual void reset(const TaskConfig &config);

  virtual void lat_reset();

  virtual void unset();

  virtual TaskStatus execute();

  virtual TaskStatus execute(ScenarioFacadeContext *context);

  // virtual bool execute(std::shared_ptr<WorldModel> world_model,
  // ScenarioFacadeContext* context);

  const std::string &name() const;

  const TaskConfig &config() const { return config_; }

protected:
  std::shared_ptr<BaseLineInfo> baseline_info_ = nullptr;
  std::shared_ptr<WorldModel> world_model_ = nullptr;
  // std::shared_ptr<ScenarioFacade> scenario_facade_;
  ScenarioFacadeContext *context_;

  TaskConfig config_;
  std::string name_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_TASK_H_
