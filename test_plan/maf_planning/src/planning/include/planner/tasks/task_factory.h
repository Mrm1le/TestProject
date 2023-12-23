#ifndef MSQUARE_DECISION_PLANNING_PLANNER_TASK_FACOTRY_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_TASK_FACOTRY_H_

#include <functional>
#include <memory>
#include <unordered_map>

#include "common/config/task_config.h"
#include "common/utils/factory.h"
#include "memory/simple_mempool.hpp" //memory
#include "planner/tasks/task.h"

namespace msquare {

#define MAP_ITEM(s) std::make_pair(#s, s)

class TaskFactory {
public:
  static void Init(
      const std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
          &default_task_config);
  static std::shared_ptr<Task> CreateTask(const TaskConfig &task_config);

  static std::unordered_map<std::string, memory::MemoryPoolPtr> &
  get_task_mempory_pools();

private:
  static util::Factory<TaskConfig::TaskType, Task,
                       std::function<Task *(const TaskConfig &config)>,
                       std::function<void(Task *ptr)>>
      task_factory_;
  static std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
      default_task_configs_;

  static std::unordered_map<std::string, memory::MemoryPoolPtr>
      task_mempory_pools_;
};

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_TASK_FACOTRY_H_
