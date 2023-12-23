#include "planner/tasks/task_factory.h"
#include "planner/behavior_planner/deciders/backup_path_decider.h"
#include "planner/behavior_planner/deciders/lane_change_decider.h"
#include "planner/behavior_planner/deciders/path_decider.h"
#include "planner/behavior_planner/deciders/speed_boundary_decider.h"
#include "planner/behavior_planner/general_motion_planner.h"
#include "planner/behavior_planner/lateral_behavior_planner.h"
#include "planner/behavior_planner/longitudinal_behavior_planner.h"
#include "planner/motion_planner/lateral_motion_planner.h"
#include "planner/motion_planner/longitudinal_motion_planner.h"
#include "planner/tasks/obstacle_decider.h"

namespace msquare {

std::unordered_map<std::string, memory::MemoryPoolPtr>
    TaskFactory::task_mempory_pools_;

util::Factory<TaskConfig::TaskType, Task,
              std::function<Task *(const TaskConfig &config)>,
              std::function<void(Task *ptr)>>
    TaskFactory::task_factory_;

std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
    TaskFactory::default_task_configs_;

std::unordered_map<std::string, memory::MemoryPoolPtr> &
TaskFactory::get_task_mempory_pools() {
  return task_mempory_pools_;
}

void TaskFactory::Init(
    const std::unordered_map<TaskConfig::TaskType, TaskConfig, std::hash<int>>
        &default_task_config) {
  // create static memory pool for all tasks， *** make sure no concurrent !!
  // ***
  if (task_mempory_pools_.size() == 0) {
    auto LBP_pool = std::make_shared<
        memory::SimpleMemoryPool<LateralBehaviorPlanner, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(LBP_pool));

    auto SBD_pool = std::make_shared<
        memory::SimpleMemoryPool<SpeedBoundaryDecider, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(SBD_pool));

    auto PD_pool =
        std::make_shared<memory::SimpleMemoryPool<PathDecider, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(PD_pool));

    auto BUD_pool = std::make_shared<
        memory::SimpleMemoryPool<BackUpPathDecider, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(BUD_pool));

    auto LOBP_pool = std::make_shared<
        memory::SimpleMemoryPool<LongitudinalBehaviorPlanner, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(LOBP_pool));

    auto LMP_pool = std::make_shared<
        memory::SimpleMemoryPool<LongitudinalMotionPlanner, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(LMP_pool));

    auto LAMP_pool = std::make_shared<
        memory::SimpleMemoryPool<LateralMotionPlanner, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(LAMP_pool));

    auto LCD_pool = std::make_shared<
        memory::SimpleMemoryPool<LaneChangeDecider, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(LCD_pool));

    auto ODC_pool = std::make_shared<
        memory::SimpleMemoryPool<ObstacleDecider, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(ODC_pool));

    auto GMP_pool = std::make_shared<
        memory::SimpleMemoryPool<GeneralMotionPlanner, TaskConfig>>();
    task_mempory_pools_.insert(MAP_ITEM(GMP_pool));

    // auto LPP_pool =
    //     std::make_shared<memory::SimpleMemoryPool<LCMDPPlanner,
    //     TaskConfig>>();
    // task_mempory_pools_.insert(MAP_ITEM(LPP_pool));
  }

  task_factory_.Clear();
  (void)task_factory_.Register(
      TaskConfig::TaskType::LATERAL_BEHAVIOR_PLANNER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["LBP_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["LBP_pool"]->Free(task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::SPEED_BOUNDARY_DECIDER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["SBD_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["SBD_pool"]->Free(task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::PATH_DECIDER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["PD_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["PD_pool"]->Free(task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::BACKUP_PATH_DECIDER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["BUD_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["BUD_pool"]->Free(task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::LONGITUDINAL_BEHAVIOR_PLANNER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["LOBP_pool"]
            ->Alloc(&config);
      },
      [](Task *task_ptr) -> void {
        return TaskFactory::get_task_mempory_pools()["LOBP_pool"]->Free(
            task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::LONGITUDINAL_MOTION_PLANNER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["LMP_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["LMP_pool"]->Free(task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::LATERAL_MOTION_PLANNER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["LAMP_pool"]
            ->Alloc(&config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["LAMP_pool"]->Free(task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::LANE_CHANGE_DECIDER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["LCD_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["LCD_pool"]->Free(task_ptr);
      });
  (void)task_factory_.Register(
      TaskConfig::TaskType::OBSTACLE_DECIDER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["ODC_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["ODC_pool"]->Free(task_ptr);
      });
  task_factory_.Register(
      TaskConfig::TaskType::GENERAL_MOTION_PLANNER,
      [](const TaskConfig &config) -> Task * {
        return (Task *)TaskFactory::get_task_mempory_pools()["GMP_pool"]->Alloc(
            &config);
      },
      [](Task *task_ptr) -> void {
        TaskFactory::get_task_mempory_pools()["GMP_pool"]->Free(task_ptr);
      });

  default_task_configs_ = default_task_config;
}

std::shared_ptr<Task> TaskFactory::CreateTask(const TaskConfig &task_config) {
  return task_factory_.CreateObject(task_config.task_type(), task_config);
}

} // namespace msquare
