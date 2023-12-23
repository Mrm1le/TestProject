#ifndef MSQUARE_DECISION_PLANNING_PLANNER_TASKS_OBSTACLE_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_TASKS_OBSTACLE_DECIDER_H_

#include "common/extract_path_bound.h"
#include "planner/tasks/task.h"

#include "data_driven_planner/common/basic_types.h"
#include "data_driven_planner/common/ddp_context.h"
#include "planner/tasks/obstacle_decider_preprocessor.h"
#include <array>

namespace msquare {

class ObstacleDecider : public Task {

public:
  ObstacleDecider(const TaskConfig &config);
  virtual ~ObstacleDecider();

  virtual void init(std::shared_ptr<WorldModel> world_model);

  void reset(const TaskConfig &config);

  void unset();

  TaskStatus execute(ScenarioFacadeContext *context);

public:
  int get_call_count() { return call_count_; };

private:
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  std::shared_ptr<BaseLineInfo> odc_baseline_info_;

  std::shared_ptr<ObstacleDeciderPreprocessor> odc_preprocessor_;

  vehicle_param veh_param_;

  ObstacleDeciderInput obstacle_decider_input_;

  ObstacleDeciderOutput obstacle_decider_output_;

private:
  void update_call_count(void) { call_count_++; };

  void clear_obstacle_decider_output(void);

  void stamp_create_time(void);

  void fake_ddp_trajectory(void);

  void load_ddp_trajectory(void);

  void load_ddp_multipath(void);

  ddp::DdpTrajectory
  process_ddp_trajectory(const ddp::DdpTrajectory &ddp_trajectory);

  void conclude_trajectory_decision(void);

  void conclude_lateral_obstacle_decision(void);

  void conclude_longitudinal_obstacle_decision(void);

  void conclude_longitudinal_obstacle_decision_new(void);

  std::vector<planning_math::Polygon2d>
  generate_ddp_virtual_lane(const ddp::DdpTrajectory &ddp_trajectory);

  void set_debug_info(void);

  void set_obstacle_decider_output(void);

  ddp::DdpTrajectory
  extend_ddp_trajectory(const ddp::DdpTrajectory &ddp_trajectory);

  int check_overlap(const Point2D &obj_frenet,
                    const pair<double, double> &obj_size,
                    const ddp::DdpTrajectory &ddp_trajectory,
                    const pair<double, double> &ego_size);

  int check_overlap(const Point2D &obj_frenet,
                    const pair<double, double> &obj_size,
                    const Point2D &ego_frenet,
                    const pair<double, double> &ego_size);

  odc_interface::Point2d env2local_pos(odc_interface::Point2d point_env);

  PointXY env2local_pos(Point2D point_env);

  int call_count_{0};

  double odc_created_time_{0.0};

  bool use_eftp_{false};

  vector<ddp::DdpTrajectory> ddp_trajectories_{};
};

} // namespace msquare
#endif