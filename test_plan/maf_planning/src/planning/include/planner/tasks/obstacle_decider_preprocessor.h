#pragma once

#include "planner/motion_planner/planner_preprocessor.h"
#include "planner/tasks/task.h"
#include "planning/common/common.h"

using namespace odc_interface;

namespace msquare {
class ObstacleDeciderPreprocessor : public PlannerPreprocessor {

public:
  ObstacleDeciderPreprocessor() : PlannerPreprocessor(){};

  virtual void process();

  ObstacleDeciderInput *get_odc_input() { return obstacle_decider_input_; };

  void mutable_external_input(ObstacleDeciderInput *odc_input,
                              ObstacleDeciderOutput *odc_output) {
    obstacle_decider_input_ = odc_input;
    obstacle_decider_output_ = odc_output;
  };

private:
  void process_time_manager();
  void update_call_count() { call_count_++; };
  void update_input_ego_state();
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

private:
  ObstacleDeciderInput *obstacle_decider_input_;
  ObstacleDeciderOutput *obstacle_decider_output_;

  Transform enu2car_;
  double odc_prec_created_time_{0.0};
  int call_count_{0};
  double t_delta_{0.1};
  vector<double> t_buffer_{0.0, 0.0};

private:
  odc_interface::Point2d env2local_pos(odc_interface::Point2d point_env);
};

} // namespace msquare