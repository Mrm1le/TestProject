#pragma once

#include "planner/motion_planner/planner_preprocessor.h"
#include "planner/tasks/task.h"

namespace msquare {

class ObstacleHeadWayPreprocessor : public PlannerPreprocessor {
public:
  ObstacleHeadWayPreprocessor() : PlannerPreprocessor(){};

  virtual void process();

private:
  void object_extraction();

  void update_headway();

  void lc_assist_decide();

  void set_headway_from_gap_select_decison();
};

} // namespace msquare
