#include "planner/motion_planner/optimizers/openspace_optimizer/environment_generator.h"

namespace msquare {

class PerpendicularScenarioAdapter {
public:
  PerpendicularScenarioAdapter();
  ~PerpendicularScenarioAdapter(){};
  bool init(planning_math::Vec2d &p_left, planning_math::Vec2d &p_right,
            std::vector<planning_math::Vec2d> &points_of_obstacles,
            Pose2D &init_pose, Pose2D &local_frame_pose, bool &is_on_left);
  bool process();
  void get_local_env_points(std::vector<planning_math::Vec2d> &points);
  void get_global_env_points(std::vector<planning_math::Vec2d> &points);
  double get_channel_width();
  double get_slot_width();
  double get_inner_space_width();
  double get_outside_space_width();
  double get_inner_obs_height();
  double get_outside_obs_height();
  // double get_front_opening_depth();
  // bool is_deep_opening_scenario();

private:
  bool is_init_plan_pose_ = false;
  bool is_replan_pose_ = false;

  EnvironmentGenerator env_generator_;
};

} // namespace msquare