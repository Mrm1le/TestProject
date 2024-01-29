#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_scenario_adapter.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <algorithm>

namespace msquare {

PerpendicularScenarioAdapter::PerpendicularScenarioAdapter() {}

bool PerpendicularScenarioAdapter::init(
    planning_math::Vec2d &p_left, planning_math::Vec2d &p_right,
    std::vector<planning_math::Vec2d> &points_of_obstacles, Pose2D &init_pose,
    Pose2D &local_frame_pose, bool &is_on_left) {
  return env_generator_.init(p_left, p_right, points_of_obstacles, init_pose,
                             local_frame_pose, is_on_left);
}

bool PerpendicularScenarioAdapter::process() {
  return env_generator_.process();
}

void PerpendicularScenarioAdapter::get_local_env_points(
    std::vector<planning_math::Vec2d> &points) {
  env_generator_.get_local_env_points(points);
}

void PerpendicularScenarioAdapter::get_global_env_points(
    std::vector<planning_math::Vec2d> &points) {
  env_generator_.get_global_env_points(points);
}

double PerpendicularScenarioAdapter::get_channel_width() {
  double channel_width = -1.0;
  if (env_generator_.get_environment().inited) {
    channel_width = env_generator_.get_environment().channel_width;
  }
  return channel_width;
}

double PerpendicularScenarioAdapter::get_inner_space_width() {
  double inner_space_width = -1.0;
  if (env_generator_.get_environment().inited) {
    inner_space_width = env_generator_.get_environment().right_slot_width;
  }
  return inner_space_width;
}

double PerpendicularScenarioAdapter::get_outside_space_width() {
  double outside_space_width = -1.0;
  if (env_generator_.get_environment().inited) {
    outside_space_width = env_generator_.get_environment().left_slot_width;
  }
  return outside_space_width;
}

double PerpendicularScenarioAdapter::get_inner_obs_height() {
  double inner_obs_height = -1.0;
  if (env_generator_.get_environment().inited) {
    inner_obs_height = env_generator_.get_environment().right_slot_obs_height;
  }
  return inner_obs_height;
}

double PerpendicularScenarioAdapter::get_outside_obs_height() {
  double outside_obs_height = -1.0;
  if (env_generator_.get_environment().inited) {
    outside_obs_height = env_generator_.get_environment().left_slot_obs_height;
  }
  return outside_obs_height;
}

double PerpendicularScenarioAdapter::get_slot_width() {
  double get_slot_width = -1.0;
  if (env_generator_.get_environment().inited) {
    get_slot_width = env_generator_.get_environment().slot_width;
  }
  return get_slot_width;
}

} // namespace msquare