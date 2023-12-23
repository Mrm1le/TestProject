#include "planner/motion_planner/apf_planner.h"
// #include "planning/common/common.h"

namespace msquare {

ApfPlanner::ApfPlanner(ApfPlannerConfigPtr cfg, double max_traj_length,
                       double min_traj_length)
    : cfg_(cfg) {
  max_point_num_ = (int)(max_traj_length / cfg_->step_size_);
  min_point_num_ = (int)(min_traj_length / cfg_->step_size_);
};

bool ApfPlanner::plan() {
  result_.clear();
  Pose2D current_pose(origin_);
  for (int index = 0; index < max_point_num_; index++) {
    planning_math::Vec2d gradient = create_gradient(current_pose);
    ApfPoint apf_point(current_pose, Point2D(gradient.x(), gradient.y()));
    result_.emplace_back(apf_point);
    if (!create_new_pose(current_pose, apf_point)) {
      break;
    }
  }
  if (result_.size() >= min_point_num_) {
    return true;
  } else {
    return false;
  }
  return false;
};

bool ApfPlanner::create_new_pose(Pose2D &current_pose,
                                 const ApfPoint &apf_point) {
  double dtheta;
  dtheta = planning_math::NormalizeAngle(apf_point.gradient_theta + M_PI -
                                         current_pose.theta);
  if (apf_point.gradient_value > cfg_->stop_gradient_value_ ||
      abs(dtheta) > M_PI / 2) {
    return false;
  } else {
    current_pose.theta = planning_math::NormalizeAngle(
        std::max(current_pose.theta - cfg_->max_dtheta_,
                 std::min(current_pose.theta + dtheta,
                          current_pose.theta + cfg_->max_dtheta_)));
    current_pose.x =
        apf_point.pose.x + cfg_->step_size_ * std::cos(current_pose.theta);
    current_pose.y =
        apf_point.pose.y + cfg_->step_size_ * std::sin(current_pose.theta);
    return true;
  }
  return true;
};

planning_math::Vec2d ApfPlanner::create_gradient(const Pose2D &current_pose) {
  planning_math::Vec2d gradient(0.0, 0.0);
  planning_math::Vec2d center =
      planning_math::Vec2d(current_pose.x + cfg_->center_to_geometry_center_ *
                                                std::cos(current_pose.theta),
                           current_pose.y + cfg_->center_to_geometry_center_ *
                                                std::sin(current_pose.theta));
  planning_math::Box2d box = planning_math::Box2d(
      center, current_pose.theta, cfg_->vehicle_length_, cfg_->vehicle_width_);
  planning_math::Vec2d vec =
      planning_math::Vec2d(current_pose.x, current_pose.y);
  for (auto obs : obstacles_) {
    auto temp_gradient = obs->get_gradient(box, vec);
    gradient += temp_gradient;
  }
  return gradient;
};

} // namespace msquare
