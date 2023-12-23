#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <limits>
#include <stdexcept>

namespace msquare {

SearchNode::SearchNode(double x, double y, double theta, double vel,
                       double delta, double wheel_base_offset) {
  this->x = x;
  this->y = y;
  this->theta = theta;

  double xy_grid_resolution_ =
      HybridAstarConfig::GetInstance()->xy_grid_resolution;

  if (std::fabs(xy_grid_resolution_) > 1e-5) {
    this->gx =
        x / xy_grid_resolution_; // parasoft-suppress AUTOSAR-A5_6_1 "f-drop"
    this->gy = y / xy_grid_resolution_;
  } else {
    this->gx = x / 1e-5;
    this->gy = y / 1e-5;
  }
  double phi_grid_resolution_ =
      HybridAstarConfig::GetInstance()->phi_grid_resolution;
  if (std::fabs(phi_grid_resolution_) > 1e-5) {
    this->gtheta =
        theta /               // parasoft-suppress AUTOSAR-A5_6_1
        phi_grid_resolution_; // parasoft-suppress AUTOSAR-A5_6_1 "f-drop"
  } else {
    this->gtheta = theta / 1e-5;
  }

  this->delta = delta;
  this->vel = vel;
  this->trajcost = 0;
  wheel_base_offset_ = wheel_base_offset;
  ComputeStringIndex();
}

void SearchNode::set_node(double x, double y, double theta, double vel,
                          double delta, double wheel_base_offset) {
  this->x = x;
  this->y = y;
  this->theta = theta;
  double xy_grid_resolution_ =
      HybridAstarConfig::GetInstance()->xy_grid_resolution;
  double phi_grid_resolution_ =
      HybridAstarConfig::GetInstance()->phi_grid_resolution;
  if (std::fabs(xy_grid_resolution_) > 1e-5) {
    this->gx =
        x / xy_grid_resolution_; // parasoft-suppress AUTOSAR-A5_6_1 "f-drop"
    this->gy =
        y / xy_grid_resolution_; // parasoft-suppress AUTOSAR-A5_6_1 "f-drop"
  } else {
    this->gx = x / 1e-5;
    this->gy = y / 1e-5;
  }
  if (std::fabs(phi_grid_resolution_) > 1e-5) {
    this->gtheta =
        theta / // parasoft-suppress AUTOSAR-A5_6_1
        HybridAstarConfig::GetInstance()
            ->phi_grid_resolution; // parasoft-suppress AUTOSAR-A5_6_1 "f-drop"

  } else {
    this->gtheta = theta / 1e-5;
  }

  this->delta = delta;
  this->vel = vel;
  this->trajcost = 0;
  wheel_base_offset_ = wheel_base_offset;
  ComputeStringIndex();
}

SearchNode::SearchNode() {
  this->x = -1;
  this->y = -1;
  this->theta = -1;
}

void SearchNode::setTrajCost() {
  const double &step_size = fabs(vel);
  if (!previous) {
    trajcost = 0;
    return;
  }

  trajcost = previous->trajcost;
  if (previous->vel * vel < 0) {
    trajcost += HybridAstarConfig::GetInstance()->traj_gear_switch_penalty;
  } else {
    trajcost +=
        std::abs(delta) * HybridAstarConfig::GetInstance()->traj_steer_penalty;
    trajcost += std::abs((delta - previous->delta)) *
                HybridAstarConfig::GetInstance()->traj_steer_change_penalty;
  }

  if (vel > 0.0) {
    trajcost +=
        HybridAstarConfig::GetInstance()->traj_forward_penalty * step_size;
  } else {
    trajcost += HybridAstarConfig::GetInstance()->traj_back_penalty * step_size;
  }
  trajcost +=
      std::min(std::abs(wheel_base_offset_ - previous->wheel_base_offset_),
               1.0) *
      1.0;

  trajcost +=
      HybridAstarConfig::GetInstance()->traj_sides_diff_penalty * side_diff;
}

} // namespace msquare
