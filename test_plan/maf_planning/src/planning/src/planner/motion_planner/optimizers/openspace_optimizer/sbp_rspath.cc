#include "common/sbp_rspath.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <iostream>

namespace msquare {

using namespace planning_math;

SbpRSPath::SbpRSPath() {
  reeds_shepp_path_ = std::shared_ptr<ReedSheppPath>(nullptr);
  for (int i = 0; i < max_nodes_num_; i++) {
    nodes_vec_.emplace_back(std::shared_ptr<SearchNode>(new SearchNode()));
  }
}

SbpRSPath::SbpRSPath(const std::shared_ptr<SearchNode> current_node,
                     std::shared_ptr<ReedSheppPath> reeds_shepp_path) {
  int difference = std::max(max_nodes_num_, (int)reeds_shepp_path->x.size()) -
                   nodes_vec_.size();
  if (difference > 0) {
    for (int i = 0; i < difference; i++) {
      nodes_vec_.emplace_back(std::shared_ptr<SearchNode>(new SearchNode()));
    }
  }
  nodes_.clear();
  reeds_shepp_path_ = reeds_shepp_path;
  nodes_.emplace_back(current_node);

  int gear;
  double x, y, phi, vel;
  std::shared_ptr<SearchNode> node;
  double rs_equevalent_delta = 0;
  double max_delta_angle_ = CarParams::GetInstance()->max_delta_angle;

  for (size_t i = 1; i < reeds_shepp_path_->x.size(); ++i) {
    x = reeds_shepp_path_->x[i];
    y = reeds_shepp_path_->y[i];
    phi = reeds_shepp_path_->phi[i];

    auto prev_node = nodes_[i - 1];
    Vec2d prev_heading_vec(std::cos(prev_node->theta),
                           std::sin(prev_node->theta));
    Vec2d prev_node_vec(prev_node->x, prev_node->y);
    Vec2d node_vec(x, y);
    gear = prev_heading_vec.InnerProd(node_vec - prev_node_vec) > 0 ? 1 : -1;
    vel = hypot(x - prev_node->x, y - prev_node->y) * gear;

    double sin_diff_phi = std::sin(phi - prev_node->theta) * gear;
    if (sin_diff_phi > 1e-6) {
      rs_equevalent_delta = max_delta_angle_;
    } else if (sin_diff_phi < -1e-6) {
      rs_equevalent_delta = -max_delta_angle_;
    } else {
      rs_equevalent_delta = 0;
    }

    nodes_vec_[i - 1]->set_node(x, y, phi, vel, rs_equevalent_delta);
    nodes_vec_[i - 1]->previous = nodes_[i - 1];
    nodes_.emplace_back(nodes_vec_[i - 1]);
  }
}

SbpRSPath::~SbpRSPath() {}

void SbpRSPath::update(const std::shared_ptr<SearchNode> current_node,
                       std::shared_ptr<ReedSheppPath> reeds_shepp_path) {
  int difference = std::max(max_nodes_num_, (int)reeds_shepp_path->x.size()) -
                   nodes_vec_.size();
  if (difference > 0) {
    for (int i = 0; i < difference; i++) {
      nodes_vec_.emplace_back(std::shared_ptr<SearchNode>(new SearchNode()));
    }
  }
  nodes_.clear();
  reeds_shepp_path_ = reeds_shepp_path;
  nodes_.emplace_back(current_node);

  int gear;
  double x, y, phi, vel;
  std::shared_ptr<SearchNode> node;
  double rs_equevalent_delta = 0;
  double max_delta_angle_ = CarParams::GetInstance()->max_delta_angle;

  for (size_t i = 1; i < reeds_shepp_path_->x.size(); ++i) {
    x = reeds_shepp_path_->x[i];
    y = reeds_shepp_path_->y[i];
    phi = reeds_shepp_path_->phi[i];

    auto prev_node = nodes_[i - 1];
    Vec2d prev_heading_vec(std::cos(prev_node->theta),
                           std::sin(prev_node->theta));
    Vec2d prev_node_vec(prev_node->x, prev_node->y);
    Vec2d node_vec(x, y);
    gear = prev_heading_vec.InnerProd(node_vec - prev_node_vec) > 0 ? 1 : -1;
    vel = hypot(x - prev_node->x, y - prev_node->y) * gear;

    double sin_diff_phi = std::sin(phi - prev_node->theta) * gear;
    if (sin_diff_phi > 1e-6) {
      rs_equevalent_delta = max_delta_angle_;
    } else if (sin_diff_phi < -1e-6) {
      rs_equevalent_delta = -max_delta_angle_;
    } else {
      rs_equevalent_delta = 0;
    }

    nodes_vec_[i - 1]->set_node(x, y, phi, vel, rs_equevalent_delta);
    nodes_vec_[i - 1]->previous = nodes_[i - 1];
    nodes_.emplace_back(nodes_vec_[i - 1]);
  }
}

void SbpRSPath::calcTrajCost() {
  for (size_t i = 1; i < nodes_.size(); ++i) {
    auto curr_node = nodes_[i];
    curr_node->setTrajCost();
  }
}

void SbpRSPath::calcObstacleCost(const std::vector<SbpObstaclePtr> obs_ptrs,
                                 const FootprintModelPtr &footprint_model) {
  double single_node_obs_cost = 0;
  for (auto node : nodes_) {
    single_node_obs_cost = 0;
    for (const SbpObstaclePtr &obs_ptr : obs_ptrs) {
      single_node_obs_cost += obs_ptr->getCost(node, footprint_model);
    }
    node->obs_cost = node->previous->obs_cost + single_node_obs_cost;
  }
}

double SbpRSPath::getCost(const std::vector<SbpObstaclePtr> obs_ptrs,
                          const FootprintModelPtr &footprint_model) {
  if (nodes_.empty()) {
    return 0.0;
  }
  calcTrajCost();
  calcObstacleCost(obs_ptrs, footprint_model);
  return nodes_.back()->trajcost + nodes_.back()->obs_cost -
         nodes_.front()->trajcost - nodes_.front()->obs_cost;
}

double SbpRSPath::getCostWithoutObstacle() {
  if (nodes_.empty()) {
    return 0.0;
  }
  calcTrajCost();
  return nodes_.back()->trajcost - nodes_.front()->trajcost;
}

bool SbpRSPath::checkCollision(
    const double step_size, const std::vector<SbpObstaclePtr> obs_ptrs,
    const FootprintModelPtr &footpint_model,
    const FootprintModelPtr &footpint_model_precise) {

  if (nodes_.empty()) {
    return false;
  }
  if (nodes_.size() < 2) {
    for (const SbpObstaclePtr &obs_ptr : obs_ptrs) {
      if (obs_ptr->checkCollision(nodes_.back(), footpint_model)) {
        return true;
      }
    }
    return false;
  }
  using namespace planning_math;
  std::shared_ptr<SearchNode> node;
  for (size_t i = 1; i + 1 < nodes_.size(); i++) {
    if (checkSingleNodeCollision(nodes_[i], obs_ptrs, footpint_model,
                                 footpint_model_precise)) {
      return true;
    }
  }

  auto prev_node = nodes_.back();
  double last_segment_length =
      std::hypot(prev_node->previous->x - prev_node->x,
                 prev_node->previous->y - prev_node->y);
  if (prev_node->previous->vel * prev_node->vel < 0) {
    if (last_segment_length < step_size) {
      // std::cerr << "HybridAstar::last_segment_length too short to follow"
      //           << std::endl;
      return true;
    }
  }

  // force direction
  if (prev_node->vel * HybridAstarConfig::GetInstance()
                           ->force_analytic_expansion_end_direction <
      0) {
    return true;
  }

  double theta1 = prev_node->previous->theta;
  double theta2 = prev_node->theta;
  double deltas_x = prev_node->x - prev_node->previous->x;
  double deltas_y = prev_node->y - prev_node->previous->y;
  last_segment_length = std::hypot(deltas_x, deltas_y);

  double average_heading =
      std::atan2((std::cos(theta1) + std::cos(theta2)) / 2,
                 (std::sin(theta1) + std::sin(theta2)) / 2);
  double arc_heading = std::atan2(deltas_y, deltas_x);
  double holonomic_error = planning_math::NormalizeAngle(
      planning_math::NormalizeAngle(theta1 - arc_heading) -
      planning_math::NormalizeAngle(theta2 - arc_heading));
  double holonomic_error_tolerance =
      0.02 * std::max(last_segment_length, step_size); // rad
  if (holonomic_error > holonomic_error_tolerance) {
    return true;
  }
  double turn_radius =
      last_segment_length /
      (2 * sin(planning_math::NormalizeAngle(theta2 - theta1) / 2.0));
  if (std::abs(turn_radius) < CarParams::GetInstance()->min_turn_radius) {
    return true;
  }

  return false;
}

bool SbpRSPath::checkSingleNodeCollision(
    std::shared_ptr<SearchNode> current_state,
    const std::vector<SbpObstaclePtr> obs_ptrs,
    const FootprintModelPtr &footpint_model,
    const FootprintModelPtr &footpint_model_precise) {
  int gear = (current_state->vel < 0.0) ? -1 : 1;
  const SearchNode &pos = *current_state;

  if (pos.previous != nullptr) {
    const SearchNode &previous_pos = *pos.previous;
    int previous_gear = (previous_pos.vel < 0.0) ? -1 : 1;
    if (previous_gear != gear && previous_pos.previous) {
      double x_previous_safe =
          previous_pos.x + previous_gear *
                               CarParams::GetInstance()->lon_inflation() *
                               cos(previous_pos.theta);
      double y_previous_safe =
          previous_pos.y + previous_gear *
                               CarParams::GetInstance()->lon_inflation() *
                               sin(previous_pos.theta);
      std::shared_ptr<SearchNode> tmp_check_node = std::make_shared<SearchNode>(
          x_previous_safe, y_previous_safe, previous_pos.theta);
      for (const SbpObstaclePtr &obs_ptr : obs_ptrs) {
        if (obs_ptr->checkCollision(tmp_check_node, footpint_model_precise)) {
          return true;
        }
      }
    }
  }

  for (const SbpObstaclePtr &obs_ptr : obs_ptrs) {
    if (obs_ptr->checkCollision(current_state, footpint_model)) {
      return true;
    }
  }

  return false;
}

} // namespace msquare