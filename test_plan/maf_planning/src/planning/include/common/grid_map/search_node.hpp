#pragma once

#include <algorithm>
#include <cmath>

#include "common/grid_map/config.h"

namespace msquare {
namespace grid {

class HAS2SearchNode {
public:
  enum class PathType {
    // NONE means this node is just a state, not path (e.g. start node)
    NONE = 0,
    LINE = 1,
    ARC = 2,
    EULER_SPIRAL = 3,
  };

  HAS2SearchNode()
      : x_(-1.0), y_(-1.0), theta_(-1.0), distance_to_obstacle_set_(false),
        boundary_cost_set_(false) {}

  HAS2SearchNode(PathType type, double x, double y, double theta,
                 double travel_distance = 0.0, double steer_angle_start = 0.0,
                 double steer_angle_end = 0.0,
                 const HAS2SearchNode *previous = nullptr,
                 double cos_theta = 3.0, double sin_theta = 3.0,
                 double spiral_t_start = 0.0, double spiral_t_end = 0.0,
                 double spiral_scale = 0.0, bool spiral_flip = false) {
    setNode(type, x, y, theta, travel_distance, steer_angle_start,
            steer_angle_end, previous, cos_theta, sin_theta, spiral_t_start,
            spiral_t_end, spiral_scale, spiral_flip);
  }

  void setNode(PathType type, double x, double y, double theta,
               double travel_distance = 0.0, double steer_angle_start = 0.0,
               double steer_angle_end = 0.0,
               const HAS2SearchNode *previous = nullptr, double cos_theta = 3.0,
               double sin_theta = 3.0, double spiral_t_start = 0.0,
               double spiral_t_end = 0.0, double spiral_scale = 0.0,
               bool spiral_flip = false) {
    this->type_ = type;
    this->x_ = x;
    this->y_ = y;
    this->theta_ = theta;
    if (cos_theta > 2.0) {
      this->cos_theta_ = std::cos(theta);
    } else {
      this->cos_theta_ = cos_theta;
    }
    if (sin_theta > 2.0) {
      this->sin_theta_ = std::sin(theta);
    } else {
      this->sin_theta_ = sin_theta;
    }

    this->steer_angle_start_ = steer_angle_start;
    this->steer_angle_end_ = steer_angle_end;
    this->travel_distance_ = travel_distance;

    this->previous_ = previous;
    if (previous == nullptr) {
      this->zigzag_num_ = 0;
    } else {
      this->zigzag_num_ = previous->zigzag_num_;
      if (this->travel_distance_ * previous->travel_distance_ < 0) {
        this->zigzag_num_++;
      }
    }

    distance_to_obstacle_set_ = false;
    boundary_cost_set_ = false;

    this->spiral_t_start_ = spiral_t_start;
    this->spiral_t_end_ = spiral_t_end;
    this->spiral_scale_ = spiral_scale;
    this->spiral_flip_ = spiral_flip;
  }

  void setNode(const HAS2SearchNode &other) {
    previous_ = other.previous_;
    type_ = other.type_;
    x_ = other.x_;
    y_ = other.y_;
    theta_ = other.theta_;
    cos_theta_ = other.cos_theta_;
    sin_theta_ = other.sin_theta_;
    steer_angle_start_ = other.steer_angle_start_;
    steer_angle_end_ = other.steer_angle_end_;
    travel_distance_ = other.travel_distance_;
    trajectory_cost_ = other.trajectory_cost_;
    heuristic_cost_ = other.heuristic_cost_;
    zigzag_num_ = other.zigzag_num_;
    distance_to_obstacle_set_ = other.distance_to_obstacle_set_;
    distance_to_obstacle_all_ = other.distance_to_obstacle_all_;
    distance_to_obstacle_front_ = other.distance_to_obstacle_front_;
    boundary_cost_set_ = other.boundary_cost_set_;
    boundary_cost_ = other.boundary_cost_;
    spiral_t_start_ = other.spiral_t_start_;
    spiral_t_end_ = other.spiral_t_end_;
    spiral_scale_ = other.spiral_scale_;
    spiral_flip_ = other.spiral_flip_;
  }

  void calcTrajectoryCost(const HybridAstarConfig &config) {
    const double &step_size = std::abs(travel_distance_);
    if (previous_ == nullptr) {
      trajectory_cost_ = 0;
      return;
    }

    trajectory_cost_ = previous_->trajectory_cost_;

    bool gear_switch = previous_->travel_distance_ * travel_distance_ < 0;
    if (gear_switch) {
      if (previous_->previous_ == nullptr) {
        trajectory_cost_ += config.traj_wrong_start_direction_penalty;
      } else {
        trajectory_cost_ += config.traj_gear_switch_penalty;
      }
    }
    trajectory_cost_ +=
        std::abs(steer_angle_start_ * 0.5 + steer_angle_end_ * 0.5) *
        config.traj_steer_penalty * step_size * 10.0;

    double steer_change =
        std::abs(steer_angle_start_ - previous_->steer_angle_end_);
    if (gear_switch || previous_->type_ == PathType::NONE) {
      trajectory_cost_ +=
          steer_change * config.traj_steer_change_penalty_gear_switch;
    } else {
      trajectory_cost_ += steer_change * config.traj_steer_change_penalty;
    }

    if (travel_distance_ > 0.0) {
      trajectory_cost_ += config.traj_forward_penalty * step_size;
    } else {
      trajectory_cost_ += config.traj_back_penalty * step_size;
    }

    if (previous_->distance_to_obstacle_set_) {
      constexpr double distance_th_1 = 1.0;
      constexpr double distance_th_2 = 0.5;
      constexpr double distance_th_3 = 0.3;
      double cost = 0.0;
      if (previous_->distance_to_obstacle_front_ < distance_th_1) {
        cost +=
            config.traj_obstacle_distance_1_penalty *
            (distance_th_1 -
             std::max(distance_th_2, previous_->distance_to_obstacle_front_));
      }
      if (previous_->distance_to_obstacle_front_ < distance_th_2) {
        cost +=
            config.traj_obstacle_distance_2_penalty *
            (distance_th_2 -
             std::max(distance_th_3, previous_->distance_to_obstacle_front_));
      }
      if (previous_->distance_to_obstacle_front_ < distance_th_3) {
        cost += config.traj_obstacle_distance_3_penalty *
                (distance_th_3 - previous_->distance_to_obstacle_front_);
      }
      trajectory_cost_ += cost * step_size;
    }

    if (previous_->boundary_cost_set_) {
      trajectory_cost_ += config.traj_boundary_cost_penalty *
                          previous_->boundary_cost_ * step_size;
    }
  }

  PathType type() const { return type_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double theta() const { return theta_; }
  double cos_theta() const { return cos_theta_; }
  double sin_theta() const { return sin_theta_; }
  double steer_angle_start() const { return steer_angle_start_; }
  double steer_angle_end() const { return steer_angle_end_; }
  double travel_distance() const { return travel_distance_; }
  double distance_to_obstacle_all() const { return distance_to_obstacle_all_; }
  double distance_to_obstacle_front() const {
    return distance_to_obstacle_front_;
  }
  bool distance_to_obstacle_set() const { return distance_to_obstacle_set_; }
  int zigzag_num() const { return zigzag_num_; }
  const HAS2SearchNode *previous() const { return previous_; }
  double trajectory_cost() const { return trajectory_cost_; }
  double heuristic_cost() const { return heuristic_cost_; }
  double boundary_cost() const { return boundary_cost_; }
  bool boundary_cost_set() const { return boundary_cost_set_; }
  double spiral_t_start() const { return spiral_t_start_; }
  double spiral_t_end() const { return spiral_t_end_; }
  double spiral_scale() const { return spiral_scale_; }
  bool spiral_flip() const { return spiral_flip_; }

  void setDistanceToObstacle(double all, double front) {
    distance_to_obstacle_set_ = true;
    distance_to_obstacle_all_ = all;
    distance_to_obstacle_front_ = front;
  }

  void addTrajectoryCost(double value) { trajectory_cost_ += value; }

  void setHeuristicCost(double value) { heuristic_cost_ = value; }

  void addHeuristicCost(double value) { heuristic_cost_ += value; }

  void setBoundaryCost(double value) {
    boundary_cost_set_ = true;
    boundary_cost_ = value;
  }

  bool isSpiralToStraight() const {
    return type_ == PathType::EULER_SPIRAL && spiral_t_end_ == 0.0;
  }

private:
  const HAS2SearchNode *previous_;

  PathType type_;
  double x_;
  double y_;
  // in rad, from x axis to y axis, zero means x axis
  double theta_;
  double cos_theta_;
  double sin_theta_;
  // in deg, from right to left, zero means forward
  double steer_angle_start_;
  double steer_angle_end_;
  // positive means forward
  double travel_distance_;
  double trajectory_cost_;
  double heuristic_cost_;
  int zigzag_num_;
  bool distance_to_obstacle_set_;
  double distance_to_obstacle_all_;
  double distance_to_obstacle_front_;
  bool boundary_cost_set_;
  double boundary_cost_;
  double spiral_t_start_;
  double spiral_t_end_;
  double spiral_scale_;
  bool spiral_flip_;
};

} // namespace grid
} // namespace msquare
