#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "curve.hpp"

namespace msquare {

namespace hybrid_a_star_2 {

class SearchNode {
private:
  enum class STurnStatus {
    NONE = 0,
    LEFT = 1,
    RIGHT = 2,
    FIRST_S_TURN = 3,
    S_TURN = 4,
  };

  static STurnStatus calcSTurnStatus(STurnStatus previous,
                                     float curvature_start,
                                     float curvature_end) {
    if (previous == STurnStatus::FIRST_S_TURN ||
        previous == STurnStatus::S_TURN) {
      return STurnStatus::S_TURN;
    }
    float previous_curvature = 0.0f;
    if (previous == STurnStatus::LEFT) {
      previous_curvature = 1.0f;
    } else if (previous == STurnStatus::RIGHT) {
      previous_curvature = -1.0f;
    }
    float min_steer =
        std::min(previous_curvature, std::min(curvature_start, curvature_end));
    float max_steer =
        std::max(previous_curvature, std::max(curvature_start, curvature_end));
    if (min_steer < 0.0f && max_steer > 0.0f) {
      return STurnStatus::FIRST_S_TURN;
    } else if (min_steer >= 0.0f && max_steer > 0.0f) {
      return STurnStatus::LEFT;
    } else if (min_steer < 0.0f && max_steer <= 0.0f) {
      return STurnStatus::RIGHT;
    }
    return STurnStatus::NONE;
  }

public:
  struct Config {
    float traj_gear_switch_penalty;
    float traj_steer_penalty;
    float traj_steer_change_penalty_gear_switch;
    float traj_steer_change_penalty;
    float traj_forward_penalty;
    float traj_back_penalty;
    float traj_obstacle_distance_1_penalty;
    float traj_obstacle_distance_2_penalty;
    float traj_obstacle_distance_3_penalty;
    float traj_boundary_cost_penalty;
    float traj_s_turn_penalty;
    float traj_end_offset_penalty;
    bool traj_penalty_start_at_reverse_as_gear_switch;
  };

  SearchNode() {
    previous_ = nullptr;
    curve_ = nullptr;
    x_ = 0.0f;
    y_ = 0.0f;
    theta_ = 0.0f;
    cos_theta_ = 0.0f;
    sin_theta_ = 0.0f;
    s_turn_status_ = 0;
    zigzag_num_ = 0;
    distance_to_obstacle_set_ = false;
    boundary_cost_set_ = false;
  }

  SearchNode(const Config &config, const curve::Curve *curve_ptr, float x,
             float y, float theta, const SearchNode *previous = nullptr,
             float cos_theta = 3.0f, float sin_theta = 3.0f,
             bool must_gear_switch_next = false) {
    init(config, curve_ptr, x, y, theta, previous, cos_theta, sin_theta,
         must_gear_switch_next);
  }

  void init(const Config &config, const curve::Curve *curve_ptr, float x,
            float y, float theta, const SearchNode *previous = nullptr,
            float cos_theta = 3.0f, float sin_theta = 3.0f,
            bool must_gear_switch_next = false) {
    curve_ = curve_ptr;
    x_ = x;
    y_ = y;
    theta_ = theta;
    if (cos_theta > 2.0f) {
      cos_theta_ = std::cos(theta);
    } else {
      cos_theta_ = cos_theta;
    }
    if (sin_theta > 2.0f) {
      sin_theta_ = std::sin(theta);
    } else {
      sin_theta_ = sin_theta;
    }
    previous_ = previous;

    STurnStatus previous_s_turn = STurnStatus::NONE;
    bool gear_switch = false;
    if (previous == nullptr) {
      setZigzagNum(0);
      setDistanceFromGearSwitch(0.0f);
    } else {
      gear_switch = travel_distance() * previous->travel_distance() < 0.0f;
      float distance_from_gear_switch = 0.0f;
      if (gear_switch) {
        setZigzagNum(previous->zigzag_num() + 1);
      } else {
        setZigzagNum(previous->zigzag_num());
        previous_s_turn = previous->s_turn_status();
        distance_from_gear_switch = previous->distance_from_gear_switch();
      }
      distance_from_gear_switch += std::abs(curve_->distance());
      setDistanceFromGearSwitch(distance_from_gear_switch);
    }
    setSTurnStatus(calcSTurnStatus(previous_s_turn, curve_->curvature_start(),
                                   curve_->curvature_end()));

    distance_to_obstacle_set_ = false;
    boundary_cost_set_ = false;
    must_gear_switch_next_ = must_gear_switch_next;

    if (previous_ == nullptr) {
      setTrajectoryCost(0.0f);
      return;
    }

    const float &step_size = std::abs(travel_distance());
    float trajectory_cost = previous_->trajectory_cost();

    bool start_at_reverse = previous_->type() == curve::Type::START &&
                            previous_->travel_distance() == 0.0f &&
                            travel_distance() < 0.0f;
    if (gear_switch || (config.traj_penalty_start_at_reverse_as_gear_switch &&
                        start_at_reverse)) {
      trajectory_cost += config.traj_gear_switch_penalty;
    }
    trajectory_cost += std::abs(steer_start() * 0.5f + steer_end() * 0.5f) *
                       config.traj_steer_penalty * step_size * 10.0f;

    float steer_change = std::abs(steer_start() - previous_->steer_end());
    if (gear_switch || previous_->type() == curve::Type::START) {
      trajectory_cost +=
          steer_change * config.traj_steer_change_penalty_gear_switch;
    } else {
      trajectory_cost += steer_change * config.traj_steer_change_penalty;
    }

    if (travel_distance() > 0.0f) {
      trajectory_cost += config.traj_forward_penalty * step_size;
    } else {
      trajectory_cost += config.traj_back_penalty * step_size;
    }

    if (previous_->distance_to_obstacle_set()) {
      constexpr float distance_th_1 = 1.0f;
      constexpr float distance_th_2 = 0.5f;
      constexpr float distance_th_3 = 0.3f;
      float cost = 0.0f;
      float distance = previous_->distance_to_obstacle();
      if (distance < distance_th_1) {
        cost += config.traj_obstacle_distance_1_penalty *
                (distance_th_1 - std::max(distance_th_2, distance));
      }
      if (distance < distance_th_2) {
        cost += config.traj_obstacle_distance_2_penalty *
                (distance_th_2 - std::max(distance_th_3, distance));
      }
      if (distance < distance_th_3) {
        cost += config.traj_obstacle_distance_3_penalty *
                (distance_th_3 - distance);
      }
      trajectory_cost += cost * step_size;
    }

    if (previous_->boundary_cost_set_) {
      trajectory_cost += config.traj_boundary_cost_penalty *
                         previous_->boundary_cost() * step_size;
    }

    if (s_turn_status() == STurnStatus::FIRST_S_TURN) {
      trajectory_cost += config.traj_s_turn_penalty;
    }

    setTrajectoryCost(trajectory_cost);
  }

  void copy(const SearchNode &other) {
    previous_ = other.previous_;
    curve_ = other.curve_;
    x_ = other.x_;
    y_ = other.y_;
    theta_ = other.theta_;
    cos_theta_ = other.cos_theta_;
    sin_theta_ = other.sin_theta_;
    trajectory_cost_ = other.trajectory_cost_;
    heuristic_cost_ = other.heuristic_cost_;
    zigzag_num_ = other.zigzag_num_;
    distance_to_obstacle_set_ = other.distance_to_obstacle_set_;
    distance_to_obstacle_ = other.distance_to_obstacle_;
    boundary_cost_set_ = other.boundary_cost_set_;
    boundary_cost_ = other.boundary_cost_;
    s_turn_status_ = other.s_turn_status_;
    must_gear_switch_next_ = other.must_gear_switch_next_;
    distance_from_gear_switch_ = other.distance_from_gear_switch_;
  }

  curve::Type type() const { return curve_->type(); }
  float x() const { return x_; }
  float y() const { return y_; }
  float theta() const { return theta_; }
  float cos_theta() const { return cos_theta_; }
  float sin_theta() const { return sin_theta_; }
  float curvature_start() const { return curve_->curvature_start(); }
  float curvature_end() const { return curve_->curvature_end(); }
  float inv_curvature_start() const { return curve_->inv_curvature_start(); }
  float inv_curvature_end() const { return curve_->inv_curvature_end(); }
  float steer_start() const { return curve_->steer_start(); }
  float steer_end() const { return curve_->steer_end(); }
  float travel_distance() const { return curve_->distance(); }
  bool forward() const { return curve_->forward(); }
  float distance_to_obstacle() const {
    return distance_to_obstacle_ * distance_to_obstacle_step;
  }
  bool distance_to_obstacle_set() const { return distance_to_obstacle_set_; }
  int zigzag_num() const { return zigzag_num_; }
  const SearchNode *previous() const { return previous_; }
  float trajectory_cost() const {
    return trajectory_cost_ * trajectory_cost_step;
  }
  float heuristic_cost() const { return heuristic_cost_ * heuristic_cost_step; }
  float boundary_cost() const { return boundary_cost_ * boundary_cost_step; }
  bool boundary_cost_set() const { return boundary_cost_set_; }
  const curve::Curve &curve() const { return *curve_; }
  bool must_gear_switch_next() const { return must_gear_switch_next_; }
  float distance_from_gear_switch() const {
    return distance_from_gear_switch_ * distance_from_gear_switch_step;
  }

  void setDistanceToObstacle(float value) {
    distance_to_obstacle_set_ = true;
    distance_to_obstacle_ = static_cast<unsigned int>(
        std::round(std::min(value, distance_to_obstacle_max) *
                   distance_to_obstacle_inv_step));
  }

  void setHeuristicCost(float value) {
    heuristic_cost_ = static_cast<unsigned int>(std::round(
        std::min(value, heuristic_cost_max) * heuristic_cost_inv_step));
  }

  void setBoundaryCost(float value) {
    boundary_cost_set_ = true;
    boundary_cost_ = static_cast<unsigned int>(std::round(
        std::min(value, boundary_cost_max) * boundary_cost_inv_step));
  }

  void setPose(float x, float y, float theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
    cos_theta_ = std::cos(theta);
    sin_theta_ = std::sin(theta);
  }

  bool inRange(float x_min, float x_max, float y_th,
               float theta_th = std::numeric_limits<float>::infinity()) const {
    return x_ > x_min && x_ < x_max && std::abs(y_) < y_th &&
           std::abs(theta_) < theta_th;
  }

  const curve::InterpolateResult interpolate(float max_distance) const {
    return curve_->interpolateBefore(max_distance, x_, y_, theta_, cos_theta_,
                                     sin_theta_);
  }

  const curve::State start() const {
    return curve_->startBefore(x_, y_, theta_, cos_theta_, sin_theta_);
  }

  const curve::State state() const {
    return curve::State(x_, y_, theta_, cos_theta_, sin_theta_,
                        curvature_end());
  }

  void addEndOffsetCost(const Config &config, float offset) {
    setTrajectoryCost(trajectory_cost() +
                      config.traj_end_offset_penalty * std::abs(offset));
  }

  static bool isParent(const SearchNode &parent, const SearchNode &child) {
    return &parent == child.previous();
  }

  static bool isBrother(const SearchNode &node1, const SearchNode &node2) {
    return node1.previous() == node2.previous();
  }

  static bool isParentOrBrother(const SearchNode &parent,
                                const SearchNode &child) {
    return isParent(parent, child) || isBrother(parent, child);
  }

private:
  STurnStatus s_turn_status() const { return STurnStatus(s_turn_status_); }

  void setSTurnStatus(STurnStatus value) {
    s_turn_status_ = static_cast<unsigned int>(value);
  }

  void setZigzagNum(unsigned int value) {
    zigzag_num_ = std::min(value, zigzag_num_max);
  }

  void setTrajectoryCost(float value) {
    trajectory_cost_ = static_cast<unsigned int>(std::round(
        std::min(value, trajectory_cost_max) * trajectory_cost_inv_step));
  }

  void setDistanceFromGearSwitch(float value) {
    distance_from_gear_switch_ = static_cast<unsigned int>(
        std::round(std::min(value, distance_from_gear_switch_max) *
                   distance_from_gear_switch_inv_step));
  }

  static constexpr unsigned int zigzag_num_bits = 6;
  static constexpr unsigned int zigzag_num_max = (1 << zigzag_num_bits) - 1;

  static constexpr unsigned int boundary_cost_bits = 12;
  static constexpr float boundary_cost_max = 1.0f;
  static constexpr float boundary_cost_step =
      boundary_cost_max / float((1 << boundary_cost_bits) - 1);
  static constexpr float boundary_cost_inv_step = 1.0f / boundary_cost_step;

  static constexpr unsigned int distance_to_obstacle_bits = 13;
  static constexpr float distance_to_obstacle_max = 2.0f;
  static constexpr float distance_to_obstacle_step =
      distance_to_obstacle_max / float((1 << distance_to_obstacle_bits) - 1);
  static constexpr float distance_to_obstacle_inv_step =
      1.0f / distance_to_obstacle_step;

  static constexpr unsigned int heuristic_cost_bits = 20;
  static constexpr float heuristic_cost_max = 500.0f;
  static constexpr float heuristic_cost_step =
      heuristic_cost_max / float((1 << heuristic_cost_bits) - 1);
  static constexpr float heuristic_cost_inv_step = 1.0f / heuristic_cost_step;

  static constexpr unsigned int trajectory_cost_bits = 20;
  static constexpr float trajectory_cost_max = 600.0f;
  static constexpr float trajectory_cost_step =
      trajectory_cost_max / float((1 << trajectory_cost_bits) - 1);
  static constexpr float trajectory_cost_inv_step = 1.0f / trajectory_cost_step;

  static constexpr unsigned int distance_from_gear_switch_bits = 12;
  static constexpr float distance_from_gear_switch_max = 4.0f;
  static constexpr float distance_from_gear_switch_step =
      distance_from_gear_switch_max /
      float((1 << distance_from_gear_switch_bits) - 1);
  static constexpr float distance_from_gear_switch_inv_step =
      1.0f / distance_from_gear_switch_step;

  const SearchNode *previous_;
  const curve::Curve *curve_;

  float x_;
  float y_;
  float theta_;
  float cos_theta_;
  float sin_theta_;

  unsigned int s_turn_status_ : 3;
  unsigned int zigzag_num_ : zigzag_num_bits;
  bool distance_to_obstacle_set_ : 1;
  unsigned int distance_to_obstacle_ : distance_to_obstacle_bits;
  bool boundary_cost_set_ : 1;
  unsigned int boundary_cost_ : boundary_cost_bits;
  unsigned int heuristic_cost_ : heuristic_cost_bits;
  unsigned int trajectory_cost_ : trajectory_cost_bits;
  bool must_gear_switch_next_ : 1;
  unsigned int distance_from_gear_switch_ : distance_from_gear_switch_bits;
};

} // namespace hybrid_a_star_2

} // namespace msquare
