#pragma once

#include <cmath>
#include <vector>

#include "curve.hpp"
#include "multi_circle_footprint_model.h"
#include "obstacle_grid.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/tmp_usage_only_global_variables.hpp"
#include "pnc/define/geometry.h"
#include "util.hpp"

namespace msquare {

namespace hybrid_a_star_2 {

class VariableStepSizeStateExpansion {
public:
  VariableStepSizeStateExpansion() {}

  void init(bool with_spiral, float min_turn_radius, float xy_grid_resolution,
            float theta_grid_resolution, float max_speed_forward,
            float max_speed_backward, float acceleration_forward,
            float acceleration_backward, float min_length_per_segment,
            int max_zigzag_allowed, bool search_max_distance,
            float lat_inflation, float lat_inflation_low, float lon_inflation,
            bool no_steer_change_gear_switch, float max_curvature_rate,
            float max_close_gear_switch_speed, int distance_option_count) {
    constexpr float theta_eps = 1e-5f;
    constexpr float xy_eps = 1e-3f;
    constexpr float deg2rad = float(M_PI) / 180.0f;
    constexpr float rad2deg = 180.0f / float(M_PI);

    with_spiral_ = with_spiral;
    min_length_per_segment_ = min_length_per_segment;
    max_zigzag_allowed_ = max_zigzag_allowed;
    search_max_distance_ = search_max_distance;
    lat_inflation_ = lat_inflation;
    lat_inflation_low_ = lat_inflation_low;
    lon_inflation_ = lon_inflation;
    max_inflation_ = std::max(lat_inflation, lon_inflation);
    x_bound_ = 0.0f;
    y_bound_ = 0.0f;
    local_frame_x_ = 0.0f;
    local_frame_y_ = 0.0f;
    local_frame_cos_theta_ = 1.0f;
    local_frame_sin_theta_ = 0.0f;
    no_steer_change_gear_switch_ = no_steer_change_gear_switch;
    candidates_for_distance_.clear();
    candidates_for_spiral_.clear();
    candidates_for_small_distance_.clear();
    candidates_for_acc_spiral_from_stop_.clear();
    curves_.clear();

    float distance_step = TmpGlobals::CONFIG_VARIABLE_STEP_SIZE_DISTANCE_STEP;
    inv_distance_step_ = 1.0f / distance_step;
    distance_option_count_ = distance_option_count;
    distance_bias_ = TmpGlobals::CONFIG_VARIABLE_STEP_SIZE_DISTANCE_BIAS;

    int steer_options =
        TmpGlobals::CONFIG_VARIABLE_STEP_SIZE_STEER_OPTION_COUNT;
    int steer_options_per_side = steer_options / 2;
    float min_arc_distance = min_turn_radius * float(steer_options_per_side) *
                             (theta_grid_resolution + theta_eps);
    float min_straight_distance = std::sqrt(2.0f) * xy_grid_resolution + xy_eps;
    for (int i = 0; i < distance_option_count_; i++) {
      float distance = distance_step * float(i + 1);
      float arc_distance = std::max(distance, min_arc_distance);
      float straight_distance = std::max(distance, min_straight_distance);
      std::vector<std::size_t> candidates;
      for (int distance_sign = -1; distance_sign < 2; distance_sign += 2) {
        for (int j = -steer_options_per_side; j <= steer_options_per_side;
             j++) {
          curves_.emplace_back();
          if (j == 0) {
            curves_.back().initAsLine(straight_distance * distance_sign);
          } else {
            curves_.back().initAsArc(
                arc_distance * distance_sign,
                float(j) / (float(steer_options_per_side) * min_turn_radius));
          }
          candidates.push_back(curves_.size() - 1);
        }
      }
      candidates_for_distance_.push_back(candidates);
    }
    nodes_buffer_.resize(steer_options * 2);

    if (search_max_distance_) {
      small_distance_option_count_ = 40;
      float small_distance_step = 0.01f;
      inv_small_distance_step_ = 1.0f / small_distance_step;
      for (int i = 0; i < small_distance_option_count_; i++) {
        float distance = small_distance_step * float(i + 1);
        std::vector<std::size_t> candidates;
        for (int distance_sign = -1; distance_sign < 2; distance_sign += 2) {
          for (int j = -steer_options_per_side; j <= steer_options_per_side;
               j++) {
            curves_.emplace_back();
            if (j == 0) {
              curves_.back().initAsLine(distance * distance_sign);
            } else {
              curves_.back().initAsArc(
                  distance * distance_sign,
                  float(j) / (float(steer_options_per_side) * min_turn_radius));
            }
            candidates.push_back(curves_.size() - 1);
          }
        }
        candidates_for_small_distance_.push_back(candidates);
      }
    }

    if (with_spiral_) {
      float min_spiral_distance = std::sqrt(2.0f) * xy_grid_resolution + 0.01f;
      candidates_for_spiral_ = getSpiralCandidate(
          steer_options, min_turn_radius, min_spiral_distance,
          max_curvature_rate, max_speed_forward, max_speed_backward,
          acceleration_forward, acceleration_backward, true, false, false,
          false);
      nodes_buffer_.resize(nodes_buffer_.size() +
                           candidates_for_spiral_.size());

      if (no_steer_change_gear_switch_) {
        candidates_for_spiral_close_gear_switch_ = getSpiralCandidate(
            steer_options, min_turn_radius, 0.0f, max_curvature_rate,
            max_close_gear_switch_speed, max_close_gear_switch_speed,
            acceleration_forward, acceleration_backward, false, false, true,
            false);
        nodes_buffer_.resize(nodes_buffer_.size() +
                             candidates_for_spiral_close_gear_switch_.size());

        candidates_for_acc_spiral_from_stop_ = getSpiralCandidate(
            steer_options, min_turn_radius, 0.0f, max_curvature_rate,
            max_speed_forward, max_speed_backward, acceleration_forward,
            acceleration_backward, false, false, false, true);
        nodes_buffer_.resize(nodes_buffer_.size() +
                             candidates_for_acc_spiral_from_stop_.size());
      }
    }
  }

  const std::vector<SearchNode *> &
  getNextStates(const SearchNode::Config &config, const SearchNode &current,
                MultiCircleFootprintModel &footprint_model,
                const ObstacleGrid &obstacle_grid) {
    if (no_steer_change_gear_switch_) {
      return getNextStatesNoSteerChangeGearSwitch(
          config, current, footprint_model, obstacle_grid);
    } else {
      return getNextStatesAllowSteerChangeGearSwitch(
          config, current, footprint_model, obstacle_grid);
    }
  }

  const std::vector<SearchNode *> &getNextStatesAllowSteerChangeGearSwitch(
      const SearchNode::Config &config, const SearchNode &current,
      MultiCircleFootprintModel &footprint_model,
      const ObstacleGrid &obstacle_grid) {
    next_.clear();
    if (current.type() == curve::Type::END) {
      return next_;
    }

    int distance_option = std::floor(
        (current.distance_to_obstacle() - distance_bias_) * inv_distance_step_);
    distance_option =
        std::min(std::max(distance_option, 1), distance_option_count_) - 1;

    float cos_theta = current.cos_theta();
    float sin_theta = current.sin_theta();

    std::size_t result_count = 0;
    const auto &candidates = candidates_for_distance_[distance_option];
    for (std::size_t i = 0; i < candidates.size(); i++) {
      std::size_t curve_id = candidates[i];
      const auto &candidate = curves_[curve_id];
      float distance_mult = candidate.distance() * current.travel_distance();
      if (current.type() == curve::Type::START) {
        if (distance_mult < 0.0f) {
          continue;
        }
      } else if (current.type() == curve::Type::EULER_SPIRAL) {
        if (distance_mult < 0.0f ||
            candidate.curvature_start() != current.curvature_end()) {
          continue;
        }
      } else {
        if (with_spiral_ && distance_mult > 0.0f &&
            candidate.curvature_start() != current.curvature_end()) {
          continue;
        }
        if (distance_mult < 0.0f &&
            candidate.curvature_start() == current.curvature_end()) {
          continue;
        }
      }
      if (current.must_gear_switch_next() && distance_mult > 0.0f) {
        continue;
      }

      bool must_gear_change_next = false;
      if (search_max_distance_ && distance_option == 0 &&
          distance_mult >= 0.0f &&
          (current.distance_to_obstacle() <
           max_inflation_ + 2.0f * obstacle_grid.max_error() +
               std::abs(candidate.distance()))) {
        curve::State next =
            candidate.endAfter(current.x(), current.y(), current.theta(),
                               current.cos_theta(), current.sin_theta());
        if (checkCollision(candidate.forward(), next, footprint_model,
                           obstacle_grid)) {
          int upper_bound = std::ceil(std::abs(candidate.distance()) *
                                      inv_small_distance_step_) -
                            1;
          int lower_bound = -1;
          while (upper_bound - lower_bound > 1 && upper_bound > 0) {
            int check = (upper_bound + lower_bound) / 2;
            const auto &curve =
                curves_[candidates_for_small_distance_[check][i]];
            curve::State next =
                curve.endAfter(current.x(), current.y(), current.theta(),
                               current.cos_theta(), current.sin_theta());
            if (checkCollision(curve.forward(), next, footprint_model,
                               obstacle_grid)) {
              upper_bound = check;
            } else {
              lower_bound = check;
            }
          }
          if (lower_bound == -1) {
            continue;
          }
          curve_id = candidates_for_small_distance_[lower_bound][i];
          must_gear_change_next = true;
        }
      }

      const auto &real_candidate = curves_[curve_id];
      if (current.type() == curve::Type::START &&
          std::abs(real_candidate.distance()) < min_length_per_segment_ &&
          must_gear_change_next) {
        continue;
      }

      addNext(config, current, real_candidate, must_gear_change_next,
              result_count);
    }

    for (const auto &curve_id : candidates_for_spiral_) {
      const auto &candidate = curves_[curve_id];
      float distance_mult = candidate.distance() * current.travel_distance();
      if (current.must_gear_switch_next() && distance_mult > 0.0f) {
        continue;
      }
      if (distance_mult < 0.0f) {
        continue;
      }
      if (candidate.curvature_start() != current.curvature_end()) {
        continue;
      }
      if ((candidate.curvature_end() > candidate.curvature_start() &&
           candidate.curvature_start() < current.curvature_start()) ||
          (candidate.curvature_end() < candidate.curvature_start() &&
           candidate.curvature_start() > current.curvature_start())) {
        continue;
      }

      addNext(config, current, candidate, false, result_count);
    }
    return next_;
  }

  const std::vector<SearchNode *> &getNextStatesNoSteerChangeGearSwitch(
      const SearchNode::Config &config, const SearchNode &current,
      MultiCircleFootprintModel &footprint_model,
      const ObstacleGrid &obstacle_grid) {
    next_.clear();
    if (current.type() == curve::Type::END) {
      return next_;
    }

    int distance_option = std::floor(
        (current.distance_to_obstacle() - distance_bias_) * inv_distance_step_);
    distance_option =
        std::min(std::max(distance_option, 1), distance_option_count_) - 1;

    float cos_theta = current.cos_theta();
    float sin_theta = current.sin_theta();

    std::size_t result_count = 0;
    for (const auto &curve_id : candidates_for_distance_[distance_option]) {
      const auto &candidate = curves_[curve_id];
      float distance_mult = candidate.distance() * current.travel_distance();
      if (current.type() == curve::Type::START && distance_mult < 0.0f) {
        continue;
      }
      if (current.must_gear_switch_next() != (distance_mult < 0.0f)) {
        continue;
      }
      if (current.type() != curve::Type::START &&
          candidate.curvature_start() != current.curvature_end()) {
        continue;
      }
      if (current.type() == curve::Type::START &&
          current.travel_distance() != 0.0 &&
          candidate.curvature_start() != current.curvature_end()) {
        continue;
      }

      addNext(config, current, candidate, false, result_count);
    }

    if (current.must_gear_switch_next()) {
      for (const auto &curve_id : candidates_for_acc_spiral_from_stop_) {
        const auto &candidate = curves_[curve_id];
        float distance_mult = candidate.distance() * current.travel_distance();
        if (distance_mult > 0.0f) {
          continue;
        }
        if (candidate.curvature_start() != current.curvature_end()) {
          continue;
        }
        if ((candidate.curvature_end() > candidate.curvature_start() &&
             candidate.curvature_start() < current.curvature_start()) ||
            (candidate.curvature_end() < candidate.curvature_start() &&
             candidate.curvature_start() > current.curvature_start())) {
          continue;
        }
        addNext(config, current, candidate, false, result_count);
      }
      return next_;
    }

    for (const auto &curve_id : candidates_for_spiral_) {
      const auto &candidate = curves_[curve_id];
      float distance_mult = candidate.distance() * current.travel_distance();
      if (distance_mult < 0.0f) {
        continue;
      }
      if (candidate.curvature_start() != current.curvature_end()) {
        continue;
      }
      if ((candidate.curvature_end() > candidate.curvature_start() &&
           candidate.curvature_start() < current.curvature_start()) ||
          (candidate.curvature_end() < candidate.curvature_start() &&
           candidate.curvature_start() > current.curvature_start())) {
        continue;
      }

      addNext(config, current, candidate, false, result_count);
    }

    for (const auto &curve_id : candidates_for_spiral_close_gear_switch_) {
      const auto &candidate = curves_[curve_id];
      float distance_mult = candidate.distance() * current.travel_distance();
      if (distance_mult < 0.0f) {
        continue;
      }
      if (candidate.curvature_start() != current.curvature_end()) {
        continue;
      }
      addNext(config, current, candidate, true, result_count);
    }
    return next_;
  }

  void setLocalInfo(Pose2D local_frame_pose, float x_bound, float y_bound) {
    x_bound_ = x_bound;
    y_bound_ = y_bound;
    local_frame_x_ = local_frame_pose.x;
    local_frame_y_ = local_frame_pose.y;
    local_frame_cos_theta_ = std::cos(local_frame_pose.theta);
    local_frame_sin_theta_ = std::sin(local_frame_pose.theta);
  }

private:
  bool isNodeOutOfRange(float x, float y) {
    float dx = x - local_frame_x_;
    float dy = y - local_frame_y_;
    float x_in_map = dx * local_frame_cos_theta_ + dy * local_frame_sin_theta_;
    float y_in_map = -dx * local_frame_sin_theta_ + dy * local_frame_cos_theta_;
    return x_in_map < 0.0f || x_in_map > x_bound_ || y_in_map < 0.0f ||
           y_in_map > y_bound_;
  }

  void addNext(const SearchNode::Config &config, const SearchNode &current,
               const curve::Curve &curve, bool must_gear_switch_next,
               std::size_t &result_count) {
    float distance_mult = curve.distance() * current.travel_distance();
    curve::State next =
        curve.endAfter(current.x(), current.y(), current.theta(),
                       current.cos_theta(), current.sin_theta());
    if (current.zigzag_num() + (distance_mult < 0.0f ? 1 : 0) >
            max_zigzag_allowed_ ||
        isNodeOutOfRange(next.x, next.y)) {
      return;
    }
    auto &node = nodes_buffer_[result_count];
    node.init(config, &curve, next.x, next.y, next.theta, &current,
              next.cos_theta, next.sin_theta, must_gear_switch_next);
    next_.push_back(&node);
    result_count++;
  }

  bool checkCollision(bool forward, const curve::State &state,
                      MultiCircleFootprintModel &footprint_model,
                      const ObstacleGrid &obstacle_grid) {
    return checkCollisionIfGearSwitch(
        forward, state.x, state.y, state.cos_theta, state.sin_theta,
        lat_inflation_, lat_inflation_low_, lon_inflation_, footprint_model,
        obstacle_grid);
  }

  std::vector<std::size_t>
  getSpiralCandidate(int steer_options, float min_turn_radius,
                     float min_spiral_distance, float curvature_rate,
                     float max_speed_forward, float max_speed_backward,
                     float acceleration_forward, float acceleration_backward,
                     bool only_allow_nearby_steer_option, bool from_mid_steer,
                     bool target_mid_steer, bool accelerate) {
    float spiral_scale_forward =
        calcSpiralScale(max_speed_forward, curvature_rate);
    float spiral_scale_backward =
        calcSpiralScale(max_speed_backward, curvature_rate);

    int steer_options_per_side = steer_options / 2;
    std::vector<std::size_t> result;
    for (int distance_sign = -1; distance_sign < 2; distance_sign += 2) {
      bool forward = distance_sign > 0;
      float max_speed = forward ? max_speed_forward : max_speed_backward;
      float acceleration =
          forward ? acceleration_forward : acceleration_backward;
      float spiral_scale =
          forward ? spiral_scale_forward : spiral_scale_backward;

      for (int start = -steer_options_per_side; start <= steer_options_per_side;
           start++) {
        if (from_mid_steer && start != 0) {
          continue;
        }
        for (int end = -steer_options_per_side; end <= steer_options_per_side;
             end++) {
          if (target_mid_steer && end != 0) {
            continue;
          }
          if (start == end) {
            continue;
          }
          if (only_allow_nearby_steer_option && std::abs(start - end) > 1) {
            continue;
          }
          float curvature_start =
              float(start) / (float(steer_options_per_side) * min_turn_radius);
          float curvature_end =
              float(end) / (float(steer_options_per_side) * min_turn_radius);
          curves_.emplace_back();
          if (accelerate) {
            int closer_end_to_start = end + (end < start ? 1 : -1);
            float curvature_closer_end_to_start =
                float(closer_end_to_start) /
                (float(steer_options_per_side) * min_turn_radius);
            float time =
                std::abs(curvature_closer_end_to_start - curvature_start) /
                curvature_rate;
            if (time * acceleration > max_speed) {
              continue;
            }
            float this_curvature_rate =
                curvature_rate *
                (curvature_end > curvature_start ? 1.0f : -1.0f);
            curves_.back().initAsAccEulerSpiral(
                forward, 0.0f, acceleration, max_speed, curvature_start,
                curvature_end, this_curvature_rate);
          } else {
            float this_spiral_scale =
                std::max(spiral_scale,
                         std::sqrt(2.0f * min_spiral_distance /
                                   std::abs(curvature_end - curvature_start)));
            curves_.back().initAsEulerSpiral(forward, curvature_start,
                                             curvature_end, this_spiral_scale);
          }
          result.push_back(curves_.size() - 1);
        }
      }
    }
    return result;
  }

  std::vector<curve::Curve> curves_;

  bool with_spiral_;
  float inv_distance_step_;
  int distance_option_count_;
  float distance_bias_;
  std::vector<std::vector<std::size_t>> candidates_for_distance_;
  std::vector<std::size_t> candidates_for_spiral_;
  std::vector<std::size_t> candidates_for_spiral_close_gear_switch_;
  std::vector<std::size_t> candidates_for_acc_spiral_from_stop_;
  float inv_small_distance_step_;
  int small_distance_option_count_;
  std::vector<std::vector<std::size_t>> candidates_for_small_distance_;
  float x_bound_;
  float y_bound_;
  float local_frame_x_;
  float local_frame_y_;
  float local_frame_cos_theta_;
  float local_frame_sin_theta_;
  std::vector<SearchNode *> next_;
  std::vector<SearchNode> nodes_buffer_;
  float min_length_per_segment_;
  int max_zigzag_allowed_;
  bool search_max_distance_;
  float lat_inflation_;
  float lat_inflation_low_;
  float lon_inflation_;
  float max_inflation_;
  bool no_steer_change_gear_switch_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
