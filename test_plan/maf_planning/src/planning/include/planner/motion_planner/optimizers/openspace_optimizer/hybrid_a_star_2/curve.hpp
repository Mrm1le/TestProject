#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "common/math/math_utils.h"

namespace msquare {

namespace hybrid_a_star_2 {

namespace curve {

enum class Type {
  UNKNOWN = 0,
  START = 1,
  END = 2,
  LINE = 3,
  ARC = 4,
  EULER_SPIRAL = 5,
  ACCELERATE_SPIRAL = 6,
};

namespace detail {

// support abs(t) < 5
// to keep error < 1e-6
// 4 <= abs(t) < 5: order >= 38
// 3 <= abs(t) < 4: order >= 26
// 2 <= abs(t) < 3: order >= 16
// 1 <= abs(t) < 2: order >= 9
// 0 <= abs(t) < 1: order >= 5
class Fresnel {
public:
  Fresnel(float lower_bound, float upper_bound, float step)
      : lower_bound_(lower_bound), inv_step_(1.0f / step) {
    int count = int(std::ceil((upper_bound - lower_bound) * inv_step_));
    s_table_.reserve(count);
    c_table_.reserve(count);
    for (int i = 0; i < count; i++) {
      s_table_.push_back(S(float(i) * step + lower_bound));
      c_table_.push_back(C(float(i) * step + lower_bound));
    }
  }

  float tabledS(float t) { return tabled(t, s_table_); }

  float tabledC(float t) { return tabled(t, c_table_); }

private:
  float S(float t, int order = 38) {
    float sub = 1.0f;
    float result = 1.0f / 3.0f;
    float t_2 = t * t;
    float t_4 = t_2 * t_2;
    for (int n = 1; n < order; n++) {
      sub *= -t_4 / float(2 * n * (2 * n + 1));
      result += sub / float(4 * n + 3);
    }
    return result * std::pow(t, 3);
  }

  float C(float t, int order = 38) {
    float sub = 1.0f;
    float result = 1.0f;
    float t_2 = t * t;
    float t_4 = t_2 * t_2;
    for (int n = 1; n < order; n++) {
      sub *= -t_4 / float((2 * n - 1) * 2 * n);
      result += sub / float(4 * n + 1);
    }
    return result * t;
  }

  float tabled(float t, const std::vector<float> &table) {
    float access = (t - lower_bound_) * inv_step_;
    int left = std::floor(access);
    float right_weight = access - float(left);
    float left_weight = 1.0f - right_weight;
    return table[left] * left_weight + table[left + 1] * right_weight;
  }

  std::vector<float> s_table_;
  std::vector<float> c_table_;
  float lower_bound_;
  float inv_step_;
};

extern Fresnel fresnel;

} // namespace detail

inline float fresnelS(float t) { return detail::fresnel.tabledS(t); }

inline float fresnelC(float t) { return detail::fresnel.tabledC(t); }

struct State {
  State() {}

  State(float x, float y, float theta, float cos_theta, float sin_theta,
        float curvature)
      : x(x), y(y), theta(theta), cos_theta(cos_theta), sin_theta(sin_theta),
        curvature(curvature) {}

  State(const State &state1, const State &state2, float state2_weight) {
    float state1_weight = 1.0f - state2_weight;
    x = state1.x * state1_weight + state2.x * state2_weight;
    y = state1.y * state1_weight + state2.y * state2_weight;
    theta = state1.theta * state1_weight + state2.theta * state2_weight;
    cos_theta =
        state1.cos_theta * state1_weight + state2.cos_theta * state2_weight;
    sin_theta =
        state1.sin_theta * state1_weight + state2.sin_theta * state2_weight;
    curvature =
        state1.curvature * state1_weight + state2.curvature * state2_weight;
  }

  bool inRange(float x_min, float x_max, float y_th,
               float theta_th = std::numeric_limits<float>::infinity()) const {
    return x > x_min && x < x_max && std::abs(y) < y_th &&
           std::abs(theta) < theta_th;
  }

  float x = 0.0f;
  float y = 0.0f;
  float theta = 0.0f;
  float cos_theta = 1.0f;
  float sin_theta = 0.0f;
  float curvature = 0.0f;
};

struct InterpolateResult {
  const std::vector<State> &states;
  float step;
};

class Curve {
public:
  Curve() {}

  void initAsStart(bool forward, bool backward) {
    type_ = Type::START;
    interpolated_max_distance_ = -std::numeric_limits<float>::infinity();

    curvature_start_ = 0.0f;
    inv_curvature_start_ = std::numeric_limits<float>::infinity();
    steer_start_ = 0.0f;
    curvature_end_ = 0.0f;
    inv_curvature_end_ = std::numeric_limits<float>::infinity();
    steer_end_ = 0.0f;
    if (forward) {
      distance_ = 1.0f;
    }
    if (backward) {
      distance_ = -1.0f;
    }

    states_.clear();
    states_.emplace_back();
  }

  void initAsEnd() {
    type_ = Type::END;
    interpolated_max_distance_ = -std::numeric_limits<float>::infinity();

    curvature_start_ = 0.0f;
    inv_curvature_start_ = std::numeric_limits<float>::infinity();
    steer_start_ = 0.0f;
    curvature_end_ = 0.0f;
    inv_curvature_end_ = std::numeric_limits<float>::infinity();
    steer_end_ = 0.0f;
    distance_ = 0.0f;

    states_.clear();
    states_.emplace_back();
  }

  void initAsLine(float distance) {
    type_ = Type::LINE;
    interpolated_max_distance_ = -std::numeric_limits<float>::infinity();

    curvature_start_ = 0.0f;
    inv_curvature_start_ = std::numeric_limits<float>::infinity();
    steer_start_ = 0.0f;
    curvature_end_ = 0.0f;
    inv_curvature_end_ = std::numeric_limits<float>::infinity();
    steer_end_ = 0.0f;
    distance_ = distance;

    int count = std::ceil(std::abs(distance) / max_resolution_);
    float step = distance / float(count);
    inv_state_resolution_ = 1.0f / std::abs(step);
    states_.clear();
    states_.reserve(count + 1);
    for (int i = 0; i < count + 1; i++) {
      states_.emplace_back(float(i) * step, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
    }
  }

  void initAsArc(float distance, float curvature) {
    type_ = Type::ARC;
    interpolated_max_distance_ = -std::numeric_limits<float>::infinity();

    curvature_start_ = curvature;
    inv_curvature_start_ = 1.0f / curvature;
    steer_start_ = toSteer(curvature);
    curvature_end_ = curvature;
    inv_curvature_end_ = inv_curvature_start_;
    steer_end_ = steer_start_;
    distance_ = distance;

    float r = inv_curvature_start_;
    int count = std::ceil(std::abs(distance) / max_resolution_);
    float step = distance * curvature / float(count);
    inv_state_resolution_ = 1.0f / std::abs(step * r);
    states_.clear();
    states_.reserve(count + 1);
    for (int i = 0; i < count + 1; i++) {
      float theta = float(i) * step;
      float sin_theta = std::sin(theta);
      float cos_theta = std::cos(theta);
      states_.emplace_back(r * sin_theta, r * (1.0f - cos_theta), theta,
                           cos_theta, sin_theta, curvature);
    }
  }

  void initAsEulerSpiral(bool forward, float curvature_start,
                         float curvature_end, float scale,
                         float max_resolution = 0.0f) {
    type_ = Type::EULER_SPIRAL;
    interpolated_max_distance_ = -std::numeric_limits<float>::infinity();

    curvature_start_ = curvature_start;
    inv_curvature_start_ = 1.0f / curvature_start;
    steer_start_ = toSteer(curvature_start);
    curvature_end_ = curvature_end;
    inv_curvature_end_ = 1.0f / curvature_end;
    steer_end_ = toSteer(curvature_end);

    float t_start = 0.5f * scale * curvature_start_;
    float t_end = 0.5f * scale * curvature_end;
    float flip_mult = forward != (t_end > t_start) ? -1.0f : 1.0f;
    t_start *= flip_mult;
    t_end *= flip_mult;
    distance_ = (t_end - t_start) * scale;

    float x_start = scale * fresnelC(t_start);
    float y_start = flip_mult * scale * fresnelS(t_start);
    float theta_start = flip_mult * t_start * t_start;
    float cos_theta_start = std::cos(theta_start);
    float sin_theta_start = std::sin(theta_start);
    if (max_resolution == 0.0f) {
      max_resolution = max_resolution_;
    }
    int count = std::ceil(std::abs(t_end - t_start) * scale / max_resolution);
    float step = (t_end - t_start) / float(count);
    inv_state_resolution_ = 1.0f / std::abs(step * scale);
    states_.clear();
    states_.reserve(count + 1);
    for (int i = 0; i < count + 1; i++) {
      float t = t_start + float(i) * step;
      float x = scale * fresnelC(t);
      float y = flip_mult * scale * fresnelS(t);
      float dx = x - x_start;
      float dy = y - y_start;
      float theta = planning_math::AngleDiff(theta_start, flip_mult * t * t);
      float sin_theta = std::sin(theta);
      float cos_theta = std::cos(theta);
      states_.emplace_back(dx * cos_theta_start + dy * sin_theta_start,
                           dx * -sin_theta_start + dy * cos_theta_start, theta,
                           cos_theta, sin_theta, flip_mult * 2.0f * t / scale);
    }
  }

  // speed_start >= 0, speed_max >= speed_start, acceleration >= 0 no matter
  // whether forward or not
  void initAsAccEulerSpiral(bool forward, float speed_start, float acceleration,
                            float max_speed, float curvature_start,
                            float curvature_end, float curvature_rate) {
    type_ = Type::ACCELERATE_SPIRAL;
    interpolated_max_distance_ = -std::numeric_limits<float>::infinity();

    curvature_start_ = curvature_start;
    inv_curvature_start_ = 1.0f / curvature_start;
    steer_start_ = toSteer(curvature_start);
    curvature_end_ = curvature_end;
    inv_curvature_end_ = 1.0f / curvature_end;
    steer_end_ = toSteer(curvature_end);
    float total_time = (curvature_end - curvature_start) / curvature_rate;
    float acc_time =
        std::min((max_speed - speed_start) / acceleration, total_time);
    distance_ = speed_start * acc_time +
                0.5f * acceleration * acc_time * acc_time +
                max_speed * (total_time - acc_time);
    distance_ *= forward ? 1.0f : -1.0f;

    int count = std::ceil(std::abs(distance_) / max_resolution_);
    inv_state_resolution_ = float(count) / std::abs(distance_);
    states_.clear();
    states_.reserve(count + 1);
    // use double cause its a iterative accumulation
    double distance_step = distance_ / double(count);
    double current_speed = speed_start;
    double current_curvature = curvature_start;
    double current_theta = 0.0f;
    double current_x = 0.0f;
    double current_y = 0.0f;
    for (int i = 0; i < count + 1; i++) {
      states_.emplace_back(current_x, current_y, current_theta,
                           std::cos(current_theta), std::sin(current_theta),
                           current_curvature);

      double time_step;
      double next_speed =
          std::sqrt(2.0 * acceleration * std::abs(distance_step) +
                    current_speed * current_speed);
      if (next_speed < max_speed) {
        time_step = (next_speed - current_speed) / acceleration;
      } else {
        next_speed = max_speed;
        float acc_time = (max_speed - current_speed) / acceleration;
        float acc_distance =
            current_speed * acc_time + 0.5 * acc_time * acc_time * acc_time;
        time_step =
            acc_time + (std::abs(distance_step) - acc_distance) / max_speed;
      }
      double next_curvature = current_curvature + curvature_rate * time_step;
      double average_curvature = (next_curvature + current_curvature) * 0.5f;
      double next_theta = current_theta + average_curvature * distance_step;
      double average_theta = (next_theta + current_theta) * 0.5f;

      current_x = current_x + std::cos(average_theta) * distance_step;
      current_y = current_y + std::sin(average_theta) * distance_step;
      current_speed = next_speed;
      current_curvature = next_curvature;
      current_theta = next_theta;
    }
  }

  void initByCut(const Curve &other, float start_ratio, float end_ratio) {
    type_ = other.type_;
    interpolated_max_distance_ = -std::numeric_limits<float>::infinity();

    distance_ = (end_ratio - start_ratio) * other.distance_;
    int count = std::ceil(std::abs(distance_) / max_resolution_);
    inv_state_resolution_ = std::abs(float(count) / distance_);
    float access_step = other.inv_state_resolution_ / inv_state_resolution_;
    states_.clear();
    states_.reserve(count + 1);
    interpolate(other.states_, states_, count + 1,
                start_ratio * std::abs(other.distance_) *
                        other.inv_state_resolution_ -
                    access_step,
                access_step);

    State start = states_[0];
    for (auto &item : states_) {
      float dx = item.x - start.x;
      float dy = item.y - start.y;
      item.x = dx * start.cos_theta + dy * start.sin_theta;
      item.y = dx * -start.sin_theta + dy * start.cos_theta;
      item.theta = planning_math::AngleDiff(start.theta, item.theta);
      float cos_theta =
          item.cos_theta * start.cos_theta + item.sin_theta * start.sin_theta;
      float sin_theta =
          item.sin_theta * start.cos_theta - item.cos_theta * start.sin_theta;
      item.cos_theta = cos_theta;
      item.sin_theta = sin_theta;
    }
    curvature_start_ = start.curvature;
    inv_curvature_start_ = 1.0f / curvature_start_;
    steer_start_ = toSteer(curvature_start_);
    curvature_end_ = states_.back().curvature;
    inv_curvature_end_ = 1.0f / curvature_end_;
    steer_end_ = toSteer(curvature_end_);
  }

  State endAfter(float x, float y, float theta, float cos_theta,
                 float sin_theta) const {
    const State &end = states_.back();
    return State(x + end.x * cos_theta + end.y * -sin_theta,
                 y + end.x * sin_theta + end.y * cos_theta,
                 planning_math::NormalizeAngle(theta + end.theta),
                 cos_theta * end.cos_theta - sin_theta * end.sin_theta,
                 sin_theta * end.cos_theta + cos_theta * end.sin_theta,
                 end.curvature);
  }

  State endAfter(const State &state) const {
    return endAfter(state.x, state.y, state.theta, state.cos_theta,
                    state.sin_theta);
  }

  State startBefore(float x, float y, float theta, float cos_theta,
                    float sin_theta) const {
    const State &end = states_.back();
    float start_cos_theta =
        cos_theta * end.cos_theta + sin_theta * end.sin_theta;
    float start_sin_theta =
        sin_theta * end.cos_theta - cos_theta * end.sin_theta;
    return State(x - end.x * start_cos_theta + end.y * start_sin_theta,
                 y - end.x * start_sin_theta - end.y * start_cos_theta,
                 theta - end.theta, start_cos_theta, start_sin_theta,
                 curvature_start_);
  }

  const InterpolateResult interpolateAfter(float max_distance, float x, float y,
                                           float theta, float cos_theta,
                                           float sin_theta) const {
    interpolate(max_distance);
    temp_interpolated_.clear();
    temp_interpolated_.reserve(interpolated_.size());
    for (const auto &state : interpolated_) {
      temp_interpolated_.emplace_back(
          x + state.x * cos_theta + state.y * -sin_theta,
          y + state.x * sin_theta + state.y * cos_theta,
          planning_math::NormalizeAngle(theta + state.theta),
          cos_theta * state.cos_theta - sin_theta * state.sin_theta,
          sin_theta * state.cos_theta + cos_theta * state.sin_theta,
          state.curvature);
    }
    return {temp_interpolated_, interpolated_step_};
  }

  const InterpolateResult interpolateBefore(float max_distance, float x,
                                            float y, float theta,
                                            float cos_theta,
                                            float sin_theta) const {
    State start = startBefore(x, y, theta, cos_theta, sin_theta);
    return interpolateAfter(max_distance, start.x, start.y, start.theta,
                            start.cos_theta, start.sin_theta);
  }

  Type type() const { return type_; };
  float distance() const { return distance_; };
  float curvature_start() const { return curvature_start_; }
  float curvature_end() const { return curvature_end_; }
  float inv_curvature_start() const { return inv_curvature_start_; }
  float inv_curvature_end() const { return inv_curvature_end_; }
  float steer_start() const { return steer_start_; }
  float steer_end() const { return steer_end_; }
  bool forward() const { return distance_ > 0.0f; };

  static void setWheelBase(float wheel_base) { wheel_base_ = wheel_base; }
  static float toSteer(float curvature) {
    return std::atan(wheel_base_ * curvature);
  }
  static void setMaxResolution(float value) { max_resolution_ = value; }

protected:
  static void interpolate(const std::vector<State> &input,
                          std::vector<State> &output, int count, float start,
                          float step) {
    for (int i = 0; i < count; i++) {
      float current = start + float(i + 1) * step;
      int left = std::floor(current);
      if (left < 0) {
        output.emplace_back(input.front());
      } else if (left + 1 > int(input.size()) - 1) {
        output.emplace_back(input.back());
      } else {
        output.emplace_back(input[left], input[left + 1],
                            current - float(left));
      }
    }
  }

  void interpolate(float max_distance) const {
    if (interpolated_max_distance_ == max_distance) {
      return;
    }
    interpolated_max_distance_ = max_distance;

    if (states_.size() == 1) {
      interpolated_.clear();
      interpolated_.emplace_back(states_[0]);
      interpolated_step_ = 0.0f;
      return;
    }

    int count = std::ceil(std::abs(distance()) / max_distance);
    interpolated_.clear();
    interpolated_.reserve(count);
    interpolated_step_ = std::abs(distance() / float(count));
    interpolate(states_, interpolated_, count, 0.0f,
                interpolated_step_ * inv_state_resolution_);
  }

  Type type_ = Type::UNKNOWN;
  float curvature_end_ = 0.0f;
  float curvature_start_ = 0.0f;
  float inv_curvature_start_ = std::numeric_limits<float>::infinity();
  float inv_curvature_end_ = std::numeric_limits<float>::infinity();
  float steer_start_ = 0.0f;
  float steer_end_ = 0.0f;
  float distance_ = 0.0f;
  std::vector<State> states_;
  float inv_state_resolution_;

  mutable std::vector<State> interpolated_;
  mutable float interpolated_max_distance_;
  mutable float interpolated_step_;

  static std::vector<State> temp_interpolated_;
  static float max_resolution_;
  static float wheel_base_;
};

} // namespace curve

} // namespace hybrid_a_star_2

} // namespace msquare
