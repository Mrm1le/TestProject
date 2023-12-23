#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "common/math/math_utils.h"

namespace msquare {

namespace hybrid_a_star_2 {

class HeuristicCost {
public:
  HeuristicCost(std::size_t reserve = 0) : inited_(false) {
    cost_.reserve(reserve);
  }

  bool inited() const { return inited_; }
  int x_count() const { return x_count_; }
  int y_count() const { return y_count_; }
  int theta_count() const { return theta_count_; }
  float cost_scale() const { return cost_scale_; }
  float max_cost() const { return 255 * cost_scale_; }

  void init(float cost_scale, float x_min, float y_min, float theta_min,
            float x_step, float y_step, float theta_step, int x_count,
            int y_count, int theta_count, bool y_flip) {
    cost_scale_ = cost_scale;
    inv_cost_scale_ = 1.0f / cost_scale;
    x_min_ = x_min;
    y_min_ = y_min;
    theta_min_ = theta_min;
    x_step_ = x_step;
    y_step_ = y_step;
    theta_step_ = theta_step;
    inv_x_step_ = 1.0f / x_step;
    inv_y_step_ = 1.0f / y_step;
    inv_theta_step_ = 1.0f / theta_step;
    x_count_ = x_count;
    y_count_ = y_count;
    theta_count_ = theta_count;
    y_flip_ = y_flip;
    cost_.resize(x_count * y_count * theta_count, 255);
    inited_ = true;
  }

  void update(float x, float y, float theta, float cost) {
    std::size_t index = getIndex(x, y, theta);
    if (index < cost_.size()) {
      cost_[index] =
          std::min(std::max(int(std::round(cost * inv_cost_scale_)), 0),
                   int(cost_[index]));
    }
  }

  float get(float forward, float x, float y, float theta, float cos_theta,
            float sin_theta, float target_x, float target_y,
            float target_theta) const {
    float dx = target_x - x;
    float dy = target_y - y;
    float x_in_current = dx * cos_theta + dy * sin_theta;
    float y_in_current = dx * -sin_theta + dy * cos_theta;
    float theta_in_current = planning_math::AngleDiff(theta, target_theta);
    if (!forward) {
      x_in_current = -x_in_current;
      theta_in_current = -theta_in_current;
    }
    std::size_t index = getIndex(x_in_current, y_in_current, theta_in_current);
    return (index < cost_.size() ? cost_[index] : 255) * cost_scale_;
  }

  bool loadFile(const std::string &file_name) {
    std::ifstream ifs(file_name);
    if (!ifs.is_open()) {
      deInit();
      return false;
    }
    ifs.seekg(0, std::ios::end);
    std::string buffer(std::size_t(ifs.tellg()), ' ');
    ifs.seekg(0);
    ifs.read(&buffer[0], buffer.size());
    loadContent(buffer);
    return true;
  }

  void loadContent(const std::string &content) {
    std::istringstream ifs(content);
    ifs.read(reinterpret_cast<char *>(&x_min_), sizeof(x_min_));
    ifs.read(reinterpret_cast<char *>(&y_min_), sizeof(y_min_));
    ifs.read(reinterpret_cast<char *>(&theta_min_), sizeof(theta_min_));
    ifs.read(reinterpret_cast<char *>(&x_step_), sizeof(x_step_));
    ifs.read(reinterpret_cast<char *>(&y_step_), sizeof(y_step_));
    ifs.read(reinterpret_cast<char *>(&theta_step_), sizeof(theta_step_));
    ifs.read(reinterpret_cast<char *>(&x_count_), sizeof(x_count_));
    ifs.read(reinterpret_cast<char *>(&y_count_), sizeof(y_count_));
    ifs.read(reinterpret_cast<char *>(&theta_count_), sizeof(theta_count_));
    ifs.read(reinterpret_cast<char *>(&cost_scale_), sizeof(cost_scale_));
    ifs.read(reinterpret_cast<char *>(&y_flip_), sizeof(y_flip_));
    cost_.resize(x_count_ * y_count_ * theta_count_);
    ifs.read(reinterpret_cast<char *>(cost_.data()),
             sizeof(std::uint8_t) * cost_.size());

    inv_x_step_ = 1.0f / x_step_;
    inv_y_step_ = 1.0f / y_step_;
    inv_theta_step_ = 1.0f / theta_step_;
    inv_cost_scale_ = 1.0f / cost_scale_;
    inited_ = true;
  }

#ifdef BUILD_IN_TEST_BAG_RECURRENT
  bool dump(std::string file_name);
  void dumpImage();
#endif // BUILD_IN_TEST_BAG_RECURRENT

private:
  std::size_t getIndex(float x, float y, float theta) const {
    if (y_flip_ && y < 0.0f) {
      y = -y;
      theta = -theta;
    }
    int ix = std::round((x - x_min_) * inv_x_step_);
    int iy = std::round((y - y_min_) * inv_y_step_);
    int itheta = std::round((theta - theta_min_) * inv_theta_step_);
    if (ix < 0 || ix >= x_count_ || iy < 0 || iy >= y_count_ || itheta < 0 ||
        itheta >= theta_count_) {
      return cost_.size();
    }
    return (ix * y_count_ + iy) * theta_count_ + itheta;
  }

  void deInit() { inited_ = false; }

  bool inited_;
  float x_min_;
  float y_min_;
  float theta_min_;
  float x_step_;
  float y_step_;
  float theta_step_;
  float inv_x_step_;
  float inv_y_step_;
  float inv_theta_step_;
  int x_count_;
  int y_count_;
  int theta_count_;
  float cost_scale_;
  float inv_cost_scale_;
  bool y_flip_;
  std::vector<std::uint8_t> cost_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
