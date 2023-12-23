#pragma once

#include <cstdint>

#include "common/math/math_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/State.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"

namespace msquare {

namespace hybrid_a_star_2 {

class BoundaryCost {
public:
  BoundaryCost() { initEmpty(0.0f, 0.0f, 0, 0, 0, 0.0, 0.0, 0.0f); }
  ~BoundaryCost() {}

  float x_min() const { return x_min_; }
  int x_num() const { return x_num_; }
  float y_min() const { return y_min_; }
  int y_num() const { return y_num_; }
  double xy_res() const { return xy_res_; }
  int theta_num() const { return theta_num_; }
  double theta_res() const { return theta_res_; }

  bool initEmpty(float x_min, float y_min, int x_num, int y_num, int theta_num,
                 double xy_res, double theta_res, float quantization_scale) {
    x_min_ = x_min;
    y_min_ = y_min;
    x_num_ = x_num;
    y_num_ = y_num;
    theta_num_ = theta_num;
    xy_res_ = xy_res;
    theta_res_ = theta_res;
    quantization_scale_ = quantization_scale;

    inv_xy_res_ = 1.0 / xy_res;
    inv_theta_res_ = 1.0 / theta_res;
    xy_num_ = x_num_ * y_num_;
    cost_.clear();
    cost_.resize(x_num_ * y_num_ * theta_num_, 0);
    return true;
  }

  bool loadFromFile(std::string file_name) {
    return loadFromContent(ReadFile(file_name));
  }

  bool loadFromContent(const std::string &content) {
    std::istringstream ifs(content);
    unsigned int verification_head;
    ifs.read((char *)&verification_head, sizeof(verification_head));
    if (verification_head != VERIFICATION_HEAD) {
      return false;
    }

    double x_min;
    ifs.read((char *)&x_min, sizeof(x_min));
    x_min_ = x_min;
    double y_min;
    ifs.read((char *)&y_min, sizeof(y_min));
    y_min_ = y_min;

    double theta_min = 0.0;
    ifs.read((char *)&theta_min, sizeof(theta_min));
    if (theta_min != 0.0) {
      return false;
    }

    ifs.read((char *)&x_num_, sizeof(x_num_));
    ifs.read((char *)&y_num_, sizeof(y_num_));
    ifs.read((char *)&theta_num_, sizeof(theta_num_));
    ifs.read((char *)&xy_res_, sizeof(xy_res_));
    ifs.read((char *)&theta_res_, sizeof(theta_res_));
    ifs.read((char *)&quantization_scale_, sizeof(quantization_scale_));

    int total_num = x_num_ * y_num_ * theta_num_;
    cost_.reserve(total_num);
    ifs.read(reinterpret_cast<char *>(cost_.data()),
             sizeof(std::uint16_t) * total_num);

    inv_xy_res_ = 1.0 / xy_res_;
    inv_theta_res_ = 1.0 / theta_res_;
    xy_num_ = x_num_ * y_num_;
    return true;
  }

#ifdef BUILD_IN_TEST_BAG_RECURRENT
  bool dumpToFile(std::string file_name, bool is_binary);
#endif // BUILD_IN_TEST_BAG_RECURRENT

  std::uint16_t setCost(int gx, int gy, int gtheta, float cost) {
    if (gx < 0 || gy < 0 || gtheta < 0 || gx >= x_num_ || gy >= y_num_ ||
        gtheta >= theta_num_) {
      return std::numeric_limits<std::uint16_t>::max();
    } else {
      float cost_quantized = std::round(cost / quantization_scale_);
      if (cost_quantized > std::numeric_limits<std::uint16_t>::max()) {
        cost_quantized = std::numeric_limits<std::uint16_t>::max();
      } else if (cost_quantized < 0) {
        cost_quantized = 0;
      }
      cost_[gtheta * xy_num_ + gy * x_num_ + gx] = cost_quantized;
      return cost_quantized;
    }
  }

  // theta should in [-pi, 2pi]
  float getCost(float x, float y, float theta) const {
    if (theta < 0.0f) {
      theta += 2.0f * float(M_PI);
    }
    int gx = std::round((x - x_min_) * inv_xy_res_);
    int gy = std::round((y - y_min_) * inv_xy_res_);
    int gtheta = std::round(theta * inv_theta_res_);
    return getCost(gx, gy, gtheta);
  }

  float getCost(int gx, int gy, int gtheta) const {
    if (gx < 0 || gy < 0 || gtheta < 0 || gx >= x_num_ || gy >= y_num_ ||
        gtheta >= theta_num_) {
      return 1.0001f;
    } else {
      return float(cost_[gtheta * xy_num_ + gy * x_num_ + gx]) *
             quantization_scale_;
    }
  }

private:
  static std::string ReadFile(const std::string &file) {
    std::ifstream ifs(file);
    if (!ifs.is_open()) {
      return std::string();
    }
    ifs.seekg(0, std::ios::end);
    std::string buffer(std::size_t(ifs.tellg()), ' ');
    ifs.seekg(0);
    ifs.read(&buffer[0], buffer.size());
    return buffer;
  }

  // head int is to verify binary file
  // the last 2 hex digits are for version control. whenever the version of file
  // is changed, the head must be modified.
  static constexpr std::uint32_t VERIFICATION_HEAD = 0x47794a02;

  double xy_res_;
  double theta_res_;
  float x_min_;
  float y_min_;
  int x_num_;
  int y_num_;
  int xy_num_;
  int theta_num_;

  std::vector<std::uint16_t> cost_;
  float quantization_scale_;

  float inv_xy_res_;
  float inv_theta_res_;
};

} // namespace hybrid_a_star_2

} // namespace msquare
