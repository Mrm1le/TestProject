#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "common/math/line_segment2d.h"

namespace msquare {
namespace planning_math {

struct CartesianCommonTerm {
  double x;
  double dx;
  double d2x;
  double d3x;
  double y;
  double dy;
  double d2y;
  double d3y;
  double yaw;
  double curve;
  double lat_acc;
  double lat_jerk;
};

struct SampleInfo {
  double eta;
  double max_jerk_abs;
  double max_jerk_square;
};

/**
 * @brief Cartesian Quintic polynominal 1d.
 */
class CartesianQuinticPoly1d {
public:
  static constexpr size_t NUM_BOUNDARY_CONDITION = 4;
  static constexpr size_t NUM_COEFF = 6;

  CartesianQuinticPoly1d() : length_{0.0}, velocity_{0.0}, max_jerk_abs_{0.0} {
    std::fill(x_.begin(), x_.end(), 0.0);
    std::fill(y_.begin(), y_.end(), 0.0);
    std::fill(eta_.begin(), eta_.end(), 0.0);
  }

  CartesianQuinticPoly1d(
      const std::array<double, NUM_BOUNDARY_CONDITION> &start,
      const std::array<double, NUM_BOUNDARY_CONDITION> &end,
      const double &velocity)
      : CartesianQuinticPoly1d(start[0], start[1], start[2], start[3], end[0],
                               end[1], end[2], end[3], velocity) {}

  CartesianQuinticPoly1d(const double x_a, const double y_a,
                         const double theta_a, const double k_a,
                         const double x_b, const double y_b,
                         const double theta_b, const double k_b,
                         const double velocity) {
    ComputeExtraInfos(x_a, y_a, x_b, y_b, velocity);

    // minimize sum of jerk square
    sample(x_a, y_a, theta_a, k_a, x_b, y_b, theta_b, k_b, velocity);
  }

  CartesianQuinticPoly1d(const CartesianQuinticPoly1d &other) {
    x_ = other.x_;
    y_ = other.y_;
    eta_ = other.eta_;
    start_ = other.start_;
    end_ = other.end_;
    unit_direction_ = other.unit_direction_;
    length_ = other.length_;
    velocity_ = other.velocity_;
    max_jerk_abs_ = other.max_jerk_abs_;
  }

  CartesianCommonTerm Evaluate(const double u) const {
    CartesianCommonTerm res{};
    res.x = x_[0] + x_[1] * u + x_[2] * u * u + x_[3] * u * u * u +
            x_[4] * u * u * u * u + x_[5] * u * u * u * u * u;
    res.y = y_[0] + y_[1] * u + y_[2] * u * u + y_[3] * u * u * u +
            y_[4] * u * u * u * u + y_[5] * u * u * u * u * u;
    res.dx = x_[1] + 2 * x_[2] * u + 3 * x_[3] * u * u +
             +4 * x_[4] * u * u * u + 5 * x_[5] * u * u * u * u;
    res.dy = y_[1] + 2 * y_[2] * u + 3 * y_[3] * u * u +
             +4 * y_[4] * u * u * u + 5 * y_[5] * u * u * u * u;
    res.d2x =
        2 * x_[2] + 6 * x_[3] * u + 12 * x_[4] * u * u + 20 * x_[5] * u * u * u;
    res.d2y =
        2 * y_[2] + 6 * y_[3] * u + 12 * y_[4] * u * u + 20 * y_[5] * u * u * u;
    res.d3x = 6 * x_[3] + 24 * x_[4] * u + 60 * x_[5] * u * u;
    res.d3y = 6 * y_[3] + 24 * y_[4] * u + 60 * y_[5] * u * u;

    double dl_sq = res.dx * res.dx + res.dy * res.dy;
    double dl_1_pt_5 = std::pow(dl_sq, 1.5);
    double dxy_cross = res.dx * res.d2y - res.d2x * res.dy;
    double dc_ds = ((res.dx * res.d3y - res.d3x * res.dy) * dl_1_pt_5 -
                    3.0 * std::sqrt(dl_sq) * dxy_cross *
                        (res.dx * res.d2x + res.dy * res.d2y)) /
                   std::pow(dl_sq, 3.0);
    double ds_dt = velocity_ / std::sqrt(dl_sq);

    res.yaw = std::atan2(res.dy, res.dx);
    res.curve = dxy_cross / dl_1_pt_5;
    res.lat_acc = velocity_ * velocity_ * res.curve;
    res.lat_jerk = velocity_ * velocity_ * dc_ds * ds_dt;

    return res;
  }

  CartesianCommonTerm Evaluate(const planning_math::Vec2d &point) const {
    double u = (length_ <= planning_math::kMathEpsilon)
                   ? 0.0
                   : (point - start_).InnerProd(unit_direction_) / length_;

    u = std::fmin(1.0, std::fmax(0.0, u));

    return Evaluate(u);
  }

  const bool isValid() const {
    const double MAX_LAT_JERK = 9.0;
    // std::cout << "length_ = " << length_
    //           << " max_jerk_abs_ = " << getMaxLatJerkAbs() << std::endl;
    return length_ > planning_math::kMathEpsilon &&
           getMaxLatJerkAbs() < MAX_LAT_JERK;
  }

  const double getMaxLatJerkAbs() const { return max_jerk_abs_; }

private:
  void sample(const double &x_a, const double &y_a, const double &theta_a,
              const double &k_a, const double &x_b, const double &y_b,
              const double &theta_b, const double &k_b,
              const double &velocity) {
    const double sin_theta_a = std::sin(theta_a);
    const double cos_theta_a = std::cos(theta_a);
    const double sin_theta_b = std::sin(theta_b);
    const double cos_theta_b = std::cos(theta_b);

    const double ETA_MIN = 2.0;
    const double ETA_MAX = 6.0;
    const double eta_step = 1.0;

    static std::vector<SampleInfo> sample_info_list;
    sample_info_list.clear();
    sample_info_list.reserve(int((ETA_MAX - ETA_MIN) / eta_step + 1));

    for (double eta = ETA_MIN; eta <= ETA_MAX; eta += eta_step) {
      SampleInfo sample_info{};
      sample_info.eta = velocity * eta;
      ComputeCoefficients(x_a, y_a, sin_theta_a, cos_theta_a, k_a, x_b, y_b,
                          sin_theta_b, cos_theta_b, k_b, sample_info.eta);

      const double u_step = 0.1;
      for (double u = 0.0; u <= 1.0; u += u_step) {
        auto res = Evaluate(u);
        sample_info.max_jerk_square += std::pow(res.lat_jerk, 2.0);
        sample_info.max_jerk_abs =
            std::fmax(sample_info.max_jerk_abs, std::fabs(res.lat_jerk));
      }
      sample_info_list.emplace_back(sample_info);
    }

    std::sort(sample_info_list.begin(), sample_info_list.end(),
              [](const SampleInfo &res1, const SampleInfo &res2) {
                return res1.max_jerk_square < res2.max_jerk_square;
              });

    ComputeCoefficients(x_a, y_a, sin_theta_a, cos_theta_a, k_a, x_b, y_b,
                        sin_theta_b, cos_theta_b, k_b,
                        sample_info_list.front().eta);
    max_jerk_abs_ = sample_info_list.front().max_jerk_abs;
  }

  void ComputeExtraInfos(const double &x_a, const double &y_a,
                         const double &x_b, const double &y_b,
                         const double &velocity) {
    const double dx = x_b - x_a;
    const double dy = y_b - y_a;

    start_ = planning_math::Vec2d(x_a, y_a);
    end_ = planning_math::Vec2d(x_b, y_b);

    length_ = hypot(dx, dy);
    unit_direction_ = (length_ <= planning_math::kMathEpsilon
                           ? planning_math::Vec2d(0, 0)
                           : planning_math::Vec2d(dx / length_, dy / length_));
    velocity_ = velocity;
  }

  void ComputeCoefficients(const double &x_a, const double &y_a,
                           const double &sin_theta_a, const double &cos_theta_a,
                           const double &k_a, const double &x_b,
                           const double &y_b, const double &sin_theta_b,
                           const double &cos_theta_b, const double &k_b,
                           const double &eta) {
    eta_[0] = eta;
    eta_[1] = eta;
    eta_[2] = 0.0;
    eta_[3] = 0.0;

    x_[0] = x_a;
    x_[1] = eta_[0] * cos_theta_a;
    x_[2] =
        0.5 * (eta_[2] * cos_theta_a - eta_[0] * eta_[0] * k_a * sin_theta_a);
    x_[3] = 10 * (x_b - x_a) - (6 * eta_[0] + 3. / 2. * eta_[2]) * cos_theta_a -
            (4 * eta_[1] - 0.5 * eta_[3]) * cos_theta_b +
            3. / 2. * eta_[0] * eta_[0] * k_a * sin_theta_a -
            0.5 * eta_[1] * eta_[1] * k_b * sin_theta_b;
    x_[4] = -15 * (x_b - x_a) +
            (8 * eta_[0] + 3. / 2. * eta_[2]) * cos_theta_a +
            (7 * eta_[1] - eta_[3]) * cos_theta_b -
            3. / 2. * eta_[0] * eta_[0] * k_a * sin_theta_a +
            eta_[1] * eta_[1] * k_b * sin_theta_b;
    x_[5] = 6 * (x_b - x_a) - (3 * eta_[0] + 0.5 * eta_[2]) * cos_theta_a -
            (3 * eta_[1] - 0.5 * eta_[3]) * cos_theta_b +
            0.5 * eta_[0] * eta_[0] * k_a * sin_theta_a -
            0.5 * eta_[1] * eta_[1] * k_b * sin_theta_b;

    y_[0] = y_a;
    y_[1] = eta_[0] * sin_theta_a;
    y_[2] =
        0.5 * (eta_[2] * sin_theta_a + eta_[0] * eta_[0] * k_a * cos_theta_a);
    y_[3] = 10 * (y_b - y_a) - (6 * eta_[0] + 3. / 2. * eta_[2]) * sin_theta_a -
            (4 * eta_[1] - 0.5 * eta_[3]) * sin_theta_b -
            3. / 2. * eta_[0] * eta_[0] * k_a * cos_theta_a +
            0.5 * eta_[1] * eta_[1] * k_b * cos_theta_b;
    y_[4] = -15 * (y_b - y_a) +
            (8 * eta_[0] + 3. / 2. * eta_[2]) * sin_theta_a +
            (7 * eta_[1] - eta_[3]) * sin_theta_b +
            3. / 2. * eta_[0] * eta_[0] * k_a * cos_theta_a -
            eta_[1] * eta_[1] * k_b * cos_theta_b;
    y_[5] = 6 * (y_b - y_a) - (3 * eta_[0] + 0.5 * eta_[2]) * sin_theta_a -
            (3 * eta_[1] - 0.5 * eta_[3]) * sin_theta_b -
            0.5 * eta_[0] * eta_[0] * k_a * cos_theta_a +
            0.5 * eta_[1] * eta_[1] * k_b * cos_theta_b;
  }

private:
  std::array<double, NUM_COEFF> x_;
  std::array<double, NUM_COEFF> y_;
  std::array<double, NUM_BOUNDARY_CONDITION> eta_;
  planning_math::Vec2d unit_direction_;
  planning_math::Vec2d start_;
  planning_math::Vec2d end_;
  double length_;
  double velocity_;
  double max_jerk_abs_;
};

} // namespace planning_math
} // namespace msquare
