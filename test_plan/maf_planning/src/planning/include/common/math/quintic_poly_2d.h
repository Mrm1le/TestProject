#pragma once
#include "common/math/quintic_poly_1d.h"
#include "common/utils/gauss_quad_constants.h"
#include <limits>

namespace msquare {
namespace planning_math {

/**
 * @brief Quintic polynominal 2d.
 */
class QuinticPoly2d {
public:
  QuinticPoly2d(const std::array<double, 5> &start,
                const std::array<double, 5> &end, const double end_t)
      : QuinticPoly2d(start[0], start[1], start[2], start[3], start[4], end[0],
                      end[1], end[2], end[3], end[4], end_t) {}

  QuinticPoly2d(const double &start_x, const double &start_y,
                const double &start_yaw, const double &start_v,
                const double &start_a, const double &end_x, const double &end_y,
                const double &end_yaw, const double &end_v, const double &end_a,
                const double &end_t) {
    double start_v_x = start_v * std::cos(start_yaw);
    double start_a_x = start_a * std::cos(start_yaw);
    double end_v_x = end_v * std::cos(end_yaw);
    double end_a_x = end_a * std::cos(end_yaw);

    double start_v_y = start_v * std::sin(start_yaw);
    double start_a_y = start_a * std::sin(start_yaw);
    double end_v_y = end_v * std::sin(end_yaw);
    double end_a_y = end_a * std::sin(end_yaw);

    std::array<double, 3> x_start = {{start_x, start_v_x, start_a_x}};
    std::array<double, 3> x_end = {{end_x, end_v_x, end_a_x}};

    std::array<double, 3> y_start = {{start_y, start_v_y, start_a_y}};
    std::array<double, 3> y_end = {{end_y, end_v_y, end_a_y}};

    end_t_ = end_t;
    x_quintic_poly_ = std::move(QuinticPoly1d(x_start, x_end, end_t));
    y_quintic_poly_ = std::move(QuinticPoly1d(y_start, y_end, end_t));
  }

  double get_x_dis(int order, double time) {
    double x_sample = x_quintic_poly_.Evaluate(order, time);
    return x_sample;
  }

  double get_y_dis(int order, double time) {
    double y_sample = y_quintic_poly_.Evaluate(order, time);
    return y_sample;
  }

  double MaxJerk() {
    double jerk = std::numeric_limits<double>::lowest();
    for (const auto &check_point : GAUSS_QUAD_5TH_POS_WITH_END) {
      double time = check_point * end_t_;
      double jerk_x = x_quintic_poly_.Evaluate(3, time);
      double jerk_y = y_quintic_poly_.Evaluate(3, time);
      double jerk_at_time = std::hypot(jerk_x, jerk_y);
      jerk = std::fmax(jerk, jerk_at_time);
    }
    return jerk;
  }

  double MaxAcceleration() {
    double acc = std::numeric_limits<double>::lowest();
    for (const auto &check_point : GAUSS_QUAD_5TH_POS_WITH_END) {
      double time = check_point * end_t_;
      double a_x = x_quintic_poly_.Evaluate(2, time);
      double a_y = y_quintic_poly_.Evaluate(2, time);
      double a_at_time = std::hypot(a_x, a_y);
      acc = std::fmax(acc, a_at_time);
    }
    return acc;
  }

private:
  double end_t_ = 0.0;
  QuinticPoly1d x_quintic_poly_;
  QuinticPoly1d y_quintic_poly_;
};

} // namespace planning_math
} // namespace msquare