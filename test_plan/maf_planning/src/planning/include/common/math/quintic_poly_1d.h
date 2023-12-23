#include <array>
#include <cmath>
#include <iostream>

namespace msquare {
namespace planning_math {

/**
 * @brief Quintic polynominal 1d.
 */
class QuinticPoly1d {
public:
  QuinticPoly1d() {
    end_s_ = {};
    coef_ = {};
  }

  QuinticPoly1d(const std::array<double, 3> &start,
                const std::array<double, 3> &end, const double end_s)
      : QuinticPoly1d(start[0], start[1], start[2], end[0], end[1], end[2],
                      end_s) {}

  QuinticPoly1d(const double &x0, const double &dx0, const double &ddx0,
                const double &x1, const double &dx1, const double &ddx1,
                const double &end_s) {
    ComputeCoefficients(x0, dx0, ddx0, x1, dx1, ddx1, end_s);
    end_s_ = end_s;
  }

  // dx0 = 0; ddx0 = 0; dx1 = 0; ddx1 = 0;
  QuinticPoly1d(const double &x0, const double &x1,
                const double &third_derivative) {
    coef_[0] = x0;
    coef_[1] = 0.;
    coef_[2] = 0.;
    coef_[3] = third_derivative / 6.;

    double cube = std::pow(third_derivative, 4) / 3840. / (x0 - x1);
    coef_[4] = cube / std::abs(cube) * std::pow(std::abs(cube), 1. / 3.);
    coef_[5] = 1.6 * coef_[4] * coef_[4] / third_derivative;
    end_s_ = -0.4 * coef_[4] / coef_[5];
  }

  QuinticPoly1d(const QuinticPoly1d &other) {
    end_s_ = other.end_s_;
    coef_ = other.coef_;
  }

  double Evaluate(const uint32_t order, const double p) const {
    switch (order) {
    case 0: {
      return ((((coef_[5] * p + coef_[4]) * p + coef_[3]) * p + coef_[2]) * p +
              coef_[1]) *
                 p +
             coef_[0];
    }
    case 1: {
      return (((5.0 * coef_[5] * p + 4.0 * coef_[4]) * p + 3.0 * coef_[3]) * p +
              2.0 * coef_[2]) *
                 p +
             coef_[1];
    }
    case 2: {
      return (((20.0 * coef_[5] * p + 12.0 * coef_[4]) * p) + 6.0 * coef_[3]) *
                 p +
             2.0 * coef_[2];
    }
    case 3: {
      return (60.0 * coef_[5] * p + 24.0 * coef_[4]) * p + 6.0 * coef_[3];
    }
    case 4: {
      return 120.0 * coef_[5] * p + 24.0 * coef_[4];
    }
    case 5: {
      return 120.0 * coef_[5];
    }
    default:
      return 0.0;
    }
  }

  const double &get_end_s() const { return end_s_; }

private:
  void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                           const double x1, const double dx1, const double ddx1,
                           const double s_end) {
    coef_[0] = x0;
    coef_[1] = dx0;
    coef_[2] = ddx0 / 2.0;

    const double s_end2 = s_end * s_end;
    const double s_end3 = s_end * s_end2;

    // the direct analytical method is at least 6 times faster than using matrix
    // inversion.
    const double c0 = (x1 - 0.5 * s_end2 * ddx0 - dx0 * s_end - x0) / s_end3;
    const double c1 = (dx1 - ddx0 * s_end - dx0) / s_end2;
    const double c2 = (ddx1 - ddx0) / s_end;

    coef_[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
    coef_[4] = (-15.0 * c0 + 7.0 * c1 - c2) / s_end;
    coef_[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / s_end2;
  }

private:
  double end_s_ = 0.0;
  std::array<double, 6> coef_;
};

} // namespace planning_math
} // namespace msquare