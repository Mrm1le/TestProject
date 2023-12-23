#pragma once

#include "common/utils/gauss_quad_constants.h"
#include "fast_math.hpp"
#include "mph_assert.h"

#include "Eigen/Core"

namespace planner_spline {

// Cubic spline interpolation coefficients of this form:
// x(s) = A * x0 + B * x1 + C * d2x0 + D * d2x1
struct Coefficients {
  double A;
  double B;
  double C;
  double D;
};

struct SamplePointCoeffInfo {
  double sample_s;
  Coefficients coeffs_at_sample_s;
  Coefficients dcoeffs_at_sample_s;
  Coefficients d2coeffs_at_sample_s;
  Coefficients d3coeffs_at_sample_s;
  size_t segment_index_at_sample_s;
};

/*
 * @brief Planner cubic spline control points
 */
template <size_t NUM_CONTROL_POINTS, typename T>
struct PlannerCubicSplineControlPoints {
  static constexpr size_t NUM_SEGMENTS = NUM_CONTROL_POINTS - 1;
  std::array<T, NUM_CONTROL_POINTS> x = {};
  std::array<T, NUM_CONTROL_POINTS> d2x = {};

  void build_control_points(
      const double &init_x, const double &init_dx_ds, const double &end_d2x_ds2,
      const std::array<double, NUM_CONTROL_POINTS> &s_at_control_points,
      const std::array<T, NUM_SEGMENTS> &x_at_end_of_segments,
      const Eigen::Matrix<double, NUM_CONTROL_POINTS - 2,
                          NUM_CONTROL_POINTS - 2>
          &inverse_interpolation_matrix) {
    // Reference: this is a C++ implementation of scipy cubic spline
    // Comparison between C++ and scipy implementation is in slider_tutorial.py
    // https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html
    // https://en.wikiversity.org/wiki/Cubic_Spline_Interpolation
    x[0] = T(init_x);
    std::copy(x_at_end_of_segments.begin(), x_at_end_of_segments.end(),
              x.begin() + 1);
    d2x[NUM_CONTROL_POINTS - 1] = T(end_d2x_ds2);
    const auto &S = s_at_control_points;

    // update the rest of path parameters
    Eigen::Matrix<T, NUM_CONTROL_POINTS - 2, 1> b_xx;
    double reciprocal_segment_2_1 = 1.0f / (S[2] - S[1]);
    double reciprocal_segment_1_0 = 1.0f / (S[1] - S[0]);
    b_xx(0, 0) = (x[2] - x[1]) * reciprocal_segment_2_1 -
                 (x[1] - x[0]) * reciprocal_segment_1_0 +
                 0.5 * (init_dx_ds - (x[1] - x[0]) * reciprocal_segment_1_0);

    // compute b_xx
    for (size_t i = 1; i < NUM_CONTROL_POINTS - 2; i++) {
      reciprocal_segment_2_1 = 1.0 / (S[i + 2] - S[i + 1]);
      reciprocal_segment_1_0 = 1.0 / (S[i + 1] - S[i]);
      b_xx(i, 0) = (x[i + 2] - x[i + 1]) * reciprocal_segment_2_1 -
                   (x[i + 1] - x[i]) * reciprocal_segment_1_0;
    }

    // handle non-zero end_d2x_ds2
    b_xx(NUM_CONTROL_POINTS - 3, 0) -=
        1.0 / 6.0 * (S[NUM_CONTROL_POINTS - 1] - S[NUM_CONTROL_POINTS - 2]) *
        d2x[NUM_CONTROL_POINTS - 1];

    // solve for d2x
    // ? is the template cast necessary?
    Eigen::Matrix<T, NUM_CONTROL_POINTS - 2, 1> solved_d2x =
        inverse_interpolation_matrix.template cast<T>() * b_xx;
    for (size_t i = 0; i < NUM_CONTROL_POINTS - 2; i++) {
      d2x[i + 1] = solved_d2x(i, 0);
    }

    // compute d2x_start
    const double segment_1_0 = S[1] - S[0];
    reciprocal_segment_1_0 = 1.0 / segment_1_0;
    d2x[0] = -3.0 * reciprocal_segment_1_0 *
             (1.0 / 6.0 * segment_1_0 * d2x[1] + init_dx_ds -
              (x[1] - x[0]) * reciprocal_segment_1_0);
  }
};

/*
 * @brief Index of sample point on a cubic spline. There are NUM_CONTROL_POINTS
 * control points, (NUM_CONTROL_POINTS-1) segements, and QUADRATURE_ORDER sample
 * points on each segment.
 */
template <size_t NUM_CONTROL_POINTS> struct Index {
  static constexpr size_t NUM_SEGMENTS = NUM_CONTROL_POINTS - 1;

  // Each segment has QUADRATURE_ORDER sample points
  size_t segment_index = 0;
  size_t quad_index =
      path_planner::QUADRATURE_ORDER; // init to an invalid number
  bool initialized = false;

  void reset() {
    segment_index = 0;
    quad_index = path_planner::QUADRATURE_ORDER;
    initialized = false;
  }

  /*
   * @brief advance current index one step forward
   */
  bool advance() {
    if (!initialized) {
      segment_index = 0;
      quad_index = 0;
      initialized = true;
      return true;
    }

    if (++quad_index < path_planner::QUADRATURE_ORDER) {
      return true;
    } else {
      // wrap quad index around, then increment segment
      quad_index = 0;
    }

    if (++segment_index < NUM_SEGMENTS) {
      return true;
    } else {
      // reached the end already
      segment_index = NUM_SEGMENTS - 1;
      quad_index = path_planner::QUADRATURE_ORDER - 1;
      return false;
    }
  }

  /*
   * @brief return index is first index
   */
  bool is_first_sample() const {
    return initialized && (quad_index == 0) && (segment_index == 0);
  }

  /*
   * @brief return flat sample point index
   */
  size_t get_flat_index() const {
    return initialized
               ? (quad_index + segment_index * path_planner::QUADRATURE_ORDER)
               : 0;
  }

  /*
   * @brief return index one step forward, but do not change current index
   */
  Index advanced() const {
    Index adv = *this;
    adv.advance();
    return adv;
  }
};

/*
 * @brief Planner cubic spline utility
 *        s_at_control_points_ are constants before optimization, handled as
 * member variable for reusing; x_at_control_points_ are variable during
 * optimization, handled as params at runtime
 */
template <size_t NUM_CONTROL_POINTS> class PlannerCubicSpline {
public:
  using IndexT = Index<NUM_CONTROL_POINTS>;
  static constexpr size_t NUM_SEGMENTS = NUM_CONTROL_POINTS - 1;
  static constexpr size_t TOTAL_QUAD_POINTS =
      NUM_SEGMENTS * path_planner::QUADRATURE_ORDER;

  PlannerCubicSpline() = default;

  PlannerCubicSpline(
      const std::array<double, NUM_CONTROL_POINTS> &s_at_control_points) {
    initialize(s_at_control_points);
  }

  /*
   * @brief initialize spline control points and sampling quad points location
   */
  void initialize(
      const std::array<double, NUM_CONTROL_POINTS> &s_at_control_points) {
    s_at_control_points_ = s_at_control_points;
    compute_inverse_interpolation_matrix();

    // populate s and interpolation coefficients at each quad point
    IndexT index;
    while (index.advance()) {
      s_at_quad_points_[index.segment_index][index.quad_index] =
          s_at_control_points_[index.segment_index] +
          (s_at_control_points_[index.segment_index + 1] -
           s_at_control_points_[index.segment_index]) *
              GAUSS_QUAD_5TH_POS[index.quad_index];

      populate_interpolation_coefficients(
          s_at_control_points_[index.segment_index + 1] -
              s_at_control_points_[index.segment_index],
          s_at_quad_points_[index.segment_index][index.quad_index] -
              s_at_control_points_[index.segment_index],
          coeffs_[index.segment_index][index.quad_index],
          dcoeffs_ds_[index.segment_index][index.quad_index],
          d2coeffs_ds2_[index.segment_index][index.quad_index],
          d3coeffs_ds3_[index.segment_index][index.quad_index]);

      weight_at_quad_points_[index.segment_index][index.quad_index] =
          GAUSS_QUAD_5TH_WT[index.quad_index];
    }
  }

  // note that this function is called during optimization
  template <typename T>
  using ControlPointT = PlannerCubicSplineControlPoints<NUM_CONTROL_POINTS, T>;

  /*
   * @brief build control points at given positions
   *
   * @param[in]     init_x                   initial control point value
   * @param[in]     init_dx_ds               initial control point derivative
   * @param[in]     end_d2x_ds2              end control point second derivative
   * @param[in]     x_at_end_of_segments     value at end of each spline segment
   * @param[out]    cps                      constructed control points
   */
  template <typename T>
  void
  build_control_points(const double &init_x, const double &init_dx_ds,
                       const double &end_d2x_ds2,
                       const std::array<T, NUM_SEGMENTS> &x_at_end_of_segments,
                       ControlPointT<T> &cps) const {
    cps.build_control_points(init_x, init_dx_ds, end_d2x_ds2,
                             s_at_control_points_, x_at_end_of_segments,
                             inverse_interpolation_matrix_);
  }

  template <typename T>
  T sample(const Coefficients &coeff, const size_t segment_index,
           const ControlPointT<T> &cps) const {
    return coeff.A * cps.x[segment_index] + coeff.B * cps.x[segment_index + 1] +
           coeff.C * cps.d2x[segment_index] +
           coeff.D * cps.d2x[segment_index + 1];
  }

  double get_s_at_index(const IndexT &index) const {
    return s_at_quad_points_[index.segment_index][index.quad_index];
  }

  template <typename T>
  T sample_value(const IndexT &index, const ControlPointT<T> &cps) const {
    mph_assert(index.initialized);
    return sample(coeffs_[index.segment_index][index.quad_index],
                  index.segment_index, cps);
  }

  template <typename T>
  T sample_value_at_s(const SamplePointCoeffInfo &sample_point_info,
                      const ControlPointT<T> &cps) const {
    return sample(sample_point_info.coeffs_at_sample_s,
                  sample_point_info.segment_index_at_sample_s, cps);
  }

  template <typename T>
  T sample_derivative(const IndexT &index, const ControlPointT<T> &cps) const {
    mph_assert(index.initialized);
    return sample(dcoeffs_ds_[index.segment_index][index.quad_index],
                  index.segment_index, cps);
  }

  template <typename T>
  T sample_derivative_at_sample_s(const SamplePointCoeffInfo &sample_point_info,
                                  const ControlPointT<T> &cps) const {
    return sample(sample_point_info.dcoeffs_at_sample_s,
                  sample_point_info.segment_index_at_sample_s, cps);
  }

  template <typename T>
  T sample_second_derivative(const IndexT &index,
                             const ControlPointT<T> &cps) const {
    mph_assert(index.initialized);
    return sample(d2coeffs_ds2_[index.segment_index][index.quad_index],
                  index.segment_index, cps);
  }

  template <typename T>
  T sample_second_derivative_at_sample_s(
      const SamplePointCoeffInfo &sample_point_info,
      const ControlPointT<T> &cps) const {
    return sample(sample_point_info.d2coeffs_at_sample_s,
                  sample_point_info.segment_index_at_sample_s, cps);
  }

  template <typename T>
  T sample_third_derivative(const IndexT &index,
                            const ControlPointT<T> &cps) const {
    mph_assert(index.initialized);
    return sample(d3coeffs_ds3_[index.segment_index][index.quad_index],
                  index.segment_index, cps);
  }

  template <typename T>
  T sample_third_derivative_at_sample_s(
      const SamplePointCoeffInfo &sample_point_info,
      const ControlPointT<T> &cps) const {
    return sample(sample_point_info.d3coeffs_at_sample_s,
                  sample_point_info.segment_index_at_sample_s, cps);
  }

  void populate_interpolation_coefficients_at_sample_s(
      SamplePointCoeffInfo &sample_point_info) const {
    mph_assert(sample_point_info.sample_s > s_at_control_points_[0]);
    mph_assert(sample_point_info.sample_s <
               s_at_control_points_[NUM_CONTROL_POINTS - 1]);

    sample_point_info.segment_index_at_sample_s = 0;
    for (size_t segment_index = 0; segment_index < NUM_SEGMENTS;
         segment_index++) {
      if (sample_point_info.sample_s <=
              s_at_control_points_[segment_index + 1] &&
          sample_point_info.sample_s > s_at_control_points_[segment_index]) {
        sample_point_info.segment_index_at_sample_s = segment_index;
        break;
      }
    }

    populate_interpolation_coefficients(
        s_at_control_points_[sample_point_info.segment_index_at_sample_s + 1] -
            s_at_control_points_[sample_point_info.segment_index_at_sample_s],
        sample_point_info.sample_s -
            s_at_control_points_[sample_point_info.segment_index_at_sample_s],
        sample_point_info.coeffs_at_sample_s,
        sample_point_info.dcoeffs_at_sample_s,
        sample_point_info.d2coeffs_at_sample_s,
        sample_point_info.d3coeffs_at_sample_s);
  }

private:
  /*
   * @brief compute inverse interpolation matrix for coefficient conversion
   */
  void compute_inverse_interpolation_matrix() {
    // Reference: this is a C++ implementation of scipy cubic spline
    // Comparison between C++ and scipy implementation is in slider_tutorial.py
    // https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html
    // https://en.wikiversity.org/wiki/Cubic_Spline_Interpolation
    static_assert(NUM_CONTROL_POINTS >= 4,
                  "At least 4 control points are needed!");

    const auto &S = s_at_control_points_;
    Eigen::Matrix<double, NUM_CONTROL_POINTS - 2, NUM_CONTROL_POINTS - 2>
        interpolation_matrix;
    interpolation_matrix.setZero();

    interpolation_matrix(0, 0) =
        1.0f / 3.0f * (S[2] - S[0]) - 1.0f / 12.0f * (S[1] - S[0]);
    interpolation_matrix(0, 1) = 1.0f / 6.0f * (S[2] - S[1]);

    for (size_t i = 1; i < NUM_CONTROL_POINTS - 3; i++) {
      interpolation_matrix(i, i - 1) = 1.0f / 6.0f * (S[i + 1] - S[i]);
      interpolation_matrix(i, i) = 1.0f / 3.0f * (S[i + 2] - S[i]);
      interpolation_matrix(i, i + 1) = 1.0f / 6.0f * (S[i + 2] - S[i + 1]);
    }

    interpolation_matrix(NUM_CONTROL_POINTS - 3, NUM_CONTROL_POINTS - 4) =
        1.0f / 6.0f * (S[NUM_CONTROL_POINTS - 2] - S[NUM_CONTROL_POINTS - 3]);
    interpolation_matrix(NUM_CONTROL_POINTS - 3, NUM_CONTROL_POINTS - 3) =
        1.0f / 3.0f * (S[NUM_CONTROL_POINTS - 1] - S[NUM_CONTROL_POINTS - 3]);

    inverse_interpolation_matrix_ = interpolation_matrix.inverse();
  }

  /*
   * @brief populate spline interplation coefficients
   */
  void populate_interpolation_coefficients(const double &cp_spacing,
                                           const double &sample_ds,
                                           Coefficients &c, Coefficients &dc,
                                           Coefficients &d2c,
                                           Coefficients &d3c) const {
    const double mult = 1.0 / 6.0;
    const double cp_spacing_inv = 1.0 / cp_spacing;
    const double cp_spacing_sq = cp_spacing * cp_spacing;

    // Cubic spline interpolation coefficients of this form:
    // x(s) = A * x0 + B * x1 + C * d2x0 + D * d2x1
    c.A = (cp_spacing - sample_ds) * cp_spacing_inv;
    c.B = 1.0 - c.A;
    c.C = mult * cp_spacing_sq * (fast_math::integer_pow_3(c.A) - c.A);
    c.D = mult * cp_spacing_sq * (fast_math::integer_pow_3(c.B) - c.B);

    // coefficients of derivatives
    dc.A = -cp_spacing_inv;
    dc.B = cp_spacing_inv;
    dc.C = mult * cp_spacing * (1.0 - 3.0 * c.A * c.A);
    dc.D = mult * cp_spacing * (3.0 * c.B * c.B - 1.0);

    // coefficients of second derivatives
    d2c.A = 0.0;
    d2c.B = 0.0;
    d2c.C = c.A;
    d2c.D = c.B;

    // coefficients of third derivatives
    d3c.A = 0.0;
    d3c.B = 0.0;
    d3c.C = -cp_spacing_inv;
    d3c.D = cp_spacing_inv;
  }

private:
  std::array<double, NUM_CONTROL_POINTS> s_at_control_points_;
  std::array<std::array<double, path_planner::QUADRATURE_ORDER>, NUM_SEGMENTS>
      s_at_quad_points_;
  Eigen::Matrix<double, NUM_CONTROL_POINTS - 2, NUM_CONTROL_POINTS - 2>
      inverse_interpolation_matrix_;

  std::array<std::array<Coefficients, path_planner::QUADRATURE_ORDER>,
             NUM_SEGMENTS>
      coeffs_;
  std::array<std::array<Coefficients, path_planner::QUADRATURE_ORDER>,
             NUM_SEGMENTS>
      dcoeffs_ds_;
  std::array<std::array<Coefficients, path_planner::QUADRATURE_ORDER>,
             NUM_SEGMENTS>
      d2coeffs_ds2_;
  std::array<std::array<Coefficients, path_planner::QUADRATURE_ORDER>,
             NUM_SEGMENTS>
      d3coeffs_ds3_;

  std::array<std::array<double, path_planner::QUADRATURE_ORDER>, NUM_SEGMENTS>
      weight_at_quad_points_;
};

// use extern template to reduce compiled module size, but still check code
// validity
// extern template class Index<path_planner::NUM_PATH_CONTROL_POINTS>;
// extern template class
// PlannerCubicSpline<path_planner::NUM_PATH_CONTROL_POINTS>;

} // namespace planner_spline