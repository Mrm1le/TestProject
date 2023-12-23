#pragma once
#include "path_planner_types.hpp"
#include "planner/motion_planner/planner_cubic_spline/planner_cubic_spline_utility.hpp"

using namespace planner_spline;
namespace path_planner {

template <typename T> class PathSpline {
public:
  static constexpr size_t N = path_planner::NUM_PATH_CONTROL_POINTS;
  static constexpr size_t N_SEG = path_planner::NUM_PATH_CONTROL_POINTS - 1;
  using IndexT = Index<N>;

  PathSpline(
      const PlannerCubicSpline<N> *path_offset_spline, const double init_x,
      const double init_y, const double init_dx_ds, const double init_dy_ds,
      const T *offset_at_end_of_segments,
      const std::array<Eigen::Vector2d, N_SEG> &ref_pos_at_end_of_segments,
      const std::array<Eigen::Vector2d, N_SEG>
          &ref_heading_at_end_of_segments) {
    // compute x and y from offset
    path_offset_spline_ = path_offset_spline;
    std::array<T, N_SEG> x_at_end_of_segments;
    std::array<T, N_SEG> y_at_end_of_segments;
    for (size_t i = 0; i < N_SEG; i++) {
      const T &offset = offset_at_end_of_segments[i];
      x_at_end_of_segments[i] = ref_pos_at_end_of_segments[i].x() -
                                ref_heading_at_end_of_segments[i].y() * offset;
      y_at_end_of_segments[i] = ref_pos_at_end_of_segments[i].y() +
                                ref_heading_at_end_of_segments[i].x() * offset;
    }
    path_offset_spline_->build_control_points(
        init_x, init_dx_ds, /*end_d2x_ds2*/ 0.0, x_at_end_of_segments, x_cps_);
    path_offset_spline_->build_control_points(
        init_y, init_dy_ds, /*end_d2y_ds2*/ 0.0, y_at_end_of_segments, y_cps_);
  }

  double get_s_at_index(const IndexT &index) const {
    return path_offset_spline_->get_s_at_index(index);
  }

  Vector2T<T> get_pos_at_index(const IndexT &index) const {
    return {path_offset_spline_->sample_value(index, x_cps_),
            path_offset_spline_->sample_value(index, y_cps_)};
  }

  Vector2T<T>
  get_pos_at_sample_s(const SamplePointCoeffInfo &sample_point_info) const {
    return {path_offset_spline_->sample_value_at_s(sample_point_info, x_cps_),
            path_offset_spline_->sample_value_at_s(sample_point_info, y_cps_)};
  }

  Vector2T<T> get_derivative_at_index(const IndexT &index) const {
    return {path_offset_spline_->sample_derivative(index, x_cps_),
            path_offset_spline_->sample_derivative(index, y_cps_)};
  }

  Vector2T<T> get_derivative_at_sample_s(
      const SamplePointCoeffInfo &sample_point_info) const {
    return {path_offset_spline_->sample_derivative_at_sample_s(
                sample_point_info, x_cps_),
            path_offset_spline_->sample_derivative_at_sample_s(
                sample_point_info, y_cps_)};
  }

  Vector2T<T> get_second_derivative_at_index(const IndexT &index) const {
    return {path_offset_spline_->sample_second_derivative(index, x_cps_),
            path_offset_spline_->sample_second_derivative(index, y_cps_)};
  }

  Vector2T<T> get_second_derivative_at_sample_s(
      const SamplePointCoeffInfo &sample_point_info) const {
    return {path_offset_spline_->sample_second_derivative_at_sample_s(
                sample_point_info, x_cps_),
            path_offset_spline_->sample_second_derivative_at_sample_s(
                sample_point_info, y_cps_)};
  }

  Vector2T<T> get_third_derivative_at_index(const IndexT &index) const {
    return {path_offset_spline_->sample_third_derivative(index, x_cps_),
            path_offset_spline_->sample_third_derivative(index, y_cps_)};
  }

  Vector2T<T> get_third_derivative_at_sample_s(
      const SamplePointCoeffInfo &sample_point_info) const {
    return {path_offset_spline_->sample_third_derivative_at_sample_s(
                sample_point_info, x_cps_),
            path_offset_spline_->sample_third_derivative_at_sample_s(
                sample_point_info, y_cps_)};
  }

  void
  compute_sample_point_info(SamplePointCoeffInfo &sample_point_info) const {
    path_offset_spline_->populate_interpolation_coefficients_at_sample_s(
        sample_point_info);
  }

private:
  // path offset spline control/quad point s values and coefficients are the
  // same for x cubic spline and y cubic spline, so shared here.
  const PlannerCubicSpline<N> *path_offset_spline_;

  // use path offset spline to get control points for x and y separately
  PlannerCubicSplineControlPoints<N, T> x_cps_;
  PlannerCubicSplineControlPoints<N, T> y_cps_;
};
} // namespace path_planner
