#pragma once
#include "path_planner_types.hpp"
#include "planner/motion_planner/planner_cubic_spline/planner_cubic_spline_utility.hpp"

using namespace planner_spline;
namespace speed_planner {

template <typename T> class AccelerationSpline {
public:
  static constexpr size_t N = speed_planner::NUM_SPEED_CONTROL_POINTS;
  static constexpr size_t N_SEG = speed_planner::NUM_SPEED_SEGMENTS;
  using IndexT = Index<N>;

  AccelerationSpline(const PlannerCubicSpline<N> *acceleration_spline,
                     const double init_x, const double init_dx_ds,
                     const T *acceleration_at_end_of_segments_input) {
    acceleration_spline_ = acceleration_spline;
    std::array<T, N_SEG> acceleration_at_end_of_segments;
    for (size_t i = 0; i < N_SEG; i++) {
      const T &offset = acceleration_at_end_of_segments_input[i];
      acceleration_at_end_of_segments[i] = offset;
    }
    acceleration_spline_->build_control_points(
        init_x, init_dx_ds, /*end_d2x_ds2*/ 0.0,
        acceleration_at_end_of_segments, acceleration_cps_);
  }

  double get_s_at_index(const IndexT &index) const {
    return acceleration_spline_->get_s_at_index(index);
  }

  T get_acceleration_at_index(const IndexT &index) const {
    return acceleration_spline_->sample_value(index, acceleration_cps_);
  }

  T get_acceleration_at_sample_s(
      const SamplePointCoeffInfo &sample_point_info) const {
    return acceleration_spline_->sample_value_at_s(sample_point_info,
                                                   acceleration_cps_);
  }

  T get_derivative_at_index(const IndexT &index) const {
    return acceleration_spline_->sample_derivative(index, acceleration_cps_);
  }

  T get_derivative_at_sample_s(
      const SamplePointCoeffInfo &sample_point_info) const {
    return acceleration_spline_->sample_derivative_at_sample_s(
        sample_point_info, acceleration_cps_);
  }

  void
  compute_sample_point_info(SamplePointCoeffInfo &sample_point_info) const {
    acceleration_spline_->populate_interpolation_coefficients_at_sample_s(
        sample_point_info);
  }

private:
  const PlannerCubicSpline<N> *acceleration_spline_;

  PlannerCubicSplineControlPoints<N, T> acceleration_cps_;
};
} // namespace speed_planner
