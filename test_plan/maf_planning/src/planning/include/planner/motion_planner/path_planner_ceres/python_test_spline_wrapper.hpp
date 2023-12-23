#pragma once
#include "path_planner_constants.hpp"
#include "planner/motion_planner/planner_cubic_spline/planner_cubic_spline_utility.hpp"
#include <Eigen/Dense>

namespace path_planner {
using namespace planner_spline;

class TestCubicSpline {
public:
  static constexpr size_t N = path_planner::NUM_PATH_CONTROL_POINTS;

  TestCubicSpline(const double &init_dx_ds, const double &end_d2x_ds2,
                  const std::array<double, N> &s_at_control_points,
                  const std::array<double, N> &x_at_control_points) {
    path_spline_.initialize(s_at_control_points);
    std::array<double, N - 1> x_at_end_of_segments;
    std::copy(x_at_control_points.begin() + 1, x_at_control_points.end(),
              x_at_end_of_segments.begin());
    path_spline_.build_control_points(x_at_control_points[0], init_dx_ds,
                                      end_d2x_ds2, x_at_end_of_segments, cps_);
  }

  void get_all_quad_points_data(Eigen::Ref<Eigen::VectorXd> all_quad_pt_s,
                                Eigen::Ref<Eigen::VectorXd> all_quad_pt_x,
                                Eigen::Ref<Eigen::VectorXd> all_quad_pt_dx,
                                Eigen::Ref<Eigen::VectorXd> all_quad_pt_d2x,
                                Eigen::Ref<Eigen::VectorXd> all_quad_pt_d3x) {
    Index<N> index;
    while (index.advance()) {
      const size_t flat_index = index.get_flat_index();
      all_quad_pt_s(flat_index) = path_spline_.get_s_at_index(index);
      all_quad_pt_x(flat_index) = path_spline_.sample_value(index, cps_);
      all_quad_pt_dx(flat_index) = path_spline_.sample_derivative(index, cps_);
      all_quad_pt_d2x(flat_index) =
          path_spline_.sample_second_derivative(index, cps_);
      all_quad_pt_d3x(flat_index) =
          path_spline_.sample_third_derivative(index, cps_);
    }
  }

private:
  PlannerCubicSplineControlPoints<N, double> cps_;
  PlannerCubicSpline<N> path_spline_;
};
} // namespace path_planner