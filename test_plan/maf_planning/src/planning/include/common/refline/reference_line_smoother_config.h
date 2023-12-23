#pragma once

namespace msquare {

struct QpSplineSmootherConfig {
  unsigned int spline_order_;       //[default = 5];
  double max_spline_length_;        // [default = 25];
  double regularization_weight_;    // [default = 0.1];
  double second_derivative_weight_; // [default = 0.0];
  double third_derivative_weight_;  // [default = 100];
  double third_derivative_weight() const { return third_derivative_weight_; };
  unsigned int spline_order() const { return spline_order_; }; //[default = 5];
  double max_spline_length() const {
    return max_spline_length_;
  }; // [default = 25];
  double regularization_weight() const {
    return regularization_weight_;
  }; // [default = 0.1];
  double second_derivative_weight() const {
    return second_derivative_weight_;
  }; // [default = 0.0];
  // double third_derivative_weight(){ return third_derivative_weight;};//
  // [default = 100];
};

struct ReferenceLineSmootherConfig {
  // The output resolution for discrete point smoother reference line is
  // directly decided by max_constraint_interval
  double max_constraint_interval_;     // [default = 5];
  double longitudinal_boundary_bound_; // [default = 1.0];
  double max_lateral_boundary_bound_;  // [default = 0.5];
  double min_lateral_boundary_bound_;  // 4 [default = 0.2];
  // The output resolution for qp smoother reference line.
  unsigned int num_of_total_points_; // [default = 500];
  double curb_shift_;                // default = 0.2];
  double lateral_buffer_;            // = 7 [default = 0.2];
  // The output resolution for spiral smoother reference line.
  double resolution_; // = 8 [default = 0.02];
  QpSplineSmootherConfig qp_spline_;

  double max_constraint_interval() const {
    return max_constraint_interval_;
  }; // [default = 5];
  double longitudinal_boundary_bound() const {
    return longitudinal_boundary_bound_;
  }; // [default = 1.0];
  double max_lateral_boundary_bound() const {
    return max_lateral_boundary_bound_;
  }; // [default = 0.5];
  double min_lateral_boundary_bound() const {
    return min_lateral_boundary_bound_;
  }; // 4 [default = 0.2];
  // The output resolution for qp smoother reference line.
  unsigned int num_of_total_points() const {
    return num_of_total_points_;
  };                                                 // [default = 500];
  double curb_shift() const { return curb_shift_; }; // default = 0.2];
  double lateral_buffer() const {
    return lateral_buffer_;
  }; // = 7 [default = 0.2];
  // The output resolution for spiral smoother reference line.
  double resolution() const { return resolution_; }; // = 8 [default = 0.02];
  QpSplineSmootherConfig qp_spline() const { return qp_spline_; };
};

} // namespace msquare
