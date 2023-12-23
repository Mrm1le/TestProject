#include "common/refline/qp_spline_reference_line_smoother.h"
#include "common/utils/frenet_coordinate_system.h"

using namespace msquare;

int main() {

  // smoother config
  ReferenceLineSmootherConfig smoother_config;
  smoother_config.max_constraint_interval_ = 3;
  smoother_config.longitudinal_boundary_bound_ = 1.0;
  smoother_config.max_lateral_boundary_bound_ = 0.5;
  smoother_config.min_lateral_boundary_bound_ = 0.2;
  smoother_config.num_of_total_points_ = 20;
  smoother_config.curb_shift_ = 0.2;
  smoother_config.lateral_buffer_ = 0.2;
  smoother_config.resolution_ = 0.02;
  smoother_config.qp_spline_.spline_order_ = 3;            //[default = 5];
  smoother_config.qp_spline_.max_spline_length_ = 25;      // [default = 25];
  smoother_config.qp_spline_.regularization_weight_ = 0.1; // [default = 0.1];
  smoother_config.qp_spline_.second_derivative_weight_ =
      0.0;                                                   // [default = 0.0];
  smoother_config.qp_spline_.third_derivative_weight_ = 100; // [default = 100];

  // confirm smoother
  std::shared_ptr<QpSplineReferenceLineSmoother> smoother_;
  smoother_ = std::make_shared<QpSplineReferenceLineSmoother>(smoother_config);

  FrenetCoordinateSystemParameters frenet_parameters_;

  frenet_parameters_.zero_speed_threshold = 0.1;
  frenet_parameters_.coord_transform_precision = 0.01;
  frenet_parameters_.step_s = 0.3;
  frenet_parameters_.coarse_step_s = 1.0;
  frenet_parameters_.optimization_gamma = 0.5;
  frenet_parameters_.max_iter = 15;

  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  std::vector<double> vx, vy;
  for (int i = 0; i < 100; i++) {
    vx.push_back(i * 1.0);
    vy.push_back(i * 1.0);
  }

  frenet_coord_.reset(
      new FrenetCoordinateSystem(vx, vy, frenet_parameters_, 0.0, 110.0));

  // // generate anchor points:
  std::vector<AnchorPoint> anchor_points;

  for (int i = 0; i < 10; i++) {
    AnchorPoint anchor;
    anchor.longitudinal_bound = smoother_config.longitudinal_boundary_bound();
    anchor.lateral_bound = smoother_config.max_lateral_boundary_bound();
    auto point = frenet_coord_->GetRefCurvePoint(i * 5.0);
    anchor.path_point.x = i * 5.0 + 10.0; // point.x;
    anchor.path_point.y = 0.0;            // point.y;
    anchor.path_point.theta = 0.0;        // frenet_coord_->GetRefCurveHeading(i
                                          // * 5.0);
    anchor.path_point.s = i * 5.0;
    anchor_points.push_back(anchor);
    MSD_LOG(INFO, "a: %lf, %lf", anchor.path_point.x, anchor.path_point.y);
  }

  for (int i = 10; i < 20; i++) {
    AnchorPoint anchor;
    anchor.longitudinal_bound = smoother_config.longitudinal_boundary_bound();
    anchor.lateral_bound = smoother_config.max_lateral_boundary_bound();
    auto point = frenet_coord_->GetRefCurvePoint(i * 5.0);
    anchor.path_point.x = i * 5.0 + 10.0; // point.x;
    anchor.path_point.y = i * 2.5 - 25.0; // point.y;
    anchor.path_point.theta = 0.0;        // frenet_coord_->GetRefCurveHeading(i
                                          // * 5.0);
    anchor.path_point.s = 50.0 + 5.5901699437393 * (i - 10.0);
    anchor_points.push_back(anchor);
    MSD_LOG(INFO, "a: %lf, %lf", anchor.path_point.y, anchor.path_point.x);
  }

  // GetAnchorPoints(raw_reference_line, &anchor_points);

  smoother_->SetAnchorPoints(anchor_points);

  if (!smoother_->Smooth()) {
    MSD_LOG(INFO, "Failed to smooth reference line with anchor points");
  }

  return 0;
}
