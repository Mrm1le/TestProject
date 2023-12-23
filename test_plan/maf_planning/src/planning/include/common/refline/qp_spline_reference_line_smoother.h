#pragma once

#include <memory>
#include <vector>

#include "common/refline//reference_line_smoother.h"
#include "common/refline//reference_point.h"
#include "common/refline/reference_line_smoother_config.h"
#include "common/refline/refline_smoother/spline_2d_solver.h"
#include "planner/message_type.h"

namespace msquare {

struct SmoothPoint {
  double x;
  double y;
  double theta;
  double curv;
};

class QpSplineReferenceLineSmoother : public ReferenceLineSmoother {
public:
  explicit QpSplineReferenceLineSmoother(
      const ReferenceLineSmootherConfig &config);

  virtual ~QpSplineReferenceLineSmoother() = default;

  void clear() {
    t_knots_.clear();
    anchor_points_.clear();
    // std::unique_ptr<Spline2dSolver> spline_solver_;
    ref_x_ = 0.0;
    ref_y_ = 0.0;
    vx_.clear();
    vy_.clear();
    smooth_points_out_.clear();
    solve_status_ = -2;
  };

  bool Smooth() override;
  //   const ReferenceLine& raw_reference_line,
  //   ReferenceLine* const smoothed_reference_line) override;

  void SetAnchorPoints(const std::vector<AnchorPoint> &achor_points) override;

  void GetRefPoint(std::vector<double> &vx, std::vector<double> &vy,
                   std::vector<SmoothPoint> &smooth_points_out);

  void GetSolveStatus(int &solve_status);

private:
  void Clear();

  bool Sampling();

  bool AddConstraint();

  bool AddKernel();

  bool Solve();

  //   bool ExtractEvaluatedPoints(
  //       const ReferenceLine& raw_reference_line, const std::vector<double>&
  //       vec_t, std::vector<common::PathPoint>* const path_points) const;

  bool GetSFromParamT(const double t, double *const s) const;

  std::uint32_t FindIndex(const double t) const;

private:
  std::vector<double> t_knots_;
  std::vector<AnchorPoint> anchor_points_;

  double ref_x_ = 0.0;
  double ref_y_ = 0.0;
  std::vector<double> vx_, vy_;
  std::vector<SmoothPoint> smooth_points_out_;
  int solve_status_ = -2; // solve status  @zyl
};

} // namespace msquare
