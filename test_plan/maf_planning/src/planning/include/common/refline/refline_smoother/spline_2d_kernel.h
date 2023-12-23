#pragma once

#include <vector>

#include "Eigen/Core"

#include "common/math/vec2d.h"
#include "common/refline/refline_smoother/spline_2d.h"

namespace msquare {

class Spline2dKernel {
public:
  Spline2dKernel(const std::vector<double> &t_knots,
                 const uint32_t spline_order);

  // customized input output
  void AddRegularization(const double regularization_param);
  bool AddKernel(const Eigen::MatrixXd &kernel, const Eigen::MatrixXd &offset,
                 const double weight);
  bool AddKernel(const Eigen::MatrixXd &kernel, const double weight);

  Eigen::MatrixXd *mutable_kernel_matrix();
  Eigen::MatrixXd *mutable_offset();

  const Eigen::MatrixXd kernel_matrix() const;
  const Eigen::MatrixXd offset() const;

  // build-in kernel methods
  void AddDerivativeKernelMatrix(const double weight);
  void AddSecondOrderDerivativeMatrix(const double weight);
  void AddThirdOrderDerivativeMatrix(const double weight);

  // reference line kernel, x_coord in strictly increasing order
  bool AddReferenceLineKernelMatrix(
      const std::vector<double> &t_coord,
      const std::vector<planning_math::Vec2d> &ref_points, const double weight);

private:
  void AddNthDerivativeKernelMatrix(const uint32_t n, const double weight);
  uint32_t find_index(const double x) const;

private:
  Eigen::MatrixXd kernel_matrix_;
  Eigen::MatrixXd offset_;
  std::vector<double> t_knots_;
  uint32_t spline_order_;
  size_t total_params_;
};

} // namespace msquare