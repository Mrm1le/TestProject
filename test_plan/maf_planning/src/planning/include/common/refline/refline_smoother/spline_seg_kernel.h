/**
 * @file : spline_seg_kernel.h
 * @brief: generating integrated kernels for smoothing spline
 *
 *           x' P x  = int_0 ^x  (f(x)^(k))^2 dx, k = 0, 1, 2, 3
 *           P is the kernel of k-th smooth kernel
 **/

#pragma once

#include <string>

#include "Eigen/Core"

#include "common/utils/macro.h"

namespace msquare {

class SplineSegKernel {
public:
  void init();
  // generating kernel matrix
  Eigen::MatrixXd Kernel(const uint32_t num_params, const double accumulated_x);

  // only support N <= 3 cases
  Eigen::MatrixXd NthDerivativeKernel(const uint32_t n,
                                      const uint32_t num_params,
                                      const double accumulated_x);

private:
  Eigen::MatrixXd DerivativeKernel(const uint32_t num_of_params,
                                   const double accumulated_x);
  Eigen::MatrixXd SecondOrderDerivativeKernel(const uint32_t num_of_params,
                                              const double accumulated_x);
  Eigen::MatrixXd ThirdOrderDerivativeKernel(const uint32_t num_of_params,
                                             const double accumulated_x);

  void IntegratedTermMatrix(const uint32_t num_of_params, const double x,
                            const std::string &type,
                            Eigen::MatrixXd *term_matrix) const;
  void CalculateFx(const uint32_t num_of_params);
  void CalculateDerivative(const uint32_t num_of_params);
  void CalculateSecondOrderDerivative(const uint32_t num_of_params);
  void CalculateThirdOrderDerivative(const uint32_t num_of_params);

  const uint32_t reserved_order_ = 5;
  Eigen::MatrixXd kernel_fx_;
  Eigen::MatrixXd kernel_derivative_;
  Eigen::MatrixXd kernel_second_order_derivative_;
  Eigen::MatrixXd kernel_third_order_derivative_;
  bool initialized_{false};

  DECLARE_SINGLETON(SplineSegKernel)
};

} // namespace msquare
