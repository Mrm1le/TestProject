#ifndef COMMON_POLYFIT_H
#define COMMON_POLYFIT_H
#if !defined(Local_Pybind)
#include <Eigen/Dense>
#endif
#if defined(Local_Pybind)
#include "eigen3/Eigen/Dense"
#endif

#include <vector>

/**
 * @brief Fit polynomial using Least Square Method.
 *
 * @param data vector of sample data (with member varibles x and y).
 * @param order Fitting order which should be larger than zero.
 * @param coefficients Coefficients vector of fitted polynomial.
 * @return bool whether polyfit succeeds or not.
 */

namespace msquare {

template <class T>
// CJ:coefficients[0]-C0,coefficients[order-1]-C highest
bool Polyfit(const std::vector<T> &data, uint8_t order,
             std::vector<double> &coefficients) {
  // input check
  if (data.size() < order + 1 || order < 1)
    return false;

  Eigen::VectorXd x_vector(data.size());
  Eigen::VectorXd y_vector(data.size());

  for (size_t i = 0; i < data.size(); ++i) {
    x_vector(i) = data[i].x;
    y_vector(i) = data[i].y;
  }

  Eigen::MatrixXd x(data.size(), order + 1);
  Eigen::VectorXd x_col = x_vector;

  x.col(0) = Eigen::VectorXd::Constant(data.size(), 1, 1);
  x.col(1) = x_vector;
  for (size_t i = 2; i < order + 1; ++i) {
    x_col = x_col.array() * x_vector.array();
    x.col(i) = x_col;
  }

  Eigen::VectorXd result =
      (x.transpose() * x).inverse() * (x.transpose()) * y_vector;

  coefficients.clear();
  coefficients.resize(result.size());
  Eigen::VectorXd::Map(&coefficients[0], result.size()) = result;
  return true;
};

} // namespace msquare
#endif
