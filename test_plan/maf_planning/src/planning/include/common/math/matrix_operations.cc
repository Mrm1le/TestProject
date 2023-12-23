/**
 * @file
 * @brief Defines some useful matrix operations.
 */

#include <cmath>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/SVD"

#include "common/math/matrix_operations.h"

namespace msquare {
namespace planning_math {

bool ContinuousToDiscrete(const Eigen::MatrixXd &m_a,
                          const Eigen::MatrixXd &m_b,
                          const Eigen::MatrixXd &m_c,
                          const Eigen::MatrixXd &m_d, const double ts,
                          Eigen::MatrixXd *ptr_a_d, Eigen::MatrixXd *ptr_b_d,
                          Eigen::MatrixXd *ptr_c_d, Eigen::MatrixXd *ptr_d_d) {
  if (ts <= 0.0) {
    // std::cout << "ContinuousToDiscrete : ts is less than or equal to zero" <<
    // std::endl;
    return false;
  }

  // Only matrix_a is mandatory to be non-zeros in matrix
  // conversion.
  if (m_a.rows() == 0) {
    // std::cout << "ContinuousToDiscrete: matrix_a size 0 " << std::endl;
    return false;
  }

  if (m_a.cols() != m_b.rows() || m_b.cols() != m_d.cols() ||
      m_c.rows() != m_d.rows() || m_a.cols() != m_c.cols()) {
    // std::cout << "ContinuousToDiscrete: matrix dimensions mismatch" <<
    // std::endl;
    return false;
  }

  Eigen::MatrixXd m_identity =
      Eigen::MatrixXd::Identity(m_a.cols(), m_a.rows());

  *ptr_a_d =
      (m_identity - ts * 0.5 * m_a).inverse() * (m_identity + ts * 0.5 * m_a);

  *ptr_b_d = std::sqrt(ts) * (m_identity - ts * 0.5 * m_a).inverse() * m_b;

  *ptr_c_d = std::sqrt(ts) * m_c * (m_identity - ts * 0.5 * m_a).inverse();

  *ptr_d_d = 0.5 * m_c * (m_identity - ts * 0.5 * m_a).inverse() * m_b + m_d;

  return true;
}

} // namespace planning_math
} // namespace msquare