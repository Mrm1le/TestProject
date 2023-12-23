#ifndef __PIECEWISE_JERK_QP_H
#define __PIECEWISE_JERK_QP_H
#include <vector>
#include "osqp_wrapper.h"

namespace msquare {
namespace parking {
class PieceWiseJerk {
 public:
  PieceWiseJerk(const size_t num_of_knots, const double delta_s,
                const Eigen::Vector3d &x_init);

  bool Solve(const size_t max_iter_num = 500);
  bool createQPDefaultSolver(const Eigen::Matrix<double, -1, -1>& H,
                      const Eigen::Matrix<double, -1, 1>& g,
                      const Eigen::Matrix<double, -1, -1>& A,
                      const Eigen::Matrix<double, -1, 1>& l,
                      const Eigen::Matrix<double, -1, 1>& u);
  bool createQPAdaptorSolver(const Eigen::Matrix<double, -1, -1>& H,
                      const Eigen::Matrix<double, -1, 1>& g,
                      const Eigen::Matrix<double, -1, -1>& A,
                      const Eigen::Matrix<double, -1, 1>& l,
                      const Eigen::Matrix<double, -1, 1>& u);
  void set_x_bounds(const std::vector<std::pair<double, double>> &x_bounds);
  void set_dx_bounds(const std::vector<std::pair<double, double>> &dx_bounds);
  void set_ddx_bounds(const std::vector<std::pair<double, double>> &ddx_bounds);
  void set_dddx_bound(const double dddx_lower_bound, const double dddx_upper_bound);
  void set_kappa_bound(const std::vector<std::pair<double, double>> &kappa_bounds);

  void set_weight_x(const double weight_x) { weight_x_ = weight_x; }
  void set_weight_dx(const double weight_dx) { weight_dx_ = weight_dx; }
  void set_weight_ddx(const double weight_ddx) { weight_ddx_ = weight_ddx; }
  void set_weight_dddx(const double weight_dddx) { weight_dddx_ = weight_dddx; }
  void set_x_ref(const std::vector<double> &weight_x_ref_vec, const std::vector<double> &x_ref);
  void set_dx_ref(const std::vector<double> &weight_dx_ref_vec, const std::vector<double> &dx_ref);
  void set_kappa_ref(const std::vector<double> &kappa_ref);
  void set_weight_bound_ref(const double weight_bound_ref);
  void set_weight_kappa(const double weight_kappa, const double weight_kappa_epsilon);

  const Eigen::VectorXd& getSolution() {return m_solution_;}

  std::vector<double> optimal_x() const { return x_; }
  std::vector<double> optimal_dx() const { return dx_; }
  std::vector<double> optimal_ddx() const { return ddx_; }
  template <typename T>
  T *CopyData(const std::vector<T> &vec) {
    T *data = new T[vec.size()];
    memcpy(data, vec.data(), sizeof(T) * vec.size());
    return data;
  }
  void FreeData(OSQPData *data);

 private:
  Eigen::MatrixXd getMatrixH();
  Eigen::VectorXd getGradientVector();
  void getGradientLbAndUbAndA(Eigen::VectorXd &lb, 
        Eigen::VectorXd &ub, Eigen::MatrixXd &A);
  void getInequalityConstraintOfXAndDdx(double *k, double *b, const double kappa_ref,
                                        const double kappa_bound, const double x_min,
                                        const double x_max);

 private:
  size_t num_of_knots_ = 0;
  //优化的输出变量　　
  std::vector<double> x_;
  std::vector<double> dx_;
  std::vector<double> ddx_;
  Eigen::Vector3d x_init_;

  std::vector<std::pair<double, double>> x_bounds_;
  std::vector<std::pair<double, double>> dx_bounds_;
  std::vector<std::pair<double, double>> ddx_bounds_;
  std::pair<double, double> dddx_bounds_;
  std::vector<std::pair<double, double>> kappa_bounds_;

  double weight_x_ = 0.0;
  double weight_dx_ = 0.0;
  double weight_ddx_ = 0.0;
  double weight_dddx_ = 0.0;
  double weight_bound_ref_ = 0.0;
  double weight_kappa_ = 0.0;
  double weight_kappa_epsilon_ = 0.0;

  double delta_s_ = 1.0;
  bool has_x_ref_ = false;
  bool has_dx_ref_ = false;
  bool has_bound_ref_ = false;
  bool has_kappa_ref_ = false;
  double weight_x_ref_ = 0.0;
  std::vector<double> x_ref_;
  std::vector<double> kappa_ref_;
  std::vector<double> dx_ref_;
  // un-uniformed weighting
  std::vector<double> weight_x_ref_vec_;
  std::vector<double> weight_dx_ref_vec_;
  bool has_end_state_ref_ = false;
  Eigen::Vector3d weight_end_state_ = {0.0, 0.0, 0.0};
  Eigen::Vector3d end_state_ref_;

  Eigen::VectorXd m_solution_;
};
} // namespace parking
} // namespace msquare

#endif
