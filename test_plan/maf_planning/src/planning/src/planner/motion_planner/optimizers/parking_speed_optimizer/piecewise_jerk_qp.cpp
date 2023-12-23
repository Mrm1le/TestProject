#include "planner/motion_planner/optimizers/parking_speed_optimizer/piecewise_jerk_qp.h"
#include "common/math/matrix_operations.h"
namespace msquare {
namespace parking {

using namespace Eigen;

PieceWiseJerk::PieceWiseJerk(const size_t num_of_knots, const double delta_s,
                             const Eigen::Vector3d& x_init) {
  // CHECK_GE(num_of_knots, 2ul);
  num_of_knots_ = num_of_knots;
  x_init_ = x_init;
  delta_s_ = delta_s;
  x_bounds_.resize(num_of_knots, std::make_pair(std::numeric_limits<double>::lowest(),
                                                std::numeric_limits<double>::max()));
  dx_bounds_.resize(num_of_knots, std::make_pair(std::numeric_limits<double>::lowest(),
                                                 std::numeric_limits<double>::max()));
  ddx_bounds_.resize(num_of_knots_, std::make_pair(std::numeric_limits<double>::lowest(),
                                                   std::numeric_limits<double>::max()));
  // kappa_bounds_.first = -0.19;
  // kappa_bounds_.second = 0.19;
  weight_x_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
  weight_dx_ref_vec_ = std::vector<double>(num_of_knots_, 0.0);
}
void PieceWiseJerk::set_x_bounds(const std::vector<std::pair<double, double>>& x_bounds) {
  // CHECK_EQ(x_bounds.size(), num_of_knots_);
  x_bounds_ = std::move(x_bounds);
}
void PieceWiseJerk::set_dx_bounds(const std::vector<std::pair<double, double>>& dx_bounds) {
  // CHECK_EQ(dx_bounds.size(), num_of_knots_);
  dx_bounds_ = std::move(dx_bounds);
}
void PieceWiseJerk::set_ddx_bounds(const std::vector<std::pair<double, double>>& ddx_bounds) {
  // CHECK_EQ(ddx_bounds.size(), num_of_knots_);

  ddx_bounds_ = std::move(ddx_bounds);
  if (std::fabs(ddx_bounds_.back().first) > 1e6) {
    //PERROR << "ddx_bounds error";
  }
}
void PieceWiseJerk::set_dddx_bound(const double dddx_lower_bound, const double dddx_upper_bound) {
  dddx_bounds_.first = dddx_lower_bound;
  dddx_bounds_.second = dddx_upper_bound;
}
void PieceWiseJerk::set_kappa_bound(const std::vector<std::pair<double, double>>& kappa_bounds) {
  // kappa_bounds_.first = kappa_lower_bound;
  // kappa_bounds_.second = kappa_upper_bound;
  kappa_bounds_ = std::move(kappa_bounds);
}
void PieceWiseJerk::set_x_ref(const std::vector<double>& weight_x_ref_vec,
                              const std::vector<double>& x_ref) {
  // CHECK_EQ(x_ref.size(), num_of_knots_);
  // CHECK_EQ(weight_x_ref_vec.size(), num_of_knots_);
  x_ref_ = std::move(x_ref);
  weight_x_ref_vec_ = std::move(weight_x_ref_vec);
  has_x_ref_ = true;
}

void PieceWiseJerk::set_dx_ref(const std::vector<double>& weight_dx_ref_vec,
                               const std::vector<double>& dx_ref) {
  // CHECK_EQ(dx_ref.size(), num_of_knots_);
  // CHECK_EQ(weight_dx_ref_vec.size(), num_of_knots_);
  dx_ref_ = std::move(dx_ref);
  weight_dx_ref_vec_ = std::move(weight_dx_ref_vec);
  has_dx_ref_ = true;
}

void PieceWiseJerk::set_kappa_ref(const std::vector<double>& kappa_ref) {
  // CHECK_EQ(kappa_ref.size(), num_of_knots_);
  kappa_ref_ = std::move(kappa_ref);
  has_kappa_ref_ = true;
}

void PieceWiseJerk::set_weight_bound_ref(const double weight_bound_ref) {
  weight_bound_ref_ = weight_bound_ref;
  has_bound_ref_ = true;
}

void PieceWiseJerk::set_weight_kappa(const double weight_kappa, const double weight_kappa_epsilon) {
  weight_kappa_ = weight_kappa;
  weight_kappa_epsilon_ = weight_kappa_epsilon;
}

bool PieceWiseJerk::createQPDefaultSolver(const Eigen::Matrix<double, -1, -1>& H,
                         const Eigen::Matrix<double, -1, 1>& g,
                         const Eigen::Matrix<double, -1, -1>& A,
                         const Eigen::Matrix<double, -1, 1>& l,
                         const Eigen::Matrix<double, -1, 1>& u) {


  // Problem settings
  OSQPSettings *settings =
      reinterpret_cast<OSQPSettings *>(c_malloc(sizeof(OSQPSettings)));

  // Populate data
  OSQPData *data = reinterpret_cast<OSQPData *>(c_malloc(sizeof(OSQPData)));

  static std::vector<c_float> P_data;
  static std::vector<c_int> P_indices;
  static std::vector<c_int> P_indptr;
  P_data.clear();
  P_indices.clear();
  P_indptr.clear();
  planning_math::DenseToCSCMatrix(H, &P_data, &P_indices, &P_indptr);

  static std::vector<c_float> A_data;
  static std::vector<c_int> A_indices;
  static std::vector<c_int> A_indptr;
  A_data.clear();
  A_indices.clear();
  A_indptr.clear();
  planning_math::DenseToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

  data->n = H.rows();
  data->m = A.rows();
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  auto mutable_g = g;
  data->q = mutable_g.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  auto mutable_l = l;
  data->l = mutable_l.data();
  auto mutable_u = u;
  data->u = mutable_u.data();

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.0;       // Change alpha parameter
  settings->eps_abs = 1.0e-03; // 1.0e-5
  settings->eps_rel = 1.0e-03; // 1.0e-5
  // settings->max_iter = 3000 + add_iter; // 10000
  settings->max_iter = 500;
  settings->polish = true;
  // settings->verbose = FLAGS_enable_osqp_debug;  // zyl
  settings->verbose = false; // false: close osqp log  true: open osqp log
                             // /thirdparty/osqp/src/util.c

  // Setup workspace
  OSQPWorkspace *work = osqp_setup(data, settings);
  c_int const status = osqp_solve(work);
  // c_int const status = osqp_solve(m_workspace.get());
  if (status == 0) {
    //PERROR << "OSQP Solve Failed. Solve Status: " << status;
    c_float* solution = work->solution->x;
    Eigen::VectorXd optimal_variable = 
        Eigen::Map<Eigen::VectorXd>(solution, work->data->n, 1);
    x_.resize(num_of_knots_);
    dx_.resize(num_of_knots_);
    ddx_.resize(num_of_knots_);

    for (uint16_t i = 0; i < num_of_knots_; i++) {
      x_[i] = optimal_variable(3 * i);
      dx_[i] = optimal_variable(3 * i + 1);
      ddx_[i] = optimal_variable(3 * i + 2);
    }
  }

  // Cleanup
  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  c_free(data);
  c_free(settings);
  if (x_.size() > 0 ) {
    return true;
  }
  return false;
}

bool PieceWiseJerk::createQPAdaptorSolver(const Eigen::Matrix<double, -1, -1>& H,
                    const Eigen::Matrix<double, -1, 1>& g,
                    const Eigen::Matrix<double, -1, -1>& A,
                    const Eigen::Matrix<double, -1, 1>& l,
                    const Eigen::Matrix<double, -1, 1>& u) {
  OsqpWrapper osqp_wrapper(H, g, A, l, u, true);
  auto workspace = osqp_wrapper.getOSQPWorkspace(H, g, A, l, u);
  size_t max_iter_num = 500;
  osqp_wrapper.settings()->max_iter = max_iter_num;
  if (osqp_wrapper.solve(workspace)) {
    VectorXd optimal_variable = osqp_wrapper.getSolution();
    m_solution_ = optimal_variable;
    x_.resize(num_of_knots_);
    dx_.resize(num_of_knots_);
    ddx_.resize(num_of_knots_);

    for (uint16_t i = 0; i < num_of_knots_; i++) {
      x_[i] = optimal_variable(3 * i);
      dx_[i] = optimal_variable(3 * i + 1);
      ddx_[i] = optimal_variable(3 * i + 2);
    }
    return true;
  }
  return false;
}

bool PieceWiseJerk::Solve(const size_t max_iter_num) {
  const size_t num_variables = num_of_knots_ * 3;
  MatrixXd H = getMatrixH();
  Eigen::VectorXd g = getGradientVector();
  // 3n个变量的上下界，三阶导数约束(n-1)个,1阶导数连续(n-1)个,0阶连续(n-1)，初始状态约束
  size_t const num_of_constraints = 3 * num_of_knots_ + 3 * (num_of_knots_ - 1);
  Eigen::VectorXd lb, ub;
  Eigen::MatrixXd A = MatrixXd::Zero(num_of_constraints, num_variables);
  lb.resize(num_of_constraints);
  ub.resize(num_of_constraints);
  getGradientLbAndUbAndA(lb, ub, A);
  bool use_default_qp = true;
  bool result = false;
  if (use_default_qp) {
    result = createQPDefaultSolver(H, g, A, lb, ub);
  } else {
    result = createQPAdaptorSolver(H, g, A, lb, ub);
  }
  return result;
}

MatrixXd PieceWiseJerk::getMatrixH() {
  const size_t num_variables = num_of_knots_ * 3;
  MatrixXd H = MatrixXd::Zero(num_variables, num_variables);

  for (size_t i = 0; i < num_of_knots_; i++) {
    H(3 * i, 3 * i) = weight_x_ + weight_x_ref_vec_[i];
    H(3 * i + 1, 3 * i + 1) = weight_dx_ + weight_dx_ref_vec_[i];
    H(3 * i + 2, 3 * i + 2) = weight_ddx_ + weight_dddx_ / (delta_s_ * delta_s_);

    if (i > 0 && i < num_of_knots_ - 1) {
      H(3 * i + 2, 3 * i + 2) += weight_dddx_ / (delta_s_ * delta_s_);
    }

    if (i < num_of_knots_ - 1) {
      H(3 * i + 2, 3 * (i + 1) + 2) = -1.0 * weight_dddx_ / (delta_s_ * delta_s_);
      H(3 * (i + 1) + 2, 3 * i + 2) = -1.0 * weight_dddx_ / (delta_s_ * delta_s_);
    }
  }
  return 2 * H;
}
Eigen::VectorXd PieceWiseJerk::getGradientVector() {
  const size_t num_variables = num_of_knots_ * 3;
  Eigen::VectorXd g = MatrixXd::Zero(num_variables, 1);
  for (size_t i = 0; i < num_of_knots_; i++) {
    if (has_x_ref_) {
      g(3 * i) += -2 * weight_x_ref_vec_[i] * x_ref_[i];
    }
    if (has_dx_ref_) {
      g(3 * i + 1) += -2 * weight_dx_ref_vec_[i] * dx_ref_[i];
    }
  }
  return g;
}

void PieceWiseJerk::getGradientLbAndUbAndA(Eigen::VectorXd& lb, Eigen::VectorXd& ub,
                                           Eigen::MatrixXd& A) {
  size_t num_constraints = 0;
  for (size_t i = 0; i < num_of_knots_; i++) {
    // for  l
    lb(3 * i) = x_bounds_[i].first;
    ub(3 * i) = x_bounds_[i].second;
    A(3 * i, 3 * i) = 1;

    // for dl
    lb(3 * i + 1) = dx_bounds_[i].first;
    ub(3 * i + 1) = dx_bounds_[i].second;
    A(3 * i + 1, 3 * i + 1) = 1;

    // for ddl
    lb(3 * i + 2) = ddx_bounds_[i].first;
    ub(3 * i + 2) = ddx_bounds_[i].second;
    A(3 * i + 2, 3 * i + 2) = 1;
  }
  num_constraints = num_constraints + 3 * num_of_knots_;
  for (size_t i = 0; i + 1 < num_of_knots_; i++) {
    // for dddl
    lb(num_constraints + i) = dddx_bounds_.first * delta_s_;
    ub(num_constraints + i) = dddx_bounds_.second * delta_s_;
    A(num_constraints + i, 3 * i + 2) = -1;
    A(num_constraints + i, 3 * (i + 1) + 2) = 1;
  }
  num_constraints = num_constraints + (num_of_knots_ - 1);
  for (size_t i = 0; i + 1 < num_of_knots_; i++) {
    // one order continuous 
    lb(num_constraints + i) = -1e-10;
    ub(num_constraints + i) = 1e-10;
    A(num_constraints + i, 3 * i + 1) = -1;
    A(num_constraints + i, 3 * (i + 1) + 1) = 1;
    A(num_constraints + i, 3 * i + 2) = -0.5 * delta_s_;
    A(num_constraints + i, 3 * (i + 1) + 2) = -0.5 * delta_s_;
  }
  num_constraints = num_constraints + (num_of_knots_ - 1);

  for (size_t i = 0; i + 1 < num_of_knots_; i++) {
    // zero order continuous
    lb(num_constraints + i) = -1e-10;
    ub(num_constraints + i) = 1e-10;
    A(num_constraints + i, 3 * i) = -1;
    A(num_constraints + i, 3 * (i + 1)) = 1;
    A(num_constraints + i, 3 * i + 1) = -delta_s_;
    A(num_constraints + i, 3 * i + 2) = -delta_s_ * delta_s_ / 3.0;
    A(num_constraints + i, 3 * (i + 1) + 2) = -delta_s_ * delta_s_ / 6.0;
  }
  num_constraints = num_constraints + (num_of_knots_ - 1);
  // reset init state
  lb(0) = x_init_[0] - 1e-6;
  ub(0) = x_init_[0] + 1e-6;

  lb(1) = x_init_[1] - 1e-6;
  ub(1) = x_init_[1] + 1e-6;

  lb(2) = x_init_[2] - 1e-6;
  ub(2) = x_init_[2] + 1e-6;

  // lb(3 * num_of_knots_ - 3 + 1) = -0.1;
  // ub(3 * num_of_knots_ - 3 + 1) = 0.1;

}

void PieceWiseJerk::getInequalityConstraintOfXAndDdx(double* k, double* b, const double kappa_ref,
                                                     const double kappa_bound, const double x_min,
                                                     const double x_max) {
  if (std::abs(kappa_ref) < 0.001) {
    *k = 0;
    *b = kappa_bound;
  } else if (kappa_ref * kappa_bound > 0) { //同号
    double lpp_max =
        (kappa_bound - kappa_ref / (1 - kappa_ref * x_min)) * std::pow((1 - kappa_ref * x_min), 2);
    double lpp_min =
        (kappa_bound - kappa_ref / (1 - kappa_ref * x_max)) * std::pow((1 - kappa_ref * x_max), 2);
    *k = (lpp_max - lpp_min) / (x_min - x_max);
    *b = lpp_max - (*k) * x_min;
  } else {
    double lpp_max =
        (kappa_bound - kappa_ref / (1 - kappa_ref * x_max)) * std::pow((1 - kappa_ref * x_max), 2);
    double lpp_min =
        (kappa_bound - kappa_ref / (1 - kappa_ref * x_min)) * std::pow((1 - kappa_ref * x_min), 2);
    *k = (lpp_max - lpp_min) / (x_max - x_min);
    *b = lpp_max - (*k) * x_max;
  }
}

} // namespace parking
} // namespace msquare
