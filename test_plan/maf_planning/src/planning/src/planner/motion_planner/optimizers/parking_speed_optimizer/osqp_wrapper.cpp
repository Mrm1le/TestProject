
#include "planner/motion_planner/optimizers/parking_speed_optimizer/osqp_wrapper.h"

namespace msquare {
namespace parking {
DataWrapper::DataWrapper() {
  m_data = (OSQPData *)c_malloc(sizeof(OSQPData));
  m_data->P = nullptr;
  m_data->A = nullptr;
}

DataWrapper::DataWrapper(int32_t n, int32_t m) {
  m_data = (OSQPData *)c_malloc(sizeof(OSQPData));
  m_data->P = nullptr;
  m_data->A = nullptr;
  setNumOfVariables(n);
  setNumOfConstraints(m);
  m_isNumberOfVariablesSet = true;
  m_isNumberOfConstraintsSet = true;
}
DataWrapper::~DataWrapper() {
  clearHessianMatrix();
  clearConstraintsMatrix();
  c_free(m_data);
}
void DataWrapper::clearHessianMatrix() {
  if (m_isHessianMatrixSet) {
    csc_spfree(m_data->P);
    m_data->P = nullptr;
    m_isHessianMatrixSet = false;
  }
}
void DataWrapper::clearConstraintsMatrix() {
  if (m_isLinearConstraintsMatrixSet) {
    csc_spfree(m_data->A);
    m_data->A = nullptr;
    m_isLinearConstraintsMatrixSet = false;
  }
}
void DataWrapper::setNumOfVariables(int32_t n) {
  m_isNumberOfVariablesSet = true;
  m_data->n = n;
}
void DataWrapper::setNumOfConstraints(int32_t m) {
  m_isNumberOfConstraintsSet = true;
  m_data->m = m;
}
void DataWrapper::setGradient(
    Eigen::Matrix<c_float, Eigen::Dynamic, 1> &gradientVector) {
  if (gradientVector.rows() != m_data->n) {
    // PERROR << "The size of gradientVector is not equal to number of
    // variables.";
    return;
  }
  m_isGradientSet = true;
  m_data->q = gradientVector.data();
}

void DataWrapper::setLowerBound(
    Eigen::Matrix<c_float, Eigen::Dynamic, 1> &lowerVector) {
  if (lowerVector.rows() != m_data->m) {
    // PERROR << "The size of lowerVector is not equal to the number of
    // constraints.";
    return;
  }
  m_isLowerBoundSet = true;
  m_data->l = lowerVector.data();
}
void DataWrapper::setUpperBound(
    Eigen::Matrix<c_float, Eigen::Dynamic, 1> &upperVector) {
  if (upperVector.rows() != m_data->m) {
    // PERROR << "The size of lowerVector is not equal to the number of
    // constraints.";
    return;
  }
  m_isUpperBoundSet = true;
  m_data->u = upperVector.data();
}
bool DataWrapper::isSet() const {
  return m_isNumberOfVariablesSet && m_isLinearConstraintsMatrixSet &&
         m_isHessianMatrixSet && m_isGradientSet &&
         m_isLinearConstraintsMatrixSet && m_isLowerBoundSet &&
         m_isUpperBoundSet;
}
OSQPData *const &DataWrapper::dataPtr() const { return m_data; }

void OsqpWrapper::OsqpWorkSpaceDeleter(OSQPWorkspace *ptr) noexcept {
  if (ptr != nullptr)
    osqp_cleanup(ptr);
}

OsqpWrapper::OsqpWrapper()
    : m_workspace(nullptr, OsqpWrapper::OsqpWorkSpaceDeleter),
      m_is_inited(false) {
  m_settings =
      std::unique_ptr<OSQPSettings>(new OSQPSettings()); /* default settings */
  osqp_set_default_settings(m_settings.get());
  // m_settings->verbose = FLAGS_v >= 5;
  m_settings->verbose = false;
  m_data = std::unique_ptr<DataWrapper>(new DataWrapper()); /* data is empty */
}
bool OsqpWrapper::isInitialized() const { return m_is_inited; }
bool OsqpWrapper::clearVariables() {
  if (!m_is_inited) {
    // PERROR << "Unable to clear OSQP Wrapper. OSQP Wrapper Is Not
    // Initialized";
    return false;
  }

  for (int32_t i = 0; i < m_workspace->data->n; i++) {
    m_workspace->x[i] = 0;
    m_workspace->x_prev[i] = 0;
    m_workspace->Px[i] = 0;
    m_workspace->Aty[i] = 0;
    m_workspace->Atdelta_y[i] = 0;
    m_workspace->delta_x[i] = 0;
    m_workspace->Pdelta_x[i] = 0;
  }

  for (int32_t i = 0; i < m_workspace->data->m; i++) {
    m_workspace->z[i] = 0;
    m_workspace->z_prev[i] = 0;
    m_workspace->y[i] = 0;
    m_workspace->Ax[i] = 0;
    m_workspace->delta_y[i] = 0;
    m_workspace->Adelta_x[i] = 0;
  }
  for (int32_t i = 0; i < m_workspace->data->n + m_workspace->data->m; i++) {
    m_workspace->xz_tilde[i] = 0;
  }
  return true;
}
bool OsqpWrapper::init() {
  if (m_is_inited) {
    // PERROR << "OSQP Wrapper has been already initialized.";
    return false;
  }
  if (!m_data->isSet()) {
    // PERROR << "OSQP Data is Not Ready.";
    return false;
  }
  OSQPWorkspace *workspace;

  // if (osqp_setup(&workspace, m_data->dataPtr(), m_settings.get()) != 0) {
  //   //PERROR << "Unable to setup the workspace.";
  //   return false;
  // }
  // m_workspace.reset(workspace);
  m_is_inited = true;
  return true;
}

void OsqpWrapper::clearWorkSpace() {
  if (m_is_inited) {
    m_workspace.reset();
    m_is_inited = false;
  }
}
bool OsqpWrapper::solve() {
  if (!m_is_inited) {
    // PERROR << "OSQP Wrapper Not Initialized. Please call init() method.";
    return false;
  }
  c_int const status = osqp_solve(m_workspace.get());
  if (status != 0) {
    // PERROR << "OSQP Solve Failed. Solve Status: " << status;
    return false;
  }
  // PDEBUG(Optimizer) << "osqp status: " << status;
  return true;
}

bool OsqpWrapper::solve(OSQPWorkspace *workspace) {
  if (!m_is_inited) {
    // PERROR << "OSQP Wrapper Not Initialized. Please call init() method.";
    return false;
  }
  c_int const status = osqp_solve(workspace);
  if (status != 0) {
    // PERROR << "OSQP Solve Failed. Solve Status: " << status;
    return false;
  }
  // PDEBUG(Optimizer) << "osqp status: " << status;
  return true;
}

const Eigen::VectorXd &OsqpWrapper::getSolution() {
  c_float *solution = m_workspace->solution->x;
  m_solution = Eigen::Map<Eigen::VectorXd>(solution, m_workspace->data->n, 1);
  return m_solution;
}

const std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>> &
OsqpWrapper::workspace() const {
  return m_workspace;
}
const std::unique_ptr<DataWrapper> &OsqpWrapper::data() const { return m_data; }
const std::unique_ptr<OSQPSettings> &OsqpWrapper::settings() const {
  return m_settings;
}

void OsqpWrapper::updateGradient(
    const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
        &gradient) {
  if (gradient.rows() != m_workspace->data->n) {
    // PERROR << "Input gradient is not correct.";
    return;
  }
  osqp_update_lin_cost(m_workspace.get(), gradient.data());
}
void OsqpWrapper::updateLowerBound(
    const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
        &lowerBound) {
  if (lowerBound.rows() != m_workspace->data->m) {
    // PERROR << "Input lowerBound size unvalid";
    return;
  }
  osqp_update_lower_bound(m_workspace.get(), lowerBound.data());
}

void OsqpWrapper::updateUpperBound(
    const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
        &upperBound) {
  if (upperBound.rows() != m_workspace->data->m) {
    // PERROR << "Input UpperBound size unvalid";
    return;
  }
  osqp_update_upper_bound(m_workspace.get(), upperBound.data());
}
void OsqpWrapper::updateBounds(
    const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
        &lowerBound,
    const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
        &upperBound) {
  if (lowerBound.rows() != m_workspace->data->m) {
    // PERROR << "Input lowerBound size unvalid";
    return;
  }
  if (upperBound.rows() != m_workspace->data->m) {
    // PERROR << "Input upperBound size unvalid";
    return;
  }
  osqp_update_bounds(m_workspace.get(), lowerBound.data(), upperBound.data());
}

void DataWrapper::setHessianMatrix(
    const Eigen::Matrix<double, -1, -1> &hessianMatrix) {
  if (!m_isNumberOfVariablesSet) {
    // PERROR << "Number Of Variables Not Set.";
    return;
  }
  if (hessianMatrix.rows() != dataPtr()->n ||
      hessianMatrix.cols() != dataPtr()->n) {
    // PERROR << "Input hessianMatrix Size Not UnValid.";
  }
  Eigen::SparseMatrix<double> const sparse_hessian = hessianMatrix.sparseView();
  setHessianSparseMatrix(sparse_hessian);
}
void DataWrapper::setConstraintsMatrix(
    const Eigen::Matrix<double, -1, -1> &constraintsMatrix) {
  if (!m_isNumberOfConstraintsSet) {
    // PERROR << "Number Of Constraints Not Set.";
    return;
  }
  if (constraintsMatrix.rows() != dataPtr()->m ||
      constraintsMatrix.cols() != dataPtr()->n) {
    // PERROR << "Input constraintsMatrix Size UnValid.";
    return;
  }
  Eigen::SparseMatrix<double> const sparse_constraints =
      constraintsMatrix.sparseView();
  setConstraintsSparseMatrix(sparse_constraints);
}

bool OsqpWrapper::isValidHessian(const Eigen::MatrixXd &H,
                                 const Eigen::VectorXd &g) {
  if (H.rows() < 1 || H.cols() != H.rows()) {
    // PERROR << "Input H Size UnValid!";
    return false;
  }
  if (g.rows() != H.rows()) {
    // PERROR << "Input g Size Unvalid!";
    return false;
  }
  return true;
}

bool OsqpWrapper::isValidConstraints(const Eigen::Index &n,
                                     const Eigen::MatrixXd &A,
                                     const Eigen::VectorXd &l,
                                     const Eigen::VectorXd &u) {
  if (A.rows() < Eigen::Index(1) || A.cols() != n) {
    // PERROR << "Input A Size UnValid!";
    return false;
  }
  if (l.rows() != A.rows()) {
    // PERROR << "Input l Size Unvalid!";
    return false;
  }
  if (u.rows() != A.rows()) {
    // PERROR << "Input u Size Unvalid!";
    return false;
  }
  return true;
}

bool OsqpWrapper::isValidSetup(const Eigen::MatrixXd &H,
                               const Eigen::VectorXd &g,
                               const Eigen::MatrixXd &A,
                               const Eigen::VectorXd &l,
                               const Eigen::VectorXd &u) {
  return isValidHessian(H, g) && isValidConstraints(H.rows(), A, l, u);
}

OsqpWrapper::OsqpWrapper(const Eigen::Matrix<double, -1, -1> &H,
                         const Eigen::Matrix<double, -1, 1> &g,
                         const Eigen::Matrix<double, -1, -1> &A,
                         const Eigen::Matrix<double, -1, 1> &l,
                         const Eigen::Matrix<double, -1, 1> &u) {
  if (!isValidSetup(H, g, A, l, u)) {
    return;
  }
  // TODO: 检查Ｈ是否正定,所有特征值均为正。
  m_workspace =
      std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>>(
          nullptr, OsqpWorkSpaceDeleter);
  m_settings = std::unique_ptr<OSQPSettings>(new OSQPSettings());
  osqp_set_default_settings(m_settings.get());
  // m_settings->verbose = FLAGS_v >= 5;
  m_settings->verbose = false;
  m_data = std::unique_ptr<DataWrapper>(new DataWrapper());
  m_data->setNumOfVariables(H.rows());
  m_data->setNumOfConstraints(A.rows());
  m_data->setHessianMatrix(H);
  auto mutable_g = g;
  m_data->setGradient(mutable_g);
  m_data->setConstraintsMatrix(A);
  auto mutable_l = l;
  m_data->setLowerBound(mutable_l);
  auto mutalbe_u = u;
  m_data->setUpperBound(mutalbe_u);
  OSQPWorkspace *workspace;
  m_is_inited = true;
}

OsqpWrapper::OsqpWrapper(const Eigen::Matrix<double, -1, -1> &H,
                         const Eigen::Matrix<double, -1, 1> &g,
                         const Eigen::Matrix<double, -1, -1> &A,
                         const Eigen::Matrix<double, -1, 1> &l,
                         const Eigen::Matrix<double, -1, 1> &u,
                         bool use_default_qp) {
  if (!isValidSetup(H, g, A, l, u)) {
    return;
  }
  // TODO: 检查Ｈ是否正定,所有特征值均为正。
}

OSQPWorkspace *
OsqpWrapper::getOSQPWorkspace(const Eigen::Matrix<double, -1, -1> &H,
                              const Eigen::Matrix<double, -1, 1> &g,
                              const Eigen::Matrix<double, -1, -1> &A,
                              const Eigen::Matrix<double, -1, 1> &l,
                              const Eigen::Matrix<double, -1, 1> &u) {
  m_workspace =
      std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>>(
          nullptr, OsqpWorkSpaceDeleter);
  m_settings = std::unique_ptr<OSQPSettings>(new OSQPSettings());
  osqp_set_default_settings(m_settings.get());
  // m_settings->verbose = FLAGS_v >= 5;
  m_settings->verbose = false;
  m_data = std::unique_ptr<DataWrapper>(new DataWrapper());
  m_data->setNumOfVariables(H.rows());
  m_data->setNumOfConstraints(A.rows());
  m_data->setHessianMatrix(H);
  auto mutable_g = g;
  m_data->setGradient(mutable_g);
  m_data->setConstraintsMatrix(A);
  auto mutable_l = l;
  m_data->setLowerBound(mutable_l);
  auto mutalbe_u = u;
  m_data->setUpperBound(mutalbe_u);
  OSQPWorkspace *workspace;

  // Setup workspace
  OSQPWorkspace *work = osqp_setup(m_data->dataPtr(), m_settings.get());
  m_workspace.reset(work);
  m_is_inited = true;
  return work;
}

} // namespace parking
} // namespace msquare
