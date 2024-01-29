#ifndef OSQP_WRAPPER_H
#define OSQP_WRAPPER_H

#include <iostream>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "osqp/osqp.h"

namespace msquare {
namespace parking {

constexpr double kNegtiveZero = -1e-10;
constexpr double kPositiveZero = 1e-10;

class DataWrapper {
public:
  DataWrapper();
  DataWrapper(int32_t n, int32_t m);
  ~DataWrapper();
  void clearHessianMatrix();
  void clearConstraintsMatrix();
  void setNumOfVariables(int32_t n);
  void setNumOfConstraints(int32_t m);

  void setHessianMatrix(const Eigen::Matrix<double, -1, -1> &hessianMatrix);
  void
  setConstraintsMatrix(const Eigen::Matrix<double, -1, -1> &constraintsMatrix);

  template <typename Derived>
  void setHessianSparseMatrix(
      const Eigen::SparseCompressedBase<Derived> &hessianMatrix);

  // template <typename Derived>
  // void setHessianMatrix(const Eigen::SparseCompressedBase<Derived>&
  // hessianMatrix);

  template <typename Derived>
  void setConstraintsSparseMatrix(
      const Eigen::SparseCompressedBase<Derived> &constraintsMatrix);

  void setGradient(Eigen::Matrix<c_float, Eigen::Dynamic, 1> &gradientVector);
  void setLowerBound(Eigen::Matrix<c_float, Eigen::Dynamic, 1> &lowerVector);
  void setUpperBound(Eigen::Matrix<c_float, Eigen::Dynamic, 1> &upperVector);
  OSQPData *const &dataPtr() const;
  bool isSet() const;

private:
  OSQPData *m_data{nullptr};
  bool m_isNumberOfVariablesSet{false};       // 要优化的变量个数
  bool m_isNumberOfConstraintsSet{false};     // 约束个数　
  bool m_isHessianMatrixSet{false};           // Hessian矩阵
  bool m_isGradientSet{false};                // Gradient矩阵
  bool m_isLinearConstraintsMatrixSet{false}; // 线性约束矩阵　
  bool m_isLowerBoundSet{false};              // 约束下界
  bool m_isUpperBoundSet{false};              // 约束上界
};

class OsqpWrapper {
public:
  OsqpWrapper();
  OsqpWrapper(const Eigen::Matrix<double, -1, -1> &H,
              const Eigen::Matrix<double, -1, 1> &g,
              const Eigen::Matrix<double, -1, -1> &A,
              const Eigen::Matrix<double, -1, 1> &l,
              const Eigen::Matrix<double, -1, 1> &u);
  OsqpWrapper(const Eigen::Matrix<double, -1, -1> &H,
              const Eigen::Matrix<double, -1, 1> &g,
              const Eigen::Matrix<double, -1, -1> &A,
              const Eigen::Matrix<double, -1, 1> &l,
              const Eigen::Matrix<double, -1, 1> &u, bool use_default_qp);

public: // 对外接口函数
  bool init();
  bool isInitialized() const;
  bool clearVariables();
  void clearWorkSpace();
  bool solve();
  bool solve(OSQPWorkspace *workspace);
  const Eigen::VectorXd &getSolution();

  // update相关函数仅用于求解器的workspace已经初始化完成情况下的更新。
  void updateGradient(
      const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
          &gradient);
  void updateLowerBound(
      const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
          &lowerBound);
  void updateUpperBound(
      const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
          &upperBound);
  void
  updateBounds(const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
                   &lowerBound,
               const Eigen::Ref<const Eigen::Matrix<c_float, Eigen::Dynamic, 1>>
                   &upperBound);

  template <typename T>
  bool updateHessianMatrix(const Eigen::SparseCompressedBase<T> &hessianMatrix);
  template <typename T>
  bool updateLinearConstraintsMatrix(
      const Eigen::SparseCompressedBase<T> &linearConstraintsMatrix);

  template <typename T, int32_t n, int32_t m>
  bool setWarmStart(const Eigen::Matrix<T, n, 1> &primalVariable,
                    const Eigen::Matrix<T, m, 1> &dualVariable);
  OSQPWorkspace *getOSQPWorkspace(const Eigen::Matrix<double, -1, -1> &H,
                                  const Eigen::Matrix<double, -1, 1> &g,
                                  const Eigen::Matrix<double, -1, -1> &A,
                                  const Eigen::Matrix<double, -1, 1> &l,
                                  const Eigen::Matrix<double, -1, 1> &u);

public: // 成员变量获取函数
  const std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>> &
  workspace() const;
  const std::unique_ptr<DataWrapper> &data() const;
  const std::unique_ptr<OSQPSettings> &settings() const;

public: // 静态工具函数
  static void OsqpWorkSpaceDeleter(OSQPWorkspace *ptr) noexcept;
  template <typename T>
  static bool createOsqpSparseMatrix(
      const Eigen::SparseCompressedBase<T> &,
      csc *&osqpSparseMatrix); // 将eigen的稀疏矩阵转为osqp的csc格式的稀疏矩阵。

private:
  bool isValidHessian(const Eigen::MatrixXd &H, const Eigen::VectorXd &g);
  bool isValidConstraints(const Eigen::Index &n, const Eigen::MatrixXd &A,
                          const Eigen::VectorXd &l, const Eigen::VectorXd &u);
  bool isValidSetup(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                    const Eigen::MatrixXd &A, const Eigen::VectorXd &l,
                    const Eigen::VectorXd &u);

private: // 成员变量　
  std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>>
      m_workspace; // 自定义删除器
  std::unique_ptr<OSQPSettings> m_settings;
  std::unique_ptr<DataWrapper> m_data;
  Eigen::VectorXd m_solution;
  bool m_is_inited;
};

// template function impl
template <typename Derived>
void DataWrapper::setHessianSparseMatrix(
    const Eigen::SparseCompressedBase<Derived> &hessianMatrix) {
  if (!m_isNumberOfVariablesSet) {
    // //PERROR << "Number of variable not set.";
    return;
  }
  if (m_isHessianMatrixSet) {
    // PWARN << "Hessian Matrix is already set.";
    return;
  }
  if (hessianMatrix.rows() != m_data->n || hessianMatrix.cols() != m_data->n) {
    // //PERROR << "The ThHessian Matrix has to be n x n size.";
    return;
  }
  // osqp 0.6.0 只要求指定Hessian的上三角部分。　
  // 注意此处的template关键字,如果没有的话会被解释为成员变量比较大小,参见C++调用模板类型对象的模板函数
  Derived hessianMatrixUpperTriangular =
      hessianMatrix.template triangularView<Eigen::Upper>();
  // 上三角矩阵转换为OSQP的csc压缩矩阵。
  OsqpWrapper::createOsqpSparseMatrix(hessianMatrixUpperTriangular, m_data->P);
  m_isHessianMatrixSet = true;
}

template <typename Derived>
void DataWrapper::setConstraintsSparseMatrix(
    const Eigen::SparseCompressedBase<Derived> &constraintsMatrix) {
  if (!m_isNumberOfConstraintsSet) {
    // //PERROR << "The number of constraints is not set.";
    return;
  }
  if (!m_isNumberOfVariablesSet) {
    // //PERROR << "The number of variables is not set.";
    return;
  }
  if (m_isLinearConstraintsMatrixSet) {
    // PWARN << "The Constraints Matrix is Already Set.";
    return;
  }
  if (constraintsMatrix.rows() != m_data->m ||
      constraintsMatrix.cols() != m_data->n) {
    // //PERROR << "The Constraints Matrix Size Is Not Correct.";
    return;
  }
  OsqpWrapper::createOsqpSparseMatrix(constraintsMatrix, m_data->A);
  m_isLinearConstraintsMatrixSet = true;
}

template <typename T>
bool OsqpWrapper::createOsqpSparseMatrix(
    const Eigen::SparseCompressedBase<T> &eigenSparseMatrix,
    csc *&osqpSparseMatrix) {
  // 此处typename告诉编译器value_type　为一个类型,采用列优先存储
  Eigen::SparseMatrix<typename T::value_type, Eigen::ColMajor> colMajorCopy =
      eigenSparseMatrix;

  c_int const rows = colMajorCopy.rows();
  c_int const cols = colMajorCopy.cols();
  c_int const numberOfNonZeroCoeff = colMajorCopy.nonZeros(); // 非零元素个数　

  const int32_t *const outerIndexPtr =
      colMajorCopy.outerIndexPtr(); // 每一列非０元素个数

  const int32_t *const innerNonZerosPtr =
      colMajorCopy.innerNonZeroPtr(); // 每一列中非０元素所在的行号。

  if (osqpSparseMatrix != nullptr) {
    // //PERROR << "osqpSparseMatrix pointer is not a null pointer! ";
    return false;
  }

  osqpSparseMatrix = csc_spalloc(rows, cols, numberOfNonZeroCoeff, 1, 0);

  int32_t innerOsqpPosition = 0;
  for (int32_t k = 0; k < cols; k++) {
    if (colMajorCopy.isCompressed()) {
      osqpSparseMatrix->p[k] = static_cast<c_int>(outerIndexPtr[k]);
    } else {
      if (k == 0) {
        osqpSparseMatrix->p[k] = 0;
      } else {
        osqpSparseMatrix->p[k] =
            osqpSparseMatrix->p[k - 1] + innerNonZerosPtr[k - 1];
      }
    }
    for (typename Eigen::SparseMatrix<typename T::value_type,
                                      Eigen::ColMajor>::InnerIterator
             it(colMajorCopy, k);
         it; ++it) {
      osqpSparseMatrix->i[innerOsqpPosition] = static_cast<c_int>(it.row());
      osqpSparseMatrix->x[innerOsqpPosition] = static_cast<c_float>(it.value());
      innerOsqpPosition++;
    }
  }
  osqpSparseMatrix->p[static_cast<int32_t>(cols)] =
      static_cast<c_int>(innerOsqpPosition);
  assert(innerOsqpPosition == numberOfNonZeroCoeff);
  return true;
}

template <typename T>
bool OsqpWrapper::updateHessianMatrix(
    const Eigen::SparseCompressedBase<T> &hessianMatrix) {
  if (!m_is_inited) {
    // //PERROR << "The OSQP Wrapper is  not initialized.";
    return false;
  }
  if (static_cast<c_int>(hessianMatrix.rows()) != m_workspace->data->n ||
      static_cast<c_int>(hessianMatrix.cols()) != m_workspace->data->n) {
    // //PERROR << "hessianMatrix Size unvalid!";
    return false;
  }
  // TODO: 只有在hessian
  // Matrix的pattern不改变的情况下才不需要重新初始化求解器，可以通计算比较新旧hessian的pattern,确定是否要进行重新初始化。
  // 　此处目前简单处理，所有情况都进行重新初始化(可能在更新hessian的情况下会损失部分性能)　
  m_data->clearHessianMatrix();
  m_data->setHessianSparseMatrix(hessianMatrix);
  clearWorkSpace();
  init();
  return true;
}
template <typename T>
bool OsqpWrapper::updateLinearConstraintsMatrix(
    const Eigen::SparseCompressedBase<T> &linearConstraintsMatrix) {
  if (!m_is_inited) {
    // //PERROR << "OSQP Wrapper Is Not Initialized.";
    return false;
  }
  if (static_cast<c_int>(linearConstraintsMatrix.rows()) !=
          m_workspace->data->m ||
      static_cast<c_int>(linearConstraintsMatrix.cols()) !=
          m_workspace->data->n) {
    // PWARN << "Input updateLinearConstraintsMatrix Size not valid!";
    return false;
  }
  // TODO：　此处同updateHessianMatrix，也需要判断新旧约束矩阵的pattern
  m_data->clearConstraintsMatrix();
  m_data->setConstraintsSparseMatrix(linearConstraintsMatrix);
  clearWorkSpace();
  init();
  return true;
}

template <typename T, int32_t n, int32_t m>
bool OsqpWrapper::setWarmStart(const Eigen::Matrix<T, n, 1> &primalVariable,
                               const Eigen::Matrix<T, m, 1> &dualVariable) {
  if (primalVariable.rows() != m_workspace->data->n) {
    // //PERROR << "The size  of primalVariable not valid!";
    return false;
  }
  if (dualVariable.rows() != m_workspace->data->m) {
    // //PERROR << "The size of dualVariable not valid!";
    return false;
  }
  osqp_warm_start(m_workspace.get(), primalVariable.data(),
                  dualVariable.data());
  return true;
}
} // namespace parking
} // namespace msquare

#endif