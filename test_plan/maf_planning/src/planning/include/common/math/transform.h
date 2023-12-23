#pragma once
#include "mph_assert.h"
#include <Eigen/Dense>
#include <cmath>
namespace msquare {
class Transform {
  Eigen::Matrix3d m_basis;
  Eigen::Vector3d m_origin;

public:
  Transform() = default;
  explicit inline Transform(Eigen::Matrix3d &m, Eigen::Vector3d &v)
      : m_basis(m), m_origin(v) {}

  explicit inline Transform(const Eigen::Vector4d &q,
                            const Eigen::Vector3d &v) {
    setRotation(q);
    setOrigin(v);
  }

  explicit inline Transform(double roll, double pitch, double yaw,
                            const Eigen::Vector3d &v) {
    setRPY(roll, pitch, yaw);
    setOrigin(v);
  }

  inline Transform(const Transform &other)
      : m_basis(other.m_basis), m_origin(other.m_origin) {}

  void setMatrix(const double &xx, const double &xy, const double &xz,
                 const double &yx, const double &yy, const double &yz,
                 const double &zx, const double &zy, const double &zz) {
    setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
  }

  void setOrigin(const Eigen::Vector3d &origin) { m_origin = origin; }

  void setValue(const double &xx, const double &xy, const double &xz,
                const double &yx, const double &yy, const double &yz,
                const double &zx, const double &zy, const double &zz) {
    m_basis << xx, xy, xz, yx, yy, yz, zx, zy, zz;
  }

  /** @brief Set the matrix from a quaternion
   *    *  @param q The Quaternion to match */
  void setRotation(const Eigen::Vector4d &q) {
    double d = q.dot(q);
    constexpr double kEpsilon = 1.0e-10;
    d = d >= 0.0 ? std::max(d, kEpsilon) : std::min(d, -kEpsilon);
    double s = double(2.0) / d;
    double xs = q.x() * s, ys = q.y() * s, zs = q.z() * s;
    double wx = q.w() * xs, wy = q.w() * ys, wz = q.w() * zs;
    double xx = q.x() * xs, xy = q.x() * ys, xz = q.x() * zs;
    double yy = q.y() * ys, yz = q.y() * zs, zz = q.z() * zs;
    setValue(double(1.0) - (yy + zz), xy - wz, xz + wy, xy + wz,
             double(1.0) - (xx + zz), yz - wx, xz - wy, yz + wx,
             double(1.0) - (xx + yy));
  }

  /** @brief Set the matrix from euler angles YPR around ZYX axes
   *    * @param eulerZ Yaw aboud Z axis
   *        * @param eulerY Pitch around Y axis
   *            * @param eulerX Roll about X axis
   *                *
   *                    * These angles are used to produce a rotation matrix.
   * The euler
   *                        * angles are applied in ZYX order. I.e a vector is
   * first rotated
   *                            * about X then Y and then Z
   *                                **/
  void setEulerYPR(double eulerZ, double eulerY, double eulerX) {
    double ci(cos(eulerX));
    double cj(cos(eulerY));
    double ch(cos(eulerZ));
    double si(sin(eulerX));
    double sj(sin(eulerY));
    double sh(sin(eulerZ));
    double cc = ci * ch;
    double cs = ci * sh;
    double sc = si * ch;
    double ss = si * sh;

    setValue(cj * ch, sj * sc - cs, sj * cc + ss, cj * sh, sj * ss + cc,
             sj * cs - sc, -sj, cj * si, cj * ci);
  }

  void setRPY(double roll, double pitch, double yaw) {
    setEulerYPR(yaw, pitch, roll);
  }

  void getMatrix(Eigen::Matrix3d &m) { m = m_basis; }

  inline Eigen::Vector3d operator()(const Eigen::Vector3d &x) const {
    return m_basis * x + m_origin;
  }

  inline Eigen::Vector3d operator*(const Eigen::Vector3d &x) const {
    return (*this)(x);
  }

  Transform inverse() const {
    Eigen::Matrix3d inv = m_basis.transpose();
    Eigen::Vector3d pos = inv * (-m_origin);
    return Transform(inv, pos);
  }
};

} // namespace msquare
