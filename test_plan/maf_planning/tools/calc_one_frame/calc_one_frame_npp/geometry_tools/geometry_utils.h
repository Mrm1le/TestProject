#pragma once
#include "maf_interface.h"
#include <Eigen/Eigen>
#include <cmath>
#include <maf_interface/maf_mla_localization.h>
#include <math.h>

#include "common.h"
#include "maf_interface.h"

namespace msquare {
namespace geometry {
const double a = 6378137.0;
const double f = 1 / 298.257223563;
const double b = a * (1 - f);
const double e1_2 = f + f * b / a;
const double RAD_TO_DEG = 57.29577951308232;
const double DEG_TO_RAD = 0.017453292519943295;
const double _c = a * a / b;
const double _e1 = sqrt(a * a - b * b) / a;
const double _e2 = sqrt(a * a - b * b) / b;
// double W(const double B_) { return sqrt(1 - pow(_e1 * sin(B_), 2)); }
inline double V(const double B_) { return sqrt(1 + pow(_e2 * cos(B_), 2)); }
// double M(const double B_) { return _c / pow(V(B_), 3); } // 子午曲率半径
inline double N(const double B_) { return _c / V(B_); } // 卯酉曲率半径

struct RotMatrix {
  double r00;
  double r01;
  double r02;
  double r10;
  double r11;
  double r12;
  double r20;
  double r21;
  double r22;
};
struct Quaternion {
  Quaternion() {}
  Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {}
  inline RotMatrix toRotationMatrix() const {
    RotMatrix rot;
    double tx = 2 * x;
    double ty = 2 * y;
    double tz = 2 * z;
    double twx = tx * w;
    double twy = ty * w;
    double twz = tz * w;
    double txx = tx * x;
    double txy = ty * x;
    double txz = tz * x;
    double tyy = ty * y;
    double tyz = tz * y;
    double tzz = tz * z;
    rot.r00 = 1 - tyy - tzz;
    rot.r01 = txy - twz;
    rot.r02 = txz + twy;
    rot.r10 = txy + twz;
    rot.r11 = 1 - txx - tzz;
    rot.r12 = tyz - twx;
    rot.r20 = txz - twy;
    rot.r21 = tyz + twx;
    rot.r22 = 1 - txx - tyy;
    return rot;
  }
  double x;
  double y;
  double z;
  double w;
};

template <typename T> struct Point3X {
  Point3X() {}
  Point3X(T x, T y) : x(x), y(y), z(0.0) {}
  Point3X(T x, T y, T z) : x(x), y(y), z(z) {}
  Point3X(const Point3X &p) { x = p.x, y = p.y, z = p.z; }
  T x;
  T y;
  T z;
  friend const bool operator!=(const Point3X &p1, const Point3X &p2) {
    return !(p1 == p2);
  }
  friend const bool operator==(const Point3X &p1, const Point3X &p2) {
    const float EPS = 1e-7;
    return fabs(p1.x - p2.x) < EPS && fabs(p1.y - p2.y) < EPS &&
           fabs(p1.z - p2.z) < EPS;
  }
};

typedef Point3X<double> Point;

inline Point ENUPoint2Ego(const Point &pt, const Point &pt_base,
                          const Quaternion &q) {
  RotMatrix R = q.toRotationMatrix(); // no transpose
  double x = pt.x - pt_base.x;
  double y = pt.y - pt_base.y;
  double z = pt.z - pt_base.z;
  // tranpose
  return Point(x * R.r00 + y * R.r10 + z * R.r20,
               x * R.r01 + y * R.r11 + z * R.r21,
               x * R.r02 + y * R.r12 + z * R.r22);
};

inline Eigen::Matrix3d
car_to_enu_transform2(const maf_mla_localization::MLALocalization &navi,
                      float &yaw) {
  Eigen::Matrix3d matrix;

  Eigen::Quaterniond quaternion(
      navi.orientation.quaternion_local.w, navi.orientation.quaternion_local.x,
      navi.orientation.quaternion_local.y, navi.orientation.quaternion_local.z);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix = quaternion.matrix();

  yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
  matrix(0, 0) = std::cos(-yaw);
  matrix(0, 1) = -std::sin(-yaw);
  matrix(0, 2) = navi.position.position_local.x;

  matrix(1, 0) = std::sin(-yaw);
  matrix(1, 1) = std::cos(-yaw);
  matrix(1, 2) = navi.position.position_local.y;

  matrix(2, 0) = 0.f;
  matrix(2, 1) = 0.f;
  matrix(2, 2) = 1.f;

  return matrix;
};

inline Eigen::Vector2d
transform_car2enu(const Eigen::Vector2d &src,
                  const Eigen::Matrix3d &transform_matrix) {
  auto position_translation =
      Eigen::Vector2d(transform_matrix(0, 2), transform_matrix(1, 2));
  Eigen::Vector2d dest;
  dest.x() =
      transform_matrix(0, 0) * src.x() + transform_matrix(1, 0) * src.y();
  dest.y() =
      transform_matrix(0, 1) * src.x() + transform_matrix(1, 1) * src.y();
  return dest + position_translation;
};

inline Eigen::Vector2d
transform_enu2car(const Eigen::Vector2d &src,
                  const Eigen::Matrix3d &transform_matrix) {
  // src是enu系的坐标
  // transform matrix是car_to_enu的
  auto enu2car_matrix = transform_matrix.inverse();
  auto position_translation =
      Eigen::Vector2d(transform_matrix(0, 2), transform_matrix(1, 2));
  Eigen::Vector2d src_base = src - position_translation;
  Eigen::Vector2d dest;
  dest.x() =
      enu2car_matrix(0, 0) * src_base.x() + enu2car_matrix(1, 0) * src_base.y();
  dest.y() =
      enu2car_matrix(0, 1) * src_base.x() + enu2car_matrix(1, 1) * src_base.y();
  return dest;
};

inline void LLH2ECEF(const double *pos, double *xyz, bool is_deg) {
  double lat = pos[1] * (is_deg ? 0.017453292519943295 : 1),
         lon = pos[0] * (is_deg ? 0.017453292519943295 : 1), h = pos[2];
  double n = N(lat), clat = cos(lat);
  xyz[0] = (n + h) * clat * cos(lon);
  xyz[1] = (n + h) * clat * sin(lon);
  xyz[2] = (n * (1 - _e1 * _e1) + h) * sin(lat);
};

inline Eigen::Vector3d LLH2ECEF(const Eigen::Vector3d &pos, bool is_deg) {
  Eigen::Vector3d xyz = {0, 0, 0};
  LLH2ECEF(pos.data(), xyz.data(), is_deg);
  return xyz;
};

inline Eigen::Matrix3d Pos2Cne(const Eigen::Vector3d &pos, bool is_deg) {
  double lon = pos[0];
  double lat = pos[1];
  if (is_deg) {
    lon *= 0.017453292519943295;
    lat *= 0.017453292519943295;
  }
  double s1 = sin(M_PI / 4.0 + lon / 2.0);
  double c1 = cos(M_PI / 4.0 + lon / 2.0);
  double s2 = sin(M_PI / 4.0 - lat / 2.0);
  double c2 = cos(M_PI / 4.0 - lat / 2.0);
  return Eigen::Quaterniond{c1 * c2, -c1 * s2, -s1 * s2, -s1 * c2}
      .toRotationMatrix();
};

inline void ECEF2LLH(const Eigen::Vector3d &xyz, Eigen::Vector3d *pos,
                     bool to_deg) {
  const double e1_2 = _e1 * _e1, r2 = xyz.head(2).squaredNorm();
  double v = a, z = xyz[2], sinp = 0;
  for (double zk = 0; fabs(z - zk) >= 1e-4;) {
    zk = z;
    sinp = z / sqrt(r2 + z * z);
    v = a / sqrt(1 - e1_2 * sinp * sinp);
    z = xyz[2] + v * e1_2 * sinp;
  }
  (*pos)[0] = r2 > 1E-12 ? atan2(xyz[1], xyz[0]) : 0.0;
  (*pos)[1] = r2 > 1E-12 ? atan(z / sqrt(r2))
                         : (xyz[2] > 0.0 ? M_PI / 2.0 : -M_PI / 2.0);
  (*pos)[2] = sqrt(r2 + z * z) - v;
  if (to_deg) {
    pos->head(2) /= DEG_TO_RAD;
  }
};

inline Eigen::Vector3d ECEF2LLH(const Eigen::Vector3d &xyz, bool to_deg) {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  ECEF2LLH(xyz, &pos, to_deg);
  return pos;
};

inline NormalPoint3D boot_2_global(const NormalPoint3D &boot_point,
                                   const NormalPoint4D &q,
                                   const NormalPoint3D &t,
                                   const PosGlobal &center) {
  // ltrans.tranformInvers
  Eigen::Vector3d pVec{boot_point.x, boot_point.y, boot_point.z};
  Eigen::Quaterniond quater{q.w, q.x, q.y, q.z};
  Eigen::Vector3d translation{t.x, t.y, t.z};
  Eigen::Vector3d pRes = quater.inverse() * (pVec - translation);
  // r2g
  Eigen::Vector3d rPVec{pRes[0], pRes[1], pRes[2]};
  Eigen::Vector3d originLLH2ECEF_ =
      LLH2ECEF({center.longitude, center.latitude, center.altitude}, true);
  Eigen::Matrix3d originPos2Cne_ =
      Pos2Cne({center.longitude, center.latitude, center.altitude}, true);
  Eigen::Vector3d vecRes =
      ECEF2LLH(originLLH2ECEF_ + originPos2Cne_.transpose() * rPVec, true);
  return {vecRes[0], vecRes[1], vecRes[2]};
};

inline maf_mla_localization::MLAEuler
Quaternion2Euler(maf_mla_localization::MLAQuaternion &q) {
  maf_mla_localization::MLAEuler angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
    angles.pitch =
        std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    angles.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

inline maf_mla_localization::MLAQuaternion
Euler2Quaternion(maf_mla_localization::MLAEuler &euler) {
  double cy = cos(euler.yaw * 0.5);
  double sy = sin(euler.yaw * 0.5);
  double cp = cos(0 * 0.5);
  double sp = sin(0 * 0.5);
  double cr = cos(0 * 0.5);
  double sr = sin(0 * 0.5);

  maf_mla_localization::MLAQuaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

inline Eigen::Matrix3d
eulerAnglesToRotationMatrix(const maf_mla_localization::MLAEuler &euler) {
  Eigen::Matrix3d R_x;
  R_x << 1, 0, 0, 0, cos(0), -sin(0), 0, sin(0), cos(0);
  Eigen::Matrix3d R_y;
  R_y << cos(0), 0, sin(0), 0, 1, 0, -sin(0), 0, cos(0);
  Eigen::Matrix3d R_z;
  R_z << cos(euler.yaw), -sin(euler.yaw), 0, sin(euler.yaw), cos(euler.yaw), 0,
      0, 0, 1;
  Eigen::Matrix3d y_x = R_y * R_x;
  return R_z * y_x;
}

inline maf_mla_localization::MLAEuler
rotationMatrixToEulerAngles(const Eigen::Matrix3d &global_R) {
  auto sy = sqrt(global_R.coeff(0, 0) * global_R.coeff(0, 0) +
                 global_R.coeff(1, 0) * global_R.coeff(1, 0));
  maf_mla_localization::MLAEuler euler{};
  if (sy < 1e-6) {
    euler.roll = atan2(-global_R.coeff(1, 2), global_R.coeff(1, 1));
    euler.pitch = atan2(-global_R.coeff(2, 0), sy);
    euler.yaw = 0;
  } else {
    euler.roll = atan2(-global_R.coeff(2, 1), global_R.coeff(2, 2));
    euler.pitch = atan2(-global_R.coeff(2, 0), sy);
    euler.yaw = atan2(global_R.coeff(1, 0), global_R.coeff(0, 0));
  }
  return euler;
}

inline void
compute_quaternion_and_euler(const double ego_yaw,
                             const Eigen::Matrix3d &transform_q_t,
                             maf_mla_localization::MLALocalization *loc) {
  loc->orientation.euler_local.yaw = ego_yaw;
  loc->orientation.quaternion_local =
      Euler2Quaternion(loc->orientation.euler_local);
  auto local_r = eulerAnglesToRotationMatrix(loc->orientation.euler_local);
  auto global_r = transform_q_t * local_r;
  loc->orientation.euler_global = rotationMatrixToEulerAngles(global_r);
  loc->orientation.quaternion_global =
      Euler2Quaternion(loc->orientation.euler_global);
  loc->orientation.available =
      (maf_mla_localization::MLAOrientation::MLA_EULER_GLOBAL |
       maf_mla_localization::MLAOrientation::MLA_EULER_LOCAL |
       maf_mla_localization::MLAOrientation::MLA_QUATERNION_GLOBAL |
       maf_mla_localization::MLAOrientation::MLA_QUATERNION_LOCAL);
}

inline Eigen::Vector3d PlusDeltaEnuAtPos(Eigen::Vector3d const &pos,
                                         Eigen::Vector3d const &denu) {
  return ECEF2LLH(LLH2ECEF(pos, true) + Pos2Cne(pos, true).transpose() * denu,
                  true);
}

inline Eigen::Matrix3d
quaternion_to_rotation_matrix(const Eigen::Quaterniond &q) {
  return q.normalized().toRotationMatrix();
}
} // namespace geometry
} // namespace msquare