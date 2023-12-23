#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>

namespace putils {

constexpr double kMathEpsilon = 1e-10;

class MathHelper {

public:
  static double randDouble() { return rand() / (double)RAND_MAX; }
  static Eigen::VectorXd randVector(int rows) {
    Eigen::VectorXd result(rows, 1);
    for (int i = 0; i < rows; i++) {
      result(i) = randDouble();
    }
    return result;
  }

  static double pointDistanceToLine(const Eigen::Vector2d &point,
                                    const Eigen::Vector2d &pre_line_point,
                                    const Eigen::Vector2d &next_line_point) {
    const double dx = next_line_point.x() - pre_line_point.x();
    const double dy = next_line_point.y() - pre_line_point.y();
    double line_length = std::hypot(dx, dy);
    Eigen::Vector2d unit_direction =
        (line_length <= kMathEpsilon
             ? Eigen::Vector2d(0, 0)
             : Eigen::Vector2d(dx / line_length, dy / line_length));
    if (line_length <= kMathEpsilon) {
      return (point - pre_line_point).norm();
    }
    const double x0 = point.x() - pre_line_point.x();
    const double y0 = point.y() - pre_line_point.y();
    const double proj = x0 * unit_direction.x() + y0 * unit_direction.y();
    if (proj <= 0.0) {
      return std::hypot(x0, y0);
    }
    if (proj >= line_length) {
      return (point - next_line_point).norm();
    }
    return std::abs(x0 * unit_direction.y() - y0 * unit_direction.x());
  }

  static void rodrigues(const Eigen::Vector3d &src, Eigen::Matrix3d &dst,
                        Eigen::Matrix3d &j0, Eigen::Matrix3d &j1,
                        Eigen::Matrix3d &j2) {

    double theta = src.norm();
    double c = cos(theta);
    double c1 = 1. - c;
    double s = sin(theta);
    double itheta = theta ? 1. / theta : 0.;
    Eigen::Vector3d r = src * itheta;
    double rx = r(0), ry = r(1), rz = r(2);
    Eigen::Matrix3d rrt = r * r.transpose();
    Eigen::Matrix3d r_x;
    r_x << 0, -rz, ry, rz, 0, -rx, -ry, rx, 0;
    Eigen::Matrix3d I;
    I.setIdentity();
    dst = c * I + c1 * rrt + s * r_x;

    double J[27];
    if (theta < 1e-10) {
      memset(J, 0, sizeof(J));
      J[5] = J[15] = J[19] = -1;
      J[7] = J[11] = J[21] = 1;
    } else {
      double rrt[] = {rx * rx, rx * ry, rx * rz, rx * ry, ry * ry,
                      ry * rz, rx * rz, ry * rz, rz * rz};
      double r_x[] = {0, -rz, ry, rz, 0, -rx, -ry, rx, 0};
      const double I[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
      double drrt[] = {rx + rx, ry, rz, ry, 0,       0,  rz, 0,  0,
                       0,       rx, 0,  rx, ry + ry, rz, 0,  rz, 0,
                       0,       0,  rx, 0,  0,       ry, rx, ry, rz + rz};
      double d_r_x_[] = {0, 0,  0, 0, 0, -1, 0, 1, 0, 0, 0, 1, 0, 0,
                         0, -1, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0};
      for (int i = 0; i < 3; i++) {
        double ri = i == 0 ? rx : i == 1 ? ry : rz;
        double a0 = -s * ri, a1 = (s - 2 * c1 * itheta) * ri, a2 = c1 * itheta;
        double a3 = (c - s * itheta) * ri, a4 = s * itheta;
        for (int k = 0; k < 9; k++)
          J[i * 9 + k] = a0 * I[k] + a1 * rrt[k] + a2 * drrt[i * 9 + k] +
                         a3 * r_x[k] + a4 * d_r_x_[i * 9 + k];
      }
    }

    j0 << J[0], J[1], J[2], J[3], J[4], J[5], J[6], J[7], J[8];
    j1 << J[9], J[10], J[11], J[12], J[13], J[14], J[15], J[16], J[17];
    j2 << J[18], J[19], J[20], J[21], J[22], J[23], J[24], J[25], J[26];
  }

  static void rodrigues(const Eigen::Vector3d &src, Eigen::Matrix3d &dst) {

    double theta = src.norm();
    double c = cos(theta);
    double c1 = 1. - c;
    double s = sin(theta);
    double itheta = theta ? 1. / theta : 0.;
    Eigen::Vector3d r = src * itheta;
    double rx = r(0), ry = r(1), rz = r(2);
    Eigen::Matrix3d rrt = r * r.transpose();
    Eigen::Matrix3d r_x;
    r_x << 0, -rz, ry, rz, 0, -rx, -ry, rx, 0;
    Eigen::Matrix3d I;
    I.setIdentity();
    dst = c * I + c1 * rrt + s * r_x;
  }

  static inline Eigen::Quaterniond yawToQuaternion(double yaw) {
    double half_yaw = 0.5f * yaw;
    Eigen::Quaterniond result;
    result.w() = cos(half_yaw);
    result.x() = 0;
    result.y() = 0;
    result.z() = sin(half_yaw);
    return result;
  }
  static inline Eigen::Matrix3d yawToRotmat(double yaw) {
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
    double sin_yaw = sin(yaw);
    result(0, 0) = result(1, 1) = cos(yaw);
    result(0, 1) = -sin_yaw;
    result(1, 0) = sin_yaw;
    return result;
  }

  static inline Eigen::Matrix2d yawToRotmat2d(double yaw) {
    Eigen::Matrix2d result = Eigen::Matrix2d::Identity();
    double sin_yaw = sin(yaw);
    result(0, 0) = result(1, 1) = cos(yaw);
    result(0, 1) = -sin_yaw;
    result(1, 0) = sin_yaw;
    return result;
  }

  static inline Eigen::Matrix2d
  extractRotYaw2dFrom3d(const Eigen::Matrix3d &rot_mat) {
    return yawToRotmat2d(extractYawFromRotMat(rot_mat));
  }

  static inline Eigen::Isometry2d pose3dTo2d(const Eigen::Isometry3d &pose) {
    Eigen::Isometry2d result = Eigen::Isometry2d::Identity();
    result.linear() = extractRotYaw2dFrom3d(pose.linear());
    result.translation() = pose.translation().topRows(2);
    return result;
  }

  static inline double quaternion2DToYaw(const Eigen::Quaterniond &quat) {
    double half_yaw = atan2(quat.z(), quat.w());
    return regularizeAngle(2 * half_yaw);
  }

  static inline double rotmat2DToYaw(const Eigen::Matrix2d &rot_mat) {
    double yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
    return regularizeAngle(yaw);
  }

  static Eigen::Vector3d
  extractYawPitchRollFromRotMat(const Eigen::Matrix3d &rot_mat) {
    Eigen::Vector3d eular = rot_mat.eulerAngles(2, 1, 0);
    // TODO : 3*(analyze this threshold)
    if (std::abs(eular(1)) + std::abs(eular(2)) > 0.8 * M_PI) {
      eular(0) = regularizeAngle(eular(0) + M_PI);
      eular(1) = regularizeAngle(eular(1) + M_PI);
      eular(2) = regularizeAngle(eular(2) + M_PI);
    } else {
      eular(0) = regularizeAngle(eular(0));
      eular(1) = regularizeAngle(eular(1));
      eular(2) = regularizeAngle(eular(2));
    }
    return eular;
  }

  static double extractYawFromRotMat(const Eigen::Matrix3d &rot_mat) {
    Eigen::Vector3d eular = rot_mat.eulerAngles(2, 1, 0);
    if (std::abs(eular(1)) + std::abs(eular(2)) > 0.8 * M_PI) {
      return regularizeAngle(eular(0) + M_PI);
    } else {
      return regularizeAngle(eular(0));
    }
  }

  static double extractYawFromRotQuat(const Eigen::Quaterniond &rot_q) {
    Eigen::Vector3d eular = rot_q.toRotationMatrix().eulerAngles(2, 1, 0);
    if (std::abs(eular(1)) + std::abs(eular(2)) > 0.8 * M_PI) {
      return RegularizeAngleTough(eular(0) + M_PI);
    } else {
      return RegularizeAngleTough(eular(0));
    }
  }

  static double regularizeAngle(double angle) {
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
  }

  static double regularizeAngleTo02PI(double angle) {
    RegularizeAngleTough(angle);
    if (angle > 2 * M_PI) {
      angle -= 2 * M_PI;
    } else if (angle < 0) {
      angle += 2 * M_PI;
    }
    return angle;
  }

  static double RegularizeAngleTough(double angle) {
    int angle_round = angle / (2 * M_PI);
    angle = angle - angle_round * (2 * M_PI);
    if (angle > M_PI) {
      angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
      angle += 2 * M_PI;
    }

    return angle;
  }

private:
  MathHelper() = delete;
  virtual ~MathHelper();
};

} // namespace putils