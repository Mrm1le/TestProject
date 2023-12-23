#ifndef __UTILS__GEOMETRY_H__
#define __UTILS__GEOMETRY_H__

#include <vector>

struct Point2D {
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  Point2D(double xx, double yy) : x(xx), y(yy) {}
};

struct Point3D {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Point3D() = default;
  Point3D(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;

  Quaternion() = default;
  Quaternion(double xx, double yy, double zz, double ww)
      : x(xx), y(yy), z(zz), w(ww) {}
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double s = 0.0;
  double kappa_ = 0.0;

  const double const_x() const { return x; }
  const double const_y() const { return y; }
  const double const_theta() const { return theta; }
  const double const_s() const { return s; }
  const double kappa() const {return kappa_; }

  void set_kappa(double kappa) { kappa_ = kappa; }

  Pose2D() = default;
  Pose2D(double xx, double yy, double tt) : x(xx), y(yy), theta(tt) {}
  Pose2D(double xx, double yy, double tt, double ss) : x(xx), y(yy), theta(tt), s(ss) {}
  Pose2D(double xx, double yy, double tt, double ss, double kappa) : x(xx), y(yy), theta(tt), s(ss), kappa_(kappa) {}
};
using Pose2DTrajectory = std::vector<Pose2D>;

struct Pose {
  Point3D position{};
  Quaternion orientation{};
};

struct Vector3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vector3() = default;
  Vector3(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};

struct Twist {
  Vector3 linear;
  Vector3 angular;
};

struct TwistWithCovariance {
  Twist twist;
  double covariance[36];
};

struct Pose3D {
  Point3D position;
  Quaternion orientation;
};

struct PoseStamped {
  Pose3D pose;
};

#endif
