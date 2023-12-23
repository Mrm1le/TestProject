#ifndef COMMON_PATH_POINT_
#define COMMON_PATH_POINT_

#include <string>

namespace msquare {

struct PathPoint {
  // coordinates
  double x;
  double y;
  double z;

  // direction on the x-y plane
  double theta;
  // curvature on the x-y plane
  double kappa;
  // curvature radius on the x-y plane
  double rho;
  // accumulated distance from beginning of the path
  double s;
  double l;

  // derivative of kappa w.r.t s.
  double dkappa;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa;
  // The lane ID where the path point is on
  std::string lane_id;

  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
  double x_derivative;
  double y_derivative;

  // zyl add:
  void set_x(double x_) { x = x_; }
  void set_y(double y_) { y = y_; }
  void set_z(double z_) { z = z_; }
  void set_s(double s_) { s = s_; }
  void set_theta(double theta_) { theta = theta_; }
  void set_kappa(double kappa_) { kappa = kappa_; }
  void set_dkappa(double dkappa_) { dkappa = dkappa_; }
  void set_ddkappa(double ddkappa_) { ddkappa = ddkappa_; }

  PathPoint(double x_ = 0, double y_ = 0, double z_ = 0, double theta_ = 0,
            double kappa_ = 0, double rho_ = 0, double s_ = 0, double l_ = 0,
            double dkappa_ = 0, double ddkappa_ = 0, double x_derivative_ = 0,
            double y_derivative_ = 0)
      : x(x_), y(y_), z(z_), theta(theta_), kappa(kappa_), rho(rho_), s(s_),
        l(l_), dkappa(dkappa_), ddkappa(ddkappa_), lane_id(std::string{""}),
        x_derivative(x_derivative_), y_derivative(y_derivative_) {}
};

} // namespace msquare
#endif