#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_solver.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/curve.h"

#include <cmath>
#include <iostream>
namespace clothoid {
void ClothoidSolver::getPointByS(double s, ClothoidPoint &clo_p) {
  clo_p.s = s;
  clo_p.k = alpha_ * s;
  clo_p.theta = 0.5 * clo_p.k * s;

  double scale = std::sqrt(alpha_ * Constant::pi_dao);
  double fresnel_x = scale * s;
  double ss, cc;
  fresnelSingle(fresnel_x, &ss, &cc);
  clo_p.x = cc / scale;
  clo_p.y = ss / scale;
}
void ClothoidSolver::getPointByTheta(double theta, ClothoidPoint &clo_p) {
  double s = std::sqrt(2 * theta * alpha_dao_);
  getPointByS(s, clo_p);
}
void ClothoidSolver::getPointByRadius(double radius, ClothoidPoint &clo_p) {
  double s = 1.0 / radius * alpha_dao_;
  getPointByS(s, clo_p);
}
void ClothoidSolver::getPointByCurvature(double k, ClothoidPoint &clo_p) {
  double s = k * alpha_dao_;
  getPointByS(s, clo_p);
}

void ClothoidSolver::getCurveByS(double s0, double s1, double step,
                                 ClothoidCurve &clo_ps) {
  s0 = std::max(s0, 0.0);
  s1 = std::max(s1, 0.0);
  if (std::abs(s1 - s0) < 1e-6) {
    clo_ps.resize(1);
    ClothoidSolver::getPointByS(s0, clo_ps[0]);
  }
  std::vector<double> s_list = clothoid::linspace(s0, s1, step);
  clo_ps.resize(s_list.size());
  for (unsigned int i = 0; i < clo_ps.size(); i++) {
    getPointByS(s_list[i], clo_ps[i]);
  }
}

void ClothoidSolver::getCurveByS(double s, double step, ClothoidCurve &clo_ps) {
  if (s < 1e-6) {
    clo_ps.resize(1);
    ClothoidSolver::getPointByS(0.0, clo_ps[0]);
  }
  int step_num = int(s / step) + 1;
  double step_normalize = s / step_num;

  clo_ps.resize(step_num + 1);
  for (int i = 0; i <= step_num; i++) {
    double si = i * step_normalize;
    getPointByS(si, clo_ps[i]);
  }
}

ClothoidCurve ClothoidSolver::getCurveByS2(double s, double step) {
  ClothoidCurve clo_curve;
  getCurveByS(s, step, clo_curve);
  return clo_curve;
}

void ClothoidSolver::getMuOffset(double radius, double &cx, double &cy,
                                 double &mu) {
  ClothoidPoint clo_p;
  getPointByRadius(radius, clo_p);
  cx = clo_p.x - radius * std::sin(clo_p.theta);
  cy = clo_p.y + radius * std::cos(clo_p.theta);
  mu = std::atan2(cx, cy);
}

} // namespace clothoid