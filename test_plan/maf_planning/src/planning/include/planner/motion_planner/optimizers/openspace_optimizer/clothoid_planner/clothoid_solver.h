#ifndef CLOTHOID_SOLVER_HEADER
#define CLOTHOID_SOLVER_HEADER

#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_parameter.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/fresnel_interface.h"

#include <vector>

namespace clothoid {

/**
 * @brief clothoid: curvature change linealy relative to s
 *  k = alpha * s
 *
 */
class ClothoidSolver {
public:
  ClothoidSolver(double alpha) : alpha_(alpha), alpha_dao_(1.0 / alpha_) {}
  ClothoidSolver() {}
  double getAlpha() { return alpha_; }

  // s >= 0
  void getPointByS(double s, ClothoidPoint &clo_p);
  void getPointByTheta(double theta, ClothoidPoint &clo_p);
  void getPointByRadius(double radius, ClothoidPoint &clo_p);
  void getPointByCurvature(double k, ClothoidPoint &clo_p);

  void getCurveByS(double s, double step, ClothoidCurve &clo_ps);
  void getCurveByS(double s0, double s1, double step, ClothoidCurve &clo_ps);
  ClothoidCurve getCurveByS2(double s, double step);

  void getMuOffset(double radius, double &cx, double &cy, double &mu);

private:
  double alpha_;     // curvature slope factor
  double alpha_dao_; // curvature slope factor
};

} // namespace clothoid

#endif