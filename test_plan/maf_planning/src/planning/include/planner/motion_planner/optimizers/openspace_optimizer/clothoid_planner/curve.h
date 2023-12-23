#ifndef CURVE_H
#define CURVE_H

#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/clothoid_parameter.h"
namespace clothoid {

void mirrorX(const ClothoidCurve &c1, ClothoidCurve &c2);
void mirrorY(const ClothoidCurve &c1, ClothoidCurve &c2);
void moveCurve(const ClothoidCurve &c1, ClothoidCurve &c2, double x, double y);
void rotateCurve(const ClothoidCurve &c1, ClothoidCurve &c2, double theta);
void transform2D(const ClothoidCurve &c1, ClothoidCurve &c2, double x, double y,
                 double theta);
void transform2D(const StraightCurve &c1, StraightCurve &c2, double x,
                 double y);

/**
 * for clothoid curve, start point is at origin
 *
 * as for S curve:
 */
void transformS2(const ClothoidCurve &c1, ClothoidCurve &c2);
void transformS3(const ClothoidCurve &c1, ClothoidCurve &c2);
void transformS4(const ClothoidCurve &c1, ClothoidCurve &c2);

std::vector<double> linspace(double s, double e, double step);

StraightCurve linspace(const StraightPoint &s1, const StraightPoint &s2,
                       double step);
StraightCurve linspace(const StraightPoint &s1, double s, double step);

CircularCurve linspaceCCW(const CircularPoint &s1, double theta,
                          double theta_step, Circle circle);
CircularCurve linspaceCCWTop(const CircularPoint &s1, double theta,
                             double theta_step, Circle c);

CircularCurve linspaceCW(double theta1, double theta2, double theta_step,
                         Circle circle);
CircularCurve linspaceCWRight(double theta1, double theta2, double theta_step,
                              Circle c);
CircularCurve clo2cir(const ClothoidCurve &clo);

void rotationalSymmetry(double sx, double sy, const StraightCurve &s1,
                        StraightCurve &s2);

} // namespace clothoid

#endif