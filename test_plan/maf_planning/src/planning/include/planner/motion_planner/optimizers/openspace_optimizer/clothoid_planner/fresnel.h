
/*                                                      fresnl.c

 *      Fresnel integrals
 *
 *
 *
 * SYNOPSIS:
 *
 * double x, S, C;
 * int fresnl();
 *
 * fresnl( x, _&S, _&C );
 *
 *
 * DESCRIPTION:
 *
 * Evaluates the Fresnel integrals
 *
 *           x
 *           -
 *          | |
 * C(x) =   |   cos(pi/2 t**2) dt,
 *        | |
 *         -
 *          0
 *
 *           x
 *           -
 *          | |
 * S(x) =   |   sin(pi/2 t**2) dt.
 *        | |
 *         -
 *          0
 *
 *
 * The integrals are approximated by rational functions if x is small.
 * For large x, auxiliary functions f(x) and g(x) are employed
 * such that
 *
 * C(x) = 0.5 + f(x) sin( pi/2 x**2 ) - g(x) cos( pi/2 x**2 )
 * S(x) = 0.5 - f(x) cos( pi/2 x**2 ) - g(x) sin( pi/2 x**2 ) .
 *
 *
 *
 * ACCURACY:
 *
 *  Relative error.
 *
 * Arithmetic  function   domain     # trials      peak         rms
 *   IEEE       S(x)      0, 10       10000       2.0e-15     3.2e-16
 *   IEEE       C(x)      0, 10       10000       1.8e-15     3.3e-16
 *   DEC        S(x)      0, 10        6000       2.2e-16     3.9e-17
 *   DEC        C(x)      0, 10        5000       2.3e-16     3.9e-17
 */

/*
   Cephes Math Library Release 2.1:  January, 1989
   Copyright 1984, 1987, 1989 by Stephen L. Moshier
 */

#ifndef FRESNL_HEADER
#define FRESNL_HEADER

namespace clothoid {

double p1evl(double x, void *coef, int N);
double polevl(double x, void *coef, int N);
int fresnel(double xxa, double *ssa, double *cca);

} // namespace clothoid

#endif