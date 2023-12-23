#include "gtest/gtest.h"
#include "common/math/quintic_poly_1d.h"

namespace msquare
{
TEST(PATH_PLANNER_TEST, QuinticSpline1D)
{
    planning_math::QuinticPoly1d quintic_poly1(/*x0*/ 3.0, /*dx0*/ 1.0, /*ddx0*/ 0.8,
                               /*x1*/ 3.6, /*dx1*/ 0.0, /*ddx1*/ 5.0, /*end_s*/ 20.0);
	EXPECT_NEAR(quintic_poly1.Evaluate(/*order*/ 0, 0.0), 3.0, 1e-8);
	EXPECT_NEAR(quintic_poly1.Evaluate(/*order*/ 1, 0.0), 1.0, 1e-8);
	EXPECT_NEAR(quintic_poly1.Evaluate(/*order*/ 2, 0.0), 0.8, 1e-8);
	EXPECT_NEAR(quintic_poly1.Evaluate(/*order*/ 0, 10.0), 42.675, 1e-8);
	EXPECT_NEAR(quintic_poly1.Evaluate(/*order*/ 0, 20.0), 3.6, 1e-8);
	EXPECT_NEAR(quintic_poly1.Evaluate(/*order*/ 1, 20.0), 0.0, 1e-8);
	EXPECT_NEAR(quintic_poly1.Evaluate(/*order*/ 2, 20.0), 5.0, 1e-8);
	std::cout << "first test passed." << std::endl;

    planning_math::QuinticPoly1d quintic_poly2(/*x0*/ -3.0, /*dx0*/ -1.0, /*ddx0*/ 0.8,
                               /*x1*/ -3.6, /*dx1*/ 8.0, /*ddx1*/ -5.0, /*end_s*/ 20.0);
	planning_math::QuinticPoly1d quintic_poly3 = quintic_poly2;
	EXPECT_NEAR(quintic_poly3.Evaluate(/*order*/ 0, 0.0), -3.0, 1e-8);
	EXPECT_NEAR(quintic_poly3.Evaluate(/*order*/ 1, 0.0), -1.0, 1e-8);
	EXPECT_NEAR(quintic_poly3.Evaluate(/*order*/ 2, 0.0), 0.8, 1e-8);
	EXPECT_NEAR(quintic_poly3.Evaluate(/*order*/ 0, 10.0), -57.675, 1e-8);
	EXPECT_NEAR(quintic_poly3.Evaluate(/*order*/ 0, 20.0), -3.6, 1e-8);
	EXPECT_NEAR(quintic_poly3.Evaluate(/*order*/ 1, 20.0), 8.0, 1e-8);
	EXPECT_NEAR(quintic_poly3.Evaluate(/*order*/ 2, 20.0), -5.0, 1e-8);
	std::cout << "second test passed." << std::endl;
}
}