#include "gtest/gtest.h"
#include "planner/motion_planner/path_planner_ceres/python_test_spline_wrapper.hpp"

namespace msquare
{
TEST(PATH_PLANNER_TEST, CubicSpline)
{
    // Assemble init condition for cubic spline
	// Test for arbitrary init conditions is shown in slider_tutorial.py
    static constexpr size_t N = path_planner::NUM_PATH_CONTROL_POINTS;
	static constexpr size_t N_OUT = path_planner::TOTAL_QUAD_POINTS;
	double init_dx_ds = 0.5;
	double end_d2x_ds2 = 0.2;
	std::array<double, N> s_at_control_points = {0, 20, 40, 60, 80};
	std::array<double, N> x_at_control_points = {-4.0, 1.6, 0.0, -1.0, -2.0};
	path_planner::TestCubicSpline cubic_spline_1d =
	    path_planner::TestCubicSpline(init_dx_ds, end_d2x_ds2, s_at_control_points,
	    x_at_control_points);
	
	// Get output
	Eigen::VectorXd all_quad_pt_s(N_OUT), all_quad_pt_x(N_OUT), all_quad_pt_dx(N_OUT),
	    all_quad_pt_d2x(N_OUT), all_quad_pt_d3x(N_OUT);
	cubic_spline_1d.get_all_quad_points_data(all_quad_pt_s, all_quad_pt_x,
	    all_quad_pt_dx, all_quad_pt_d2x, all_quad_pt_d3x);
	std::cout << "dump here"<< std::endl;
	
	// Test at sample point 1
	EXPECT_EQ(all_quad_pt_s.size(), 20);
	EXPECT_NEAR(all_quad_pt_s[10], 40.93820, 1e-4);
	EXPECT_NEAR(all_quad_pt_x[10], -0.00793, 1e-4);
	EXPECT_NEAR(all_quad_pt_dx[10], 0.00128, 1e-4);
	EXPECT_NEAR(all_quad_pt_d2x[10], 0.01951, 1e-4);
	EXPECT_NEAR(all_quad_pt_d3x[10], -0.00395, 1e-4);
	std::cout << "first test passed. " << std::endl;

    // Test at sample point 2
	EXPECT_NEAR(all_quad_pt_s[15], 60.93820, 1e-4);
	EXPECT_NEAR(all_quad_pt_x[15], -1.34614, 1e-4);
	EXPECT_NEAR(all_quad_pt_dx[15], -0.39136, 1e-4);
	EXPECT_NEAR(all_quad_pt_d2x[15], -0.04380, 1e-4);
	EXPECT_NEAR(all_quad_pt_d3x[15], 0.01279, 1e-4);
	std::cout << "second test passed. " << std::endl;
}
}