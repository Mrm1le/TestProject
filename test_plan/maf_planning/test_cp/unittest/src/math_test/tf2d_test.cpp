#include "common/math/math_utils.h"
#include "gtest/gtest.h"



// using planning_math = msquare::planning_math;
using namespace msquare::planning_math;
class Tf2dTest : public ::testing::Test{
public:
    static void SetUpTestCase() {
        other_frame_ = Pose2D(4.0, 4.0, M_PI/2.0);
        box2d_ = Box2d(Vec2d(2.0, 2.0),0.0, 4.0, 2.0);
        line2d_ = LineSegment2d(Vec2d(0.0, 4.0), Vec2d(4.0, 0.0));
    }

public:
    static Pose2D other_frame_;
    static Box2d box2d_;
    static LineSegment2d line2d_;
};

Pose2D Tf2dTest::other_frame_;
Box2d Tf2dTest::box2d_;
LineSegment2d Tf2dTest::line2d_;


TEST_F(Tf2dTest, TestBox){
    // global to local
    Box2d local_box= tf2d(Tf2dTest::other_frame_, Tf2dTest::box2d_);
    EXPECT_DOUBLE_EQ(local_box.center_x(), -2.0);
    EXPECT_DOUBLE_EQ(local_box.center_y(), 2.0);
    EXPECT_DOUBLE_EQ(local_box.length(), Tf2dTest::box2d_.length());
    EXPECT_DOUBLE_EQ(local_box.width(), Tf2dTest::box2d_.width());

    // local to global
    Box2d global_box= tf2d_inv(Tf2dTest::other_frame_, Tf2dTest::box2d_);
    EXPECT_DOUBLE_EQ(global_box.center_x(), 2.0);
    EXPECT_DOUBLE_EQ(global_box.center_y(), 6.0);
    EXPECT_DOUBLE_EQ(global_box.length(), Tf2dTest::box2d_.length());
    EXPECT_DOUBLE_EQ(global_box.width(), Tf2dTest::box2d_.width());
}

TEST_F(Tf2dTest, TestLine){
    double thresh = 1e-6;
    // global to local
    LineSegment2d local_line= tf2d(Tf2dTest::other_frame_, Tf2dTest::line2d_);
    EXPECT_NEAR(local_line.start().x(), 0.0, thresh);
    EXPECT_NEAR(local_line.start().y(), 4.0, thresh);
    EXPECT_NEAR(local_line.end().x(), -4.0, thresh);
    EXPECT_NEAR(local_line.end().y(), 0.0, thresh);

    // local to global
    LineSegment2d global_line= tf2d_inv(Tf2dTest::other_frame_, Tf2dTest::line2d_);
    EXPECT_NEAR(global_line.start().x(), 0.0, thresh);
    EXPECT_NEAR(global_line.start().y(), 4.0, thresh);
    EXPECT_NEAR(global_line.end().x(), 4.0, thresh);
    EXPECT_NEAR(global_line.end().y(), 8.0, thresh);
}

