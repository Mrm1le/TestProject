#include "common/math/line2d.h"

#include "gtest/gtest.h"

#include "common/math/line_segment2d.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"

namespace msquare {
namespace planning_math {

namespace {

Line2d line1({0, 0}, {0, 1});
Line2d line2({0, 0}, {1, 0});
Line2d line3({0, 0}, {1, 1});
Line2d line4({1, 0}, M_PI_4);
Line2d line5({0, 1}, -M_PI_4);
Line2d line6({1, 1}, {0, 1});
Line2d line7({1, 2}, {5, 4});

}  // namespace

TEST(Line2dTest, DistanceTo) {
  EXPECT_NEAR(line1.DistanceTo({0, 0}), 0.0, 1e-5);
  EXPECT_NEAR(line1.DistanceTo({1, 1}), 1.0, 1e-5);
  EXPECT_NEAR(line1.DistanceTo({-1, -1}), 1.0, 1e-5);
  EXPECT_NEAR(line2.DistanceTo({1, 1}), 1.0, 1e-5);
  EXPECT_NEAR(line2.DistanceTo({0, 0}), 0.0, 1e-5);
  EXPECT_NEAR(line2.DistanceTo({0, 1}), 1.0, 1e-5);
}

TEST(Line2dTest, IsPointIn) {
  EXPECT_TRUE(line1.IsPointIn({0, 0}));
  EXPECT_TRUE(line1.IsPointIn({0, 10}));
  EXPECT_TRUE(line3.IsPointIn({2, 2}));
  EXPECT_FALSE(line1.IsPointIn({-3, 0}));
  EXPECT_FALSE(line1.IsPointIn({2, 2}));
  EXPECT_FALSE(line2.IsPointIn({-4, -2}));
}

TEST(Line2dTest, GetIntersect) {
  Vec2d point;
  EXPECT_FALSE(line2.GetIntersect(line6, &point));

  EXPECT_TRUE(line1.GetIntersect(line2, &point));
  EXPECT_NEAR(point.x(), 0, 1e-5);
  EXPECT_NEAR(point.y(), 0, 1e-5);
  EXPECT_TRUE(line1.GetIntersect(line7, &point));
  EXPECT_NEAR(point.x(), 0, 1e-5);
  EXPECT_NEAR(point.y(), 1.5, 1e-5);
  EXPECT_TRUE(line3.GetIntersect(line7, &point));
  EXPECT_NEAR(point.x(), 3, 1e-5);
  EXPECT_NEAR(point.y(), 3, 1e-5);
}

TEST(Line2dTest, GetPerpendicularFoot) {
  Vec2d foot_pt;
  EXPECT_NEAR(line7.GetPerpendicularFoot({0, 0}, &foot_pt), 0.6 * std::sqrt(5.0),
              1e-5);
  EXPECT_NEAR(foot_pt.x(), -0.6, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 1.2, 1e-5);
  EXPECT_NEAR(line7.GetPerpendicularFoot({3, 3}, &foot_pt), 0.0, 1e-5);
  EXPECT_NEAR(foot_pt.x(), 3.0, 1e-5);
  EXPECT_NEAR(foot_pt.y(), 3.0, 1e-5);
}

TEST(Line2dTest, GetBisector) {
  Line2d bisector;
  Vec2d point;
  EXPECT_FALSE(line2.GetBisector(line6, &bisector));

  EXPECT_TRUE(line1.GetBisector(line2, &bisector));
  EXPECT_TRUE(line1.GetIntersect(line2, &point));
  EXPECT_NEAR(bisector.unit_direction().x(), std::sqrt(0.5), 1e-5);
  EXPECT_NEAR(bisector.unit_direction().y(), std::sqrt(0.5), 1e-5);
  EXPECT_TRUE(bisector.IsPointIn(point));
  EXPECT_NEAR(line1.DistanceTo(point), line2.DistanceTo(point), 1e-5);

  EXPECT_TRUE(line4.GetBisector(line5, &bisector));
  EXPECT_TRUE(line4.GetIntersect(line5, &point));
  EXPECT_NEAR(bisector.unit_direction().x(), 1.0, 1e-5);
  EXPECT_NEAR(bisector.unit_direction().y(), 0.0, 1e-5);
  EXPECT_TRUE(bisector.IsPointIn(point));
  EXPECT_NEAR(line4.DistanceTo(point), line5.DistanceTo(point), 1e-5);
}

}  // namespace planning_math
}  // namespace apollo
