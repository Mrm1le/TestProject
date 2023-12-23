#include "parking_apa_planner.h"
#include "common_util.h"
#include "gtest/gtest.h"

using namespace msquare::parking;
using namespace msquare::planning_math;

class AstarPlannerTest : public ::testing::Test{
public:
    static void SetUpTestCase() {
        initConfig();
        msquare::HybridAstarConfig::GetInstance()->verbose = true;

        std::string json_file = "/home/ros/catkin_ws/src/maf_planning/test/unittest/resources/trajectory.json";
        std::vector<msquare::TrajectoryPoint> traj_ps;
        readConfig(traj_ps, osd_, json_file);
        
        planWithStrategy(msquare::StrategyParams::GetInstance(), osd_, sbp_res_);
    }

public:
    static msquare::SearchBasedPlannerPtr planner_;
    static OpenspaceDeciderOutput osd_;
    static msquare::SbpResult sbp_res_;
};
msquare::SearchBasedPlannerPtr AstarPlannerTest::planner_;
OpenspaceDeciderOutput AstarPlannerTest::osd_;
msquare::SbpResult AstarPlannerTest::sbp_res_;

TEST_F(AstarPlannerTest, TestPerformance){
    EXPECT_EQ(AstarPlannerTest::sbp_res_.status, msquare::SbpStatus::SUCCESS);
    EXPECT_EQ(AstarPlannerTest::sbp_res_.x.size(), 21);
    EXPECT_EQ(AstarPlannerTest::planner_->getSearchPoints().size(), 192);

    Pose2D pose;
    pose.x = AstarPlannerTest::osd_.target_state.path_point.x + 0.1;
    pose.y = AstarPlannerTest::osd_.target_state.path_point.y + 0.1;
    pose.theta = AstarPlannerTest::osd_.target_state.path_point.theta;
    std::vector<msquare::planning_math::LineSegment2d> map;
    AstarPlannerTest::planner_->Update(AstarPlannerTest::osd_.map_boundary, map);
    EXPECT_EQ(AstarPlannerTest::planner_->getResult().status, msquare::SbpStatus::EXCEPTION);

    std::shared_ptr<msquare::HybridAstar> planner = std::make_shared<msquare::HybridAstar>(100);
    EXPECT_EQ(planner->getResult().x.size(),0);
}


