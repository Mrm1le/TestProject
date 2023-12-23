
#include "planner/motion_planner/openspace_motion_planner/path_sampler.h"
#include "parking_apa_planner.h"
#include "gtest/gtest.h"


using namespace msquare::parking;



TEST(PathSamplerTest, ZigzagPath){
    std::string json_file = "/home/ros/catkin_ws/src/maf_planning/test/unittest/resources/trajectory.json";
    msquare::parking::OpenspaceDeciderOutput osd;
    std::vector<msquare::TrajectoryPoint> traj_ps;
    readConfig(traj_ps, osd, json_file);

    std::shared_ptr<msquare::parking::WorldModel> world_model = std::make_shared<msquare::parking::WorldModel>();
    PathSampler sampler(world_model);
    sampler.set_path(traj_ps);
    sampler.calculate();
    EXPECT_FALSE(sampler.is_arrived());
    EXPECT_FALSE(sampler.is_at_last_segment());

    Pose2D pose{-2880.75, 1087.70, -2.2089};
    EXPECT_FALSE(sampler.is_arrived(pose, 1.0));
}

