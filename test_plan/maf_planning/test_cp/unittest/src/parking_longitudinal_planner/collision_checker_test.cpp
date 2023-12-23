#include "gtest/gtest.h"
#include "pnc/define/geometry.h"
#include "common/config/vehicle_param.h"
#include "planner/behavior_planner/deciders/collision_checker.h"
#include <iostream>

using namespace msquare::parking;
class CollisionCheckerTest : public ::testing::Test{
public:
    static void SetUpTestCase() {
        std::string car_param_file = "/home/ros/catkin_ws/src/maf_planning/resource/"
                            "config/scenario_configs_json/parking/"
                            "vehicle_param.yaml";
        if (!msquare::VehicleParam::Instance()->loadFile(car_param_file)) {
            std::cout<<"VehicleParam load file failed."<<std::endl;
        }   
        CollisionCheckerTest::setParams(false);
    }

    static void setParams(bool reverse){
        double back_comp_length = reverse ? 0.0
              : -msquare::VehicleParam::Instance()->front_edge_to_center +
                    msquare::VehicleParam::Instance()->length;
        collision_checker_.set_params(
            msquare::VehicleParam::Instance()->front_edge_to_center - msquare::VehicleParam::Instance()->length / 2.0,
            back_comp_length,
            reverse
        );
    }

public:
    static CollisionChecker collision_checker_;
};

CollisionChecker CollisionCheckerTest::collision_checker_ = CollisionChecker();

CollisionChecker& collision_checker = CollisionCheckerTest::collision_checker_;

TEST_F(CollisionCheckerTest, TestModelSpec){
    double thres = 0.05;
    msquare::planning_math::Vec2d obs_point(1,1);

    CollisionCheckerTest::setParams(false);
    Pose2D ego_pose{0.0, 0.0, 0.0};
    // collision_checker.set_point(ego_pose);
    // collision_checker.collision_check(obs_point, thres);

    // EXPECT_TRUE(collision_checker.is_point());
    // EXPECT_FALSE(collision_checker.is_trajectory());
}
TEST_F(CollisionCheckerTest, TestPointCollision){
    double thres = 0.05;
    msquare::planning_math::Vec2d obs_point;
    CollisionCheckStatus check_status;

    // // Pose2D
    // Pose2D ego_pose{0.0, 0.0, 0.0};
    // collision_checker.set_point(ego_pose);
    // obs_point = msquare::planning_math::Vec2d(1.0, 1.0);
    // check_status = collision_checker.collision_check(obs_point, thres);
    // EXPECT_TRUE(check_status.is_collision);

    // obs_point = msquare::planning_math::Vec2d(4.0, 1.0);
    // check_status = collision_checker.collision_check(obs_point, thres);
    // EXPECT_FALSE(check_status.is_collision);

    // msquare::planning_math::Box2d obs_box({2.0, 3.0}, 0, 1.9, 2.0);
    // check_status = collision_checker.collision_check(obs_box, thres);
    // EXPECT_FALSE(check_status.is_collision);

    // msquare::planning_math::LineSegment2d obs_line({4.0, 0.0}, {0.0, 3.0});
    // check_status = collision_checker.collision_check(obs_line, thres);
    // EXPECT_TRUE(check_status.is_collision);

    // msquare::planning_math::Polygon2d obs_polygon(obs_box);
    // check_status = collision_checker.collision_check(obs_polygon, thres);
    // EXPECT_FALSE(check_status.is_collision);

    //  // Pose2D
    // msquare::PathPoint ego_path_point(0.0, 0.0, 0.0, 0.0, 0.0);
    // collision_checker.set_point(ego_path_point);
    // obs_point = msquare::planning_math::Vec2d(1.0, 1.0);
    // check_status = collision_checker.collision_check(obs_point, thres);
    // EXPECT_TRUE(check_status.is_collision);

    // obs_point = msquare::planning_math::Vec2d(4.0, 1.0);
    // check_status = collision_checker.collision_check(obs_point, thres);
    // EXPECT_FALSE(check_status.is_collision);

    // obs_box = msquare::planning_math::Box2d({2.0, 3.0}, 0, 1.9, 2.0);
    // check_status = collision_checker.collision_check(obs_box, thres);
    // EXPECT_FALSE(check_status.is_collision);

    // obs_line = msquare::planning_math::LineSegment2d({4.0, 0.0}, {0.0, 3.0});
    // check_status = collision_checker.collision_check(obs_line, thres);
    // EXPECT_TRUE(check_status.is_collision);

    // obs_polygon = msquare::planning_math::Polygon2d(obs_box);
    // check_status = collision_checker.collision_check(obs_polygon, thres);
    // EXPECT_FALSE(check_status.is_collision);
}

TEST_F(CollisionCheckerTest, TestTrajectorytCollision){
    double thres = 0.05;
    CollisionCheckStatus check_status;

    // trajectory: y = x, x ➡ [-5, 5]
    std::vector<msquare::PathPoint> traj_pathpoint;
    std::vector<Pose2D> traj_pose2d;
    for(double x = -5.0; x < 5.0; x += 0.3){
        traj_pathpoint.emplace_back(x, x, 0, M_PI/4.0, 0);
        traj_pose2d.emplace_back(x, x, M_PI/4.0);
    }

    // PathPoint
    msquare::planning_math::Vec2d obs_point = msquare::planning_math::Vec2d(1.0, 1.0);
    check_status =
        collision_checker.collision_check(traj_pathpoint, obs_point, thres);
    EXPECT_TRUE(check_status.is_collision);

    obs_point = msquare::planning_math::Vec2d(4.0, 1.0);
    check_status =
        collision_checker.collision_check(traj_pathpoint, obs_point, thres);
    EXPECT_FALSE(check_status.is_collision);

    msquare::planning_math::Box2d obs_box({0.0, 4.0}, 0, 1.9, 2.0);
    check_status =
        collision_checker.collision_check(traj_pathpoint, obs_box, thres);
    EXPECT_FALSE(check_status.is_collision);

    msquare::planning_math::LineSegment2d obs_line({4.0, 0.0}, {0.0, 3.0});
    check_status =
        collision_checker.collision_check(traj_pathpoint, obs_line, thres);
    EXPECT_TRUE(check_status.is_collision);

    msquare::planning_math::Polygon2d obs_polygon(obs_box);
    check_status =
        collision_checker.collision_check(traj_pathpoint, obs_polygon, thres);
    EXPECT_FALSE(check_status.is_collision);

    // Pose2D
    collision_checker.set_trajectory(traj_pose2d);

    obs_point = msquare::planning_math::Vec2d(1.0, 1.0);
    check_status =
        collision_checker.collision_check(traj_pose2d, obs_point, thres);
    EXPECT_TRUE(check_status.is_collision);

    obs_point = msquare::planning_math::Vec2d(4.0, 1.0);
    check_status =
        collision_checker.collision_check(traj_pose2d, obs_point, thres);
    EXPECT_FALSE(check_status.is_collision);

    obs_box = msquare::planning_math::Box2d({0.0, 4.0}, 0, 1.9, 2.0);
    check_status =
        collision_checker.collision_check(traj_pose2d, obs_box, thres);
    EXPECT_FALSE(check_status.is_collision);

    obs_line = msquare::planning_math::LineSegment2d({4.0, 0.0}, {0.0, 3.0});
    check_status =
        collision_checker.collision_check(traj_pose2d, obs_line, thres);
    EXPECT_TRUE(check_status.is_collision);

    obs_polygon = msquare::planning_math::Polygon2d(obs_box);
    check_status =
        collision_checker.collision_check(traj_pose2d, obs_polygon, thres);
    EXPECT_FALSE(check_status.is_collision);
}




