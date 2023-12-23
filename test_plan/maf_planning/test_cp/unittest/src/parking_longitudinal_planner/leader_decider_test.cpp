#include "gtest/gtest.h"
#include "pnc/define/geometry.h"
#include "world_model_info.h"
#include "planner/behavior_planner/deciders/leader_decider.h"
#include <iostream>


TEST(LeaderDecider, TestLeaderId){
    std::string bag_name = "/home/ros/catkin_ws/src/maf_planning/test/unittest/resources/world_model.bag";
    std::shared_ptr<msquare::parking::WorldModel> world_model = std::make_shared<msquare::parking::WorldModel>();

    getWorldModel(world_model, bag_name, 300);

    world_model->init(msquare::SceneType::PARKING_APA);
    world_model->update_members();

    std::shared_ptr<msquare::parking::LeaderDecider> leader_decider = std::make_shared<msquare::parking::LeaderDecider>(world_model);
    leader_decider ->execute();

    using PC = msquare::parking::PlanningContext;
    PC::Instance()->mutable_planning_status()->scenario.status_type = msquare::parking::StatusType::APA;

    EXPECT_TRUE(PC::Instance()->mutable_planning_status()->scenario.status_type == msquare::parking::StatusType::APA);
}
