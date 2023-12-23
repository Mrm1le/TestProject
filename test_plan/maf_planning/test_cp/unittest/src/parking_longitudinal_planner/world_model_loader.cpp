#include "world_model_info.h"
#include "gtest/gtest.h"

TEST(WorldModel, Loader){
    std::string bag_name = "/home/ros/Downloads/apa_bag/world_model.bag";
    // msquare::parking::bagReadTest(bag_name);

    std::shared_ptr<msquare::parking::WorldModel> world_model = std::make_shared<msquare::parking::WorldModel>();

    getWorldModel(world_model, bag_name, 300);
    Pose2D ego_pose = world_model-> get_ego_state().ego_pose;
    EXPECT_NEAR(ego_pose.x, 71.4461, 1e-3);
}

