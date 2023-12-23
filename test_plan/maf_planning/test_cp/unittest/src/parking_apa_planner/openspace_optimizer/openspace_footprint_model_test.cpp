#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.h"
#include "parking_apa_planner.h"
#include "common/sbp_obstacle_line.h"

#include "gtest/gtest.h"

namespace msquare {

class OpenspaceFootprintTest : public ::testing::Test{
public:
  static void SetUpTestCase() {
    initConfig();
    double lat_inflation_ = CarParams::GetInstance()->lat_inflation();
    double shrink_ratio_for_lines =
        CarParams::GetInstance()->shrink_ratio_for_lines_;

    FootprintModelPtr box_model = std::make_shared<BoxFootprintModel>(
        VehicleParam::Instance(), lat_inflation_, shrink_ratio_for_lines);
    FootprintModelPtr mirror_model = std::make_shared<CircleFootprintModel>(
        VehicleParam::Instance(),
        CarParams::GetInstance()->inflation_rearview_mirror,
        shrink_ratio_for_lines);
    footprint_model_ =
        std::make_shared<CompositeFootprintModel>(
            std::vector<FootprintModelPtr>({box_model, mirror_model}));

    footprint_model_precise_ = std::make_shared<PolygonFootprintModel>(
        VehicleParam::Instance(),
        (EgoModelType)HybridAstarConfig::GetInstance()
            ->footprint_model_precise_,
        lat_inflation_, shrink_ratio_for_lines);
  }

public:
    static FootprintModelPtr footprint_model_;
    static FootprintModelPtr footprint_model_precise_;
};

FootprintModelPtr OpenspaceFootprintTest::footprint_model_;
FootprintModelPtr OpenspaceFootprintTest::footprint_model_precise_;

TEST_F(OpenspaceFootprintTest, composite_footprint_model){
  Pose2D curr_pose(0, 0, 0), next_pose(7, 0, 0);
  footprint_model_->updatePose(curr_pose);

  planning_math::Vec2d pt1(1.0, 0), pt2(7.0, 0.2);
  std::vector<planning_math::Vec2d> pt_vec {pt1, pt2};
  EXPECT_TRUE(footprint_model_->checkOverlap(curr_pose, pt1));
  EXPECT_FALSE(footprint_model_->checkOverlap(curr_pose, pt2));
  EXPECT_TRUE(footprint_model_->checkOverlap(curr_pose, pt_vec));

  planning_math::Vec2d start_pt1(-1.5, 0), end_pt1(0, 1.2);
  planning_math::Vec2d start_pt2(4.0, -3.0), end_pt2(4.0, 3.0);
  planning_math::LineSegment2d line_seg1(start_pt1, end_pt1);
  planning_math::LineSegment2d line_seg2(start_pt2, end_pt2);
  std::vector<planning_math::LineSegment2d> line_seg_vec {line_seg1, line_seg2};
  EXPECT_TRUE(footprint_model_->checkOverlap(curr_pose, line_seg1));
  EXPECT_FALSE(footprint_model_->checkOverlap(curr_pose, line_seg2));
  EXPECT_TRUE(footprint_model_->checkOverlap(curr_pose, line_seg_vec));

  EXPECT_TRUE(footprint_model_->checkTraceOverlap(curr_pose, next_pose, {line_seg2}));

  planning_math::Vec2d center(0.0, 3.0);
  planning_math::Box2d box1(center, 0.0, 5.0, 1.5);
  planning_math::Box2d box2(center, M_PI_2, 5.0, 1.5);
  EXPECT_FALSE(footprint_model_->checkOverlap(curr_pose, box1));
  EXPECT_TRUE(footprint_model_->checkOverlap(curr_pose, box2));
}

TEST_F(OpenspaceFootprintTest, polygon_footprint_model){
  Pose2D curr_pose(0, 0, 0), next_pose(7, 0, 0);
  footprint_model_precise_->updatePose(curr_pose);

  planning_math::Vec2d pt1(1.0, 0), pt2(7.0, 0.2);
  std::vector<planning_math::Vec2d> pt_vec {pt1, pt2};
  EXPECT_TRUE(footprint_model_precise_->checkOverlap(curr_pose, pt1));
  EXPECT_FALSE(footprint_model_precise_->checkOverlap(curr_pose, pt2));
  EXPECT_TRUE(footprint_model_precise_->checkOverlap(curr_pose, pt_vec));

  planning_math::Vec2d start_pt1(-1.5, 0), end_pt1(0, 1.2);
  planning_math::Vec2d start_pt2(4.0, -3.0), end_pt2(4.0, 3.0);
  planning_math::LineSegment2d line_seg1(start_pt1, end_pt1);
  planning_math::LineSegment2d line_seg2(start_pt2, end_pt2);
  std::vector<planning_math::LineSegment2d> line_seg_vec {line_seg1, line_seg2};
  EXPECT_TRUE(footprint_model_precise_->checkOverlap(curr_pose, line_seg1));
  EXPECT_FALSE(footprint_model_precise_->checkOverlap(curr_pose, line_seg2));
  EXPECT_TRUE(footprint_model_precise_->checkOverlap(curr_pose, line_seg_vec));

  EXPECT_FALSE(footprint_model_precise_->checkTraceOverlap(curr_pose, next_pose, {line_seg2}));

  planning_math::Vec2d center(0.0, 3.0);
  planning_math::Box2d box1(center, 0.0, 5.0, 1.5);
  planning_math::Box2d box2(center, M_PI_2, 5.0, 1.5);
  EXPECT_FALSE(footprint_model_precise_->checkOverlap(curr_pose, box1));
  EXPECT_TRUE(footprint_model_precise_->checkOverlap(curr_pose, box2));
}

TEST(OpenspaceFootprintModel, TestCalculateDistance){
    double lat_inflation = msquare::CarParams::GetInstance()->lat_inflation();
    double shrink_ratio_for_lines =  msquare::CarParams::GetInstance()->shrink_ratio_for_lines_;
    std::shared_ptr<msquare::BoxFootprintModel> box_model = std::make_shared<msquare::BoxFootprintModel>(msquare::VehicleParam::Instance(), lat_inflation, shrink_ratio_for_lines);
    std::shared_ptr<msquare::CircleFootprintModel> circle_model = std::make_shared<msquare::CircleFootprintModel>(msquare::VehicleParam::Instance(), msquare::CarParams::GetInstance()->inflation_rearview_mirror, shrink_ratio_for_lines);
    std::shared_ptr<msquare::CompositeFootprintModel> composite_footprint_model = std::make_shared<msquare::CompositeFootprintModel>(std::vector<msquare::FootprintModelPtr>({box_model, circle_model}));
    std::shared_ptr<msquare::PolygonFootprintModel> polygon_model = std::make_shared<PolygonFootprintModel>(
      VehicleParam::Instance(),
      (EgoModelType)HybridAstarConfig::GetInstance()->footprint_model_precise_,
      lat_inflation, shrink_ratio_for_lines);

    std::string json_file = "/home/ros/catkin_ws/src/maf_planning/test/unittest/resources/trajectory.json";
    std::vector<msquare::TrajectoryPoint> traj_ps;
    msquare::parking::OpenspaceDeciderOutput osd;
    readConfig(traj_ps, osd, json_file);

    std::shared_ptr<msquare::SbpObstacleLine> obs_ptr = std::make_shared<msquare::SbpObstacleLine>(osd.obstacle_lines);

    msquare::ObstacleLine obs_line(osd.lines[0],msquare::OpenspaceObstacleType::WALL);


    Pose2D pose1{}, pose2{};

    try{
        box_model->calculateDistance(pose1, &obs_line);
    }catch(std::exception& ex){
        EXPECT_STREQ("BoxFootprintModel::calculateDistance not implemented!", ex.what());
    }
    std::cout<<"3"<<std::endl;

    try{
        composite_footprint_model->calculateDistance(pose1,&obs_line);        
    }catch(std::exception& ex){
        EXPECT_STREQ("BoxFootprintModel::calculateDistance not implemented!", ex.what());
    }

    try{
        polygon_model->calculateDistance(pose1,&obs_line);        
    }catch(std::exception& ex){
        EXPECT_STREQ("PolygonFootprintModel::calculateDistance not implemented!", ex.what());
    }

    EXPECT_NEAR(circle_model->calculateDistance(pose1, &obs_line),3068.0774, 1e-3);
    EXPECT_FALSE(circle_model->checkTraceOverlap(pose1, pose2, osd.obstacle_lines));
}


}