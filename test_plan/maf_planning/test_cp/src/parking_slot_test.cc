#include "common/angle_parking_slot.h"
#include "common/parallel_parking_slot.h"
#include "common/parking_lot.h"
#include "common/parking_slot_factory.h"
#include "gtest/gtest.h"

/**
 * by far, nearest corners of slot must be 0 and 3 for vertical slot
 */

namespace {

using namespace msquare::parking;

std::map<LotType, std::string> parking_slot_cfg_map = {
    {LotType::PREPENDICULAR,
     "/home/ros/catkin_ws/src/maf_planning/resource/config/"
     "scenario_configs_json/parking/lot_config.yaml"},
    {LotType::PARALLEL,
     "/home/ros/catkin_ws/src/maf_planning/resource/config/"
     "scenario_configs_json/parking/parallel_parking_slot.yml"}};

TEST(VerticalSlot, corners) {
  /*
  shape:
   |--------
   |
   |--------
  */
  std::vector<double> data_x({3.0, -3.0, -3.0, 3.0});
  std::vector<double> data_y({1.2, 1.2, -1.2, -1.2});
  std::vector<Point3D> corners;
  for (int i = 0; i < 4; ++i) {
    corners.push_back(Point3D{data_x[i], data_y[i], 0});
  }
  std::shared_ptr<ParkingSlotInterface> parking_slot;
  parking_slot = std::make_shared<BaseParkingSlot>(
      parking_slot_cfg_map.at(LotType::PREPENDICULAR), corners, true);

  parking_slot = std::make_shared<BaseParkingSlot>(
      parking_slot_cfg_map.at(LotType::PREPENDICULAR), corners, true);

  double box_heading;
  box_heading = parking_slot->getBox().heading();
  EXPECT_NEAR(box_heading, 0.0, 1e-6);
}

// TEST(ParallelSlot, corners) {
//   /*
//   shape:
//    |        |
//    |        |
//    |--------|
//   */
//   std::vector<double> data_x({-3.0, -3.0, 3.0, 3.0});
//   std::vector<double> data_y({1.2, -1.2, -1.2, 1.2});
//   std::vector<Point3D> corners;
//   std::vector<Point3D> original_corners;
//   for (int i = 0; i < 4; ++i) {
//     corners.push_back(Point3D{data_x[i], data_y[i], 0});
//     original_corners.push_back(Ved2d{data_x[i], data_y[i]})
//   }
//   std::shared_ptr<ParkingSlotInterface> parking_slot;
//   parking_slot = std::make_shared<ParallelParkingSlot>(
//       parking_slot_cfg_map.at(LotType::PARALLEL), original_corners, corners, false);

//   double box_heading;
//   box_heading = parking_slot->getBox().heading();
//   EXPECT_NEAR(box_heading, 0.0, M_PI);
// }

class ParkingSlotFactoryTest : public testing::Test {
protected:
  static std::shared_ptr<ParkingSlotFactory> factory_;
  static void SetUpTestCase() {
    factory_ = std::make_shared<ParkingSlotFactory>(parking_slot_cfg_map);
  }
  static void TearDownTestCase() { factory_ = nullptr; }
};
std::shared_ptr<ParkingSlotFactory> ParkingSlotFactoryTest::factory_ = nullptr;

TEST_F(ParkingSlotFactoryTest, CreateVerticalLeft) {
  std::vector<double> data_x({3.0, -3.0, -3.0, 3.0});
  std::vector<double> data_y({1.2, 1.2, -1.2, -1.2});
  std::vector<Point3D> corners;
  for (int i = 0; i < 4; ++i) {
    corners.push_back(Point3D{data_x[i], data_y[i], 0});
  }

  Pose2D init_pose{5, 0, M_PI_2};

  std::shared_ptr<ParkingSlotInterface> parking_slot =
      factory_->create(corners, corners, init_pose);

  EXPECT_TRUE(parking_slot->isOnLeft());
  Pose2D parking_in_pose =
      parking_slot->getParkingInPose(4.0, 1.5, 0, Pose2D{0, 0, 0});
  EXPECT_GE(parking_in_pose.x, -3.0);
  EXPECT_LE(parking_in_pose.x, 0.0);
}

TEST_F(ParkingSlotFactoryTest, CreateParallelLeft) {
  std::vector<double> data_x({-3.0, -3.0, 3.0, 3.0});
  std::vector<double> data_y({1.2, -1.2, -1.2, 1.2});
  std::vector<Point3D> corners;
  for (int i = 0; i < 4; ++i) {
    corners.push_back(Point3D{data_x[i], data_y[i], 0});
  }

  Pose2D init_pose{0, 5, M_PI};

  std::shared_ptr<ParkingSlotInterface> parking_slot =
      factory_->create(corners, corners, init_pose);

  EXPECT_TRUE(parking_slot->isOnLeft());
  Pose2D parking_in_pose =
      parking_slot->getParkingInPose(4.0, 1.5, 0, Pose2D{0, 0, 0});
  EXPECT_GE(parking_in_pose.x, 0.0);
  EXPECT_LE(parking_in_pose.x, 3.0);
}

TEST_F(ParkingSlotFactoryTest, CreateParallelRight) {
  std::vector<double> data_x({-3.0, -3.0, 3.0, 3.0});
  std::vector<double> data_y({1.2, -1.2, -1.2, 1.2});
  std::vector<Point3D> corners;
  for (int i = 0; i < 4; ++i) {
    corners.push_back(Point3D{data_x[i], data_y[i], 0});
  }

  Pose2D init_pose{0, 5, 0};

  std::shared_ptr<ParkingSlotInterface> parking_slot =
      factory_->create(corners, corners, init_pose);

  EXPECT_TRUE(!parking_slot->isOnLeft());
  Pose2D parking_in_pose =
      parking_slot->getParkingInPose(4.0, 1.5, 0, Pose2D{0, 0, 0});
  EXPECT_GE(parking_in_pose.x, -3.0);
  EXPECT_LE(parking_in_pose.x, 0.0);
}

TEST_F(ParkingSlotFactoryTest, AngleSlotWings) {
  double half_slot_length = 3.0;
  double half_slot_width = 1.2;
  /*
  shape:
   |--------
   |
   |--------
  */
  std::vector<double> data_x({3.0, -3.0, -3.0, 3.0});
  std::vector<double> data_y({1.2, 1.2, -1.2, -1.2});
  /*
  shape:
     /--------
    /
   /--------
  */
  std::vector<double> original_data_x({half_slot_length + half_slot_width,
                                       -half_slot_length + half_slot_width,
                                       -half_slot_length - half_slot_width,
                                       half_slot_length - half_slot_width});
  std::vector<double> original_data_y(
      {half_slot_width, half_slot_width, -half_slot_width, -half_slot_width});
  std::vector<Point3D> corners;
  std::vector<Point3D> original_corners;
  for (int i = 0; i < 4; ++i) {
    corners.push_back(Point3D{data_x[i], data_y[i], 0});
    original_corners.push_back(
        Point3D{original_data_x[i], original_data_y[i], 0});
  }

  Pose2D init_pose{0, 5, M_PI_4};
  std::shared_ptr<ParkingSlotInterface> parking_slot =
      factory_->create(corners, original_corners, init_pose);

  using namespace msquare::planning_math;
  std::vector<LineSegment2d> front_wings = parking_slot->genFrontWings(1.0);
  EXPECT_NEAR(NormalizeAngle(front_wings.front().heading()), M_PI_4, 1e-6);
  EXPECT_NEAR(NormalizeAngle(front_wings.back().heading()), -M_PI * 3 / 4,
              1e-6);

  EXPECT_TRUE(front_wings.front().start() ==
              Vec2d(original_corners[0].x, original_corners[0].y));
  EXPECT_TRUE(front_wings.back().start() ==
              Vec2d(original_corners[3].x, original_corners[3].y));
}

TEST_F(ParkingSlotFactoryTest, AngleSlotBackwardIn) {
  double half_slot_length = 3.0;
  double half_slot_width = 1.2;
  /*
  shape:
   |--------
   |
   |--------
  */
  std::vector<double> data_x({3.0, -3.0, -3.0, 3.0});
  std::vector<double> data_y({1.2, 1.2, -1.2, -1.2});
  /*
  shape:
     /--------
    /
   /--------
  */
  std::vector<double> original_data_x({half_slot_length + half_slot_width,
                                       -half_slot_length + half_slot_width,
                                       -half_slot_length - half_slot_width,
                                       half_slot_length - half_slot_width});
  std::vector<double> original_data_y(
      {half_slot_width, half_slot_width, -half_slot_width, -half_slot_width});
  std::vector<Point3D> corners;
  std::vector<Point3D> original_corners;
  for (int i = 0; i < 4; ++i) {
    corners.push_back(Point3D{data_x[i], data_y[i], 0});
    original_corners.push_back(
        Point3D{original_data_x[i], original_data_y[i], 0});
  }

  Pose2D init_pose{0, 5, M_PI_4};
  std::shared_ptr<ParkingSlotInterface> parking_slot =
      factory_->create(corners, original_corners, init_pose);

  using namespace msquare::planning_math;
  Pose2D target_pose = parking_slot->getParkingInPose(4, 1, 0, init_pose);
  EXPECT_GT(0, target_pose.x);
  EXPECT_NEAR(std::cos(target_pose.theta), 1, 1e-6);
}

TEST_F(ParkingSlotFactoryTest, AngleSlotForwardIn) {
  double half_slot_length = 3.0;
  double half_slot_width = 1.2;
  /*
  shape:
   |--------
   |
   |--------
  */
  std::vector<double> data_x({3.0, -3.0, -3.0, 3.0});
  std::vector<double> data_y({1.2, 1.2, -1.2, -1.2});
  /*
  shape:
     /--------
    /
   /--------
  */
  std::vector<double> original_data_x({half_slot_length + half_slot_width,
                                       -half_slot_length + half_slot_width,
                                       -half_slot_length - half_slot_width,
                                       half_slot_length - half_slot_width});
  std::vector<double> original_data_y(
      {half_slot_width, half_slot_width, -half_slot_width, -half_slot_width});
  std::vector<Point3D> corners;
  std::vector<Point3D> original_corners;
  for (int i = 0; i < 4; ++i) {
    corners.push_back(Point3D{data_x[i], data_y[i], 0});
    original_corners.push_back(
        Point3D{original_data_x[i], original_data_y[i], 0});
  }

  Pose2D init_pose{0, 5, -M_PI * 3 / 4};
  std::shared_ptr<ParkingSlotInterface> parking_slot =
      factory_->create(corners, original_corners, init_pose);

  using namespace msquare::planning_math;
  Pose2D target_pose = parking_slot->getParkingInPose(4, 1, 0, init_pose);
  EXPECT_LT(0, target_pose.x);
  EXPECT_NEAR(std::cos(target_pose.theta), -1, 1e-6);
}

} // namespace