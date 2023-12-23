#include <cstdint>

#include "common/parking_slot_factory.h"
#include "common/angle_parking_slot.h"
#include "common/parallel_parking_slot.h"
#include "common/parking_lot.h"
#include "common/planning_context.h"

namespace msquare {
namespace parking {
using namespace planning_math;

ParkingSlotFactory::ParkingSlotFactory(
    const std::map<std::uint8_t, std::string> &cfg_map)
    : cfg_map_(cfg_map) {}

ParkingSlotFactory::~ParkingSlotFactory() {}

std::shared_ptr<ParkingSlotInterface> ParkingSlotFactory::create(
    const ParkingSlotType &type, const std::vector<Point3D> corners,
    const std::vector<Point3D> physical_corners, const Pose2D &initial_pose) {
  std::vector<Vec2d> physical_corners_vec;
  for (const Point3D &pt : physical_corners) {
    physical_corners_vec.push_back(Vec2d(pt.x, pt.y));
  }
  corners_ = corners;

  std::ostringstream slot_debug;
  for(const auto& cpt: corners_){
    slot_debug<<cpt.x<<" "<<cpt.y<<" ";
  }
  slot_debug<<initial_pose.x<<" "<<initial_pose.y<<" "<< initial_pose.theta;
  MSD_LOG(ERROR, "ParkingSlotFactory slot_debug=%s", slot_debug.str().c_str());
  MSD_LOG(ERROR, "initial_pose x = %.3f, y = %.3f theta = %.3f",initial_pose.x, initial_pose.y, initial_pose.theta);

  checkCorners();
  initBox();
  calcRelativeGeometryInfo(initial_pose);
  type_ = type;
  MSD_LOG(ERROR, "ParkingSlotFactory is_relative_left_=%d.", is_relative_left_);

  switch (type_.value) {
  case ParkingSlotType::PERPENDICULAR:
    return std::make_shared<BaseParkingSlot>(cfg_map_.at(type_.value), corners_,
                                             is_relative_left_);
    break;
  case ParkingSlotType::OBLIQUE:
    return std::make_shared<AngleParkingSlot>(cfg_map_.at(type_.value),
                                              corners_, is_relative_left_,
                                              physical_corners_vec);
    break;
  case ParkingSlotType::PARALLEL:
    return std::make_shared<ParallelParkingSlot>(cfg_map_.at(type_.value),
                                                 corners_, physical_corners_vec,
                                                 is_relative_left_);
    break;
  default:
    break;
  }

  return nullptr;
}

void ParkingSlotFactory::calcRelativeGeometryInfo(const Pose2D &initial_pose) {
  Vec2d box_center_local = tf2d(initial_pose, box_.center());
  is_relative_left_ = box_center_local.y() > 0;
}

void ParkingSlotFactory::checkCorners() {
  if (corners_.size() != 4) {
    // throw std::invalid_argument(
    //     "[Parking slot] must construct with 4 cornres.");
  }
  Vec2d edge_0_1(Vec2d(corners_.at(1).x, corners_.at(1).y) -
                 Vec2d(corners_.at(0).x, corners_.at(0).y));
  Vec2d edge_1_2(Vec2d(corners_.at(2).x, corners_.at(2).y) -
                 Vec2d(corners_.at(1).x, corners_.at(1).y));
  if (edge_0_1.CrossProd(edge_1_2) < 0) {
    MSD_LOG(WARN, "slot corners are arranged clockwise!");
  }
}

void ParkingSlotFactory::initBox() {
  Pose2D lot_front_pose;
  lot_front_pose.x = (corners_[0].x + corners_[3].x) / 2;
  lot_front_pose.y = (corners_[0].y + corners_[3].y) / 2;
  Pose2D lot_back_pose;
  lot_back_pose.x = (corners_[1].x + corners_[2].x) / 2;
  lot_back_pose.y = (corners_[1].y + corners_[2].y) / 2;
  Pose2D lot_center_pose;
  lot_center_pose.x = (lot_front_pose.x + lot_back_pose.x) / 2;
  lot_center_pose.y = (lot_front_pose.y + lot_back_pose.y) / 2;
  lot_center_pose.theta = atan2(lot_front_pose.y - lot_back_pose.y,
                                lot_front_pose.x - lot_back_pose.x);
  double lot_length = std::hypot(lot_front_pose.y - lot_back_pose.y,
                                 lot_front_pose.x - lot_back_pose.x);
  double lot_width =
      std::hypot(corners_[0].x - corners_[3].x, corners_[0].y - corners_[3].y);

  box_ = planning_math::Box2d(
      planning_math::Vec2d{lot_center_pose.x, lot_center_pose.y},
      lot_center_pose.theta, lot_length, lot_width);
}

} // namespace parking
} // namespace msquare