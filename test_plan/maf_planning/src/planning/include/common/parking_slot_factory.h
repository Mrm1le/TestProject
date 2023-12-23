#pragma once

#include <cstdint>
#include <map>

#include "common/math/math_utils.h"
#include "common/parking_slot_interface.h"
#include "common/planning_context.h"
#include "planning/common/common.h"

namespace msquare {
namespace parking {

class ParkingSlotFactory {
private:
  /**
   * @brief calculate relative heading and side (left/right)
   * with the help of box_
   */
  void calcRelativeGeometryInfo(const Pose2D &initial_pose);

  /**
   * @brief Ensure number of corners equals 4, and reverse rank if
   * corners is distributed clockwise.
   */
  void checkCorners();
  void initBox();

  std::map<std::uint8_t, std::string> cfg_map_;
  bool is_relative_left_;
  std::vector<Point3D> corners_;
  planning_math::Box2d box_;
  ParkingSlotType type_;

public:
  ParkingSlotFactory(const std::map<std::uint8_t, std::string> &cfg_map);
  ~ParkingSlotFactory();

  std::shared_ptr<ParkingSlotInterface>
  create(const ParkingSlotType &type, const std::vector<Point3D> corners,
         const std::vector<Point3D> original_corners,
         const Pose2D &initial_pose);
};

} // namespace parking
} // namespace msquare
