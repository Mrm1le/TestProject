#pragma once

#include <memory>

#include "worldmodel/utility/geometry.hpp"

namespace msd_worldmodel {
namespace worldmodel_v1 {

class CoordinateTransformer {
public:
  // Set ego car ENU position and Euler pose
  virtual void setENUOrigin(const Point3D &point, double roll, double pitch,
                            double yaw) = 0;

  // Set ego car ENU position and quaternion pose
  virtual void setENUOrigin(const Point3D &point, double w, double x, double y,
                            double z) = 0;

  // Transform a ENU position to car position
  virtual Point3D ENUToCar(const Point3D &point) const = 0;

  // Transform a car position to ENU position
  virtual Point3D CarToENU(const Point3D &point) const = 0;

  // Get ENU origin position
  virtual Point3D getENUOrigin() const = 0;

  // Instantiate
  static std::shared_ptr<CoordinateTransformer> make();

  virtual ~CoordinateTransformer() = default;
};

} // namespace worldmodel_v1
} // namespace msd_worldmodel
