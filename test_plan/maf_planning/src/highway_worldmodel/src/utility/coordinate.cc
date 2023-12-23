#include "worldmodel/utility/coordinate.hpp"

namespace msd_worldmodel {
namespace worldmodel_v1 {

class CoordinateTransformerImpl : public CoordinateTransformer {
public:
  CoordinateTransformerImpl() = default;
  void setENUOrigin(const Point3D &point, double roll, double pitch,
                    double yaw) override {
    enu_origin_ = point;
    rotate_matrix_ = rollPitchYawToMatrix(roll, pitch, yaw).transpose();
  }
  void setENUOrigin(const Point3D &point, double w, double x, double y,
                    double z) override {
    enu_origin_ = point;
    rotate_matrix_ = quaternionToMatrix(w, x, y, z).transpose();
  }
  Point3D ENUToCar(const Point3D &point) const override {
    return rotate_matrix_ * (point - enu_origin_);
  }
  Point3D CarToENU(const Point3D &point) const override {
    return (rotate_matrix_.transpose() * point).eval() + enu_origin_;
  }
  Point3D getENUOrigin() const override { return enu_origin_; }

private:
  Point3D enu_origin_;
  Eigen::Matrix3d rotate_matrix_;
};

std::shared_ptr<CoordinateTransformer> CoordinateTransformer::make() {
  return std::make_shared<CoordinateTransformerImpl>();
}

} // namespace worldmodel_v1
} // namespace msd_worldmodel
