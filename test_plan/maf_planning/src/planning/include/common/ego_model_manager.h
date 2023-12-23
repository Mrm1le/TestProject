#ifndef EGO_MODEL_MANAGER_H
#define EGO_MODEL_MANAGER_H

#include "common/config/vehicle_param.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "pnc/define/geometry.h"
#include "pnc/define/path_point.h"

namespace msquare {
enum class EgoModelType : int {
  ORIGIN = 0,
  DOUBLE_BOX = 1,        // wider in front, thinner in rear
  POLYGON = 2,           // same shape as double box using polygon
  DECAGON = 3,           // use trapezoid to replace rounded corner in front
  TETRADECAGON = 4,      // based on decagon, extract mirror
  RECTANGLE_HEXAGON = 5, // same shape as tetradecagon, use two overlapped shape
  HEXAGON = 6,           // rounded front corner
  PENTAGON = 7,          // bullet-shaped
  HEXADECAGON = 8,       // based on retradecagon，cut back corners
  WHEEL_BASE = 9,        // car width, wheel_base length
};

class EgoModelManager {
public:
  EgoModelManager();

  inline EgoModelManager &set_reverse(const bool reverse) {
    reverse_ = reverse;
    return *this;
  }

  inline EgoModelManager &set_deviation_length(const double deviation_length) {
    deviation_length_ = deviation_length;
    return *this;
  }

  inline EgoModelManager &set_cut_length(const double cut_length) {
    cut_length_ = cut_length;
    return *this;
  }

  inline EgoModelManager &set_lat_expansion(const double lat_expansion) {
    lat_expansion_ = lat_expansion;
    return *this;
  }

  inline EgoModelManager &set_lon_expansion(const double lon_expansion) {
    lon_expansion_ = lon_expansion;
    return *this;
  }

  inline EgoModelManager &
  set_fillet_cutting_length(const double fillet_cutting_length) {
    fillet_cutting_length_ = fillet_cutting_length;
    return *this;
  }

  bool set_model_type(EgoModelType ego_model_type); // to delete

  EgoModelType get_ego_model_type();

  const planning_math::Polygon2d &
  get_ego_model_polygon(const EgoModelType ego_model_type,
                        const PathPoint &center_point);

  const double& chassis_height() const {
    return chassis_height_;
  }

private:
  bool set_params(const double &deviation_length, const double &cut_length,
                  const bool &reverse);
  bool set_expansion(double lat_expansion = 0.0,
                     double fillet_cutting_length = 0.0,
                     double lon_expansion = 0.0);
  bool set_origin_model(double lat_expansion = 0.0, double lon_expansion = 0.0);
  // bool set_doublebox_model();
  bool set_polygon_model(double lat_expansion = 0.0,
                         double lon_expansion = 0.0);
  bool set_pentagon_model(double lat_expansion = 0.0,
                          double fillet_cutting_length = 0.0);
  bool set_hexagon_model(double lat_expansion = 0.0,
                         double fillet_cutting_length = 0.15);
  bool set_decagon_model(double lat_expansion = 0.0,
                         double fillet_cutting_length = 0.0);
  bool set_tetradecagon_model(double lat_expansion = 0.0,
                              double fillet_cutting_length = 0.15);
  bool set_hexadecagon_model(double lat_expansion = 0.0,
                             double fillet_cutting_length = 0.0);
  bool set_wheel_base_model(double lat_expansion = 0.0,
                            double lon_expansion = 0.0);
  bool set_geometry_contour_model(
      const bool is_reverse, const EgoModelType &ego_model_type,
      const PathPoint &center_point,
      const std::vector<planning_math::Vec2d> &origin_contour_point);

private:
  EgoModelType ego_model_type_;
  std::vector<planning_math::Vec2d> origin_model_points_;
  std::vector<planning_math::Vec2d> polygon_model_points_;
  std::vector<planning_math::Vec2d> pentagon_model_points_;
  std::vector<planning_math::Vec2d> hexagon_model_points_;
  std::vector<planning_math::Vec2d> decagon_model_points_;
  std::vector<planning_math::Vec2d> tetradecagon_model_points_;
  std::vector<planning_math::Vec2d> hexadecagon_model_points_;
  planning_math::Polygon2d ego_model_polygon_;
  VehicleParam *vehicle_param_ = nullptr;
  double deviation_length_{0};
  double cut_length_{0};
  bool reverse_{false};
  double cut_scaler_;
  double comp_length_;
  double lat_expansion_{0};
  double lon_expansion_{0};
  double fillet_cutting_length_{0};
  double back_light_len_;
  double back_light_height_;
  double chassis_cutting_rolling_length_ = 0.0;
  double chassis_height_ = 0.0;
  PathPoint point_;

  bool init();
};

} // namespace msquare

#endif
