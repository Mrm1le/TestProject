#include "common/ego_model_manager.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
// #include "planning/common/logging.h"
#include "planning/common/statistic.h"
#include <iostream>

namespace msquare {

namespace {
constexpr double kChassisHeight = 0.17;
constexpr double kCutBackLength = 0.78 - 0.24;
} // namespace

EgoModelManager::EgoModelManager() { (void)init(); }

bool EgoModelManager::init() {
  vehicle_param_ = VehicleParam::Instance();
  const auto &rolling_radius = vehicle_param_->wheel_rolling_radius;
  chassis_cutting_rolling_length_ = std::sqrt(
      rolling_radius * rolling_radius -
      (rolling_radius - kChassisHeight) * (rolling_radius - kChassisHeight));
  ego_model_type_ = EgoModelType::ORIGIN;
  origin_model_points_.resize(4);
  polygon_model_points_.resize(8);
  pentagon_model_points_.resize(5);
  hexagon_model_points_.resize(6);
  decagon_model_points_.resize(10);
  tetradecagon_model_points_.resize(14);
  hexadecagon_model_points_.resize(16);
  ego_model_polygon_ = planning_math::Polygon2d();
  (void)set_params(0, 0, false);
  (void)set_expansion(0, 0);
  (void)set_origin_model();
  chassis_height_ = kChassisHeight;
  return true;
}
bool EgoModelManager::set_params(const double &deviation_length,
                                 const double &cut_length,
                                 const bool &reverse) {
  deviation_length_ = deviation_length;
  cut_length_ = cut_length;
  cut_scaler_ = reverse ? -1.0 : 1.0;
  reverse_ = reverse;
  comp_length_ = deviation_length_ + cut_length_ * cut_scaler_ / 2.0;
  back_light_len_ =
      CarParams::GetInstance()->car_config.car_only_config.back_light_len;
  back_light_height_ =
      CarParams::GetInstance()->car_config.car_only_config.back_light_height;

  return true;
}

bool EgoModelManager::set_model_type(EgoModelType ego_model_type) {
  ego_model_type_ = ego_model_type;
  return true;
}

bool EgoModelManager::set_expansion(double lat_expansion,
                                    double fillet_cutting_length,
                                    double lon_expansion) {
  lat_expansion_ = lat_expansion;
  fillet_cutting_length_ = fillet_cutting_length;
  lon_expansion_ = lon_expansion;
  return true;
}

EgoModelType EgoModelManager::get_ego_model_type() { return ego_model_type_; }

const planning_math::Polygon2d &
EgoModelManager::get_ego_model_polygon(const EgoModelType ego_model_type,
                                       const PathPoint &center_point) {
  STAT(EgoModelManager_get_ego_model_polygon);

  ego_model_type_ = ego_model_type;
  point_ = center_point;

  switch (ego_model_type) {
  case EgoModelType::ORIGIN:
    set_origin_model();
    break;
  case EgoModelType::DOUBLE_BOX:
    // set_doublebox_model();
    break;
  case EgoModelType::POLYGON:
    set_polygon_model();
    break;
  case EgoModelType::DECAGON:
    set_decagon_model();
    break;
  case EgoModelType::TETRADECAGON:
    set_tetradecagon_model();
    break;
  case EgoModelType::RECTANGLE_HEXAGON:
    break;
  case EgoModelType::HEXAGON:
    set_hexagon_model();
    break;
  case EgoModelType::PENTAGON:
    set_pentagon_model();
    break;
  case EgoModelType::HEXADECAGON:
    if (VehicleParam::Instance()
            ->geometry_contour_.is_using_geometry_contour()) {
      set_geometry_contour_model(
          reverse_, EgoModelType::HEXADECAGON, center_point,
          VehicleParam::Instance()
              ->geometry_contour_.hexadecagon_contour_point());
    } else {
      set_hexadecagon_model();
    }
    break;
  case EgoModelType::WHEEL_BASE:
    set_wheel_base_model();
    break;
  default:
    ego_model_type_ = EgoModelType::ORIGIN;
    set_origin_model();
    break;
  }
  return ego_model_polygon_;
}

bool EgoModelManager::set_origin_model(double lat_expansion,
                                       double lon_expansion) {
  (void)set_expansion(lat_expansion, 0.0, lon_expansion);
  ego_model_type_ = EgoModelType::ORIGIN;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);
  double center_x = point_.x + point_theta_cos * deviation_length_;
  double center_y = point_.y + point_theta_sin * deviation_length_;
  double dx1 = point_theta_cos * (vehicle_param_->length / 2.0);
  double dy1 = point_theta_sin * (vehicle_param_->length / 2.0);
  double dx2 = point_theta_sin * (vehicle_param_->width / 2.0 + lat_expansion_);
  double dy2 =
      -point_theta_cos * (vehicle_param_->width / 2.0 + lat_expansion_);
  origin_model_points_[0].set_point(
      center_x + dx1 + dx2 + point_theta_cos * lon_expansion_,
      center_y + dy1 + dy2 + point_theta_sin * lon_expansion_);
  origin_model_points_[1].set_point(
      center_x + dx1 - dx2 + point_theta_cos * lon_expansion_,
      center_y + dy1 - dy2 + point_theta_sin * lon_expansion_);
  origin_model_points_[2].set_point(center_x - dx1 - dx2, center_y - dy1 - dy2);
  origin_model_points_[3].set_point(center_x - dx1 + dx2, center_y - dy1 + dy2);
  ego_model_polygon_.set_points(origin_model_points_);

  return true;
}

bool EgoModelManager::set_polygon_model(double lat_expansion,
                                        double lon_expansion) {
  (void)set_expansion(lat_expansion, 0.0, lon_expansion);
  ego_model_type_ = EgoModelType::POLYGON;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);
  if (reverse_) {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0);
    double dx1_front =
        point_theta_cos * (vehicle_param_->length / 4.0 - cut_length_ / 2.0);
    double dy1_front =
        point_theta_sin * (vehicle_param_->length / 4.0 - cut_length_ / 2.0);
    double dx2_front = point_theta_sin * vehicle_param_->width / 2.0;
    double dy2_front = -point_theta_cos * vehicle_param_->width / 2.0;
    double dx1_rear = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_rear = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_rear =
        point_theta_sin * vehicle_param_->width_wo_rearview_mirror / 2.0;
    double dy2_rear =
        -point_theta_cos * vehicle_param_->width_wo_rearview_mirror / 2.0;
    polygon_model_points_[0].set_point(center_x_front - dx1_front + dx2_front,
                                       center_y_front - dy1_front + dy2_front);
    polygon_model_points_[1].set_point(center_x_front + dx1_front + dx2_front,
                                       center_y_front + dy1_front + dy2_front);
    polygon_model_points_[2].set_point(center_x_front + dx1_front - dx2_front,
                                       center_y_front + dy1_front - dy2_front);
    polygon_model_points_[3].set_point(center_x_front - dx1_front - dx2_front,
                                       center_y_front - dy1_front - dy2_front);
    polygon_model_points_[4].set_point(center_x_rear + dx1_rear - dx2_rear,
                                       center_y_rear + dy1_rear - dy2_rear);
    polygon_model_points_[5].set_point(center_x_rear - dx1_rear - dx2_rear,
                                       center_y_rear - dy1_rear - dy2_rear);
    polygon_model_points_[6].set_point(center_x_rear - dx1_rear + dx2_rear,
                                       center_y_rear - dy1_rear + dy2_rear);
    polygon_model_points_[7].set_point(center_x_rear + dx1_rear + dx2_rear,
                                       center_y_rear + dy1_rear + dy2_rear);
  } else {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double dx1_front = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_front = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_front =
        point_theta_sin * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dy2_front =
        -point_theta_cos * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dx1_rear =
        point_theta_cos * (vehicle_param_->length / 4.0 - cut_length_ / 2.0);
    double dy1_rear =
        point_theta_sin * (vehicle_param_->length / 4.0 - cut_length_ / 2.0);
    double dx2_rear =
        point_theta_sin *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    double dy2_rear =
        -point_theta_cos *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    polygon_model_points_[0].set_point(center_x_front - dx1_front + dx2_front,
                                       center_y_front - dy1_front + dy2_front);
    polygon_model_points_[1].set_point(center_x_front + dx1_front + dx2_front +
                                           point_theta_cos * lon_expansion_,
                                       center_y_front + dy1_front + dy2_front +
                                           point_theta_sin * lon_expansion_);
    polygon_model_points_[2].set_point(center_x_front + dx1_front - dx2_front +
                                           point_theta_cos * lon_expansion_,
                                       center_y_front + dy1_front - dy2_front +
                                           point_theta_sin * lon_expansion_);
    polygon_model_points_[3].set_point(center_x_front - dx1_front - dx2_front,
                                       center_y_front - dy1_front - dy2_front);
    polygon_model_points_[4].set_point(center_x_rear + dx1_rear - dx2_rear,
                                       center_y_rear + dy1_rear - dy2_rear);
    polygon_model_points_[5].set_point(center_x_rear - dx1_rear - dx2_rear,
                                       center_y_rear - dy1_rear - dy2_rear);
    polygon_model_points_[6].set_point(center_x_rear - dx1_rear + dx2_rear,
                                       center_y_rear - dy1_rear + dy2_rear);
    polygon_model_points_[7].set_point(center_x_rear + dx1_rear + dx2_rear,
                                       center_y_rear + dy1_rear + dy2_rear);
  }
  ego_model_polygon_.set_points(polygon_model_points_);
  return true;
}

bool EgoModelManager::set_pentagon_model(double lat_expansion,
                                         double fillet_cutting_length) {
  (void)set_expansion(lat_expansion, fillet_cutting_length);
  ego_model_type_ = EgoModelType::PENTAGON;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);
  double center_x = point_.x + point_theta_cos * comp_length_;
  double center_y = point_.y + point_theta_sin * comp_length_;
  double dx1 = point_theta_cos * (vehicle_param_->length / 2.0);
  double dy1 = point_theta_sin * (vehicle_param_->length / 2.0);
  double dx2 =
      point_theta_sin *
      (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
  double dy2 =
      -point_theta_cos *
      (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
  double triangle_a = (vehicle_param_->width_wo_rearview_mirror -
                       vehicle_param_->bumper_length) /
                          2.0 -
                      fillet_cutting_length_;
  double triangle_b =
      vehicle_param_->light_to_front_edge - fillet_cutting_length_;
  double tip_length = (triangle_b * vehicle_param_->width_wo_rearview_mirror) /
                          (2.0 * triangle_a) -
                      triangle_b;
  pentagon_model_points_[0].set_point(center_x - dx1 + dx2,
                                      center_y - dy1 + dy2);
  pentagon_model_points_[1].set_point(
      center_x + dx1 - triangle_b * point_theta_cos + dx2,
      center_y + dy1 - triangle_b * point_theta_sin + dy2);
  pentagon_model_points_[2].set_point(
      center_x + dx1 + tip_length * point_theta_cos,
      center_y + dy1 + tip_length * point_theta_sin);
  pentagon_model_points_[3].set_point(
      center_x + dx1 - triangle_b * point_theta_cos - dx2,
      center_y + dy1 - triangle_b * point_theta_sin - dy2);
  pentagon_model_points_[4].set_point(center_x - dx1 - dx2,
                                      center_y - dy1 - dy2);
  ego_model_polygon_.set_points(pentagon_model_points_);
  return true;
}

bool EgoModelManager::set_hexagon_model(double lat_expansion,
                                        double fillet_cutting_length) {
  (void)set_expansion(lat_expansion, fillet_cutting_length);
  ego_model_type_ = EgoModelType::HEXAGON;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);
  double center_x = point_.x + point_theta_cos * comp_length_;
  double center_y = point_.y + point_theta_sin * comp_length_;
  double dx1 = point_theta_cos * (vehicle_param_->length / 2.0);
  double dy1 = point_theta_sin * (vehicle_param_->length / 2.0);
  double dx2 =
      point_theta_sin *
      (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
  double dy2 =
      -point_theta_cos *
      (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
  double triangle_a = (vehicle_param_->width_wo_rearview_mirror -
                       vehicle_param_->bumper_length) /
                          2.0 -
                      fillet_cutting_length_;
  double triangle_b =
      vehicle_param_->light_to_front_edge - fillet_cutting_length_;
  double triangle_c = std::hypot(triangle_a, triangle_b);
  double mirror_width = 0.11;
  hexagon_model_points_[0].set_point(center_x - dx1 + dx2,
                                     center_y - dy1 + dy2);
  hexagon_model_points_[1].set_point(
      center_x + dx1 - triangle_b * point_theta_cos + dx2,
      center_y + dy1 - triangle_b * point_theta_sin + dy2);
  hexagon_model_points_[2].set_point(
      center_x + dx1 + dx2 - point_theta_sin * (triangle_a),
      center_y + dy1 + dy2 + point_theta_cos * (triangle_a));
  hexagon_model_points_[3].set_point(
      center_x + dx1 - dx2 + point_theta_sin * (triangle_a),
      center_y + dy1 - dy2 - point_theta_cos * (triangle_a));
  hexagon_model_points_[4].set_point(
      center_x + dx1 - triangle_b * point_theta_cos - dx2,
      center_y + dy1 - triangle_b * point_theta_sin - dy2);
  hexagon_model_points_[5].set_point(center_x - dx1 - dx2,
                                     center_y - dy1 - dy2);
  ego_model_polygon_.set_points(hexagon_model_points_);
  return true;
}

bool EgoModelManager::set_decagon_model(double lat_expansion,
                                        double fillet_cutting_length) {
  (void)set_expansion(lat_expansion, fillet_cutting_length);
  ego_model_type_ = EgoModelType::DECAGON;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);
  double triangle_a =
      (vehicle_param_->width - vehicle_param_->bumper_length) / 2.0 -
      fillet_cutting_length_;
  double triangle_b =
      vehicle_param_->light_to_front_edge - fillet_cutting_length_;
  double triangle_c = std::hypot(triangle_a, triangle_b);
  if (reverse_) {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0);
    double dx1_front = point_theta_cos * (vehicle_param_->length / 4.0 +
                                          cut_length_ * cut_scaler_ / 2.0);
    double dy1_front = point_theta_sin * (vehicle_param_->length / 4.0 +
                                          cut_length_ * cut_scaler_ / 2.0);
    double dx2_front = point_theta_sin * vehicle_param_->width / 2.0;
    double dy2_front = -point_theta_cos * vehicle_param_->width / 2.0;
    double dx1_rear = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_rear = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_rear =
        point_theta_sin * vehicle_param_->width_wo_rearview_mirror / 2.0;
    double dy2_rear =
        -point_theta_cos * vehicle_param_->width_wo_rearview_mirror / 2.0;
    decagon_model_points_[0].set_point(center_x_front - dx1_front + dx2_front,
                                       center_y_front - dy1_front + dy2_front);
    decagon_model_points_[1].set_point(
        center_x_front + dx1_front - point_theta_cos * triangle_b + dx2_front,
        center_y_front - point_theta_sin * triangle_b + dy1_front + dy2_front);
    decagon_model_points_[2].set_point(
        center_x_front + dx1_front + dx2_front - point_theta_sin * triangle_a,
        center_y_front + dy1_front + dy2_front - point_theta_cos * triangle_a);
    decagon_model_points_[3].set_point(
        center_x_front + dx1_front - dx2_front + point_theta_sin * triangle_a,
        center_y_front + dy1_front - dy2_front + point_theta_cos * triangle_a);
    decagon_model_points_[4].set_point(
        center_x_front + dx1_front - point_theta_cos * triangle_b - dx2_front,
        center_y_front - point_theta_sin * triangle_b + dy1_front - dy2_front);
    decagon_model_points_[5].set_point(center_x_front - dx1_front - dx2_front,
                                       center_y_front - dy1_front - dy2_front);
    decagon_model_points_[6].set_point(center_x_rear + dx1_rear - dx2_rear,
                                       center_y_rear + dy1_rear - dy2_rear);
    decagon_model_points_[7].set_point(center_x_rear - dx1_rear - dx2_rear,
                                       center_y_rear - dy1_rear - dy2_rear);
    decagon_model_points_[8].set_point(center_x_rear - dx1_rear + dx2_rear,
                                       center_y_rear - dy1_rear + dy2_rear);
    decagon_model_points_[9].set_point(center_x_rear + dx1_rear + dx2_rear,
                                       center_y_rear + dy1_rear + dy2_rear);

  } else {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double dx1_front = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_front = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_front =
        point_theta_sin * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dy2_front =
        -point_theta_cos * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dx1_rear = point_theta_cos * (vehicle_param_->length / 4.0 +
                                         cut_length_ * cut_scaler_ / 2.0);
    double dy1_rear = point_theta_sin * (vehicle_param_->length / 4.0 +
                                         cut_length_ * cut_scaler_ / 2.0);
    double dx2_rear =
        point_theta_sin *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    double dy2_rear =
        -point_theta_cos *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    decagon_model_points_[0].set_point(center_x_front - dx1_front + dx2_front,
                                       center_y_front - dy1_front + dy2_front);
    decagon_model_points_[1].set_point(
        center_x_front + dx1_front - point_theta_cos * triangle_b + dx2_front,
        center_y_front - point_theta_sin * triangle_b + dy1_front + dy2_front);
    decagon_model_points_[2].set_point(
        center_x_front + dx1_front + dx2_front - point_theta_sin * triangle_a,
        center_y_front + dy1_front + dy2_front - point_theta_cos * triangle_a);
    decagon_model_points_[3].set_point(
        center_x_front + dx1_front - dx2_front + point_theta_sin * triangle_a,
        center_y_front + dy1_front - dy2_front + point_theta_cos * triangle_a);
    decagon_model_points_[4].set_point(
        center_x_front + dx1_front - point_theta_cos * triangle_b - dx2_front,
        center_y_front - point_theta_sin * triangle_b + dy1_front - dy2_front);
    decagon_model_points_[5].set_point(center_x_front - dx1_front - dx2_front,
                                       center_y_front - dy1_front - dy2_front);
    decagon_model_points_[6].set_point(center_x_rear + dx1_rear - dx2_rear,
                                       center_y_rear + dy1_rear - dy2_rear);
    decagon_model_points_[7].set_point(center_x_rear - dx1_rear - dx2_rear,
                                       center_y_rear - dy1_rear - dy2_rear);
    decagon_model_points_[8].set_point(center_x_rear - dx1_rear + dx2_rear,
                                       center_y_rear - dy1_rear + dy2_rear);
    decagon_model_points_[9].set_point(center_x_rear + dx1_rear + dx2_rear,
                                       center_y_rear + dy1_rear + dy2_rear);
  }
  ego_model_polygon_.set_points(decagon_model_points_);
  return true;
}

bool EgoModelManager::set_tetradecagon_model(double lat_expansion,
                                             double fillet_cutting_length) {
  (void)set_expansion(lat_expansion, fillet_cutting_length);
  ego_model_type_ = EgoModelType::TETRADECAGON;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);
  double triangle_a = (vehicle_param_->width_wo_rearview_mirror -
                       vehicle_param_->bumper_length) /
                          2.0 -
                      fillet_cutting_length_;
  double triangle_b =
      vehicle_param_->light_to_front_edge - fillet_cutting_length_;
  double triangle_c = std::hypot(triangle_a, triangle_b);
  double mirror_length = 0.3;
  double mirror_width = 0.11;
  if (reverse_) {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0);
    double dx1_front = point_theta_cos * (vehicle_param_->length / 4.0);
    double dy1_front = point_theta_sin * (vehicle_param_->length / 4.0);
    double dx2_front = point_theta_sin * vehicle_param_->width / 2.0;
    double dy2_front = -point_theta_cos * vehicle_param_->width / 2.0;
    double dx1_rear = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_rear = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_rear =
        point_theta_sin * vehicle_param_->width_wo_rearview_mirror / 2.0;
    double dy2_rear =
        -point_theta_cos * vehicle_param_->width_wo_rearview_mirror / 2.0;
    tetradecagon_model_points_[0].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_front);
    tetradecagon_model_points_[1].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front);
    tetradecagon_model_points_[2].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front - point_theta_sin * mirror_width,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front + point_theta_cos * mirror_width);

    tetradecagon_model_points_[3].set_point(
        center_x_front + dx2_front - point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front + dy2_front + point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));
    tetradecagon_model_points_[4].set_point(
        center_x_front + dx1_front + dx2_front -
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front + dy2_front +
            point_theta_cos * (mirror_width + triangle_a));
    tetradecagon_model_points_[5].set_point(
        center_x_front + dx1_front - dx2_front +
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front - dy2_front -
            point_theta_cos * (mirror_width + triangle_a));
    tetradecagon_model_points_[6].set_point(
        center_x_front - dx2_front + point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front - dy2_front - point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));

    tetradecagon_model_points_[7].set_point(
        center_x_front -
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front + point_theta_sin * mirror_width,
        center_y_front -
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front - point_theta_cos * mirror_width);
    tetradecagon_model_points_[8].set_point(
        center_x_front -
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front,
        center_y_front -
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front);
    tetradecagon_model_points_[9].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_front);

    tetradecagon_model_points_[10].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_rear);
    tetradecagon_model_points_[11].set_point(
        center_x_rear - dx1_rear - dx2_rear,
        center_y_rear - dy1_rear - dy2_rear);
    tetradecagon_model_points_[12].set_point(
        center_x_rear - dx1_rear + dx2_rear,
        center_y_rear - dy1_rear + dy2_rear);
    tetradecagon_model_points_[13].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_rear);

  } else {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double dx1_front = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_front = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_front =
        point_theta_sin * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dy2_front =
        -point_theta_cos * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dx1_rear = point_theta_cos * (vehicle_param_->length / 4.0 -
                                         cut_length_ * cut_scaler_ / 2.0);
    double dy1_rear = point_theta_sin * (vehicle_param_->length / 4.0 -
                                         cut_length_ * cut_scaler_ / 2.0);
    double dx2_rear =
        point_theta_sin *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    double dy2_rear =
        -point_theta_cos *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    tetradecagon_model_points_[0].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_front);
    tetradecagon_model_points_[1].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front);
    tetradecagon_model_points_[2].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front - point_theta_sin * mirror_width,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front + point_theta_cos * mirror_width);

    tetradecagon_model_points_[3].set_point(
        center_x_front + dx2_front - point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front + dy2_front + point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));
    tetradecagon_model_points_[4].set_point(
        center_x_front + dx1_front + dx2_front -
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front + dy2_front +
            point_theta_cos * (mirror_width + triangle_a));
    tetradecagon_model_points_[5].set_point(
        center_x_front + dx1_front - dx2_front +
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front - dy2_front -
            point_theta_cos * (mirror_width + triangle_a));
    tetradecagon_model_points_[6].set_point(
        center_x_front - dx2_front + point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front - dy2_front - point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));

    tetradecagon_model_points_[7].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front + point_theta_sin * mirror_width,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front - point_theta_cos * mirror_width);
    tetradecagon_model_points_[8].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front);
    tetradecagon_model_points_[9].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_front);

    tetradecagon_model_points_[10].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_rear);
    tetradecagon_model_points_[11].set_point(
        center_x_rear - dx1_rear - dx2_rear,
        center_y_rear - dy1_rear - dy2_rear);
    tetradecagon_model_points_[12].set_point(
        center_x_rear - dx1_rear + dx2_rear,
        center_y_rear - dy1_rear + dy2_rear);
    tetradecagon_model_points_[13].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_rear);
  }
  ego_model_polygon_.set_points(tetradecagon_model_points_);
  return true;
}

bool EgoModelManager::set_hexadecagon_model(double lat_expansion,
                                            double fillet_cutting_length) {
  (void)set_expansion(lat_expansion, fillet_cutting_length);
  ego_model_type_ = EgoModelType::HEXADECAGON;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);

  double triangle_a = (vehicle_param_->width_wo_rearview_mirror -
                       vehicle_param_->bumper_length) /
                          2.0 -
                      fillet_cutting_length_;
  double triangle_b =
      vehicle_param_->light_to_front_edge - fillet_cutting_length_;
  double triangle_c = std::hypot(triangle_a, triangle_b);
  double mirror_length = 0.10;
  double mirror_width = 0.11;
  double back_triangle_a = back_light_len_;
  double back_triangle_b = back_light_height_;
  if (reverse_) {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0);
    double dx1_front = point_theta_cos * (vehicle_param_->length / 4.0);
    double dy1_front = point_theta_sin * (vehicle_param_->length / 4.0);
    double dx2_front = point_theta_sin * vehicle_param_->width / 2.0;
    double dy2_front = -point_theta_cos * vehicle_param_->width / 2.0;
    double dx1_rear = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_rear = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_rear =
        point_theta_sin * vehicle_param_->width_wo_rearview_mirror / 2.0;
    double dy2_rear =
        -point_theta_cos * vehicle_param_->width_wo_rearview_mirror / 2.0;
    hexadecagon_model_points_[0].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_front);
    hexadecagon_model_points_[1].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front);
    hexadecagon_model_points_[2].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front - point_theta_sin * mirror_width,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front + point_theta_cos * mirror_width);
    hexadecagon_model_points_[3].set_point(
        center_x_front + dx2_front - point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front + dy2_front + point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));
    hexadecagon_model_points_[4].set_point(
        center_x_front + dx1_front + dx2_front -
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front + dy2_front +
            point_theta_cos * (mirror_width + triangle_a));
    hexadecagon_model_points_[5].set_point(
        center_x_front + dx1_front - dx2_front +
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front - dy2_front -
            point_theta_cos * (mirror_width + triangle_a));
    hexadecagon_model_points_[6].set_point(
        center_x_front - dx2_front + point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front - dy2_front - point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));

    hexadecagon_model_points_[7].set_point(
        center_x_front -
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front + point_theta_sin * mirror_width,
        center_y_front -
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front - point_theta_cos * mirror_width);
    hexadecagon_model_points_[8].set_point(
        center_x_front -
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front,
        center_y_front -
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front);
    hexadecagon_model_points_[9].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_front);
    hexadecagon_model_points_[10].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_rear);
    hexadecagon_model_points_[11].set_point(
        center_x_rear -
            point_theta_cos * (vehicle_param_->length / 4.0 - back_triangle_b) -
            dx2_rear,
        center_y_rear -
            point_theta_sin * (vehicle_param_->length / 4.0 - back_triangle_b) -
            dy2_rear);
    hexadecagon_model_points_[12].set_point(
        center_x_rear - dx1_rear -
            point_theta_sin * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a),
        center_y_rear - dy1_rear +
            point_theta_cos * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a));
    hexadecagon_model_points_[13].set_point(
        center_x_rear - dx1_rear +
            point_theta_sin * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a),
        center_y_rear - dy1_rear -
            point_theta_cos * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a));

    hexadecagon_model_points_[14].set_point(
        center_x_rear -
            point_theta_cos * (vehicle_param_->length / 4.0 - back_triangle_b) +
            dx2_rear,
        center_y_rear -
            point_theta_sin * (vehicle_param_->length / 4.0 - back_triangle_b) +
            dy2_rear);

    hexadecagon_model_points_[15].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_rear);
  } else {
    double center_x_front =
        point_.x +
        point_theta_cos * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_y_front =
        point_.y +
        point_theta_sin * (deviation_length_ + vehicle_param_->length / 4.0);
    double center_x_rear =
        point_.x +
        point_theta_cos * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double center_y_rear =
        point_.y +
        point_theta_sin * (deviation_length_ - vehicle_param_->length / 4.0 +
                           cut_length_ * cut_scaler_ / 2.0);
    double dx1_front = point_theta_cos * vehicle_param_->length / 4.0;
    double dy1_front = point_theta_sin * vehicle_param_->length / 4.0;
    double dx2_front =
        point_theta_sin * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dy2_front =
        -point_theta_cos * (vehicle_param_->width / 2.0 + lat_expansion_);
    double dx1_rear = point_theta_cos * (vehicle_param_->length / 4.0 -
                                         cut_length_ * cut_scaler_ / 2.0);
    double dy1_rear = point_theta_sin * (vehicle_param_->length / 4.0 -
                                         cut_length_ * cut_scaler_ / 2.0);
    double dx2_rear =
        point_theta_sin *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    double dy2_rear =
        -point_theta_cos *
        (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion_);
    hexadecagon_model_points_[0].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_front);
    hexadecagon_model_points_[1].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front);
    hexadecagon_model_points_[2].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dx2_front - point_theta_sin * mirror_width,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) +
            dy2_front + point_theta_cos * mirror_width);

    hexadecagon_model_points_[3].set_point(
        center_x_front + dx2_front - point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front + dy2_front + point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));
    hexadecagon_model_points_[4].set_point(
        center_x_front + dx1_front + dx2_front -
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front + dy2_front +
            point_theta_cos * (mirror_width + triangle_a));
    hexadecagon_model_points_[5].set_point(
        center_x_front + dx1_front - dx2_front +
            point_theta_sin * (mirror_width + triangle_a),
        center_y_front + dy1_front - dy2_front -
            point_theta_cos * (mirror_width + triangle_a));
    hexadecagon_model_points_[6].set_point(
        center_x_front - dx2_front + point_theta_sin * mirror_width +
            point_theta_cos * (vehicle_param_->length / 4.0 - triangle_b),
        center_y_front - dy2_front - point_theta_cos * mirror_width +
            point_theta_sin * (vehicle_param_->length / 4.0 - triangle_b));

    hexadecagon_model_points_[7].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front + point_theta_sin * mirror_width,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front - point_theta_cos * mirror_width);
    hexadecagon_model_points_[8].set_point(
        center_x_front +
            point_theta_cos *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dx2_front,
        center_y_front +
            point_theta_sin *
                (vehicle_param_->length / 4.0 -
                 vehicle_param_->front_edge_to_mirror + mirror_length) -
            dy2_front);
    hexadecagon_model_points_[9].set_point(
        center_x_front +
            point_theta_cos * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_front,
        center_y_front +
            point_theta_sin * (vehicle_param_->length / 4.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_front);

    hexadecagon_model_points_[10].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) -
            dy2_rear);
    hexadecagon_model_points_[11].set_point(
        center_x_rear -
            point_theta_cos * (vehicle_param_->length / 4.0 - back_triangle_b) -
            dx2_rear,
        center_y_rear -
            point_theta_sin * (vehicle_param_->length / 4.0 - back_triangle_b) -
            dy2_rear);
    hexadecagon_model_points_[12].set_point(
        center_x_rear - dx1_rear -
            point_theta_sin * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a),
        center_y_rear - dy1_rear +
            point_theta_cos * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a));
    hexadecagon_model_points_[13].set_point(
        center_x_rear - dx1_rear +
            point_theta_sin * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a),
        center_y_rear - dy1_rear -
            point_theta_cos * (vehicle_param_->width_wo_rearview_mirror / 2.0 -
                               back_triangle_a));

    hexadecagon_model_points_[14].set_point(
        center_x_rear -
            point_theta_cos * (vehicle_param_->length / 4.0 - back_triangle_b) +
            dx2_rear,
        center_y_rear -
            point_theta_sin * (vehicle_param_->length / 4.0 - back_triangle_b) +
            dy2_rear);
    hexadecagon_model_points_[15].set_point(
        center_x_rear + dx1_rear +
            point_theta_cos * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dx2_rear,
        center_y_rear + dy1_rear +
            point_theta_sin * (vehicle_param_->length / 2.0 -
                               vehicle_param_->front_edge_to_mirror) +
            dy2_rear);
  }
  ego_model_polygon_.set_points(hexadecagon_model_points_);
  return true;
}

bool EgoModelManager::set_wheel_base_model(double lat_expansion,
                                           double lon_expansion) {
  (void)set_expansion(lat_expansion, 0.0, lon_expansion);
  ego_model_type_ = EgoModelType::WHEEL_BASE;
  double point_theta_sin = sin(point_.theta);
  double point_theta_cos = cos(point_.theta);
  double center_x =
      point_.x + point_theta_cos * (vehicle_param_->wheel_base / 2.0);
  double center_y =
      point_.y + point_theta_sin * (vehicle_param_->wheel_base / 2.0);
  double dx1 = point_theta_cos * (vehicle_param_->wheel_base / 2.0 +
                                  chassis_cutting_rolling_length_);
  double dy1 = point_theta_sin * (vehicle_param_->wheel_base / 2.0 +
                                  chassis_cutting_rolling_length_);
  double dx2 = point_theta_sin *
               (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion);
  double dy2 = -point_theta_cos *
               (vehicle_param_->width_wo_rearview_mirror / 2.0 + lat_expansion);
  origin_model_points_[0].set_point(
      center_x + dx1 + dx2 + point_theta_cos * lon_expansion_,
      center_y + dy1 + dy2 + point_theta_sin * lon_expansion_);
  origin_model_points_[1].set_point(
      center_x + dx1 - dx2 + point_theta_cos * lon_expansion_,
      center_y + dy1 - dy2 + point_theta_sin * lon_expansion_);
  origin_model_points_[2].set_point(center_x - dx1 - dx2, center_y - dy1 - dy2);
  origin_model_points_[3].set_point(center_x - dx1 + dx2, center_y - dy1 + dy2);
  ego_model_polygon_.set_points(origin_model_points_);

  return true;
}

bool EgoModelManager::set_geometry_contour_model(
    const bool is_reverse, const EgoModelType &ego_model_type,
    const PathPoint &center_point,
    const std::vector<planning_math::Vec2d> &origin_contour_point) {
  ego_model_type_ = ego_model_type;
  std::vector<planning_math::Vec2d> model_points;
  for (const auto &pt : origin_contour_point) {
    planning_math::Vec2d origin_pt = pt;
    // cut back when ego reverse
    if (!is_reverse && pt.x() < 0.0) {
      origin_pt.set_x(pt.x() + kCutBackLength);
    }

    planning_math::Vec2d new_pt =
        planning_math::tf2d_inv(center_point, origin_pt);
    model_points.emplace_back(new_pt);
  }
  ego_model_polygon_.set_points(model_points);
  return true;
}

} // namespace msquare
