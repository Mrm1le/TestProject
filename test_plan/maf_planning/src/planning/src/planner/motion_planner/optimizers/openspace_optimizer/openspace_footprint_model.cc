#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_footprint_model.h"
#include "common/math/math_utils.h"
#include <iostream>
#include <stdexcept>

namespace msquare {

using namespace planning_math;

inline Box2d getWheelSafeBox(const Pose2D &current_pose, double wheel_base,
                             double wheel_rolling_radius, double width) {
  Vec2d local_front_center(wheel_base + wheel_rolling_radius, 0);
  Vec2d local_rear_center(-wheel_rolling_radius, 0);
  LineSegment2d local_axis(local_rear_center, local_front_center);
  return Box2d(tf2d_inv(current_pose, local_axis), width);
}

BoxFootprintModel::BoxFootprintModel(const VehicleParam *vehicle_param,
                                     const double lat_inflation,
                                     const double shrink_ratio_for_lines)
    : vehicle_param_(vehicle_param), lat_inflation_(lat_inflation),
      shrink_ratio_for_lines_(shrink_ratio_for_lines),
      inflation_for_points_(
          std::max(HybridAstarConfig::GetInstance()->inflation_for_points_ +
                       kMathEpsilon,
                   kMathEpsilon)) {}

BoxFootprintModel::~BoxFootprintModel() {}

double BoxFootprintModel::calculateDistance(const Pose2D &current_pose,
                                            const ObstacleLine *obstacle) {
  // throw std::runtime_error(
  //     "BoxFootprintModel::calculateDistance not implemented!");
  return 1e19;
}

void BoxFootprintModel::updatePose(const Pose2D &current_pose) {
  center_ = getCenter(current_pose);
  geometry_cache_.pose = current_pose;
  geometry_cache_.box =
      Box2d(center_, current_pose.theta, vehicle_param_->length,
            vehicle_param_->width_wo_rearview_mirror + 2 * lat_inflation_);
  geometry_cache_.box_shrinked = getWheelSafeBox(
      current_pose, vehicle_param_->wheel_base,
      vehicle_param_->wheel_rolling_radius,
      vehicle_param_->width_wo_rearview_mirror * shrink_ratio_for_lines_);

  initMaxMmin();
}

void BoxFootprintModel::initMaxMmin() {
  max_x_ = geometry_cache_.box.max_x();
  min_x_ = geometry_cache_.box.min_x();
  max_y_ = geometry_cache_.box.max_y();
  min_y_ = geometry_cache_.box.min_y();
}

bool BoxFootprintModel::checkOverlap(
    const Pose2D &current_pose, const planning_math::LineSegment2d &obstacle,
    bool is_virtual) {
  if (is_virtual && shrink_ratio_for_lines_ < 1e-6) {
    return false;
  }

  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Box2d &box = is_virtual ? geometry_cache_.box_shrinked : geometry_cache_.box;
  return box.HasOverlap(obstacle);
}

bool BoxFootprintModel::checkOverlap(const Pose2D &current_pose,
                                     const planning_math::Vec2d &obstacle,
                                     bool is_virtual) {
  if (is_virtual && shrink_ratio_for_lines_ < 1e-6) {
    return false;
  }

  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Box2d &box = is_virtual ? geometry_cache_.box_shrinked : geometry_cache_.box;

  return box.DistanceTo(obstacle) < inflation_for_points_;
}

bool BoxFootprintModel::checkOverlap(const Pose2D &current_pose,
                                     const planning_math::Box2d &box) {
  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }
  return geometry_cache_.box.HasOverlap(box);
}

bool BoxFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles,
    bool is_virtual) {
  if (is_virtual && shrink_ratio_for_lines_ < 1e-6) {
    return false;
  }

  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Box2d &box = is_virtual ? geometry_cache_.box_shrinked : geometry_cache_.box;

  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (box.HasOverlap(obstacles.at(i))) {
      return true;
    }
  }

  return false;
}

bool BoxFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::Vec2d> &obstacles, bool is_virtual) {
  if (is_virtual && shrink_ratio_for_lines_ < 1e-6) {
    return false;
  }

  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Box2d &box = is_virtual ? geometry_cache_.box_shrinked : geometry_cache_.box;

  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (box.DistanceTo(obstacles.at(i)) < inflation_for_points_) {
      return true;
    }
  }

  return false;
}

bool BoxFootprintModel::checkTraceOverlap(
    const Pose2D &current_pose, const Pose2D &next_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles) {
  static bool first_called = true;
  if (next_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(next_pose);
  }

  Box2d box =
      Box2d(getCenter(current_pose), current_pose.theta, vehicle_param_->length,
            vehicle_param_->width_wo_rearview_mirror + 2 * lat_inflation_);
  Box2d &box_next = geometry_cache_.box;
  std::vector<Vec2d> box_corners;
  box.GetAllCorners(&box_corners);
  std::vector<Vec2d> box_next_corners;
  box_next.GetAllCorners(&box_next_corners);

  int key_trace_idx = 0;
  Vec2d move_vec(next_pose.x - current_pose.x, next_pose.y - current_pose.y);
  Vec2d heading_vec = Vec2d::CreateUnitVec2d(current_pose.theta);
  double cross_prod = heading_vec.CrossProd(move_vec);
  double inner_prod = heading_vec.InnerProd(move_vec);
  if (inner_prod > 0 && cross_prod < 0) {
    key_trace_idx = 1;
  }
  if (inner_prod > 0 && cross_prod > 0) {
    key_trace_idx = 0;
  }
  if (inner_prod < 0 && cross_prod > 0) {
    key_trace_idx = 3;
  }
  if (inner_prod < 0 && cross_prod < 0) {
    key_trace_idx = 2;
  }
  LineSegment2d trace(box_corners.at(key_trace_idx),
                      box_next_corners.at(key_trace_idx));

  for (const LineSegment2d &line : obstacles) {
    if (trace.HasIntersect(line)) {
      return true;
    }
  }
  // std::cout<< "BoxFootprintModel::checkTraceOverlap false\n";
  return false;
}

PolygonFootprintModel::PolygonFootprintModel(
    const VehicleParam *vehicle_param, EgoModelType type,
    const double lat_inflation, const double shrink_ratio_for_lines)
    : vehicle_param_(vehicle_param), ego_model_type_(type), lat_inflation_(lat_inflation),
      shrink_ratio_for_lines_(shrink_ratio_for_lines),
      inflation_for_points_(
          std::max(HybridAstarConfig::GetInstance()->inflation_for_points_ +
                       kMathEpsilon,
                   kMathEpsilon)) {
  (void)ego_model_manager_.set_lat_expansion(lat_inflation_);
}

PolygonFootprintModel::~PolygonFootprintModel() {}

double PolygonFootprintModel::calculateDistance(const Pose2D &current_pose,
                                                const ObstacleLine *obstacle) {
  // throw std::runtime_error(
  //     "PolygonFootprintModel::calculateDistance not implemented!");
  return 1e19;
}

void PolygonFootprintModel::initMaxMmin() {
  max_x_ = geometry_cache_.polygon.max_x();
  min_x_ = geometry_cache_.polygon.min_x();
  max_y_ = geometry_cache_.polygon.max_y();
  min_y_ = geometry_cache_.polygon.min_y();
}

void PolygonFootprintModel::updatePose(const Pose2D &current_pose) {
  planning_math::Vec2d center = getCenter(current_pose);
  PathPoint center_pp(center.x(), center.y(), 0, current_pose.theta);
  geometry_cache_.pose = current_pose;
  geometry_cache_.polygon = ego_model_manager_.get_ego_model_polygon(ego_model_type_, center_pp);
  geometry_cache_.box_shrinked = getWheelSafeBox(
      current_pose, vehicle_param_->wheel_base,
      vehicle_param_->wheel_rolling_radius,
      vehicle_param_->width_wo_rearview_mirror * shrink_ratio_for_lines_);

  initMaxMmin();
}

bool PolygonFootprintModel::checkOverlap(
    const Pose2D &current_pose, const planning_math::LineSegment2d &obstacle,
    bool is_virtual) {

  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Polygon2d &polygon = geometry_cache_.polygon;
  Box2d &box_shrinked = geometry_cache_.box_shrinked;

  if (!is_virtual) {
    return polygon.HasOverlap(obstacle);
  } else {
    if (shrink_ratio_for_lines_ < 1e-6) {
      return false;
    } else {
      return box_shrinked.HasOverlap(obstacle);
    }
  }
}

bool PolygonFootprintModel::checkOverlap(const Pose2D &current_pose,
                                         const planning_math::Vec2d &obstacle,
                                         bool is_virtual) {
  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Polygon2d &polygon = geometry_cache_.polygon;
  Box2d &box_shrinked = geometry_cache_.box_shrinked;

  if (!is_virtual) {
    return polygon.DistanceTo(obstacle) < inflation_for_points_;
  } else {
    if (shrink_ratio_for_lines_ < 1e-6) {
      return false;
    } else {
      return box_shrinked.DistanceTo(obstacle) < inflation_for_points_;
    }
  }
}

bool PolygonFootprintModel::checkOverlap(const Pose2D &current_pose,
                                         const planning_math::Box2d &box) {
  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Polygon2d &polygon = geometry_cache_.polygon;

  return polygon.HasOverlap(Polygon2d(box));
}

bool PolygonFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles,
    bool is_virtual) {

  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Polygon2d &polygon = geometry_cache_.polygon;
  Box2d &box_shrinked = geometry_cache_.box_shrinked;

  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (!is_virtual) {
      if (polygon.HasOverlap(obstacles.at(i))) {
        return true;
      }
    } else {
      if (shrink_ratio_for_lines_ < 1e-6) {
        continue;
      }
      if (box_shrinked.HasOverlap(obstacles.at(i))) {
        return true;
      }
    }
  }

  return false;
}

bool PolygonFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::Vec2d> &obstacles, bool is_virtual) {
  static bool first_called = true;
  if (current_pose != geometry_cache_.pose || first_called) {
    first_called = false;
    updatePose(current_pose);
  }

  Polygon2d &polygon = geometry_cache_.polygon;
  Box2d &box_shrinked = geometry_cache_.box_shrinked;

  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (!is_virtual) {
      if (polygon.DistanceTo(obstacles.at(i)) < inflation_for_points_) {
        return true;
      }
    } else {
      if (shrink_ratio_for_lines_ < 1e-6) {
        continue;
      }
      if (box_shrinked.DistanceTo(obstacles.at(i)) < inflation_for_points_) {
        return true;
      }
    }
  }

  return false;
}

bool PolygonFootprintModel::checkTraceOverlap(
    const Pose2D &current_pose, const Pose2D &next_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles) {
  return false;
}

CircleFootprintModel::CircleFootprintModel(const VehicleParam *vehicle_param,
                                           double inflation,
                                           double shrink_ratio)
    : vehicle_param_(vehicle_param), inflation_(inflation),
      shrink_ratio_(shrink_ratio),
      inflation_for_points_(
          std::max(HybridAstarConfig::GetInstance()->inflation_for_points_ +
                       kMathEpsilon,
                   kMathEpsilon)) {
  if (shrink_ratio < 0 || shrink_ratio > 1.0) {
    // throw std::invalid_argument(
    //     "CircleFootprintModel:: shrink_ratio out of [0,1]");
  }
}

CircleFootprintModel::~CircleFootprintModel() {}

double CircleFootprintModel::calculateDistance(const Pose2D &current_pose,
                                               const ObstacleLine *obstacle) {
  return obstacle->DistanceTo(getCenter(current_pose)) -
         vehicle_param_->width / 2;
}

void CircleFootprintModel::initMaxMmin(const Pose2D &current_pose) {
  double radius = vehicle_param_->width / 2 + inflation_;
  max_x_ = center_.x() + radius;
  min_x_ = center_.x() - radius;
  max_y_ = center_.y() + radius;
  min_y_ = center_.y() - radius;
}

void CircleFootprintModel::updatePose(const Pose2D &current_pose) {
  center_ = getCenter(current_pose);
  initMaxMmin(current_pose);
}

bool CircleFootprintModel::checkOverlap(
    const Pose2D &current_pose, const planning_math::LineSegment2d &obstacle,
    bool is_virtual) {
  double radius = vehicle_param_->width / 2 + inflation_;
  if (is_virtual) {
    return false;
  }

  if (current_pose != cache_pose_) {
    updatePose(current_pose);
  }

  return obstacle.DistanceTo(center_) <= radius;
}

bool CircleFootprintModel::checkOverlap(const Pose2D &current_pose,
                                        const planning_math::Vec2d &obstacle,
                                        bool is_virtual) {
  double radius = vehicle_param_->width / 2;
  if (!is_virtual) {
    radius += inflation_;
  } else {
    radius *= shrink_ratio_;
  }

  if (current_pose != cache_pose_) {
    updatePose(current_pose);
  }

  return obstacle.DistanceTo(center_) <= radius + inflation_for_points_;
}

bool CircleFootprintModel::checkOverlap(const Pose2D &current_pose,
                                        const planning_math::Box2d &box) {

  if (current_pose != cache_pose_) {
    cache_pose_ = current_pose;
    updatePose(current_pose);
  }
  double radius = vehicle_param_->width / 2;

  return box.DistanceTo(center_) <= radius;
}

bool CircleFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles,
    bool is_virtual) {
  double radius = vehicle_param_->width / 2;
  if (!is_virtual) {
    radius += inflation_;
  } else {
    radius *= shrink_ratio_;
  }

  if (current_pose != cache_pose_) {
    updatePose(current_pose);
  }

  for (const LineSegment2d &line : obstacles) {
    if (line.min_x() > center_.x() + radius)
      continue;
    if (line.max_x() < center_.x() - radius)
      continue;
    if (line.min_y() > center_.y() + radius)
      continue;
    if (line.max_y() < center_.y() - radius)
      continue;

    if (line.DistanceTo(center_) <= radius) {
      return true;
    }
  }

  return false;
}

bool CircleFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::Vec2d> &obstacles, bool is_virtual) {
  double radius = vehicle_param_->width / 2;
  if (!is_virtual) {
    radius += inflation_ + inflation_for_points_;
  } else {
    radius *= shrink_ratio_;
    radius += inflation_for_points_;
  }

  if (current_pose != cache_pose_) {
    updatePose(current_pose);
  }

  double radius_square = radius * radius + 1e-3;

  for (const Vec2d &point : obstacles) {
    if (point.x() < center_.x() - radius)
      continue;
    if (point.x() > center_.x() + radius)
      continue;
    if (point.y() < center_.y() - radius)
      continue;
    if (point.y() > center_.y() + radius)
      continue;

    if ((pow(point.x() - center_.x(), 2) + pow(point.y() - center_.y(), 2)) <
        radius_square) {
      return true;
    }
  }

  return false;
}
bool CircleFootprintModel::checkTraceOverlap(
    const Pose2D &current_pose, const Pose2D &next_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles) {
  return false;
}

CompositeFootprintModel::CompositeFootprintModel(
    const std::vector<FootprintModelPtr> &models)
    : models_(models) {}

CompositeFootprintModel::~CompositeFootprintModel() {}

double
CompositeFootprintModel::calculateDistance(const Pose2D &current_pose,
                                           const ObstacleLine *obstacle) {
  double min_dis = std::numeric_limits<double>::max();
  for (FootprintModelPtr &model : models_) {
    min_dis =
        std::min(min_dis, model->calculateDistance(current_pose, obstacle));
  }

  return min_dis;
}
void CompositeFootprintModel::updatePose(const Pose2D &current_pose) {
  for (FootprintModelPtr &model : models_) {
    model->updatePose(current_pose);
  }
  initMaxMmin();
}

void CompositeFootprintModel::initMaxMmin() {
  max_x_ = std::numeric_limits<double>::lowest();
  min_x_ = std::numeric_limits<double>::max();
  max_y_ = std::numeric_limits<double>::lowest();
  min_y_ = std::numeric_limits<double>::max();
  for (FootprintModelPtr &model : models_) {
    max_x_ = std::max(model->max_x(), max_x_);
    min_x_ = std::min(model->min_x(), min_x_);
    max_y_ = std::max(model->max_y(), max_y_);
    min_y_ = std::min(model->min_y(), min_y_);
  }
}

bool CompositeFootprintModel::checkOverlap(
    const Pose2D &current_pose, const planning_math::LineSegment2d &obstacle,
    bool is_virtual) {
  for (FootprintModelPtr &model : models_) {
    if (model->checkOverlap(current_pose, obstacle, is_virtual)) {
      return true;
    }
  }

  return false;
}

bool CompositeFootprintModel::checkOverlap(const Pose2D &current_pose,
                                           const planning_math::Vec2d &obstacle,
                                           bool is_virtual) {
  for (FootprintModelPtr &model : models_) {
    if (model->checkOverlap(current_pose, obstacle, is_virtual)) {
      return true;
    }
  }

  return false;
}

bool CompositeFootprintModel::checkOverlap(const Pose2D &current_pose,
                                           const planning_math::Box2d &box) {
  for (FootprintModelPtr &model : models_) {
    if (model->checkOverlap(current_pose, box)) {
      return true;
    }
  }

  return false;
}

bool CompositeFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles,
    bool is_virtual) {
  for (FootprintModelPtr &model : models_) {
    if (model->checkOverlap(current_pose, obstacles, is_virtual)) {
      return true;
    }
  }

  return false;
}

bool CompositeFootprintModel::checkOverlap(
    const Pose2D &current_pose,
    const std::vector<planning_math::Vec2d> &obstacles, bool is_virtual) {
  for (FootprintModelPtr &model : models_) {
    if (model->checkOverlap(current_pose, obstacles, is_virtual)) {
      return true;
    }
  }

  return false;
}

bool CompositeFootprintModel::checkTraceOverlap(
    const Pose2D &current_pose, const Pose2D &next_pose,
    const std::vector<planning_math::LineSegment2d> &obstacles) {
  for (FootprintModelPtr &model : models_) {
    if (model->checkTraceOverlap(current_pose, next_pose, obstacles)) {
      return true;
    }
  }

  return false;
}

} // namespace msquare
