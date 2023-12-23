#ifndef _APF_PLANNER_H_
#define _APF_PLANNER_H_

#include "common/config/lateral_planning_config.h"
#include "common/math/box2d.h"
#include "common/math/math_utils.h"
#include "common/utils/geometry.h"
#include <common/utils/frenet_coordinate_system.h>
#include <iostream>
#include <memory>
#include <string>

namespace msquare {

struct ObstaclePotentialParam {

  double repulsion_weight_;
  double repulsion_max_distance_;
  double repulsion_2_weight_;
  double repulsion_2_attenuation_;
  ObstaclePotentialParam(double repulsion_weight, double repulsion_max_distance,
                         double repulsion_2_weight,
                         double repulsion_2_attenuation)
      : repulsion_weight_(repulsion_weight),
        repulsion_max_distance_(repulsion_max_distance),
        repulsion_2_weight_(repulsion_2_weight),
        repulsion_2_attenuation_{repulsion_2_attenuation} {};
};

class ApfElementInterface {

private:
  /* data */
public:
  virtual ~ApfElementInterface() = default;
  virtual planning_math::Vec2d
  get_gradient(const planning_math::Box2d &box,
               const planning_math::Vec2d &vec) = 0;
};

typedef std::shared_ptr<ApfElementInterface> ApfElementPtr;

class ApfObstacleLine : public ApfElementInterface {

private:
  ObstaclePotentialParam param_;
  planning_math::LineSegment2d line_;

public:
  ApfObstacleLine(const planning_math::LineSegment2d &line,
                  const ObstaclePotentialParam &param)
      : line_(line), param_(param){};

  virtual planning_math::Vec2d get_gradient(const planning_math::Box2d &box,
                                            const planning_math::Vec2d &vec) {
    double dis = get_distance(box);
    double delta_s = 0.001;
    if (dis < 5 * param_.repulsion_2_attenuation_ && dis > 0.03) {
      planning_math::Box2d box_dx(box);
      planning_math::Box2d box_dy(box);
      box_dx.Shift(planning_math::Vec2d(delta_s, 0));
      box_dy.Shift(planning_math::Vec2d(0, delta_s));
      double u_0 = get_potential(box);
      double u_dx = get_potential(box_dx);
      double u_dy = get_potential(box_dy);
      return planning_math::Vec2d((u_dx - u_0) / delta_s,
                                  (u_dy - u_0) / delta_s);
    }
    return planning_math::Vec2d(0, 0);
  };

private:
  double get_distance(const planning_math::Box2d &box) {
    return box.DistanceTo(line_);
  };
  double get_potential(const planning_math::Box2d &box) {
    double dis = get_distance(box);
    double field = param_.repulsion_2_weight_ *
                   exp(-dis / param_.repulsion_2_attenuation_);
    if (dis < param_.repulsion_max_distance_) {
      if (dis > 0.0) {
        dis = std::max(dis, 1e-5);
      } else {
        dis = std::min(dis, -1e-5);
      }
      if (param_.repulsion_max_distance_ > 0.0) {
        param_.repulsion_max_distance_ =
            std::max(param_.repulsion_max_distance_, 1e-5);
      } else {
        param_.repulsion_max_distance_ =
            std::min(param_.repulsion_max_distance_, -1e-5);
      }

      field += 0.5 * param_.repulsion_weight_ *
               (1 / dis - 1 / param_.repulsion_max_distance_) *
               (1 / dis - 1 / param_.repulsion_max_distance_);
    }
    return field;
  };
};

class ApfObstaclePoint : public ApfElementInterface {

private:
  ObstaclePotentialParam param_;
  planning_math::Vec2d point_;

public:
  ApfObstaclePoint(const planning_math::Vec2d &point,
                   const ObstaclePotentialParam &param)
      : point_(point), param_(param){};

  virtual planning_math::Vec2d get_gradient(const planning_math::Box2d &box,
                                            const planning_math::Vec2d &vec) {
    double dis = get_distance(box);
    double delta_s = 0.001;
    if (dis < 5 * param_.repulsion_2_attenuation_ && dis > 0.03) {
      planning_math::Box2d box_dx(box);
      planning_math::Box2d box_dy(box);
      box_dx.Shift(planning_math::Vec2d(delta_s, 0));
      box_dy.Shift(planning_math::Vec2d(0, delta_s));
      double u_0 = get_potential(box);
      double u_dx = get_potential(box_dx);
      double u_dy = get_potential(box_dy);
      return planning_math::Vec2d((u_dx - u_0) / delta_s,
                                  (u_dy - u_0) / delta_s);
    }
    return planning_math::Vec2d(0, 0);
  };

private:
  double get_potential(const planning_math::Box2d &box) {
    double dis = get_distance(box);
    double field = param_.repulsion_2_weight_ *
                   exp(-dis / param_.repulsion_2_attenuation_);
    if (dis < param_.repulsion_max_distance_) {
      if (dis > 0.0) {
        dis = std::max(dis, 1e-5);
      } else {
        dis = std::min(dis, -1e-5);
      }
      if (param_.repulsion_max_distance_ > 0.0) {
        param_.repulsion_max_distance_ =
            std::max(param_.repulsion_max_distance_, 1e-5);
      } else {
        param_.repulsion_max_distance_ =
            std::min(param_.repulsion_max_distance_, -1e-5);
      }

      field += 0.5 * param_.repulsion_weight_ *
               (1 / dis - 1 / param_.repulsion_max_distance_) *
               (1 / dis - 1 / param_.repulsion_max_distance_);
    }
    return field;
  };
  double get_distance(const planning_math::Box2d &box) {
    return box.DistanceTo(point_);
  }
};

class ApfDirectionalField : public ApfElementInterface {

private:
  double theta_;
  double intensity_;
  planning_math::Vec2d origin_;
  planning_math::Vec2d gradient_;

public:
  ApfDirectionalField(const double &intensity, const double theta,
                      const planning_math::Vec2d &origin)
      : intensity_(intensity), theta_(theta), origin_(origin) {
    gradient_ = planning_math::Vec2d(cos(theta_) * intensity_,
                                     sin(theta_) * intensity_);
  };
  virtual planning_math::Vec2d get_gradient(const planning_math::Box2d &box,
                                            const planning_math::Vec2d &vec) {
    return gradient_;
  };

private:
  double get_potential(const planning_math::Box2d &box) {
    return (box.center() - origin_).InnerProd(gradient_);
  };
};

class ApfFrenetField : public ApfElementInterface {

private:
  double s_intensity_;
  double l_intensity_;
  std::shared_ptr<FrenetCoordinateSystem> coord_;

public:
  ApfFrenetField(const double &s_intensity, const double l_intensity,
                 std::shared_ptr<FrenetCoordinateSystem> coord)
      : s_intensity_(s_intensity), l_intensity_(l_intensity), coord_(coord){};

  virtual planning_math::Vec2d get_gradient(const planning_math::Box2d &box,
                                            const planning_math::Vec2d &vec) {
    Point2D cart_point = {vec.x(), vec.y()}, frenet_point;
    if (coord_->CartCoord2FrenetCoord(cart_point, frenet_point) ==
        TRANSFORM_STATUS::TRANSFORM_FAILED) {
      Point2D start_point = coord_->GetRefCurvePoint(0.0);
      Point2D end_point =
          coord_->GetRefCurvePoint(std::max(coord_->GetLength() - 0.01, 0.0));
      double ref_theta;
      if (distance(cart_point, start_point) < distance(cart_point, end_point)) {
        ref_theta = coord_->GetRefCurveHeading(0.0);
      } else {
        ref_theta = coord_->GetRefCurveHeading(
            std::max(coord_->GetLength() - 0.01, 0.0));
      }
      double gradient_x = s_intensity_ * cos(ref_theta + M_PI) +
                          l_intensity_ * cos(ref_theta + M_PI + M_PI_2);
      double gradient_y = s_intensity_ * sin(ref_theta + M_PI) +
                          l_intensity_ * sin(ref_theta + M_PI + M_PI_2);
      return planning_math::Vec2d(gradient_x, gradient_y);
    } else {
      double ref_theta = coord_->GetRefCurveHeading(frenet_point.x);
      double gradient_x = s_intensity_ * cos(ref_theta + M_PI) +
                          l_intensity_ * cos(ref_theta + M_PI + M_PI_2);
      double gradient_y = s_intensity_ * sin(ref_theta + M_PI) +
                          l_intensity_ * sin(ref_theta + M_PI + M_PI_2);
      return planning_math::Vec2d(gradient_x, gradient_y);
    }
    return planning_math::Vec2d(0.0, 0.0);
  };

private:
  double get_potential(const planning_math::Box2d &box) { return 0.0; };
  double distance(Point2D point1, Point2D point2) {
    return planning_math::Vec2d(point2.x - point1.x, point2.y - point1.y)
        .Length();
  }
};

class ApfPlannerConfig {

public:
  double step_size_;
  double center_to_geometry_center_;
  double vehicle_length_;
  double vehicle_width_;
  double min_turning_radius_;
  double max_dtheta_;
  double stop_gradient_value_;
  ApfPlannerConfig() {
    step_size_ = 2.0;
    center_to_geometry_center_ = 1.5;
    vehicle_length_ = 5.0;
    vehicle_width_ = 2.0;
    min_turning_radius_ = 10.0;
    max_dtheta_ = step_size_ / min_turning_radius_;
    stop_gradient_value_ = 300.0;
  };
  ApfPlannerConfig(const ApfDeciderConfig *cfg) {
    step_size_ = cfg->gradient_descent_distance_;
    center_to_geometry_center_ = cfg->vehicle_length_added_;
    vehicle_length_ = cfg->vehicle_length_;
    vehicle_width_ = cfg->vehicle_width_;
    min_turning_radius_ = cfg->min_turning_radius_;
    stop_gradient_value_ = cfg->stop_gradient_value_;
  };
  void load_config(const ApfDeciderConfig *cfg) {
    step_size_ = cfg->gradient_descent_distance_;
    center_to_geometry_center_ = cfg->vehicle_length_added_;
    vehicle_length_ = cfg->vehicle_length_;
    vehicle_width_ = cfg->vehicle_width_;
    min_turning_radius_ = cfg->min_turning_radius_;
    stop_gradient_value_ = cfg->stop_gradient_value_;
  };
};

typedef std::shared_ptr<ApfPlannerConfig> ApfPlannerConfigPtr;

struct ApfPoint {
  Pose2D pose;
  Point2D gradient;
  double gradient_value;
  double gradient_theta;
  ApfPoint(Pose2D a, Point2D b) : pose(a), gradient(b) {
    gradient_theta = std::atan2(b.y, b.x);
    gradient_value = std::sqrt(b.x * b.x + b.y * b.y);
  }
  ApfPoint(){};
};
typedef std::vector<ApfPoint> ApfTraj;

class ApfPlanner {

public:
  ApfPlanner(ApfPlannerConfigPtr cfg, double max_traj_length = 50.0,
             double min_traj_length = 20.0);
  bool plan();
  void set_obstacles(std::vector<ApfElementPtr> obstacles) {
    obstacles_ = obstacles;
  };
  void set_traj_length(double max_traj_length, double min_traj_length) {
    max_point_num_ = (int)(max_traj_length / cfg_->step_size_);
    min_point_num_ = (int)(min_traj_length / cfg_->step_size_);
  }
  void set_origin(Pose2D origin) { origin_ = origin; }
  const std::vector<ApfPoint> &get_result() { return result_; };

private:
  planning_math::Vec2d create_gradient(const Pose2D &current_pose);
  bool create_new_pose(Pose2D &current_pose, const ApfPoint &apf_point);

private:
  ApfPlannerConfigPtr cfg_;
  Pose2D origin_;
  int max_point_num_;
  int min_point_num_;
  std::vector<ApfElementPtr> obstacles_;
  ApfTraj result_;
};

} // namespace msquare

#endif
