#include "planner/behavior_planner/deciders/collision_checker.h"
// #include <iostream>
namespace msquare {

namespace parking {

CollisionChecker::CollisionChecker() { (void)init(); }

bool CollisionChecker::init() {
  cut_length_ = 0.0;
  cut_scaler_ = 1.0;
  deviation_length_ = 0.0;
  check_type_ = CheckType::TRAJECTORY;
  ego_model_ = EgoModelManager();
  return true;
}

bool CollisionChecker::set_params(const double deviation_length,
                                  const double cut_length, const bool reverse) {
  deviation_length_ = deviation_length;
  cut_length_ = cut_length;
  cut_scaler_ = reverse ? -1.0 : 1.0;
  reverse_ = reverse;
  (void)ego_model_.set_deviation_length(deviation_length_)
      .set_cut_length(cut_length_)
      .set_reverse(reverse_);
  // ego_model_type_ = ego_model_type;
  // use_double_box_ = use_double_box;
  // use_polygon_ = use_polygon;
  // use_decagon_ = use_decagon;
  // use_tetradecagon_ = use_tetradecagon;
  // use_rectangle_hexagon_ = use_rectangle_hexagon;

  // if (ego_model_type_ == EgoModelType::POLYGON && (cut_length >
  // VehicleParam::Instance()->length) / 2.0) {
  //   ego_model_type_ == EgoModelType::DOUBLE_BOX;
  // }
  // std::cout <<"modeldebug ego model = "<<int(ego_model_type_)<<std::endl;
  return true;
}

void CollisionChecker::collision_check(const PathPoint &path_point,
                                       const planning_math::Box2d &obs,
                                       const double collision_threshold,
                                       CollisionCheckStatus &result) {
  if (ego_model_.get_ego_model_type() == EgoModelType::PENTAGON) {
    double temp_dis =
        ego_model_.get_ego_model_polygon(EgoModelType::PENTAGON, path_point)
            .DistanceTo(obs);
    if (temp_dis < result.min_distance) {
      result.min_distance = temp_dis;
      if (result.min_distance < collision_threshold) {
        result.is_collision = true;
        return;
      }
    }
  } else {
    double temp_dis =
        ego_model_.get_ego_model_polygon(EgoModelType::ORIGIN, path_point)
            .DistanceTo(obs);
    if (temp_dis < result.min_distance) {
      result.min_distance = temp_dis;
      if (result.min_distance < collision_threshold) {
        result.is_collision = true;
      }
    }
    if (result.is_collision) {
      double second_check_dis =
          ego_model_
              .get_ego_model_polygon(EgoModelType::HEXADECAGON, path_point)
              .DistanceTo(obs);
      if (second_check_dis < result.min_distance) {
        result.min_distance = second_check_dis;
      } else {
        if (second_check_dis > collision_threshold) {
          result.is_collision = false;
          result.min_distance = second_check_dis;
        }
      }
    }
  }
}

void CollisionChecker::collision_check(const PathPoint &path_point,
                                       const planning_math::Polygon2d &obs,
                                       const double collision_threshold,
                                       CollisionCheckStatus &result) {
  double temp_dis =
      ego_model_.get_ego_model_polygon(EgoModelType::ORIGIN, path_point)
          .DistanceTo(obs);

  if (temp_dis < result.min_distance) {
    result.min_distance = temp_dis;
    if (result.min_distance < collision_threshold) {
      result.is_collision = true;
    }
  }
  if (result.is_collision) {
    double second_check_dis =
        ego_model_.get_ego_model_polygon(EgoModelType::HEXADECAGON, path_point)
            .DistanceTo(obs);
    if (second_check_dis < result.min_distance) {
      result.min_distance = second_check_dis;
    } else {
      if (second_check_dis > collision_threshold) {
        result.is_collision = false;
        result.min_distance = second_check_dis;
      }
    }
  }
}

void CollisionChecker::collision_check(const PathPoint &path_point,
                                       const planning_math::LineSegment2d &obs,
                                       const double collision_threshold,
                                       CollisionCheckStatus &result) {
  if (check_type_ == CheckType::TRAJECTORY) {
    double temp_dis = 0;
    if (result.s < 0.6) {
      if (ego_model_.get_ego_model_type() == EgoModelType::WHEEL_BASE) {
        temp_dis =
            ego_model_
                .get_ego_model_polygon(EgoModelType::WHEEL_BASE, path_point)
                .DistanceTo(obs);
      } else {
        temp_dis =
            ego_model_
                .get_ego_model_polygon(EgoModelType::HEXADECAGON, path_point)
                .DistanceTo(obs);
      }
    } else {
      if (ego_model_.get_ego_model_type() == EgoModelType::POLYGON) {
        temp_dis =
            ego_model_.get_ego_model_polygon(EgoModelType::POLYGON, path_point)
                .DistanceTo(obs);

      } else if (ego_model_.get_ego_model_type() == EgoModelType::DECAGON) {
        temp_dis =
            ego_model_.get_ego_model_polygon(EgoModelType::DECAGON, path_point)
                .DistanceTo(obs);

      } else if (ego_model_.get_ego_model_type() == EgoModelType::HEXADECAGON) {
        temp_dis =
            ego_model_
                .get_ego_model_polygon(EgoModelType::HEXADECAGON, path_point)
                .DistanceTo(obs);

      } else if (ego_model_.get_ego_model_type() == EgoModelType::WHEEL_BASE) {
        temp_dis =
            ego_model_
                .get_ego_model_polygon(EgoModelType::WHEEL_BASE, path_point)
                .DistanceTo(obs);

      } else {
        temp_dis =
            ego_model_.get_ego_model_polygon(EgoModelType::ORIGIN, path_point)
                .DistanceTo(obs);
      }
    }

    if (temp_dis < result.min_distance) {
      result.min_distance = temp_dis;
      if (result.min_distance < collision_threshold) {
        result.is_collision = true;
      }
    }
  } else {
    result.min_distance = std::min(
        result.min_distance,
        ego_model_.get_ego_model_polygon(EgoModelType::ORIGIN, path_point)
            .DistanceTo(obs));
    if (result.min_distance < collision_threshold) {
      result.is_collision = true;
    }
  }
}

void CollisionChecker::collision_check(const PathPoint &path_point,
                                       const planning_math::Vec2d &obs,
                                       const double collision_threshold,
                                       CollisionCheckStatus &result) {
  double temp_dis = 100.0;
  if (ego_model_.get_ego_model_type() == EgoModelType::POLYGON) {
    temp_dis =
        ego_model_.get_ego_model_polygon(EgoModelType::POLYGON, path_point)
            .DistanceTo(obs);
  } else if (ego_model_.get_ego_model_type() == EgoModelType::HEXADECAGON) {
    temp_dis =
        ego_model_.get_ego_model_polygon(EgoModelType::HEXADECAGON, path_point)
            .DistanceTo(obs);
  } else {
    temp_dis =
        ego_model_.get_ego_model_polygon(EgoModelType::ORIGIN, path_point)
            .DistanceTo(obs);
  }
  if (temp_dis < result.min_distance) {
    result.min_distance = temp_dis;
    if (result.min_distance < collision_threshold) {
      result.is_collision = true;
    }
  }
}

} // namespace parking

} // namespace msquare
