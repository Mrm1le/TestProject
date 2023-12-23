#pragma once
#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"

namespace msquare {

enum class OpenspaceObstacleType : unsigned {
  VIRTUAL = 0,
  BOUNDARY = 1,  // map boundary
  ROADBOARD = 2, //
  PARKINGLINE = 3,
  WALL = 4,
  CAR = 5,
  HUMAN = 6,
  FREESPACE = 7
};

class ObstacleLine : public planning_math::LineSegment2d {
public:
  ObstacleLine(const planning_math::Vec2d &start,
               const planning_math::Vec2d &end,
               const OpenspaceObstacleType obs_type)
      : planning_math::LineSegment2d(start, end), type_(obs_type){};

  ObstacleLine(const planning_math::LineSegment2d &line_obs,
               const OpenspaceObstacleType obs_type)
      : planning_math::LineSegment2d(line_obs), type_(obs_type){};

  ~ObstacleLine(){};
  const OpenspaceObstacleType type() const { return type_; };

  bool is_physical() const {
    return type_ == OpenspaceObstacleType::CAR ||
           type_ == OpenspaceObstacleType::HUMAN ||
           type_ == OpenspaceObstacleType::WALL ||
           type_ == OpenspaceObstacleType::FREESPACE;
  }

private:
  OpenspaceObstacleType type_;
};

class ObstacleBox : public planning_math::Box2d {
public:
  ObstacleBox(const planning_math::Vec2d &center, const double heading,
              const double length, const double width,
              const OpenspaceObstacleType obs_type)
      : planning_math::Box2d(center, heading, length, width), type_(obs_type){};

  ObstacleBox(const planning_math::Box2d &box_obs,
              const OpenspaceObstacleType obs_type)
      : planning_math::Box2d(box_obs), type_(obs_type){};

  ~ObstacleBox(){};
  const OpenspaceObstacleType type() const { return type_; };

private:
  OpenspaceObstacleType type_;
};

} // namespace msquare