#pragma once
#include "State.hpp"
#include "common/footprint_model.h"

namespace msquare {

enum class ObstacleHeightType {
  // *****this is only for compatibility ****
  //     OBSTACLES should always be set height type
  // unknown obstacles are treated as HIGH obstacle
  UNKNOWN = 0,

  // lowest obstacle, vehicle may run over it (e.g. speed bump)
  MAY_RUN_OVER = 1,

  // lower than bumper height, wheels are not allowd to run over, but bumpers
  // are allowd (e.g. wheel-stop, curb)
  LOW = 2,

  // lower than rear-view-mirror, wheels, bumpers, are not allowd to collide,
  // but rear-view-mirror can be ignored in collision check
  MID = 3,

  // the default high-obstacle, must check collision of wheels, bumpers, and
  // rear-view-mirror
  HIGH = 4,

  // the high-hanging obstacle (above mirror hight), only upper-body collision
  // check needed
  HANGING_HIGH = 5,

  // TODO: Jinwei: temp impl: map_lines should be considered as low in parallel
  // and be ignored in vertical / oblique
  MAP_LINE = 6,

  HEIGHT_TYPE_NUM
};

class SbpObstacleInterface {
private:
  /* data */
  ObstacleHeightType height_type_ = ObstacleHeightType::UNKNOWN;

public:
  virtual ~SbpObstacleInterface() = default;
  virtual double getCost(const SearchNodePtr &node,
                         const FootprintModelPtr &footpint_model) = 0;
  virtual bool checkCollision(const SearchNodePtr &node,
                              const FootprintModelPtr &footpint_model) = 0;
  virtual double getDistance(const planning_math::Vec2d &point) = 0;
  virtual std::vector<planning_math::Vec2d>
  getNearestPoints(const planning_math::LineSegment2d &ego_centerline) = 0;
  virtual std::vector<planning_math::Vec2d>
  getDiscretePoints(double step) const {
    return std::vector<planning_math::Vec2d>();
  };

  virtual ObstacleHeightType getHeightType() const { return height_type_; }
  virtual void setHeightType(ObstacleHeightType height_type) {
    height_type_ = height_type;
  }
};

typedef std::shared_ptr<SbpObstacleInterface> SbpObstaclePtr;

} // namespace msquare
