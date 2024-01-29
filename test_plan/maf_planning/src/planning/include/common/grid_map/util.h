#pragma once

namespace msquare {
namespace grid {

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

  HEIGHT_TYPE_NUM
};

} // namespace grid
} // namespace msquare