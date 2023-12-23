#ifndef _PLANNING_COMMON_TYPE_H
#define _PLANNING_COMMON_TYPE_H

#include <stdint.h>

namespace msquare {

struct RefPointFrenet {
  double s;
  double lane_width = 3.2;
  double left_lane_border = 1.6;
  double right_lane_border = 1.6;
  double left_road_border = 1.6;
  double right_road_border = 1.6;

  // 1 : physical; 0 : virtual
  uint16_t left_road_border_type;
  uint16_t right_road_border_type;
};

} // namespace msquare

#endif