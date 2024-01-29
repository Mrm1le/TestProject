#ifndef SPEED_PLANNER_UTIL_H
#define SPEED_PLANNER_UTIL_H

#include <vector>

namespace msquare {
namespace parking {

struct STElement {
  double t = 0;
  double s = 0;
  double v = 0;
  double a = 0;
  double jerk = 0;
  STElement(double _t = 0.0, double _s = 0.0, double _v = 0.0, double _a = 0.0,
            double _jerk = 0.0)
      : t(_t), s(_s), v(_v), a(_a), jerk(_jerk) {}
};
using VecST = std::vector<STElement>;

} // end namespace parking
} // end namespace msquare

#endif