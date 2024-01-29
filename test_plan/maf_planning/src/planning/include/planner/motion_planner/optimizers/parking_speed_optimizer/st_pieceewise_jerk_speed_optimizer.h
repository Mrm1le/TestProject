

#ifndef DEV_ST_PIECEWISE_JERK_SPEED_OPTIMIZER_H
#define DEV_ST_PIECEWISE_JERK_SPEED_OPTIMIZER_H

#include "planner/behavior_planner/parking/speed_planner_util.h"
#include <memory>
#include <vector>

namespace msquare {
namespace parking {

struct SpeedPlannerCofig {
  double s_w;     // weight for s
  double v_w;     // weight for v
  double a_w;     // weight for a
  double j_w;     // weight for jerk
  double s_ref_w; // weight for s_ref
  double v_ref_w; // weight for v_ref
  double dt;      // delta t
  double a_lo;    // for acc
  double a_up;
  double v_lo; // for velocity
  double v_up;
  double j_lo; // for jerk
  double j_up;
  SpeedPlannerCofig() = default;
  SpeedPlannerCofig(double dt, bool is_reverse) { init(dt, is_reverse); }
  void init(double dt, bool is_reverse);
};

class PiecewiseJerkSpeedOptimizer {
public:
  PiecewiseJerkSpeedOptimizer(const SpeedPlannerCofig &speed_config)
      : config_(speed_config) {}
  bool makeOptimize(const double init_speed, const double init_a,
                    std::vector<std::vector<double>> *path, const VecST &vec_st,
                    const msquare::parking::VecST &first_st_obs,
                    const msquare::parking::VecST &second_st_obs);

private:
  SpeedPlannerCofig config_;
};

} // namespace parking
} // namespace msquare

#endif // DEV_ST_PIECEWISE_JERK_SPEED_OPTIMIZER_H