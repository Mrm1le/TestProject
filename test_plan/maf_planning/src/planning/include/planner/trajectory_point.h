#pragma once

#include "nlohmann/json.hpp"
#include "pnc/define/path_point.h"

namespace msquare {

struct TrajectoryPoint {
  // path point
  PathPoint path_point{};

  // linear velocity
  double v; // in [m/s]
  // linear acceleration
  double a;
  // relative time from beginning of the trajectory
  double relative_time;

  double steer;
  // probability only for prediction trajectory point
  double prediction_prob;
  // speed direction
  double velocity_direction;
  double sigma_x = 0.0;
  double sigma_y = 0.0;
  double sigma_yaw = 0.0;
  double relative_ego_yaw;
  TrajectoryPoint(double x_ = 0, double y_ = 0, double z_ = 0,
                  double theta_ = 0, double s_ = 0, double v_ = 0,
                  double a_ = 0, double relative_time_ = 0, double steer_ = 0,
                  double prediction_prob_ = 0, double velocity_direction_ = 0,
                  double sigma_x_ = 0, double relative_ego_yaw_ = 0)
      : path_point(PathPoint(x_, y_, z_, theta_, 0.0, 0.0, s_)), v(v_), a(a_),
        relative_time(relative_time_), steer(steer_),
        prediction_prob(prediction_prob_),
        velocity_direction(velocity_direction_), sigma_x(sigma_x_),
        relative_ego_yaw(relative_ego_yaw_){};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PathPoint, x, y, z, theta)
#ifdef BUILD_IN_TEST_BAG_RECURRENT
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(TrajectoryPoint, path_point, v,
                                                a, relative_time, steer,
                                                sigma_x, sigma_y, sigma_yaw)
#else  // BUILD_IN_TEST_BAG_RECURRENT
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TrajectoryPoint, path_point, v, a,
                                   relative_time, steer, sigma_x, sigma_y,
                                   sigma_yaw)
#endif // BUILD_IN_TEST_BAG_RECURRENT

} // namespace msquare