#ifndef CLOTHOID_PARAMETER
#define CLOTHOID_PARAMETER
#include "common/utils/geometry.h"
#include "nlohmann/json.hpp"
#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/local_log.h"
#include <iostream>
#include <vector>

namespace clothoid {

struct Parameter {
  // geometry shape
  double width = 1.96;
  double length = 5.098;
  double width_with_rearview = 2.118;
  double wheel_base = 3.100;
  double front_to_rear = 4.015;
  double back_to_rear = 1.083;
  double rear_to_center = 1.466;
  double steering_angle_ratio = 14.5;
  double init_steer_angle = 300; // start angle at csc path

  double front_corner_width = 0.595;
  double front_corner_length = 3.645;
  double wheel_radius = 0.335;

  // inflation
  double lat = 0.12;
  double lon = 0.25;
  double straight_lat = 0.2;
  double straight_lon = 0.3;
  double obs_inflation = 0.0;

  double min_block_len = 0.1;
  double side_safe_distance = 0.3; // corner safe distance
  double max_accum_error = 1e-3;
  double target_lat_error = 0.1;
  int escape_sdjust_time = 5; // times for trying to calculate escape pose
  double max_step_height = 0.08;
  double straight_line = 15.0;
  double s_straight = 0.5;
  double s_first_straight = 1.0;
  double in_slot_curve_thres = 0.2;

  double min_radius = 5.015;
  double back_light_len = 0.25;
  double back_light_height = 0.30;
  double steering_speed = 340;         //  deg/s
  double max_steering_angle = 460;     // deg
  double turning_speed = 0.5;          // m/s
  double radical_turning_speed = 0.35; // m/s
  double step = 0.1;

  double max_alpha;
  double radical_alpha; // for adjust out slot

  double invalid_lat_offset = 0.5;
  double virtual_wall_interval = 0.1;

  double check_empty_length = 3.5;
  bool is_min_r_priority = false;

  // NLOHMANN_DEFINE_TYPE_INTRUSIVE(max_alpha, min_radius)
};

struct Circle {
  Circle() {}
  Circle(double _x, double _y, double _r) : x(_x), y(_y), r(_r) {}

  double x;
  double y;
  double r;
};

using StraightPoint = Pose2D;
using CircularPoint = StraightPoint;

struct ClothoidPoint : StraightPoint {
  double k; // curvature
  double s;
};

using StraightCurve = std::vector<StraightPoint>;
using CircularCurve = std::vector<CircularPoint>;
using ClothoidCurve = std::vector<ClothoidPoint>;

namespace Constant {
const double pi = 3.1415926535897;
const double pi_dao = 1.0 / 3.1415926535897;

} // namespace Constant

enum RotateType { LEFT_FORWARD, LEFT_BCAKWARD, RIGHT_FORWARD, RIGHT_BCAKWARD };

} // namespace clothoid

#endif