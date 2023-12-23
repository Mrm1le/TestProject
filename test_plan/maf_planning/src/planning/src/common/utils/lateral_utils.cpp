#include "common/utils/lateral_utils.h"
#include "common/config_context.h"
#include "common/utils/pose2d_utils.hpp"
#include "mtime_core/mtime.h"
#include "pnc/define/geometry.h"
#include <algorithm>
namespace msquare {

double calc_poly1d(const std::vector<double> &coefs, double x) {
  double result = 0;

  for (std::size_t i = 0; i < coefs.size(); i++) {
    result = result * x + coefs[i];
  }

  return result;
}

double get_dist(double x, double y, const std::vector<double> &y_x) {
  double search_range = 5;
  double tol = 0.02;
  double x_low = x - search_range;
  double x_upp = x + search_range;

  double invphi = (std::sqrt(5.0) - 1.0) / 2.0;
  double invphi2 = (3.0 - std::sqrt(5.0)) / 2.0;

  double h = 2 * search_range;
  int n = int(std::ceil(std::log(tol / h) / std::log(invphi)));

  double x_left = x_low + invphi2 * h;
  double x_right = x_low + invphi * h;

  double d_left =
      std::pow(x_left - x, 2) + std::pow(calc_poly1d(y_x, x_left) - y, 2);

  double d_right =
      std::pow(x_right - x, 2) + std::pow(calc_poly1d(y_x, x_right) - y, 2);

  for (int i = 0; i < n - 1; i++) {
    if (d_left < d_right) {
      x_upp = x_right;
      x_right = x_left;
      d_right = d_left;
      h = invphi * h;
      x_left = x_low + invphi2 * h;
      d_left =
          std::pow(x_left - x, 2) + std::pow(calc_poly1d(y_x, x_left) - y, 2);
    } else {
      x_low = x_left;
      x_left = x_right;
      d_left = d_right;
      h = invphi * h;
      x_right = x_low + invphi * h;
      d_right =
          std::pow(x_right - x, 2) + std::pow(calc_poly1d(y_x, x_right) - y, 2);
    }
  }

  double v_low = calc_poly1d(y_x, x_low);
  double v_upp = calc_poly1d(y_x, x_upp);

  double k = (v_low - y) * h - (x_low - x) * (v_upp - v_low);

  double v_min;
  int sgn = (k >= 0) ? -1 : 1;
  if (d_left < d_right) {
    v_min = std::min(std::pow(x_low - x, 2) + std::pow(v_low - y, 2), d_left);
  } else {
    v_min = std::min(std::pow(x_upp - x, 2) + std::pow(v_upp - y, 2), d_right);
  }

  return sgn * std::sqrt(v_min);
}

bool calc_dist(const std::vector<double> &path_y, double x, double y,
               double length, double width, double theta, double &d_center,
               double &d_edge_max, double &d_edge_min) {
  if (path_y.empty()) {
    return false;
  }

  std::vector<Point2D> edge_points{{0.5 * length, 0.5 * width},
                                   {0.5 * length, -0.5 * width},
                                   {-0.5 * length, -0.5 * width},
                                   {-0.5 * length, 0.5 * width}};

  std::vector<double> d_edges;

  double v_sin = std::sin(theta);
  double v_cos = std::cos(theta);

  for (auto &point : edge_points) {
    point.x = point.x * v_cos - point.y * v_sin;
    point.y = point.x * v_sin + point.y * v_cos;

    d_edges.push_back(get_dist(point.x, point.y, path_y));
  }

  d_center = get_dist(x, y, path_y);
  d_edge_max = *std::max_element(d_edges.begin(), d_edges.end());
  d_edge_min = *std::min_element(d_edges.begin(), d_edges.end());

  return true;
}

double get_system_time() { return MTIME()->timestamp().sec(); }

double get_boot_time() { return MTIME()->timestamp().sec(); }

double cal_lat_offset(double ego_vel, double dash_length) {
  double kLateralAccMax = 3.0 - std::fmax(ego_vel, 3) * 0.05;
  double kDeltaRateMax = 0.4 - std::fmax(ego_vel, 3) * 0.005;
  double kLateralJerkMax = 5.0 - std::fmax(ego_vel, 3) * 0.15;

  double ddl_limit = std::min(
      kLateralAccMax / pow(fmax(ego_vel, 3.0), 2.),
      ConfigurationContext::Instance()
              ->get_vehicle_param()
              .max_front_wheel_angle /
          ConfigurationContext::Instance()->get_vehicle_param().wheel_base);
  double physical_limit =
      kDeltaRateMax /
      ConfigurationContext::Instance()->get_vehicle_param().wheel_base /
      std::fmax(ego_vel, 0.1);
  double comfort_limit = kLateralJerkMax / pow(ego_vel, 3);
  double dddl_limit = std::fmin(physical_limit, comfort_limit);

  double l = 0;
  double dl = 0;
  double ddl = 0;
  double ddd_max = dddl_limit;
  double delta_s = clip(ego_vel * 0.2, 5.0, 0.5);
  for (int i = 0; i * delta_s < dash_length / 2.0; i++) {
    if (i * delta_s < dash_length / 4.0)
      ddd_max = dddl_limit;
    else
      ddd_max = -dddl_limit;
    l += dl * delta_s + ddl * pow(delta_s, 2) / 2. +
         ddd_max * pow(delta_s, 3) / 6.;
    dl += ddl * delta_s + ddd_max * pow(delta_s, 2) / 2.;
    ddl += ddd_max * delta_s;
    ddl = clip(ddl, ddl_limit, -ddl_limit);
  }

  return l * 2.0;
}

} // namespace msquare
