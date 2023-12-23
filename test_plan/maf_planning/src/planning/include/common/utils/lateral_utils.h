#ifndef LATERAL_UTILS_H
#define LATERAL_UTILS_H
#include <cmath>
#include <time.h>
#include <vector>
namespace msquare {

double calc_poly1d(const std::vector<double> &coefs, double x);

double get_dist(double x, double y, const std::vector<double> &y_x);

bool calc_dist(const std::vector<double> &path_y, double x, double y,
               double length, double width, double theta, double &d_center,
               double &d_edge_max, double &d_edge_min);
double get_system_time();
double get_boot_time();

double cal_lat_offset(double ego_vel, double dash_length);
} // namespace msquare

#endif
