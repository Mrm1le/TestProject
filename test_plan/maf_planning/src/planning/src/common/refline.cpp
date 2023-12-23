#include "common/refline.h"

namespace msquare {

bool equal_zero(double a) {
  if (a > -std::numeric_limits<double>::epsilon() &&
      a < std::numeric_limits<double>::epsilon()) {
    return true;
  }

  return false;
}

bool calc_projection(double x, double y, double x0, double y0, double x1,
                     double y1, double &v_x, double &v_y, double &t) {
  if (std::fabs(x0 - x1) < 1e-8 && std::fabs(y0 - y1) < 1e-8) {
    return false;
  }

  double diff_x = x - x0;
  double diff_y = y - y0;
  double diff_x1 = x1 - x0;
  double diff_y1 = y1 - y0;

  if (equal_zero(diff_x1) && equal_zero(diff_y1)) {
    t = 0.0;
  } else {
    t = (diff_x * diff_x1 + diff_y * diff_y1) /
        (diff_x1 * diff_x1 + diff_y1 * diff_y1);
  }

  t = clip(t, 1.0, 0.0);
  v_x = x0 + diff_x1 * t;
  v_y = y0 + diff_y1 * t;

  return true;
}

void discrete(double start, double end, double gap,
              std::vector<double> &output) {
  output.clear();
  for (double value = start; value < end; value += gap) {
    output.push_back(value);
  }
}

void calc_cartesian_frenet(const std::vector<PathPoint> &path_points, double x,
                           double y, double &s, double &l, double &v_s,
                           double &v_l, double &theta, bool get_theta,
                           const double *v, const double *yaw) {
  int index_min = 0;

  double min_dist =
      std::pow(path_points[0].x - x, 2) + std::pow(path_points[0].y - y, 2);

  for (size_t i = 1; i < path_points.size(); i++) {
    double temp_dist =
        std::pow(path_points[i].x - x, 2) + std::pow(path_points[i].y - y, 2);

    if (temp_dist < min_dist) {
      min_dist = temp_dist;
      index_min = i;
    }
  }

  double pi = std::atan(1.0) * 4;

  double x0, y0, x1, y1;
  double matched_x = 0.0;
  double matched_y = 0.0;
  double t = 0.0;
  if (index_min == 0) {
    x0 = path_points[index_min].x;
    y0 = path_points[index_min].y;
    x1 = path_points[index_min + 1].x;
    y1 = path_points[index_min + 1].y;
    matched_x = path_points[index_min].x;
    matched_y = path_points[index_min].y;
    s = path_points[index_min].s;
    if (get_theta || v != nullptr) {
      theta = path_points[index_min].theta;
    }
  } else {
    x0 = path_points[index_min - 1].x;
    y0 = path_points[index_min - 1].y;
    x1 = path_points[index_min].x;
    y1 = path_points[index_min].y;

    (void)calc_projection(x, y, x0, y0, x1, y1, matched_x, matched_y, t);
    s = path_points[index_min - 1].s +
        (path_points[index_min].s - path_points[index_min - 1].s) * t;

    if (get_theta || v != nullptr) {
      if (path_points[index_min - 1].theta > pi / 2 &&
          path_points[index_min].theta < -pi / 2) {
        theta = path_points[index_min - 1].theta +
                (path_points[index_min].theta + 2 * pi -
                 path_points[index_min - 1].theta) *
                    t;

        if (theta > 2 * pi) {
          theta -= 2 * pi;
        }
      } else if (path_points[index_min - 1].theta < -pi / 2 &&
                 path_points[index_min].theta > pi / 2) {
        theta = path_points[index_min - 1].theta +
                (path_points[index_min].theta - 2 * pi -
                 path_points[index_min - 1].theta) *
                    t;

        if (theta < -2 * pi) {
          theta += 2 * pi;
        }
      } else {
        theta =
            path_points[index_min - 1].theta +
            (path_points[index_min].theta - path_points[index_min - 1].theta) *
                t;
      }
    }
  }

  l = std::sqrt(std::pow(matched_x - x, 2) + std::pow(matched_y - y, 2));
  double k = (y0 - y) * (x1 - x0) - (x0 - x) * (y1 - y0);
  if (k >= 0) {
    l = -l;
  }

  if (v != nullptr && yaw != nullptr) {
    v_l = (*v) * std::sin(*yaw - theta);
    v_s = (*v) * std::cos(*yaw - theta);
  }
}

void calc_frenet_cartesian(const std::vector<PathPoint> &path_points, double s,
                           double l, double &x, double &y) {
  size_t index = 0;
  size_t i_low = 0;
  size_t i_upp = (int)path_points.size() - 1;

  while (i_upp > i_low + 1) {
    if (path_points[(i_low + i_upp) / 2].s > s) {
      i_upp = (i_low + i_upp) / 2;
    } else {
      i_low = (i_low + i_upp) / 2;
    }
  }

  if (s >= path_points[i_upp].s) {
    index = i_upp;
  } else {
    index = i_low;
  }

  double pi = std::atan(1.0) * 4;

  double x_r, y_r, theta_r;

  if (index < (int)path_points.size() - 1) {
    double s0 = path_points[index].s;
    double s1 = path_points[index + 1].s;
    double t = equal_zero(s1 - s0) ? 0 : ((s - s0) / (s1 - s0));

    if (path_points[index].theta > pi / 2 &&
        path_points[index + 1].theta < -pi / 2) {
      theta_r = path_points[index].theta + (path_points[index + 1].theta +
                                            2 * pi - path_points[index].theta) *
                                               t;

      if (theta_r > 2 * pi) {
        theta_r -= 2 * pi;
      }
    } else if (path_points[index].theta < -pi / 2 &&
               path_points[index + 1].theta > pi / 2) {
      theta_r = path_points[index].theta + (path_points[index + 1].theta -
                                            2 * pi - path_points[index].theta) *
                                               t;

      if (theta_r < -2 * pi) {
        theta_r += 2 * pi;
      }
    } else {
      theta_r = path_points[index].theta +
                (path_points[index + 1].theta - path_points[index].theta) * t;
    }

    x_r = path_points[index].x +
          (path_points[index + 1].x - path_points[index].x) * t;
    y_r = path_points[index].y +
          (path_points[index + 1].y - path_points[index].y) * t;
  } else {
    theta_r = path_points[index].theta;
    x_r = path_points[index].x;
    y_r = path_points[index].y;
  }

  x = x_r - l * std::sin(theta_r);
  y = y_r + l * std::cos(theta_r);
}

RawRefLine::RawRefLine(int position) {
  exist_ = false;
  position_ = position;
  neighbours_.fill(nullptr);
}

RawRefLine::RawRefLine(const RawRefLine &raw_refline) {
  exist_ = raw_refline.exist_;
  position_ = raw_refline.position_;
  waypoints_ = raw_refline.waypoints_;
  point_ids_ = raw_refline.point_ids_;
  neighbours_ = raw_refline.neighbours_;
}

RawRefLine &RawRefLine::operator=(const RawRefLine &raw_refline) {
  if (this == &raw_refline) {
    return *this;
  }
  exist_ = raw_refline.exist_;
  position_ = raw_refline.position_;
  waypoints_ = raw_refline.waypoints_;
  point_ids_ = raw_refline.point_ids_;
  neighbours_ = raw_refline.neighbours_;
  return *this;
}

void RawRefLine::update(const std::vector<ReferenceLinePointDerived> &waypoints,
                        double ego_vel) {
  waypoints_.clear();
  point_ids_.clear();

  exist_ = (waypoints.size() > 0);
  if (exist_ == false) {
    return;
  }

  std::vector<double> x_points;
  std::vector<double> y_points;

  double min_dist = DBL_MAX;
  int nearest_point_idx = -1;
  int front_point_idx = -1;
  nearest_waypoint_.clear();
  front_waypoint_.clear();
  for (size_t i = 0; i < waypoints.size(); i++) {
    x_points.push_back(waypoints[i].car_point.x);
    y_points.push_back(waypoints[i].car_point.y);
    point_ids_.insert(std::make_pair(waypoints[i].track_id, true));
    // find nearest waypoint
    if (std::abs(waypoints[i].car_point.x) < 20.0) {
      double dist = hypot(waypoints[i].car_point.x, waypoints[i].car_point.y);
      if (dist < min_dist) {
        min_dist = dist;
        nearest_point_idx = i;
      }
    }
    // calculate width at max(10m, 2s * ego_vel) front distance
    double front_distance = std::max(10.0, 2.0 * ego_vel);
    if (front_point_idx < 0 && waypoints[i].car_point.x > front_distance &&
        waypoints[i].car_point.x < front_distance + 10.0) {
      front_point_idx = i;
    }
  }
  if (nearest_point_idx >= 0) {
    nearest_waypoint_.push_back(waypoints[nearest_point_idx]);
  }
  if (front_point_idx >= 0) {
    front_waypoint_.push_back(waypoints[front_point_idx]);
  }

  bool push_back = false;
  for (size_t i = 0; i + 1 < waypoints.size(); i++) {
    if (x_points[i] > -50 && x_points[i] < 150) {
      waypoints_.emplace_back(x_points[i], y_points[i]);
    }

    double dist = std::sqrt(std::pow(x_points[i + 1] - x_points[i], 2) +
                            std::pow(y_points[i + 1] - y_points[i], 2));

    if (dist < 0.01 && waypoints_.size() > 0) {
      waypoints_.pop_back();
    } else if (dist > 2.0) {
      int num_insert = int(std::ceil(dist / 2.0));
      double dx = (x_points[i + 1] - x_points[i]) / num_insert;
      double dy = (y_points[i + 1] - y_points[i]) / num_insert;
      double x_tmp = x_points[i];
      double y_tmp = y_points[i];

      for (int j = 0; j < num_insert - 1; j++) {
        x_tmp += dx;
        y_tmp += dy;

        if (x_tmp > -50) {
          push_back = true;
        }

        if (push_back && x_tmp <= 80) {
          waypoints_.emplace_back(x_tmp, y_tmp);
        } else if (push_back && x_tmp > 80) {
          break;
        }
      }
    }

    if (i == waypoints.size() - 2 && x_points[i + 1] < 80) {
      waypoints_.emplace_back(x_points[i + 1], y_points[i + 1]);
    }
  }

  if (waypoints_.size() == 2) {
    Point2D point((waypoints_[0].x + waypoints_[1].x) / 2.0,
                  (waypoints_[0].y + waypoints_[1].y) / 2.0);
    waypoints_.insert(waypoints_.begin() + 1, point);
  }
}

double RawRefLine::min_square_dist(double x, double y) {
  int index_min = 0;
  if (waypoints_.size() < 3) {
    return kInvalidDist;
  }

  double diff_x =          // parasoft-suppress AUTOSAR-A8_5_0-a-2
      waypoints_[0].x - x; // parasoft-suppress AUTOSAR-A8_5_0-a-2 "f-drop"
  double diff_y =          // parasoft-suppress AUTOSAR-A8_5_0-a-2
      waypoints_[0].y - y; // parasoft-suppress AUTOSAR-A8_5_0-a-2 "f-drop"
  double min_dist = diff_x * diff_x + diff_y * diff_y;

  for (size_t i = 1; i < waypoints_.size(); i++) {
    diff_x = waypoints_[i].x - x;
    diff_y = waypoints_[i].y - y;

    double tmp_dist = diff_x * diff_x + diff_y * diff_y;
    if (tmp_dist < min_dist) {
      min_dist = tmp_dist;
      index_min = i;
    }
  }

  double x0, y0, x1, y1, x2, y2;
  if (index_min == 0) {
    x0 = waypoints_[index_min].x;
    y0 = waypoints_[index_min].y;
    x1 = waypoints_[index_min + 1].x;
    y1 = waypoints_[index_min + 1].y;
    x2 = x1;
    y2 = y1;
  } else {
    x0 = waypoints_[index_min - 1].x;
    y0 = waypoints_[index_min - 1].y;
    x1 = waypoints_[index_min].x;
    y1 = waypoints_[index_min].y;
    if (index_min + 1 < waypoints_.size()) {
      x2 = waypoints_[index_min + 1].x;
      y2 = waypoints_[index_min + 1].y;
    } else {
      x2 = x1;
      y2 = y1;
    }
  }

  double v_x, v_y, t, v_x1, v_y1, t1;
  bool ret = calc_projection(x, y, x0, y0, x1, y1, v_x, v_y, t);
  if (ret == false) {
    return kInvalidDist;
  }

  diff_x = v_x - x;
  diff_y = v_y - y;

  min_dist = diff_x * diff_x + diff_y * diff_y;
  bool ret1 = calc_projection(x, y, x1, y1, x2, y2, v_x1, v_y1, t1);
  if (ret1 == false) {
    return kInvalidDist;
  }

  double diff_x1 = v_x1 - x;
  double diff_y1 = v_y1 - y;

  double min_dist1 = diff_x1 * diff_x1 + diff_y1 * diff_y1;
  double k = (y0 - y) * (x1 - x0) - (x0 - x) * (y1 - y0);
  // MSD_LOG(INFO, "Lane::*****************************: "
  //                "waypoints_[index_min - 1].x: %f, waypoints_[index_min -
  //                1].y: %f, waypoints_[index_min].x: %f,
  //                waypoints_[index_min].y: %f" "waypoints_[index_min + 1].x:
  //                %f, waypoints_[index_min + 1].y: %f" "x0: %f, y0: %f, x1:
  //                %f, y1: %f, x2: %f, y2: %f", waypoints_[index_min - 1].x,
  //                waypoints_[index_min - 1].y, waypoints_[index_min].x,
  //                waypoints_[index_min].y, waypoints_[index_min + 1].x,
  //                waypoints_[index_min + 1].y, x0, y0, x1, y1, x2, y2);
  if (index_min != 0 && index_min + 1 < waypoints_.size() &&
      min_dist1 < min_dist) {
    min_dist = min_dist1;
    k = (y1 - y) * (x2 - x1) - (x1 - x) * (y2 - y1);
  }
  // MSD_LOG(INFO, "Lane: %%%%%%%%%%%%%%%%%%%%%%%%%: min_dist: %f, min_dist1:
  // %f", min_dist, min_dist1);
  if (k >= 0) {
    min_dist = 0 - min_dist;
  }

  return min_dist;
}

void RawRefLine::save_context(RawRefLineContext &context) const {
  context.exist = exist_;
  context.position = position_;
  context.waypoints = waypoints_;
  context.point_ids = point_ids_;
}

void RawRefLine::restore_context(const RawRefLineContext &context) {
  exist_ = context.exist;
  position_ = context.position;
  waypoints_ = context.waypoints;
  point_ids_ = context.point_ids;
}

void RawRefLine::reset() {
  exist_ = false;
  position_ = RefLinePosition::UNKNOWN_REFLINE;
  waypoints_.clear();
  point_ids_.clear();
  neighbours_.fill(nullptr);
}

RefLine::RefLine() {
  position_ = RefLinePosition::UNKNOWN_REFLINE;
  master_ = nullptr;
}

void RefLine::update_pathpoints() {
  if (master_ == nullptr) {
    return;
  }

  const std::vector<Point2D> &waypoints = master_->waypoints();
  if (waypoints.size() < 2) {
    path_points_.clear();
    ppath_.reset();
    return;
  }

  double interp_gap = 0.5;
  std::vector<double> x_points, y_points;

  for (size_t i = 0; i < waypoints.size(); i++) {
    x_points.push_back(waypoints[i].x);
    y_points.push_back(waypoints[i].y);
  }

  path_points_.clear();
  ppath_.reset();

  std::vector<double> u_points{0};

  for (size_t i = 0; i + 1 < x_points.size(); i++) {
    double dist = std::sqrt(std::pow(x_points[i + 1] - x_points[i], 2) +
                            std::pow(y_points[i + 1] - y_points[i], 2));

    u_points.push_back(u_points[i] + dist);
  }

  msquare::planning_math::spline x_spline, y_spline;
  x_spline.set_points(u_points, x_points);
  y_spline.set_points(u_points, y_points);

  std::vector<double> us;
  discrete(u_points[0], u_points.back(), interp_gap, us);

  size_t index = 0;
  for (size_t i = 0; i < us.size(); i++) {
    PathPoint pp;
    pp.x = x_spline(us[i]);
    pp.y = y_spline(us[i]);

    if (i == 0) {
      pp.s = 0;
    } else {
      double dist = std::sqrt(std::pow(pp.x - path_points_[i - 1].x, 2) +
                              std::pow(pp.y - path_points_[i - 1].y, 2));

      pp.s = path_points_[i - 1].s + dist;
    }

    // something strange
    for (size_t j = 0; j + 1 < u_points.size(); j++) {
      if (j < index) {
        continue;
      }

      if (us[i] >= u_points[j] && us[i] < u_points[j + 1]) {
        index = j;
        break;
      }
      index++;
    }

    pp.theta = std::atan2(y_spline.deriv(1, us[i]), x_spline.deriv(1, us[i]));
    pp.kappa = 0;
    path_points_.push_back(pp);
  }

  set_ppath(u_points, x_spline, y_spline);
}

void RefLine::set_ppath(const std::vector<double> &u_points,
                        const msquare::planning_math::spline &x_spline,
                        const msquare::planning_math::spline &y_spline) {
  ppath_.reset(new PPath(u_points[0], u_points.back(), x_spline, y_spline));
}

void RefLine::cartesian_frenet(double x, double y, double &s, double &l,
                               double &theta, bool get_theta) {
  if (path_points_.size() < 2) {
    return;
  }

  double v_s = 0;
  double v_l = 0;
  calc_cartesian_frenet(path_points_, x, y, s, l, v_s, v_l, theta, get_theta);
}

void RefLine::frenet_cartesian(double s, double l, double &x, double &y) {
  if (path_points_.empty() == true) {
    return;
  }

  calc_frenet_cartesian(path_points_, s, l, x, y);
}

void RefLine::save_context(FixRefLineContext &context) const {
  context.position = position_;
  context.path_points = path_points_;
}

void RefLine::restore_context(const FixRefLineContext &context) {
  position_ = context.position;
  path_points_ = context.path_points;
}

} // namespace msquare
