#include "common/math/math_utils.h"
#include "common/utils/spline.h"

#include <cmath>
#include <utility>

namespace msquare {
namespace planning_math {

double Sqr(const double x) { return x * x; }

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

int RandomInt(const int s, const int t, unsigned int rand_seed) {
  if (s >= t) {
    return s;
  }
  return s + rand_r(&rand_seed) % (t - s + 1);
}

double RandomDouble(const double s, const double t, unsigned int rand_seed) {
  return s + (t - s) / 16383.0 * (rand_r(&rand_seed) & 16383);
}

// Gaussian
double Gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

// 2-dimension Gaussian
double Gaussian2d(const double u1, const double u2, const double std1,
                  const double std2, const double x1, const double x2,
                  const double rho) {
  return (1.0 / 2 * M_PI * std1 * std2 * std::sqrt(1 - rho * rho)) *
         std::exp(-((x1 - u1) * (x1 - u1) / (std1 * std1) +
                    (x2 - u2) * (x2 - u2) / (std2 * std2) -
                    2 * rho * (x1 - u1) * (x2 - u2) / (std1 * std2)) /
                  (2 * (1 - rho * rho)));
}

// Sigmoid
double Sigmoid(const double x) { return 1.0 / (1.0 + std::exp(-x)); }

// Eigen::Vector2d RotateVector2d(const Eigen::Vector2d &v_in,
//                                const double theta) {
//   const double cos_theta = std::cos(theta);
//   const double sin_theta = std::sin(theta);

//   auto x = cos_theta * v_in.x() - sin_theta * v_in.y();
//   auto y = sin_theta * v_in.x() + cos_theta * v_in.y();

//   return {x, y};
// }

double interps(const std::vector<double> &y, const std::vector<double> &x,
               const double &x_interp) {
  if (y.size() == 0) {
    return 0.0;
  } else if (y.size() == 1) {
    return y[0];
  } else {
    double s = x_interp;
    for (int j = 0; j < (int)y.size() - 1; j++) {
      if (s >= x[j] && s <= x[j + 1]) {
        return y[j] + (y[j + 1] - y[j]) / (x[j + 1] - x[j]) * (s - x[j]);
        break;
      } else if (j == (y.size() - 2)) {
        return y[j + 1];
      }
    }
  }
  return 0.0;
}

void interpsVector(const std::vector<double> &y, const std::vector<double> &x,
                   const std::vector<double> &x_interps,
                   std::vector<double> &y_interps) {
  mph_assert(y.size() == x.size());
  for (std::size_t i = 0; i < x_interps.size(); i++) {
    if (y.size() == 0) {
      y_interps.push_back(0.);
    } else if (y.size() == 1) {
      y_interps.push_back(y[0]);
    } else {
      double s = x_interps[i];
      for (int j = 0; j < (int)y.size() - 1; j++) {
        if (s >= x[j] && s <= x[j + 1]) {
          y_interps.push_back(y[j] + (y[j + 1] - y[j]) / (x[j + 1] - x[j]) *
                                         (s - x[j]));
          break;
        } else if (j == (y.size() - 2)) {
          y_interps.push_back(y[j + 1]);
        }
      }
    }
  }
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

Pose2D calc_projection_point(const Pose2D &point1, const Pose2D &point2,
                             const Pose2D &point) {
  Pose2D projection_point;
  double k =
      ((point.x - point1.x) * (point2.x - point1.x) +
       (point.y - point1.y) * (point2.y - point1.y)) /
      std::pow(std::hypot(point2.x - point1.x, point2.y - point1.y), 2.0);
  k = std::min(std::max(0.0, k), 1.0);
  projection_point.x = point1.x + (point2.x - point1.x) * k;
  projection_point.y = point1.y + (point2.y - point1.y) * k;
  projection_point.theta = std::atan2(
      std::sin(point1.theta) * (1.0 - k) + std::sin(point2.theta) * k,
      std::cos(point1.theta) * (1.0 - k) + std::cos(point2.theta) * k);
  // std::cout << "TB k: " << k << std::endl;
  return projection_point;
}

Pose2D calc_projection_point2(const Pose2D &point1, const Pose2D &point2,
                              const Pose2D &point) {
  Pose2D projection_point;
  double k =
      ((point.x - point1.x) * (point2.x - point1.x) +
       (point.y - point1.y) * (point2.y - point1.y)) /
      std::pow(std::hypot(point2.x - point1.x, point2.y - point1.y), 2.0);
  projection_point.x = point1.x + (point2.x - point1.x) * k;
  projection_point.y = point1.y + (point2.y - point1.y) * k;
  k = std::min(std::max(0.0, k), 1.0);
  projection_point.theta = std::atan2(
      std::sin(point1.theta) * (1.0 - k) + std::sin(point2.theta) * k,
      std::cos(point1.theta) * (1.0 - k) + std::cos(point2.theta) * k);
  // std::cout << "TB k: " << k << std::endl;
  return projection_point;
}

LineSegment2d tf2d(const Pose2D &local_frame, const LineSegment2d &line) {
  return LineSegment2d(tf2d(local_frame, line.start()),
                       tf2d(local_frame, line.end()));
}

Box2d tf2d(const Pose2D &local_frame, const Box2d &box) {
  return Box2d(tf2d(local_frame, box.center()),
               NormalizeAngle(box.heading() - local_frame.theta), box.length(),
               box.width());
}

Pose2D tf2d(const Pose2D &local_frame, const Pose2D &pose) {
  Pose2D Point_local;
  rotate2d(pose.x - local_frame.x, pose.y - local_frame.y, -local_frame.theta,
           0.0, 0.0, Point_local.x, Point_local.y);
  Point_local.theta = NormalizeAngle(pose.theta - local_frame.theta);
  return Point_local;
}

LineSegment2d tf2d_inv(const Pose2D &local_frame,
                       const LineSegment2d &line_local) {
  return LineSegment2d(tf2d_inv(local_frame, line_local.start()),
                       tf2d_inv(local_frame, line_local.end()));
}

Box2d tf2d_inv(const Pose2D &local_frame, const Box2d &box_local) {
  return Box2d(tf2d_inv(local_frame, box_local.center()),
               NormalizeAngle(box_local.heading() + local_frame.theta),
               box_local.length(), box_local.width());
}

Pose2D tf2d_inv(const Pose2D &local_frame, const Pose2D &p_local) {
  Pose2D p_global;
  rotate2d(p_local.x, p_local.y, local_frame.theta, local_frame.x,
           local_frame.y, p_global.x, p_global.y);
  p_global.theta = NormalizeAngle(p_local.theta + local_frame.theta);
  return p_global;
}

void get_rotate_matrix(float rotate_angle, float *rotate_matrix_ptr) {
  float cos_theta = cos(M_PI / 2 - rotate_angle);
  float sin_theta = sin(M_PI / 2 - rotate_angle);
  rotate_matrix_ptr[0] = cos_theta;
  rotate_matrix_ptr[1] = sin_theta;
  rotate_matrix_ptr[2] = -sin_theta;
  rotate_matrix_ptr[3] = cos_theta;
}

double getRemainDistance(std::vector<Pose2D> &traj_pose_array_,
                         std::vector<float> &traj_vel_array_,
                         const Pose2D ego_pose) {
  std::vector<Pose2D>::iterator iter = traj_pose_array_.begin();
  std::vector<Pose2D>::iterator iter_min = traj_pose_array_.begin();

  double s = 0.0;
  double traj_length = 0.0;
  double dist = 100.0;
  double dist_min = 100.0;
  double max_length = 20.0;

  Pose2D traj_point;
  std::vector<Pose2D> trajectory;
  int index = 0;
  int index_min = 0;
  for (iter = traj_pose_array_.begin(); iter != traj_pose_array_.end();
       iter++) {
    dist = std::hypot(iter->x - ego_pose.x, iter->y - ego_pose.y);
    if (dist < dist_min) {
      dist_min = dist;
      iter_min = iter;
      index_min = index;
    }
    index++;
  }
  Pose2D projection_point;
  if (iter_min != traj_pose_array_.begin()) {
    projection_point = calc_projection_point(
        Pose2D((iter_min - 1)->x, (iter_min - 1)->y, (iter_min - 1)->theta),
        Pose2D((iter_min)->x, (iter_min)->y, (iter_min)->theta),
        Pose2D(ego_pose.x, ego_pose.y, 0.0));

    if (std::abs(traj_vel_array_.at(index_min - 1)) > 0.01) {
      s += std::hypot(projection_point.x - iter_min->x,
                      projection_point.y - iter_min->y);
      if (s > 0.01) {
        traj_point.x = projection_point.x;
        traj_point.y = projection_point.y;
        traj_point.theta = projection_point.theta;
        trajectory.push_back(traj_point);
      }
    }
  }
  if ((iter_min) != traj_pose_array_.end() &&
      (iter_min + 1) != traj_pose_array_.end()) {
    projection_point = calc_projection_point(
        Pose2D((iter_min)->x, (iter_min)->y, (iter_min)->theta),
        Pose2D((iter_min + 1)->x, (iter_min + 1)->y, (iter_min + 1)->theta),
        Pose2D(ego_pose.x, ego_pose.y, 0.0));
    if (std::abs(traj_vel_array_.at(index_min)) > 0.01 && trajectory.empty()) {
      s += std::hypot(projection_point.x - (iter_min + 1)->x,
                      projection_point.y - (iter_min + 1)->y);
      index_min++;
      iter_min++;
      if (s > 0.01) {
        traj_point.x = projection_point.x;
        traj_point.y = projection_point.y;
        traj_point.theta = projection_point.theta;
        trajectory.push_back(traj_point);
      }
    }
  }
  index = index_min;
  bool reach_end = false;
  for (iter = iter_min; iter != traj_pose_array_.end(); iter++) {
    s += std::hypot(iter->x - iter_min->x, iter->y - iter_min->y);
    traj_point.x = iter->x;
    traj_point.y = iter->y;
    traj_point.theta = iter->theta;
    trajectory.push_back(traj_point);
    iter_min = iter;
    if (std::abs(traj_vel_array_.at(index)) < 0.01 && !reach_end) {
      traj_length = s;
      reach_end = true;
    }
    if (s > max_length) {
      break;
    }
    index++;
  }
  if (!reach_end) {
    traj_length = s;
  }
  return traj_length;
}

Eigen::AngleAxisd Quat2AxisAngle(Eigen::Quaterniond q) {
  Eigen::AngleAxisd rotation_vector(q);
  return rotation_vector;
}
Eigen::Quaterniond EulerZYX2Quat(Eigen::Vector3d &euler_zyx) {
  double c1 = std::cos(euler_zyx.x() / 2.0);
  double c2 = std::cos(euler_zyx.y() / 2.0);
  double c3 = std::cos(euler_zyx.z() / 2.0);
  double s1 = std::sin(euler_zyx.x() / 2.0);
  double s2 = std::sin(euler_zyx.y() / 2.0);
  double s3 = std::sin(euler_zyx.z() / 2.0);
  Eigen::Quaterniond q(c1 * c2 * c3 + s1 * s2 * s3, c1 * c2 * s3 - c3 * s1 * s2,
                       c1 * c3 * s2 + c2 * s1 * s3,
                       c2 * c3 * s1 - c1 * s2 * s3);
  return q;
}
Eigen::AngleAxisd EulerZYX2AxisAngle(Eigen::Vector3d &euler_zyx) {
  Eigen::Quaterniond q = EulerZYX2Quat(euler_zyx);
  return Quat2AxisAngle(q);
}
Eigen::Matrix3d EulerZYX2Rotm(Eigen::Vector3d &euler_zyx) {
  Eigen::Quaterniond q = EulerZYX2Quat(euler_zyx);
  return q.toRotationMatrix();
}
Eigen::Matrix2d Angle2Rotm2d(const double &angle) {
  Eigen::Vector3d euler_zyx(angle, 0.0, 0.0);
  Eigen::Matrix3d R = EulerZYX2Rotm(euler_zyx);
  return R.block(0, 0, 2, 2);
}

double getRemainDistanceControlWay(std::vector<Pose2D> &traj_pose_array_,
                                   std::vector<float> &traj_vel_array_,
                                   const Pose2D ego_pose) {
  int PARKING_PATH_EXTEND_POINTS = 10;
  double match_point_dist = 9999.99;
  int match_point_index = 0;
  Eigen::Vector2d ref_pos_i = Eigen::Vector2d::Zero();
  if (traj_pose_array_.empty()) {
    return 0;
  }
  for (int i = 0; i < static_cast<int>(traj_pose_array_.size()) - 1; ++i) {
    ref_pos_i << traj_pose_array_[i].x, traj_pose_array_[i].y;
    Eigen::Vector2d dp(ego_pose.x - ref_pos_i.x(), ego_pose.y - ref_pos_i.y());
    double dist = dp.norm();
    if (dist < match_point_dist) {
      match_point_dist = dist;
      match_point_index = i;
    }
  }
  int match_point_search_start_index = 0;
  if (match_point_index == 0) {
    match_point_search_start_index = 0;
  } else {
    match_point_search_start_index = match_point_index - 1;
  }
  int count = 0;
  int dp_count = 0;
  std::vector<double> dy_vec;
  std::vector<double> dx_vec;
  std::vector<double> s_fit_vec;
  for (int i = match_point_search_start_index;
       i < static_cast<int>(traj_pose_array_.size()); ++i) {
    Eigen::Vector2d ref_pos_i = Eigen::Vector2d::Zero();
    ref_pos_i << traj_pose_array_[i].x, traj_pose_array_[i].y;
    Eigen::Vector2d dis_pos_i(ref_pos_i.x() - ego_pose.x,
                              ref_pos_i.y() - ego_pose.y);
    Eigen::Matrix2d rotm2d = Angle2Rotm2d(ego_pose.theta);
    Eigen::Vector2d dis_pos_b = rotm2d.transpose() * dis_pos_i;
    if (count == 0) {
      s_fit_vec.push_back(0.0);
      dx_vec.push_back(dis_pos_b.x());
      dy_vec.push_back(dis_pos_b.y());
    } else {
      Eigen::Vector2d dp =
          dis_pos_b - Eigen::Vector2d(dx_vec.back(), dy_vec.back());
      // to prevent the failure of spline
      if (std::fabs(dp.norm()) >= 0.001) {
        s_fit_vec.push_back(s_fit_vec.back() + dp.norm());
        dx_vec.push_back(dis_pos_b.x());
        dy_vec.push_back(dis_pos_b.y());
      } else {
        dp_count++;
      }
    }
    // if (i == pred_pos_match_index) {
    //   s_match_pred = s_fit_vec.back();
    // }
    count++;
  }

  double s_right_side = 0.0;
  double s_left_side = 0.0;
  double s_mid = 0.0;
  if (s_fit_vec.size() > 2) {
    s_left_side = s_fit_vec[0];  // ref_info_ptr_->match_point_index_ - 1
    s_right_side = s_fit_vec[2]; // ref_info_ptr_->match_point_index_ + 1
  }
  planning_math::spline dx_spline;
  planning_math::spline dy_spline;
  dx_spline.set_points(s_fit_vec, dx_vec);
  dy_spline.set_points(s_fit_vec, dy_vec);
  int iter_time = 10;
  double s_proj = 0.0;
  for (int i = 0; i < iter_time; ++i) {
    s_mid = (s_left_side + s_right_side) / 2.0;
    double f_mid = dx_spline(s_mid) * dx_spline.deriv(1, s_mid) +
                   dy_spline(s_mid) * dy_spline.deriv(1, s_mid);
    double f_left = dx_spline(s_left_side) * dx_spline.deriv(1, s_left_side) +
                    dy_spline(s_left_side) * dy_spline.deriv(1, s_left_side);
    double f_right =
        dx_spline(s_right_side) * dx_spline.deriv(1, s_right_side) +
        dy_spline(s_right_side) * dy_spline.deriv(1, s_right_side);
    if (f_left * f_mid <= 0) {
      s_right_side = s_mid;
    } else if (f_right * f_mid < 0) {
      s_left_side = s_mid;
    } else {
      s_mid = s_left_side;
      break;
    }
  }
  s_proj = s_mid;
  Eigen::Vector2d project_pos = Eigen::Vector2d::Zero();
  double project_pos_x = dx_spline(s_proj);
  double project_pos_y = dy_spline(s_proj);
  project_pos << project_pos_x, project_pos_y;
  // projection_point.x = project_pos_x;
  // projection_point.y = project_pos_y;
  int s_final_index = (int)traj_pose_array_.size() -
                      PARKING_PATH_EXTEND_POINTS - match_point_index - dp_count;
  double s_without_extend = 0;
  if (s_fit_vec.size() > 0) {
    s_final_index = planning_math::Clamp(
        s_final_index, 0, static_cast<int>(s_fit_vec.size()) - 1);
    s_without_extend = s_fit_vec[s_final_index];
  }
  double remain_s_plan = s_without_extend - s_proj;
  return remain_s_plan;
}

} // namespace planning_math
} // namespace msquare
