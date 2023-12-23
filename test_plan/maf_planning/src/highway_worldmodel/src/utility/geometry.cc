#include "worldmodel/utility/geometry.hpp"

namespace msd_worldmodel {
namespace worldmodel_v1 {

double scanLineToSegment(const Point2D &origin, const Point2D &A,
                         const Point2D &B) {
  auto A_prime = (A - origin).eval();
  auto B_prime = (B - origin).eval();
  return (-B_prime.x()) / (A_prime.x() - B_prime.x()) *
             (A_prime.y() - B_prime.y()) +
         B_prime.y();
}

// double scanLineToPolyline(const Point2D &origin, double yaw,
//                           const Points2D &polyline, size_t &seg_begin_idx,
//                           bool is_left) {
//   Eigen::Matrix2d rotate_matrix;
//   auto sin_yaw = std::sin(yaw);
//   auto cos_yaw = std::cos(yaw);
//   rotate_matrix << cos_yaw, sin_yaw, -sin_yaw, cos_yaw;
//   auto rotated_origin = (rotate_matrix * origin).eval();
//   auto rotated_points = (rotate_matrix * polyline).eval();
//   // Over limit index means unknown
//   seg_begin_idx = rotated_points.cols();

Point2D rotatePoint(const Eigen::Matrix2d &rotate_matrix,
                    const Point2D &origin) {
  return (rotate_matrix * origin).eval();
}

Eigen::Matrix2d getRoateMatrix(const double &yaw) {
  Eigen::Matrix2d rotate_matrix;
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
  rotate_matrix << cos_yaw, sin_yaw, -sin_yaw, cos_yaw;
  return rotate_matrix;
}

float calculateDistance(const float &x1, const float &y1, const float &x2,
                        const float &y2) {
  auto diff_x = x1 - x2;
  auto diff_y = y1 - y2;
  return std::sqrt(diff_x * diff_x + diff_y + diff_y);
}
//   // First, find the cross segments
//   double A_x, B_x, A_y, B_y;
//   if (rotated_points(0, rotated_points.cols() - 1) < rotated_origin.x() - 2)
//   {
//     return invalid_distance * (is_left ? 1 : -1);
//   } else if (rotated_points(0, rotated_points.cols() - 1) <=
//              rotated_origin.x()) {
//     // If last point is behind the target point
//     A_x = rotated_points(0, rotated_points.cols() - 2);
//     A_y = rotated_points(1, rotated_points.cols() - 2);
//     B_x = rotated_points(0, rotated_points.cols() - 1);
//     B_y = rotated_points(1, rotated_points.cols() - 1);
//     seg_begin_idx = rotated_points.cols() - 2;
//   } else if (rotated_points(0, 0) >= rotated_origin.x() + 2) {
//     return invalid_distance * (is_left ? 1 : -1);
//   } else if (rotated_points(0, 0) >= rotated_origin.x()) {
//     // If first point is ahead the target point
//     A_x = rotated_points(0, 0);
//     A_y = rotated_points(1, 0);
//     B_x = rotated_points(0, 1);
//     B_y = rotated_points(1, 1);
//     seg_begin_idx = 0;
//   } else {
//     size_t target_i = 1;
//     for (int i = 1; i < rotated_points.cols(); ++i) {
//       if (rotated_points(0, i) >= rotated_origin.x()) {
//         target_i = i;
//         break;
//       }
//     }
//     A_x = rotated_points(0, target_i - 1);
//     A_y = rotated_points(1, target_i - 1);
//     B_x = rotated_points(0, target_i);
//     B_y = rotated_points(1, target_i);
//     seg_begin_idx = target_i - 1;
//   }

//   return scanLineToSegment(rotated_origin, Point2D(A_x, A_y),
//                            Point2D(B_x, B_y));
// }

// double scanLineToPolygon(const Point2D &origin, double yaw,
//                          const Points2D &polygon) {
//   if (polygon.cols() < 2) {
//     return std::numeric_limits<double>::max();
//   }
//   Eigen::Matrix2d rotate_matrix;
//   auto sin_yaw = std::sin(yaw);
//   auto cos_yaw = std::cos(yaw);
//   rotate_matrix << cos_yaw, sin_yaw, -sin_yaw, cos_yaw;
//   auto rotated_origin = (rotate_matrix * origin).eval();
//   auto rotated_points = (rotate_matrix * polygon).eval();
//   double ret = std::numeric_limits<double>::max();
//   for (size_t i = 0; i < static_cast<size_t>(rotated_points.cols()) - 1; ++i)
//   {
//     // 二者的x值异号
//     if ((rotated_points(0, i) <= rotated_origin.x()) ==
//         (rotated_points(0, i + 1) > rotated_origin.x())) {
//       Point2D A(rotated_points(0, i), rotated_points(1, i));
//       Point2D B(rotated_points(0, i + 1), rotated_points(1, i + 1));
//       auto dist = scanLineToSegment(rotated_origin, A, B);
//       if (std::abs(dist) < std::abs(ret)) {
//         ret = dist;
//       }
//     }
//   }
//   return ret;
// }

Eigen::Matrix3d rollPitchYawToMatrix(double roll, double pitch, double yaw) {
  auto cy = std::cos(yaw);
  auto cp = std::cos(pitch);
  auto cr = std::cos(roll);
  auto sy = std::sin(yaw);
  auto sp = std::sin(pitch);
  auto sr = std::sin(roll);

  Eigen::Matrix3d ret;
  ret << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp,
      sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr;
  return ret;
}

Eigen::Vector4d rollPitchYawToQuaternion(double roll, double pitch,
                                         double yaw) {
  auto sr = std::sin(roll / 2);
  auto cr = std::cos(roll / 2);
  auto sp = std::sin(pitch / 2);
  auto cp = std::cos(pitch / 2);
  auto sy = std::sin(yaw / 2);
  auto cy = std::cos(yaw / 2);
  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  Eigen::Vector4d ret;
  ret << x, y, z, w;
  return ret;
}

Eigen::Matrix3d quaternionToMatrix(double w, double x, double y, double z) {
  auto xy = x * y;
  auto wz = w * z;
  auto xz = x * z;
  auto wy = w * y;
  auto yz = y * z;
  auto wx = w * x;
  auto x2 = x * x;
  auto y2 = y * y;
  auto z2 = z * z;

  Eigen::Matrix3d ret;
  ret << 1 - 2 * (y2 + z2), 2 * (xy - wz), 2 * (xz + wy), 2 * (xy + wz),
      1 - 2 * (x2 + z2), 2 * (yz - wx), 2 * (xz - wy), 2 * (yz + wx),
      1 - 2 * (x2 + y2);
  return ret;
}

Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z) {
  Eigen::Vector3d ret;
  ret << std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
      std::asin(2 * (w * y - z * x)),
      std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return ret;
}

double calcCurvature(const Point2D &pt1, const Point2D &pt2,
                     const Point2D &pt3) {
  double a = std::sqrt(std::pow((pt1.x() - pt2.x()), 2) +
                       std::pow((pt1.y() - pt2.y()), 2));
  double b = std::sqrt(std::pow((pt1.x() - pt3.x()), 2) +
                       std::pow((pt1.y() - pt3.y()), 2));
  double c = std::sqrt(std::pow((pt2.x() - pt3.x()), 2) +
                       std::pow((pt2.y() - pt3.y()), 2));
  double r = (a * b * c) /
             std::sqrt((a + b - c) * (a - b + c) * (b + c - a) * (a + b + c));
  return 1 / r;
}

float getPolyLaneYCoordinate(std::vector<float> coeff,
                             const float &x_coordinate) {
  const int fitting_dim = coeff.size();
  float res = 0.f;
  for (int index = int(fitting_dim - 1); index >= 0; --index)
    res = res * x_coordinate + coeff[index];
  return res;
}

float getPolyLaneYGradient(const float &x_coordinate,
                           std::vector<float> coeff) {
  const int fitting_dim = coeff.size();
  float res = 0.f;
  for (int index = int(fitting_dim - 1); index >= 1; --index)
    res = res * x_coordinate + coeff[index] * index;
  return res;
}

} // namespace worldmodel_v1
} // namespace msd_worldmodel
