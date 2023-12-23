#pragma once

#include <Eigen/Eigen>
#include <limits>

namespace msd_worldmodel {
namespace worldmodel_v1 {

typedef Eigen::Vector2d Point2D;
typedef Eigen::Vector3d Point3D;
typedef Eigen::Matrix2Xd Points2D; // (2xn) matrix
typedef Eigen::Matrix3Xd Points3D; // (3xn) matrix

constexpr double deg_to_rad = M_PI / 180.0;
constexpr double earth_radius = 6378.137;
constexpr float invalid_distance = std::numeric_limits<float>::max();

// hdmap::data::Point to Point3D
/***
template <typename T> Point3D toPoint3D(const T &pt) {
  return Point3D{pt.x(), pt.y(), pt.z()};
}

template <typename T> Point2D toPoint2D(const T &pt) {
  return Point2D{pt.x(), pt.y()};
}

inline Point3D toPoint3D(const Point2D &pt) {
  return Point3D{pt.x(), pt.y(), 0};
}
***/

// Calculate scan line distance from P(x0, y0) to a segment A(x1, y1) - B(x2,
// y2)
double scanLineToSegment(const Point2D &origin, const Point2D &A,
                         const Point2D &B);

Point2D rotatePoint(const Eigen::Matrix2d &rotate_matrix,
                    const Point2D &origin);
Eigen::Matrix2d getRoateMatrix(const double &yaw);

// Calculate the rotate matrix from Euler pose
Eigen::Matrix3d rollPitchYawToMatrix(double roll, double pitch, double yaw);

// Calculate the quaternion from Euler post
Eigen::Vector4d rollPitchYawToQuaternion(double roll, double pitch, double yaw);

// Calculate the rotate matrix from quaternion
Eigen::Matrix3d quaternionToMatrix(double w, double x, double y, double z);

// Calculate the Euler pose from quaternion (in RPY)
Eigen::Vector3d quaternionToEuler(double w, double x, double y, double z);

// Calculate curvature by 3 points
double calcCurvature(const Point2D &pt1, const Point2D &pt2,
                     const Point2D &pt3);

float calculateDistance(const float &x1, const float &y1, const float &x2,
                        const float &y2);

float getPolyLaneYCoordinate(std::vector<float> coeff,
                             const float &x_coordinate);
float getPolyLaneYGradient(const float &x_coordinate, std::vector<float> coeff);

} // namespace worldmodel_v1
} // namespace msd_worldmodel
