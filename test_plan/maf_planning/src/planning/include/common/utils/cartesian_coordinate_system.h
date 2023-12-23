
#ifndef __CARTESIAN_COORDINATE_SYSTEM_H__
#define __CARTESIAN_COORDINATE_SYSTEM_H__

#include <cmath>

#include "pnc/define/geometry.h"

const double PI = 3.1415926;

struct Segment2D {
  Point2D point[2];
};

double CrossProduct(const Segment2D &s1, const Segment2D &s2);
double DotProduct(const Segment2D &s1, const Segment2D &s2);

double Norm(const Segment2D &s1);

double PointsSquareDistance(const Point2D &p1, const Point2D &p2);

// Cartesian state of a point
struct CartesianState {
  double x;
  double y;
  double speed;
  double yaw;
  double curvature;
  double acceleration;
};

class CartesianCoordinateSystem {
private:
  Pose2D m_origin;

public:
  CartesianCoordinateSystem(Pose2D origin);
  Point2D CartCoord2CurCartCoord(const Pose2D &prev_origin,
                                 const Point2D &prev_rel_coord);
  Pose2D CartPose2CurCartPose(const Pose2D &prev_origin,
                              const Pose2D &prev_rel_pos);
  Point2D CurCartCoord2CartCoord(const Pose2D &future_origin,
                                 const Point2D &cur_rel_coord);
  Pose2D CurCartPose2CartPose(const Pose2D &future_origin,
                              const Pose2D &cur_rel_pos);
};
#endif
