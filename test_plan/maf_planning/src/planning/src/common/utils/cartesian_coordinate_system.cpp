

#include "common/utils/cartesian_coordinate_system.h"

double CrossProduct(const Segment2D &s1, const Segment2D &s2) {
  double product;
  double xx1 = s1.point[1].x - s1.point[0].x;
  double yy1 = s1.point[1].y - s1.point[0].y;
  double xx2 = s2.point[1].x - s2.point[0].x;
  double yy2 = s2.point[1].y - s2.point[0].y;
  product = xx1 * yy2 - yy1 * xx2;
  return product;
}

double DotProduct(const Segment2D &s1, const Segment2D &s2) {
  double product;
  double xx1 = s1.point[1].x - s1.point[0].x;
  double yy1 = s1.point[1].y - s1.point[0].y;
  double xx2 = s2.point[1].x - s2.point[0].x;
  double yy2 = s2.point[1].y - s2.point[0].y;
  product = xx1 * xx2 + yy1 * yy2;
  return product;
}

double Norm(const Segment2D &s1) {
  //    double product;
  double xx1 = s1.point[1].x - s1.point[0].x;
  double yy1 = s1.point[1].y - s1.point[0].y;

  return sqrt(xx1 * xx1 + yy1 * yy1);
}

double PointsSquareDistance(const Point2D &p1, const Point2D &p2) {
  return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2);
}

CartesianCoordinateSystem::CartesianCoordinateSystem(Pose2D origin)
    : m_origin(origin) {}

Point2D CartesianCoordinateSystem::CartCoord2CurCartCoord(
    const Pose2D &prev_origin, const Point2D &prev_rel_coord) {
  // Need further validation
  double dis_to_prev_origin =
      sqrt(pow(prev_rel_coord.x, 2) + pow(prev_rel_coord.y, 2));
  double prev_vector_angle = atan2l(prev_rel_coord.y, prev_rel_coord.x);
  double xabs = prev_origin.x +
                dis_to_prev_origin * cos(prev_vector_angle + prev_origin.theta);
  double yabs = prev_origin.y +
                dis_to_prev_origin * sin(prev_vector_angle + prev_origin.theta);
  ;

  double cur_vector_angle =
      atan2l(yabs - m_origin.y, xabs - m_origin.x) - m_origin.theta;
  double dis_to_cur_origin =
      sqrt(pow(yabs - m_origin.y, 2) + pow(xabs - m_origin.x, 2));

  Point2D cur_rel_coord;
  cur_rel_coord.x = dis_to_cur_origin * cos(cur_vector_angle);
  cur_rel_coord.y = dis_to_cur_origin * sin(cur_vector_angle);
  return cur_rel_coord;
}
Pose2D
CartesianCoordinateSystem::CartPose2CurCartPose(const Pose2D &prev_origin,
                                                const Pose2D &prev_rel_pos) {
  // Need further validation
  double dis_to_prev_origin =
      sqrt(pow(prev_rel_pos.x, 2) + pow(prev_rel_pos.y, 2));
  double prev_vector_angle = atan2l(prev_rel_pos.y, prev_rel_pos.x);
  double xabs = prev_origin.x +
                dis_to_prev_origin * cos(prev_vector_angle + prev_origin.theta);
  double yabs = prev_origin.y +
                dis_to_prev_origin * sin(prev_vector_angle + prev_origin.theta);
  ;

  double cur_vector_angle =
      atan2l(yabs - m_origin.y, xabs - m_origin.x) - m_origin.theta;
  double dis_to_cur_origin =
      sqrt(pow(yabs - m_origin.y, 2) + pow(xabs - m_origin.x, 2));

  Pose2D cur_rel_pose;
  cur_rel_pose.x = dis_to_cur_origin * cos(cur_vector_angle);
  cur_rel_pose.y = dis_to_cur_origin * sin(cur_vector_angle);

  double yawshift = m_origin.theta - prev_origin.theta;
  cur_rel_pose.theta = prev_rel_pos.theta - yawshift;
  cur_rel_pose.theta = (cur_rel_pose.theta > PI) ? (cur_rel_pose.theta - 2 * PI)
                                                 : cur_rel_pose.theta;
  cur_rel_pose.theta = (cur_rel_pose.theta < -PI)
                           ? (cur_rel_pose.theta + 2 * PI)
                           : cur_rel_pose.theta;

  return cur_rel_pose;
}

Point2D CartesianCoordinateSystem::CurCartCoord2CartCoord(
    const Pose2D &future_origin, const Point2D &cur_rel_coord) {
  // Need further validation
  double dis_to_cur_origin =
      sqrt(pow(cur_rel_coord.x, 2) + pow(cur_rel_coord.y, 2));
  double cur_vector_angle = atan2l(cur_rel_coord.y, cur_rel_coord.x);
  double xabs =
      m_origin.x + dis_to_cur_origin * cos(cur_vector_angle + m_origin.theta);
  double yabs =
      m_origin.y + dis_to_cur_origin * sin(cur_vector_angle + m_origin.theta);
  ;

  double future_vector_angle =
      atan2l(yabs - future_origin.y, xabs - future_origin.x) -
      future_origin.theta;
  double dis_to_future_origin =
      sqrt(pow(yabs - future_origin.y, 2) + pow(xabs - future_origin.x, 2));

  Point2D future_rel_coord;
  future_rel_coord.x = dis_to_future_origin * cos(future_vector_angle);
  future_rel_coord.y = dis_to_future_origin * sin(future_vector_angle);
  return future_rel_coord;
}
Pose2D
CartesianCoordinateSystem::CurCartPose2CartPose(const Pose2D &future_origin,
                                                const Pose2D &cur_rel_pos) {
  // Need further validation
  double dis_to_cur_origin =
      sqrt(pow(cur_rel_pos.x, 2) + pow(cur_rel_pos.y, 2));
  double cur_vector_angle = atan2l(cur_rel_pos.y, cur_rel_pos.x);
  double xabs =
      m_origin.x + dis_to_cur_origin * cos(cur_vector_angle + m_origin.theta);
  double yabs =
      m_origin.y + dis_to_cur_origin * sin(cur_vector_angle + m_origin.theta);
  ;

  double future_vector_angle =
      atan2l(yabs - future_origin.y, xabs - future_origin.x) -
      future_origin.theta;
  double dis_to_future_origin =
      sqrt(pow(yabs - future_origin.y, 2) + pow(xabs - future_origin.x, 2));

  Pose2D future_rel_pose;
  future_rel_pose.x = dis_to_future_origin * cos(future_vector_angle);
  future_rel_pose.y = dis_to_future_origin * sin(future_vector_angle);
  double yawshift = future_origin.theta - m_origin.theta;
  future_rel_pose.theta = cur_rel_pos.theta - yawshift;
  future_rel_pose.theta = (future_rel_pose.theta > PI)
                              ? (future_rel_pose.theta - 2 * PI)
                              : future_rel_pose.theta;
  future_rel_pose.theta = (future_rel_pose.theta < -PI)
                              ? (future_rel_pose.theta + 2 * PI)
                              : future_rel_pose.theta;

  return future_rel_pose;
}
