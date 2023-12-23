/*
 * polygon.hpp
 *
 *  Created on: Apr 13, 2020
 *      Author: dozen
 */

#ifndef SRC_MODULES_MATH_ALGORITHMS_POLYGON_POLYGON_HPP_
#define SRC_MODULES_MATH_ALGORITHMS_POLYGON_POLYGON_HPP_

#include <Eigen/Core>
#include <iostream>
#include <stack>
#include <vector>

namespace putils {

class Polygon {
public:
  static double getPolygonArea(const std::vector<Eigen::Vector2d> &polygon) {
    double area = 0.f;
    for (size_t i = 0; i < polygon.size(); i++) {
      size_t j = (i + 1) % polygon.size();
      area += polygon[i].x() * polygon[j].y();
      area -= polygon[i].y() * polygon[j].x();
    }
    return 0.5f * std::abs(area);
  }

  static bool pointInPolygon(const Eigen::Vector2d &point,
                             const std::vector<Eigen::Vector2d> &polygon) {
    for (auto &p : polygon) {
      if (std::fabs(p.y() - point.y()) < 1e-5 &&
          std::fabs(p.x() - point.x()) < 1e-5)
        return true;
    }
    // 水平/垂直交叉点数判别法（适用于任意多边形）
    bool is_in_polygon = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
      bool flag1 =
          ((polygon[i].y() >= point.y()) != (polygon[j].y() >= point.y()));
      bool flag2 = (point.x() <= (polygon[j].x() - polygon[i].x()) *
                                         (point.y() - polygon[i].y()) /
                                         (polygon[j].y() - polygon[i].y()) +
                                     polygon[i].x());
      if (flag1 && flag2)
        is_in_polygon = !is_in_polygon;
    }
    return is_in_polygon;
  }
  static bool pointInAnyPolygons(
      const Eigen::Vector2d &point,
      const std::vector<std::vector<Eigen::Vector2d>> &polygons) {
    for (auto &p : polygons) {
      if (pointInPolygon(point, p)) {
        return true;
      }
    }
    return false;
  }

  static double
  getIntersectPolygonArea(const std::vector<Eigen::Vector2d> &track_polygon,
                          const std::vector<Eigen::Vector2d> &det_polygon) {
    std::vector<Eigen::Vector2d> cross_polygon;
    getIntersectPolygon(track_polygon, det_polygon, cross_polygon);

    return getPolygonArea(cross_polygon);
  }

  static void
  getIntersectPolygon(const std::vector<Eigen::Vector2d> &track_polygon,
                      const std::vector<Eigen::Vector2d> &det_polygon,
                      std::vector<Eigen::Vector2d> &cross_polygon) {
    cross_polygon.clear();
    // get point in polygon
    for (auto &it : track_polygon) {
      if (pointInPolygon(it, det_polygon))
        cross_polygon.push_back(it);
    }
    for (auto &it : det_polygon) {
      if (pointInPolygon(it, track_polygon))
        cross_polygon.push_back(it);
    }

    // deal equal point
    std::vector<int> equal_id;
    for (size_t i = 0; i < cross_polygon.size(); i++) {
      for (size_t j = i + 1; j < cross_polygon.size(); j++) {
        if (std::fabs(cross_polygon[i].x() - cross_polygon[j].x()) < 1e-5 &&
            std::fabs(cross_polygon[i].y() - cross_polygon[j].y()) < 1e-5)
          equal_id.push_back(j); // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
      }
    }
    std::sort(equal_id.begin(), equal_id.end());
    int num = 0;
    for (auto &id : equal_id) {
      cross_polygon.erase(cross_polygon.begin() + id - num);
      num++;
    }

    // get point of two line intersection
    for (size_t i = 0; i < track_polygon.size(); i++) {
      Eigen::Vector2d track_P0 = track_polygon[i];
      Eigen::Vector2d track_P1 = track_polygon[(i + 1) % track_polygon.size()];
      for (size_t j = 0; j < det_polygon.size(); j++) {
        Eigen::Vector2d det_P0 = det_polygon[j];
        Eigen::Vector2d det_P1 = det_polygon[(j + 1) % det_polygon.size()];

        Eigen::Vector2d inter_point;
        if (segmentIntersection(track_P0, track_P1, det_P0, det_P1,
                                inter_point))
          cross_polygon.push_back(inter_point);
      }
    }
    pointOrderedFromPolygon(cross_polygon);
  }

  static bool segmentIntersection(const Eigen::Vector2d &track_P0,
                                  const Eigen::Vector2d &track_P1,
                                  const Eigen::Vector2d &det_P0,
                                  const Eigen::Vector2d &det_P1,
                                  Eigen::Vector2d &inter_point) {
    double t_delta_x = track_P1.x() - track_P0.x();
    double t_delta_y = track_P1.y() - track_P0.y();
    double d_delta_x = det_P1.x() - det_P0.x();
    double d_delta_y = det_P1.y() - det_P0.y();

    double td_delta_x = track_P0.x() - det_P0.x();
    double td_delta_y = track_P0.y() - det_P0.y();

    double area = t_delta_x * d_delta_y - d_delta_x * t_delta_y;
    double area_1 = t_delta_x * td_delta_y - t_delta_y * td_delta_x;
    double area_2 = d_delta_x * td_delta_y - d_delta_y * td_delta_x;

    if (std::fabs(area) < 1e-5) // todo::get point in coincide line
      return false;
    bool is_positive_area = area > 0;
    if ((area_1 < 0.f) == is_positive_area)
      return false;
    if ((area_2 < 0.f) == is_positive_area)
      return false;
    if ((area_1 >= area) == is_positive_area ||
        (area_2 >= area) == is_positive_area)
      return false;

    double ratio = area_2 / area;
    inter_point.x() = track_P0.x() + ratio * t_delta_x;
    inter_point.y() = track_P0.y() + ratio * t_delta_y;
    return true;
  }

  static inline int comparePointWithAngle(const void *a, const void *b) {
    if (((Eigen::Vector3d *)a)->z() < ((Eigen::Vector3d *)b)->z())
      return -1;
    else if (((Eigen::Vector3d *)a)->z() > ((Eigen::Vector3d *)b)->z())
      return 1;
    else
      return 0;
  }

  static inline bool comparePointWithAngleSort(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
    if (a.z() < b.z())
      return false;
    else if (a.z() > b.z())
      return true;
    else
      return false;
  }

  static void pointOrderedFromPolygon(std::vector<Eigen::Vector2d> &polygon) {
    if (polygon.empty())
      return;

    Eigen::Vector2d center_point = Eigen::Vector2d::Zero();
    for (auto &it : polygon) {
      center_point.x() += it.x();
      center_point.y() += it.y();
    }
    center_point.x() /= polygon.size();
    center_point.y() /= polygon.size();

    std::vector<Eigen::Vector3d> points_with_angle = {};
    for (size_t i = 0; i < polygon.size(); i++) {
      Eigen::Vector3d temp_point;
      temp_point.x() = polygon[i].x();
      temp_point.y() = polygon[i].y();
      temp_point.z() = // z is actually angle
          atan2((polygon[i].y() - center_point.y()),
                (polygon[i].x() - center_point.x()));
      points_with_angle.emplace_back(temp_point);
    }
    std::sort(points_with_angle.begin(),points_with_angle.end(),comparePointWithAngleSort);

    for (size_t i = 0; i < polygon.size(); i++) {
      polygon[i].x() = points_with_angle[i].x();
      polygon[i].y() = points_with_angle[i].y();
    }
  }

  // A utility function to find next to top in a stack
  static Eigen::Vector2d nextToTop(std::stack<Eigen::Vector2d> &S) {
    Eigen::Vector2d p = S.top();
    S.pop();
    Eigen::Vector2d res = S.top();
    S.push(p);
    return res;
  }

  // To find orientation of ordered triplet (p, q, r).
  // The function returns following values
  // 0 --> p, q and r are colinear
  // 1 --> Clockwise
  // 2 --> Counterclockwise
  static int orientation(Eigen::Vector2d p, Eigen::Vector2d q,
                         Eigen::Vector2d r) {
    int val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);

    if (val == 0)
      return 0;               // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
  }

  // A function used by library function qsort() to sort an array of
  // points with respect to the first point
  static int compareAngleWithFirstPoint(const void *vp1, const void *vp2,
                                        void *vp0) {
    Eigen::Vector2d *p0 = (Eigen::Vector2d *)vp0;
    Eigen::Vector2d *p1 = (Eigen::Vector2d *)vp1;
    Eigen::Vector2d *p2 = (Eigen::Vector2d *)vp2;

    // Find orientation
    int o = orientation(*p0, *p1, *p2);
    if (o == 0)
      return ((*p0 - *p2).squaredNorm() >= (*p0 - *p1).squaredNorm()) ? -1 : 1;

    return (o == 2) ? -1 : 1;
  }

  // 求凸包多边形
  static void getConvexHull(const std::vector<Eigen::Vector2d> &input_polygon,
                            std::vector<Eigen::Vector2d> &convex_hull) {
    std::vector<Eigen::Vector2d> polygon = input_polygon;
    convex_hull.clear();
    if (polygon.size() < 3) {
      return;
    }

    int n = polygon.size();

    // Find the bottommost point
    int ymin = polygon[0][1], min = 0;
    for (int i = 1; i < n; i++) {
      int y = polygon[i][1];
      // Pick the bottom-most or chose the left most point in case of tie
      if ((y < ymin) || (ymin == y && polygon[i][0] < polygon[min][0]))
        ymin = polygon[i][1], min = i;
    }

    // Place the bottom-most point at first position
    std::swap(polygon[0], polygon[min]);

    // Sort n-1 points with respect to the first point.
    // A point p1 comes before p2 in sorted output if p2
    // has larger polar angle (in counterclockwise
    // direction) than p1
    // qsort_r(&polygon[1], n-1, sizeof(Eigen::Vector2d),
    // compareAngleWithFirstPoint, &polygon[0]);
    std::sort(polygon.begin() + 1, polygon.end(),
              [&](Eigen::Vector2d &a, Eigen::Vector2d &b) {
                return compareAngleWithFirstPoint(&a, &b, &polygon[0]) < 0;
              });

    // If two or more points make same angle with p0,
    // Remove all but the one that is farthest from p0
    // Remember that, in above sorting, our criteria was
    // to keep the farthest point at the end when more than
    // one points have same angle.
    int m = 1; // Initialize size of modified array
    for (int i = 1; i < n; i++) {
      // Keep removing i while angle of i and i+1 is same
      // with respect to p0
      while (i < n - 1 &&
             orientation(polygon[0], polygon[i], polygon[i + 1]) == 0)
        i++; // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"

      polygon[m] = polygon[i];
      m++; // Update size of modified array
    }

    // If modified array of points has less than 3 points,
    // convex hull is not possible
    if (m < 3) {
      return;
    }

    // Create an empty stack and push first three points
    // to it.
    std::stack<Eigen::Vector2d> S;
    S.push(polygon[0]);
    S.push(polygon[1]);
    S.push(polygon[2]);

    // Process remaining n-3 points
    for (int i = 3; i < m; i++) {
      // Keep removing top while the angle formed by
      // points next-to-top, top, and points[i] makes
      // a non-left turn
      int ret;
      while ((ret = orientation(nextToTop(S), S.top(), polygon[i])) != 2) {
        S.pop();
        // add patch
        if (S.size() < 2) {
          (void)printf("!!! Polygon warning: convexhull stack size %d < 2\n",
                       (int)S.size());
          return;
        }
      }
      S.push(polygon[i]);
    }

    // Now stack has the output points, print contents of stack
    while (!S.empty()) {
      Eigen::Vector2d p = S.top();
      convex_hull.push_back(p);
      S.pop();
    }
  }

  // 已知两个以下角点，返回空
  // 已知三个角点，扩展为平行四边形
  // 已知四个角点，返回四边形
  static void getExtendedQuad(const std::vector<Eigen::Vector2d> &input_polygon,
                              std::vector<Eigen::Vector2d> &quad) {
    quad.clear();
    std::vector<Eigen::Vector2d> polygon = input_polygon;
    if (polygon.size() <= 2) {
      return;
    }

    if (polygon.size() == 3) {
      double a = (polygon[0] - polygon[1]).norm();
      double b = (polygon[2] - polygon[1]).norm();
      double c = (polygon[0] - polygon[2]).norm();
      int rect_point_idx = 0;
      if (a >= b && a >= c) {
        rect_point_idx = 2;
      } else if (b >= a && b >= c) {
        rect_point_idx = 0;
      } else if (c >= a && c >= b) {
        rect_point_idx = 1;
      }

      // 求扩展点
      Eigen::Vector2d extended_point(0, 0);
      for (int i = 0; i < 3; i++) {
        if (i == rect_point_idx) {
          extended_point -= polygon[i];
        } else {
          extended_point += polygon[i];
        }
      }
      polygon.push_back(extended_point);

      quad = polygon;
    }

    if (polygon.size() == 4) {
      quad = polygon;
    }

    if (polygon.size() > 4) {
      // printf("!!! Polygon warning: more than 4 pts in polygon\n");
      quad = polygon;
      return;
    }

    pointOrderedFromPolygon(quad);
    return;
  }

private:
  Polygon() = delete;
  virtual ~Polygon();
};

} // namespace putils
#endif /* SRC_MODULES_MATH_ALGORITHMS_POLYGON_POLYGON_HPP_ */
