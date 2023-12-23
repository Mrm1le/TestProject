#include "common/math/math_utils.h"
#include <iostream>

Pose2D calc_projection_point(const Pose2D &point1, const Pose2D &point2,
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

Pose2D calc_projection_point2(const Pose2D &point1, const Pose2D &point2,
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

int main(){
    Pose2D p1, p2, p3, p4;
    p1 = Pose2D(1.0, 1.0, 0.0);
    p2 = Pose2D(3.0, 3.0, 0.0);
    p3 = Pose2D(2.0, 1.0, 0.0);
    std::vector<Pose2D> p3_s = {
        Pose2D(2.0, 1.0, 0.0),
        Pose2D(1.0, 0.0, 0.0),
        Pose2D(2.0, -3.0, 0.0),
        Pose2D(7.0, 1.0, 0.0)
    };

    for(auto p: p3_s){
        p4 = calc_projection_point(p1, p2, p);
        std::cout<<p4.x << " " << p4.y;
        p4 = calc_projection_point2(p1, p2, p);
        std::cout<<" bf:"<<p4.x << " " << p4.y<<std::endl;

    }



    p4 = calc_projection_point(p1, p2, p3);
    std::cout<<p4.x << " " << p4.y<<std::endl;

    return 0;
}