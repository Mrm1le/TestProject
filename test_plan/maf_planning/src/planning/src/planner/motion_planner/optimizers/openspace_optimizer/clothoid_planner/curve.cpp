#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/curve.h"
namespace clothoid {

void transformS2(const ClothoidCurve &c1, ClothoidCurve &c2) {
  ClothoidCurve c0;
  mirrorX(c1, c0);
  moveCurve(c0, c2, 0, -c0[0].y);
}
void transformS3(const ClothoidCurve &c1, ClothoidCurve &c2) {
  mirrorY(c1, c2);
}
void transformS4(const ClothoidCurve &c1, ClothoidCurve &c2) {
  ClothoidCurve c01, c02;
  mirrorY(c1, c01);
  mirrorX(c01, c02);
  moveCurve(c02, c2, -c02.front().x, -c02.front().y);
}

void mirrorX(const ClothoidCurve &c1, ClothoidCurve &c2) {
  unsigned int size = c1.size();
  c2.resize(size);
  for (int i = 0; i < size; i++) {
    c2[i].x = c1[i].x;
    c2[i].y = c1[size - i - 1].y;
    c2[i].theta = -c1[size - i - 1].theta;
    c2[i].s = c1[size - i - 1].s;
    c2[i].k = c1[i].k;
  }
}
void mirrorY(const ClothoidCurve &c1, ClothoidCurve &c2) {
  unsigned int size = c1.size();
  c2.resize(size);
  for (int i = 0; i < size; i++) {
    c2[i].x = c1[i].x;
    c2[i].y = -c1[i].y;
    c2[i].theta = -c1[i].theta;
    c2[i].s = c1[i].s;
    c2[i].k = c1[i].k;
  }
}
void moveCurve(const ClothoidCurve &c1, ClothoidCurve &c2, double x, double y) {
  unsigned int size = c1.size();
  c2.resize(size);
  for (int i = 0; i < size; i++) {
    c2[i].x = c1[i].x + x;
    c2[i].y = c1[i].y + y;
    c2[i].theta = c1[i].theta;
    c2[i].s = c1[i].s;
    c2[i].k = c1[i].k;
  }
}

void rotateCurve(const ClothoidCurve &c1, ClothoidCurve &c2, double theta) {
  unsigned int size = c1.size();
  c2.resize(size);
  double cc1 = std::cos(theta);
  double ss1 = std::sin(theta);
  for (int i = 0; i < size; i++) {
    c2[i].x = c1[i].x * cc1 - ss1 * c1[i].y;
    c2[i].y = c1[i].x * ss1 + cc1 * c1[i].y;
    c2[i].theta = c1[i].theta + theta;
    c2[i].s = c1[i].s;
    c2[i].k = c1[i].k;
  }
}
void transform2D(const ClothoidCurve &c1, ClothoidCurve &c2, double x, double y,
                 double theta) {
  ClothoidCurve c0;
  rotateCurve(c1, c0, theta);
  moveCurve(c0, c2, x, y);
}

void transform2D(const StraightCurve &c1, StraightCurve &c2, double x,
                 double y) {
  for (auto &p : c1) {
    c2.emplace_back(p.x + x, p.y + y, p.theta);
  }
}

StraightCurve linspace(const StraightPoint &p1, const StraightPoint &p2,
                       double step) {
  double s_all = std::hypot(p1.x - p2.x, p1.y - p2.y);
  double direction =
      std::cos(p1.theta) * (p2.x - p1.x) + std::sin(p1.theta) * (p2.y - p1.y);
  std::vector<double> s_list =
      linspace(0, (direction > 0 ? s_all : -s_all), step);
  StraightCurve path;
  path.reserve(s_list.size());
  double cs = std::cos(p1.theta);
  double ss = std::sin(p1.theta);
  for (auto s : s_list) {
    StraightPoint p0;
    p0.x = p1.x + cs * s;
    p0.y = p1.y + ss * s;
    p0.theta = p1.theta;
    path.push_back(p0);
  }
  return path;
}

std::vector<double> linspace(double s, double e, double step) {
  step = std::abs(step);
  if (std::abs(s - e) < 1e-6) {
    return std::vector<double>{s};
  }
  if (std::abs(s - e) < step) {
    return std::vector<double>{s, e};
  }
  int step_num = int(std::abs(e - s) / step) + 1;
  double step_nor = (e - s) / step_num;
  std::vector<double> res(step_num + 1);
  for (int i = 0; i <= step_num; i++) {
    res[i] = s + i * step_nor;
  }

  return res;
}
StraightCurve linspace(const StraightPoint &s1, double s, double step) {
  std::vector<double> s_list = linspace(0, s, step);
  StraightCurve path;
  path.reserve(s_list.size());
  double cs = std::cos(s1.theta);
  double ss = std::sin(s1.theta);
  for (auto s : s_list) {
    path.emplace_back(s1.x + cs * s, s1.y + ss * s, s1.theta);
  }
  return path;
}
/**
 * @brief counter-clockwise from bottom point as the start
 *
 * @param s1
 * @param theta
 * @param theta_step
 * @param c
 * @return CircularCurve
 */
CircularCurve linspaceCCW(const CircularPoint &s1, double theta,
                          double theta_step, Circle c) {
  CircularCurve cir1;
  int theta_step_num = int(std::abs(theta) / theta_step) + 1;
  double theta_step_nor = theta / theta_step_num;
  double theta_temp;
  for (int i = 0; i <= theta_step_num; i++) {
    theta_temp = s1.theta + i * theta_step_nor;
    cir1.emplace_back(c.x + c.r * std::sin(theta_temp),
                      c.y - c.r * std::cos(theta_temp), theta_temp);
  }
  return cir1;
}

/**
 * @brief counter-clockwise from top point as the start
 *
 * @param s1
 * @param theta
 * @param theta_step
 * @param c
 * @return CircularCurve
 */
CircularCurve linspaceCCWTop(const CircularPoint &s1, double theta,
                             double theta_step, Circle c) {
  CircularCurve cir1;
  int theta_step_num = int(std::abs(theta) / theta_step) + 1;
  double theta_step_nor = theta / theta_step_num;
  double theta_temp;
  for (int i = 0; i <= theta_step_num; i++) {
    theta_temp = s1.theta + i * theta_step_nor;
    cir1.emplace_back(c.x - c.r * std::sin(theta_temp),
                      c.y - c.r + c.r * std::cos(theta_temp), theta_temp);
  }
  return cir1;
}

/**
 * @brief clockwise and from left point
 *
 * @param theta1
 * @param theta2
 * @param theta_step
 * @param circle
 * @return CircularCurve
 */
CircularCurve linspaceCW(double theta1, double theta2, double theta_step,
                         Circle c) {
  CircularCurve cir;
  double delta_theta = theta2 - theta1;
  int theta_step_num = int(std::abs(delta_theta) / theta_step) + 1;
  double theta_step_nor = delta_theta / theta_step_num;
  double theta_temp;
  for (int i = 0; i <= theta_step_num; i++) {
    theta_temp = theta1 + i * theta_step_nor;
    cir.emplace_back(c.x - c.r * std::cos(theta_temp),
                     c.y + c.r * std::sin(theta_temp), M_PI_2 - theta_temp);
  }
  return cir;
}

/**
 * @brief clockwise and from bottom point
 *
 * @param theta1
 * @param theta2
 * @param theta_step
 * @param circle
 * @return CircularCurve
 */
CircularCurve linspaceCWRight(double theta1, double theta2, double theta_step,
                              Circle c) {
  CircularCurve cir;
  double delta_theta = theta2 - theta1;
  int theta_step_num = int(std::abs(delta_theta) / theta_step) + 1;
  double theta_step_nor = delta_theta / theta_step_num;
  double theta_temp;
  for (int i = 0; i <= theta_step_num; i++) {
    theta_temp = theta1 + i * theta_step_nor;
    cir.emplace_back(c.x + c.r * std::cos(theta_temp),
                     c.y - c.r * std::sin(theta_temp), M_PI_2 - theta_temp);
  }
  return cir;
}

CircularCurve clo2cir(const ClothoidCurve &clo) {
  CircularCurve cir;
  cir.reserve(clo.size());
  for (auto &p : clo) {
    cir.emplace_back(p.x, p.y, p.theta);
  }
  return cir;
}

void rotationalSymmetry(double sx, double sy, const StraightCurve &s1,
                        StraightCurve &s2) {
  double x_x = 2 * sx;
  double y_y = 2 * sy;
  s2.clear();
  s2.reserve(s1.size());
  for (std::vector<StraightPoint>::const_iterator it = s1.end() - 1;
       it >= s1.begin(); it--) {
    s2.emplace_back(x_x - it->x, y_y - it->y, it->theta);
  }
}

} // namespace clothoid