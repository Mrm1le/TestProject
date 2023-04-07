#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <string>

// 定义点结构体
struct Point {
  double x;
  double y;
  double angle;
};

// 定义圆弧结构体
struct Arc {
  double cx;
  double cy;
  double radius;
  double angle;
};

std::vector<Point> read_trajectory(const std::string &filename)
{
    std::vector<Point> points;
    std::ifstream file(filename);
    double x, y, angle;
    while (file >> x >> y >> angle)
    {
        points.emplace_back(Point{x, y, angle});
    }
    return points;
}

/*
// 样条插值方法拟合多段圆弧
std::vector<Arc> fitArcs(const std::vector<Point>& points) {
  std::vector<Arc> arcs;  // 存储拟合出的圆弧
  int n = points.size();
  if (n < 2) return arcs;

  std::vector<double> t(n);  // 定义参数t数组
  for (int i = 0; i < n; ++i) {
    if (i == 0) {
      t[i] = 0.0;
    } else {
      double dx = points[i].x - points[i-1].x;
      double dy = points[i].y - points[i-1].y;
      t[i] = t[i-1] + std::sqrt(dx*dx + dy*dy);
    }
  }

  std::vector<double> k(n);  // 定义二次导数k数组
  k[0] = k[n-1] = 0.0;
  for (int i = 1; i < n-1; ++i) {
    double dx1 = points[i].x - points[i-1].x;
    double dy1 = points[i].y - points[i-1].y;
    double dx2 = points[i+1].x - points[i].x;
    double dy2 = points[i+1].y - points[i].y;
    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
    double a = len1 / (len1 + len2);
    double b = 1.0 - a;
    double d = 2.0 * (a/len1 + b/len2);
    k[i] = 6.0 * (dx2/dy2 - dx1/dy1) / (len1*len2*(dx1/dy1 + dx2/dy2));
  }

  // 求解圆弧参数
  for (int i = 0; i < n-1; ++i) {
    double x1 = points[i].x;
    double y1 = points[i].y;
    double x2 = points[i+1].x;
    double y2 = points[i+1].y;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double len = std::sqrt(dx*dx + dy*dy);
    double angle1 = points[i].angle;
    double angle2 = points[i+1].angle;
    double dangle = angle2 - angle1;
    if (dangle > M_PI) {
      dangle -= 2.0 * M_PI;
    } else if (dangle < -M_PI) {
      dangle += 2.0 * M_PI;
    }
    double a = std::sin(dangle - 2.0*k[i]*len) / (2.0*len);
double b = k[i] / a;
double r = std::abs(a);
double cx = x1 + a*dy;
double cy = y1 - a*dx;
double start_angle = std::atan2(y1-cy, x1-cx);
double end_angle = std::atan2(y2-cy, x2-cx);
if (dangle < 0.0) {
  std::swap(start_angle, end_angle);
}
if (start_angle > end_angle) {
  end_angle += 2.0 * M_PI;
}
Arc arc = {cx, cy, r, start_angle, end_angle};
arcs.push_back(arc);

}

return arcs;
}


// // RANSAC算法拟合圆弧曲线
// Arc fitArcRANSAC(const std::vector<Point>& points, double max_error, int max_iterations) {
//   std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
//   std::uniform_int_distribution<int> dist(0, points.size()-1);
//   double best_cost = std::numeric_limits<double>::max();
//   Arc best_arc;
//   for (int iter = 0; iter < max_iterations; ++iter) {
//     // 随机选择3个点作为圆弧拟合的基础
//     std::vector<int> indices(3);
//     std::unordered_set<int> index_set;
//     while (indices.size() < 3) {
//       int index = dist(rng);
//       if (index_set.count(index) == 0) {
//         indices.push_back(index);
//         index_set.insert(index);
//       }
//     }
//     // 根据选定的3个点拟合圆弧
//     Arc arc = fitArc3(points[indices[0]], points[indices[1]], points[indices[2]]);
//     // 计算拟合误差
//     double cost = 0.0;
//     for (const Point& p : points) {
//       double d = std::abs(distanceToArc(arc, p));
//       cost += (d <= max_error) ? d : max_error;
//     }
//     // 更新最优圆弧
//     if (cost < best_cost) {
//       best_cost = cost;
//       best_arc = arc;
//     }
//   }
//   return best_arc;
// }

*/

double angleDifference(double a1, double a2) {
    double diff = fmod(a2 - a1 + M_PI, 2 * M_PI);
    if (diff < 0) {
        diff += 2 * M_PI;
    }
    return diff - M_PI;
}


// 最小二乘法拟合圆弧曲线
Arc fitArcLeastSquares(const std::vector<Point>& points) {
  double x_sum = 0.0;
  double y_sum = 0.0;
  double xy_sum = 0.0;
  double x2_sum = 0.0;
  double y2_sum = 0.0;
  for (const Point& p : points) {
    x_sum += p.x;
    y_sum += p.y;
    xy_sum += p.x * p.y;
    x2_sum += p.x * p.x;
    y2_sum += p.y * p.y;
  }
  double n = static_cast<double>(points.size());
  double x_mean = x_sum / n;
  double y_mean = y_sum / n;
  double u1 = x2_sum + y2_sum - n * (x_mean * x_mean + y_mean * y_mean);
  double u2 = x_sum - n * x_mean;
  double u3 = y_sum - n * y_mean;
  double u4 = x2_sum - n * x_mean * x_mean;
  double u5 = xy_sum - n * x_mean * y_mean;
  double u6 = y2_sum - n * y_mean * y_mean;
  double denom = u1 * (u4 * u6 - u5 * u5) - u2 * (u2 * u6 - u3 * u5) + u5 * (u2 * u3 - u4 * u3);
  if (std::abs(denom) < 1e-8) {
    return Arc(0.0, 0.0, 0.0, std::numeric_limits<double>::max());
  }
  double x0 = (u4 * u6 - u5 * u5) * u2 + u5 * (u3 * u4 - u5 * u2) - u6 * (u2 * u3 - u4 * u3);
  double y0 = (u4 * u6 - u5 * u5) * u3 + u5 * (u2 * u3 - u4 * u3) - u4 * (u2 * u3 - u4 * u3);
  x0 /= denom;
  y0 /= denom;
  double r = std::sqrt((u2 * u2 + u3 * u3 + x0 * x0 + y0 * y0 - 2.0 * (u2 * x0 + u3 * y0)) / n);
  double theta1 = std::atan2(points[0].y - y0, points[0].x - x0);
  double theta2 = std::atan2(points.back().y - y0, points.back().x - x0);
  double angle = angleDifference(theta2, theta1);
  return Arc(x0, y0, r, angle);
}


void write_curve(const std::vector<Arc> &circles, const std::string &filename)
{
    std::ofstream file(filename);
    for (const auto &circle : circles)
    {
        file << circle.cx << " " << circle.cy << " " << circle.radius << "\n";
    }
}

int main() {
// std::vector<Point> points = {{0.0, 0.0, 0.0},
// {1.0, 0.0, 0.0},
// {2.0, 1.0, M_PI/4},
// {3.0, 2.0, M_PI/2},
// {4.0, 3.0, M_PI*3/4},
// {5.0, 4.0, M_PI}};
auto points = read_trajectory("trajectory.txt");
std::vector<Arc> arcs = fitArcLeastSquares(points);
for (const auto& arc : arcs) {
std::cout << "cx = " << arc.cx << ", cy = " << arc.cy
<< ", radius = " << arc.radius
<< ", start_angle = " << arc.start_angle
<< ", end_angle = " << arc.end_angle << std::endl;
}

write_curve(arcs, "curve.txt");
return 0;
}


// 上述代码定义了一个Point结构体，用于存储轨迹点的横坐标、纵坐标和航向角信息；定义了一个Arc结构体，用于存储拟合出的圆弧的圆心坐标、半径和起始角度、终止角度信息。

// fitArcs函数通过样条插值方法拟合多段圆弧，该方法先计算轨迹点之间的距离，并根据距离计算出参数t数组；然后根据轨迹点之间的方向角差和距离，计算出二次导数k数组；
// 最后，根据轨迹点之间的距离和角度差，计算出每个圆弧的圆心坐标、半径和起始角度、终止角度信息，并存储在arcs数组中返回。

// 在主函数中，定义了一个示例点集，调用fitArcs函数拟合圆弧，并输出拟合出的圆心坐标、半径和起始角度、终止角度信息。

// 需要注意的是，上述代码只是一种简单的圆弧拟合方法，可能无法满足所有的应用需求。在实际应用中，还需要考虑诸如拟合精度、计算效率、对异常点的处理等问题，需要根据具体应用场景进行相应的改进和

// 田谷歌
// 继续

// 改进。另外，上述代码只是实现了圆弧的拟合，如果需要将拟合出的圆弧用于机器人路径规划等应用，还需要将圆弧离散化为一系列路径点，例如使用等距离离散化或等时间离散化等方法。

// 下面是一个等距离离散化的示例代码：

// c++

// 将拟合出的圆弧等距离离散化为一系列路径点
// std::vector<Point> discreteArc(const Arc& arc, double step) {
//   std::vector<Point> points;
//   double len = std::abs(arc.radius * (arc.end_angle - arc.start_angle));
//   int n = std::ceil(len / step);
//   if (n < 2) {
//     n = 2;
//   }
//   double dtheta = (arc.end_angle - arc.start_angle) / (n - 1);
//   for (int i = 0; i < n; ++i) {
//     double theta = arc.start_angle + i*dtheta;
//     double x = arc.cx + arc.radius * std::cos(theta);
//     double y = arc.cy + arc.radius * std::sin(theta);
//     double yaw = theta + M_PI/2.0;
//     points.push_back({x, y, yaw});
//   }
//   return points;
// }

// 该代码定义了一个discreteArc函数，输入参数为拟合出的圆弧和等距离步长，输出为一系列路径点。
// 该函数首先计算出圆弧的长度len，然后根据步长计算出等距离离散化的路径点数n，如果n小于2，则强制设置为2；
// 最后根据圆弧的起始角度、终止角度和等距离步长，计算出每个路径点的横坐标、纵坐标和航向角，并存储在points数组中返回。

// 需要注意的是，上述等距离离散化的方法会产生离散化误差，因此在实际应用中，需要根据实际需求和场景选择合适的离散化方法，并进行相应的误差控制。

