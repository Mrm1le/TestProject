#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

struct Point {
    double x;
    double y;
    double heading;
};
void fit_circles(const std::vector<Point>& points, std::vector<std::tuple<Point, Point, double, double, double>>& circles) {
    // 矩阵大小
    const int rows = points.size() - 1;
    const int cols = 3;

    // 构造系数矩阵和目标矩阵
    Eigen::MatrixXd A(rows, cols);
    Eigen::VectorXd b(rows);

    for (int i = 0; i < rows; i++) {
        double xi = points[i].x;
        double yi = points[i].y;
        double xi1 = points[i + 1].x;
        double yi1 = points[i + 1].y;
        double hi = points[i].heading;

        A(i, 0) = xi - xi1;
        A(i, 1) = yi - yi1;
        A(i, 2) = std::tan(hi) * (xi - xi1) - (yi - yi1);
        b(i) = (xi - xi1) * (xi - xi1) + (yi - yi1) * (yi - yi1);
    }

    // 求解最小二乘解
    Eigen::VectorXd x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // 生成圆弧
    double cx, cy, r, theta1, theta2;
    Point p1, p2;
    for (int i = 0; i < rows; i++) {
        p1 = points[i];
        p2 = points[i + 1];
        cx = (p1.x + p2.x) / 2.0 + (p1.y - p2.y) * x(2) / (2.0 * x(0) * x(2) - x(1) * x(1));
        cy = (p1.y + p2.y) / 2.0 + (p2.x - p1.x) * x(2) / (2.0 * x(1) * x(2) - x(0) * x(1));
        r = std::sqrt((p1.x - cx) * (p1.x - cx) + (p1.y - cy) * (p1.y - cy));
        theta1 = std::atan2(p1.y - cy, p1.x - cx);
        theta2 = std::atan2(p2.y - cy, p2.x - cx);

        // 调整角度差距
        double delta_theta = theta2 - theta1;
        if (delta_theta > M_PI) {
            delta_theta -= 2 * M_PI;
        } else if (delta_theta < -M_PI) {
            delta_theta += 2 * M_PI;
        }

        // 添加圆弧
        circles.emplace_back(p1, p2, r, theta1, theta1 + delta_theta);
    }
}

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

int main() {
    // std::vector<Point> points = {{0, 0, 0},
    //                              {1, 1, M_PI / 4},
    //                              {2, 0, -M_PI / 4},
    //                              {3, 1, M_PI / 4},
    //                              {4, 0, -M_PI / 4},
    //                              {5, 1, M_PI / 4},
    //                              {6, 0, -M_PI / 4},
    //                              {7, 1, M_PI / 4},
    //                              {8, 0, -M_PI / 4},
    //                              {9, 1, M_PI / 4}};

    std::vector<Point> points = read_trajectory("trajectory.txt");

    std::vector<std::tuple<Point, Point, double, double, double>> circles;
    fit_circles(points, circles);

    for (const auto& circle : circles) {
        Point p1 = std::get<0>(circle);
        Point p2 = std::get<1>(circle);
        double r = std::get<2>(circle);
        double theta1 = std::get<3>(circle);
        double theta2 = std::get<4>(circle);

        std::cout << "Circle: (" << p1.x << ", " << p1.y << ") -> (" << p2.x << ", " << p2.y << "), radius=" << r
                  << ", theta1=" << theta1 << ", theta2=" << theta2 << std::endl;
    }

    return 0;
}
