#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

const double PI = 3.14159265358979323846;

struct Point
{
    double x, y, yaw;
};

struct Arc
{
    Point start_point, end_point;
    double radius, start_yaw, end_yaw;
};

double get_distance(Point p1, Point p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double get_angle(Point p1, Point p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

void fit_arc(const Point& p1, const Point& p2, double r, Arc& arc)
{
    Point mid = {(p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, 0.0};
    double d = get_distance(p1, p2);
    double chord_length = sqrt(4 * r * r - d * d) / 2.0;
    double angle = p1.yaw;
    double start_yaw = angle - PI / 2.0;
    double end_yaw = start_yaw + asin(chord_length / r) * 2;
    arc.start_point.x = mid.x + r * cos(start_yaw);
    arc.start_point.y = mid.y + r * sin(start_yaw);
    arc.start_point.yaw = start_yaw;
    arc.end_point.x = mid.x + r * cos(end_yaw);
    arc.end_point.y = mid.y + r * sin(end_yaw);
    arc.end_point.yaw = end_yaw;
    arc.radius = r;
    arc.start_yaw = start_yaw;
    arc.end_yaw = end_yaw;
}


void fit_arcs(const std::vector<Point>& points, double r, std::vector<Arc>& arcs)
{
    arcs.clear();
    if (points.size() < 2) {
        return;
    }
    Arc arc;
    for (int i = 0; i < points.size() - 1; i++) {
        fit_arc(points[i], points[i + 1], r, arc);
        if (i == 0) {
            arcs.push_back(arc);
        } else {
            // Check if the end of previous arc is the same as the start of current arc
            if (fabs(arcs.back().end_point.x - arc.start_point.x) < 1e-6 &&
                fabs(arcs.back().end_point.y - arc.start_point.y) < 1e-6 &&
                fabs(arcs.back().end_yaw - arc.start_yaw) < 1e-6) {
                arcs.back().end_point = arc.end_point;
                arcs.back().end_yaw = arc.end_yaw;
            } else {
                arcs.push_back(arc);
            }
        }
    }
}

int main()
{
    std::vector<Point> points1 = {
        {0, 0, 0},
        {5, 0, 0},
        {5, 5, 0},
        {0, 5, 0},
        {0, 0, 0}
    };
    std::vector<Arc> arcs1;
    fit_arcs(points1, 2, arcs1);
    for (const auto& arc : arcs1) {
        std::cout << "Start Point: (" << arc.start_point.x << ", " << arc.start_point.y << ")" << std::endl;
        std::cout << "End Point: (" << arc.end_point.x << ", " << arc.end_point.y << ")" << std::endl;
        std::cout << "Radius: " << arc.radius << std::endl;
        std::cout << "Start Yaw: " << arc.start_yaw << std::endl;
        std::cout << "End Yaw: " << arc.end_yaw << std::endl << std::endl;
    }

    std::vector<Point> points2 = {
        {0, 0, 0},
        {5, 0, 0},
        {5, 5, 0},
        {0, 5, 0},
        {0, 0, 0}
    };
    std::vector<Arc> arcs2;
    fit_arcs(points2, 4, arcs2);
    for (const auto& arc : arcs2) {
        std::cout << "Start Point: (" << arc.start_point.x << ", " << arc.start_point.y << ")" << std::endl;
        std::cout << "End Point: (" << arc.end_point.x << ", " << arc.end_point.y << ")" << std::endl;
        std::cout << "Radius: " << arc.radius << std::endl;
        std::cout << "Start Yaw: " << arc.start_yaw << std::endl;
        std::cout << "End Yaw: " << arc.end_yaw << std::endl << std::endl;
    }

    std::vector<Point> points3 = {
        {0, 0, 0},
        {5, 0, 0},
        {5, 5, 0},
        {0, 5, 0},
        {0, 0, 0}
    };
    std::vector<Arc> arcs3;
    fit_arcs(points3, 1, arcs3);
    for (const auto& arc : arcs3) {
        std::cout << "Start Point: (" << arc.start_point.x << ", " << arc.start_point.y << ")" << std::endl;
        std::cout << "End Point: (" << arc.end_point.x << ", " << arc.end_point.y << ")" << std::endl;
        std::cout << "Radius: " << arc.radius << std::endl;
        std::cout << "Start Yaw: " << arc.start_yaw << std::endl;
        std::cout << "End Yaw: " << arc.end_yaw << std::endl << std::endl;
    }

    return 0;
}
// 