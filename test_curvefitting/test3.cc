#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

using namespace std;

// 定义一个表示点的结构体
struct Point
{
    double x;
    double y;
    double heading;
};

// 定义一个表示圆弧的结构体
struct Arc
{
    Point start;
    Point end;
    double radius;
    double startAngle;
    double endAngle;
};

// 定义一个计算两点之间距离的函数
double distance(Point p1, Point p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return sqrt(dx * dx + dy * dy);
}

// 定义一个计算圆弧拟合误差的函数
double arcError(Point start, Point end, double radius, double startAngle, double endAngle)
{
    double error = 0;
    double step = (endAngle - startAngle) / 100.0;
    for (double angle = startAngle; angle < endAngle; angle += step)
    {
        Point p;
        p.x = start.x + radius * cos(angle);
        p.y = start.y + radius * sin(angle);
        double d = distance(p, end);
        if (d > error)
        {
            error = d;
        }
    }
    return error;
}

// 定义一个使用最小二乘法拟合圆弧的函数
Arc fitArc(Point *points, int start, int end)
{
    double x1 = points[start].x;
    double y1 = points[start].y;
    double x2 = points[end].x;
    double y2 = points[end].y;
    double dx = x2 - x1;
    double dy = y2 - y1;
    double d = sqrt(dx * dx + dy * dy);
    double heading = atan2(dy, dx);
    double chord = distance(points[start], points[end]);
    double radius = chord / 2.0 + pow(d, 2.0) / (8.0 * chord);
    double distanceToCenter = sqrt(pow(radius, 2.0) - pow(chord / 2.0, 2.0));
    Point center;
    center.x = x1 + (chord / 2.0 + distanceToCenter) * cos(heading + M_PI / 2.0);
    center.y = y1 + (chord / 2.0 + distanceToCenter) * sin(heading + M_PI / 2.0);
    double startAngle = atan2(points[start].y - center.y, points[start].x - center.x);
    double endAngle = atan2(points[end].y - center.y, points[end].x - center.x);
    double error = arcError(points[start], points[end], radius, startAngle, endAngle);
    Arc arc;
    arc.start = points[start];
    arc.end = points[end];
    arc.radius = radius;
    arc.startAngle = startAngle;
    arc.endAngle = endAngle;
    return arc;
}

// 定义一个使用Ramer-Douglas-Peucker算法拟合多段圆弧的函数
vector<Arc> fitArcs(Point *points, int start, int end, double tolerance)
{
    vector<Arc> arcs;
    double maxError = 0;
    int index = 0;
    for (int i = start + 1; i < end - 1; i++)
    {
        double error = arcError(points[start], points[end], distance(points[start], points[end]) / 2.0, 0, M_PI);
        if (error > maxError)
        {
            maxError = error;
            index = i;
        }
    }
    if (maxError > tolerance)
    {
        vector<Arc> leftArcs = fitArcs(points, start, index, tolerance);
        vector<Arc> rightArcs = fitArcs(points, index, end, tolerance);
        arcs.insert(arcs.end(), leftArcs.begin(), leftArcs.end() - 1);
        arcs.insert(arcs.end(), rightArcs.begin(), rightArcs.end());
    }
    else
    {
        Arc arc = fitArc(points, start, end);
        arcs.push_back(arc);
    }
    return arcs;
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

int main()
{
    // 假设已经读入了轨迹点并存储在一个Point类型的vector中;
    vector<Point> points = read_trajectory("trajectory.txt");

    // 定义拟合误差容限
    double tolerance = 0.1;

    // 对整个轨迹点序列进行圆弧拟合
    vector<Arc> arcs = fitArcs(&points[0], 0, points.size() - 1, tolerance);
    std::cout << arcs.size() << std::endl;

    // 输出每个圆弧的起点、终点、半径、起点角度、终点角度
    for (int i = 0; i < arcs.size(); i++)
    {
        Arc arc = arcs[i];
        cout << "Arc " << i + 1 << ": (" << arc.start.x << ", " << arc.start.y << ") to (" << arc.end.x << ", " << arc.end.y << "), radius = " << arc.radius << ", start angle = " << arc.startAngle << ", end angle = " << arc.endAngle << endl;
    }

    return 0;
}

//
// 需要注意的是，上述代码中的圆弧拟合算法并不是完美的，可能会出现一些误差。如果您需要更高精度的圆弧拟合算法，可以考虑使用其他算法，如曲线拟合算法。
