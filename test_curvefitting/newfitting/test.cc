#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
using namespace Eigen;

enum Direction
{
    Forward = 0,
    Backward,
    ForwardLeft,
    ForwardRight,
    BackwardLeft,
    BackwardRight,
    Unknown
};

// 定义轨迹点结构体
struct Point
{
    double x; // x 坐标
    double y; // y 坐标
    double heading;
};

struct Vector
{
    double startX;
    double startY;
    double endX;
    double endY;
};
// 定义圆弧结构体
struct Circle
{
    double centerX; // 圆心 x 坐标
    double centerY; // 圆心 y 坐标
    double radius;  // 半径
    Point start_point;
    Point end_point;
};

double dotProduct(const Vector &v1, const Vector &v2)
{
    double deltaX1 = v1.endX - v1.startX;
    double deltaY1 = v1.endY - v1.startY;
    double deltaX2 = v2.endX - v2.startX;
    double deltaY2 = v2.endY - v2.startY;
    return deltaX1 * deltaX2 + deltaY1 * deltaY2;
}

double vectorLength(const Vector &v)
{
    double deltaX = v.endX - v.startX;
    double deltaY = v.endY - v.startY;
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

bool isAngleGreater90(const Vector &v1, const Vector &v2)
{
    double dot = dotProduct(v1, v2);
    double len1 = vectorLength(v1);
    double len2 = vectorLength(v2);
    double cosTheta = dot / (len1 * len2);
    return cosTheta < 0;
}

// 判断向量和给定角度之间的夹角是否大于90度
bool isAngleGreater90(const Vector &v, double angleRad)
{
    double vLength = vectorLength(v);
    double dotProduct = (v.endX - v.startX) * std::cos(angleRad) + (v.endY - v.startY) * std::sin(angleRad);
    return dotProduct < 0;
}

// 判断圆弧的绘制方向
bool IsCircleClockWise(const Circle &circle)
{
    // 计算起点相对于圆心的极角
    double startAngle = std::atan2(circle.start_point.y - circle.centerY, circle.start_point.x - circle.centerX);
    // std::cout << "circle.start_point.y: " << circle.start_point.y << " circle.centerY: " << circle.centerY << std::endl;
    // std::cout << "circle.start_point.x: " << circle.start_point.x << " circle.centerX: " << circle.centerX << std::endl;
    // std::cout << "startAngle: " << startAngle << std::endl;
    // 计算终点相对于圆心的极角
    double endAngle = std::atan2(circle.end_point.y - circle.centerY, circle.end_point.x - circle.centerX);

    // std::cout << "circle.end_point.y: " << circle.end_point.y << " circle.centerY: " << circle.centerY << std::endl;
    // std::cout << "circle.end_point.x: " << circle.end_point.x << " circle.centerX: " << circle.centerX << std::endl;
    // std::cout << "endAngle: " << endAngle << std::endl;
    // 计算极角差值
    double angleDiff = endAngle - startAngle;
    // 将极角差值限制在 [-π, π] 范围内
    if (angleDiff > M_PI)
        angleDiff -= 2 * M_PI;
    else if (angleDiff < -M_PI)
        angleDiff += 2 * M_PI;

    //  std::cout << "angleDiff: " << angleDiff << std::endl;
    // 判断绘制方向
    if (angleDiff > 0) // 极角差值大于零，判定为逆时针
        return false;
    else
        return true;
}

// 使用最小二乘法进行圆弧拟合
Circle fitCircleToPoints(const std::vector<Point> &points)
{
    int n = points.size();
    MatrixXd A(n, 3);
    VectorXd b(n);

    for (int i = 0; i < n; i++)
    {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = 1;
        b(i) = -1 * points[i].x * points[i].x - points[i].y * points[i].y;
    }

    VectorXd x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);

    double centerX = -x(0) / 2;
    double centerY = -x(1) / 2;
    double radius = std::sqrt(centerX * centerX + centerY * centerY - x(2));

    Circle circle;
    circle.centerX = centerX;
    circle.centerY = centerY;
    circle.radius = radius;
    circle.start_point = points.front();
    circle.end_point = points.back();
    return circle;
}

std::vector<Point> read_trajectory(const std::string &filename)
{
    std::vector<Point> points;
    std::ifstream file(filename);
    double x, y, heading;
    while (file >> x >> y >> heading)
    {
        points.emplace_back(Point{x, y, heading});
    }
    return points;
}

// 比较函数，按照元组中的第一个 int 值进行升序排序
bool compareTuple(const std::tuple<int, Point, Direction> &t1, const std::tuple<int, Point, Direction> &t2)
{
    return std::get<0>(t1) < std::get<0>(t2);
}

int main()
{
    // setp1 粗拟合
    auto trajectory = read_trajectory("/home/thj/Desktop/TestProject/test_curvefitting/newfitting/trajectory.txt");

    // 分段拟合圆弧
    std::vector<Circle> circles;
    const int segmentLength = 10;                                              // 每段轨迹的长度
    std::vector<std::tuple<int, Point, Direction>> shift_points = {};          // 换挡点(id, coordinate， direction)
    std::vector<std::tuple<int, Point, Direction>> inflection_points = {};     // 折点(id, coordinate， direction)
    std::vector<std::tuple<int, Point, Direction>> new_inflection_points = {}; // 折点(id, coordinate， direction) 补全方向信息
    std::vector<std::tuple<int, Point, Direction>> inflection_points_ = {};    // 折点(id, coordinate， direction) 最终的折点信息

    // 添加起点到折点信息中
    if (isAngleGreater90(Vector{trajectory[0].x, trajectory[0].y, trajectory[1].x, trajectory[1].y}, trajectory[0].heading)) // Backward
        shift_points.emplace_back(std::make_tuple(0, Point{trajectory[0].x, trajectory[0].y}, Backward));
    else
        shift_points.emplace_back(std::make_tuple(0, Point{trajectory[0].x, trajectory[0].y}, Forward));

    // 识别换挡点
    for (int i = 0; i < trajectory.size() - 2; ++i)
    {
        if (isAngleGreater90(Vector{trajectory[i].x, trajectory[i].y, trajectory[i + 1].x, trajectory[i + 1].y},
                             Vector{trajectory[i + 1].x, trajectory[i + 1].y, trajectory[i + 2].x, trajectory[i + 2].y}))
            if (isAngleGreater90(Vector{trajectory[i + 1].x, trajectory[i + 1].y, trajectory[i + 2].x, trajectory[i + 2].y}, trajectory[i].heading))
                shift_points.emplace_back(std::make_tuple(i, Point{trajectory[i + 1].x, trajectory[i + 1].y}, Backward));
            else
                shift_points.emplace_back(std::make_tuple(i, Point{trajectory[i + 1].x, trajectory[i + 1].y}, Forward));
    }
    // 识别折点
    for (int i = segmentLength; i < trajectory.size(); i += segmentLength)
    {
        inflection_points.emplace_back(std::make_tuple(i, Point{trajectory[i].x, trajectory[i].y}, Unknown));
    }

    // 剔除相邻点
    for (int i = 1; i < inflection_points.size(); ++i)
    {
        for (int j = 0; j < shift_points.size(); ++j)
        {
            if (std::abs(std::get<0>(inflection_points[i]) - std::get<0>(shift_points[j])) < (segmentLength / 2))
            {
                inflection_points.erase(inflection_points.begin() + i);
                --i;
            }
        }
    }
    // 换挡点添加到折点中
    inflection_points.insert(inflection_points.end(), shift_points.begin(), shift_points.end());

    // 使用 std::sort 对 折点 进行排序，传入自定义的比较函数 compareTuple
    std::sort(inflection_points.begin(), inflection_points.end(), compareTuple);

    // 把轨迹上最后一个点添加到折点
    if (trajectory.size() - std::get<0>(inflection_points.back()) > (segmentLength / 2))
        inflection_points.emplace_back(std::make_tuple(static_cast<int>(trajectory.size() - 1), trajectory.back(), Unknown));
    else
    {
        inflection_points.erase(inflection_points.end());
        inflection_points.emplace_back(std::make_tuple(static_cast<int>(trajectory.size() - 1), trajectory.back(), Unknown));
    }

    // 补全折点的方向信息
    for (int i = 0; i < inflection_points.size(); ++i)
    {
        if (std::get<2>(inflection_points[i]) == Unknown)
        {
            new_inflection_points.emplace_back(std::make_tuple(std::get<0>(inflection_points[i]),
                                                               std::get<1>(inflection_points[i]),
                                                               std::get<2>(new_inflection_points[i - 1])));
        }
        else
            new_inflection_points.emplace_back(inflection_points[i]);
    }
    // 拟合圆弧
    for (int i = 0; i < new_inflection_points.size() - 1; ++i)
    {
        std::vector<Point> segmentPoints(trajectory.begin() + std::get<0>(new_inflection_points[i]),
                                         trajectory.begin() + std::get<0>(new_inflection_points[i + 1]) + 1);

        // std::for_each(segmentPoints.begin(), segmentPoints.end(), [](Point point){
        //     std::cout << point.x << " ";
        // });
        // std::cout << std::endl;
        Circle circle = fitCircleToPoints(segmentPoints);
        static Circle circle_last = {};
        // std::cerr << std::sqrt(std::pow(circle.centerX - circle_last.centerX, 2) + std::pow(circle.centerY - circle_last.centerY, 2)) << " "
        //           << std::abs(circle.radius - circle_last.radius) << std::endl;
        // 粗拟合，合并相似轨迹
        if (std::sqrt(std::pow(circle.centerX - circle_last.centerX, 2) + std::pow(circle.centerY - circle_last.centerY, 2)) < 0.1 &&
            std::abs(circle.radius - circle_last.radius) < 0.1)
            new_inflection_points.erase(new_inflection_points.begin() + i);
        memcpy(&circle_last, &circle, sizeof(Circle));
    }

    // 拟合圆弧
    for (int i = 0; i < new_inflection_points.size() - 1; ++i)
    {
        std::vector<Point> segmentPoints(trajectory.begin() + std::get<0>(new_inflection_points[i]), trajectory.begin() +
                                                                                                         std::get<0>(new_inflection_points[i + 1]) + 1);

        // std::for_each(segmentPoints.begin(), segmentPoints.end(), [](Point point){
        //     std::cout << point.x << " ";
        // });
        // std::cout << std::endl;
        Circle circle = fitCircleToPoints(segmentPoints);
        circles.emplace_back(circle);
    }

    for (int i = 0; i < circles.size(); ++i)
    {
        if (IsCircleClockWise(circles[i]))
        {
            if (std::get<2>(new_inflection_points[i]) == Forward)
                inflection_points_.emplace_back(std::make_tuple(std::get<0>(new_inflection_points[i]), std::get<1>(new_inflection_points[i]), ForwardRight));
            else if (std::get<2>(new_inflection_points[i]) == Backward)
                inflection_points_.emplace_back(std::make_tuple(std::get<0>(new_inflection_points[i]), std::get<1>(new_inflection_points[i]), BackwardLeft));
            else
                inflection_points_.emplace_back(std::make_tuple(std::get<0>(new_inflection_points[i]), std::get<1>(new_inflection_points[i]), std::get<2>(inflection_points_[i - 1])));
        }
        else
        {
            if (std::get<2>(new_inflection_points[i]) == Forward)
                inflection_points_.emplace_back(std::make_tuple(std::get<0>(new_inflection_points[i]), std::get<1>(new_inflection_points[i]), ForwardLeft));
            else if (std::get<2>(new_inflection_points[i]) == Backward)
                inflection_points_.emplace_back(std::make_tuple(std::get<0>(new_inflection_points[i]), std::get<1>(new_inflection_points[i]), BackwardRight));
            else
                inflection_points_.emplace_back(std::make_tuple(std::get<0>(new_inflection_points[i]), std::get<1>(new_inflection_points[i]), std::get<2>(inflection_points_[i - 1])));
        }
    }
    inflection_points_.emplace_back(std::make_tuple(std::get<0>(new_inflection_points.back()), std::get<1>(new_inflection_points.back()), std::get<2>(inflection_points_.back())));

    // 圆弧交界处平滑处理
    // for (int i = 0; i < circles.size() - 1; i++)
    // {
    //     Circle &currentCircle = circles[i];
    //     Circle &nextCircle = circles[i + 1];
    //     double distance = std::sqrt((currentCircle.centerX - nextCircle.centerX) * (currentCircle.centerX - nextCircle.centerX) +
    //                                 (currentCircle.centerY - nextCircle.centerY) * (currentCircle.centerY - nextCircle.centerY));

    //     // 如果相邻圆弧之间的距离小于两个圆弧半径之和，则进行平滑处理
    //     if (distance < currentCircle.radius + nextCircle.radius)
    //     {
    //         double ratio = currentCircle.radius / (currentCircle.radius + nextCircle.radius);
    //         nextCircle.centerX = currentCircle.centerX + ratio * (nextCircle.centerX - currentCircle.centerX);
    //         nextCircle.centerY = currentCircle.centerY + ratio * (nextCircle.centerY - currentCircle.centerY);
    //     }
    // }

    // 输出拟合得到的圆弧信息
    for (int i = 0; i < circles.size(); i++)
    {
        std::cout << circles[i].centerX << " " << circles[i].centerY << " " << circles[i].radius << " " << circles[i].start_point.x << " " << circles[i].start_point.y << " " << circles[i].end_point.x << " " << circles[i].end_point.y;
        std::cout << std::endl;
    }

    for (int i = 0; i < inflection_points_.size(); i++)
    {
        std::cout << std::get<0>(inflection_points_[i]) << " " << std::get<1>(inflection_points_[i]).x << " "
                  << std::get<1>(inflection_points_[i]).y << " " << std::get<2>(inflection_points_[i]) << std::endl;
    }

    // step 2: 细拟合
    return 0;
}
