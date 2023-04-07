#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
using namespace Eigen;

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

bool isAngleGreaterThan90Degrees(const Vector &v1, const Vector &v2)
{
    double dot = dotProduct(v1, v2);
    double len1 = vectorLength(v1);
    double len2 = vectorLength(v2);
    double cosTheta = dot / (len1 * len2);
    return cosTheta < 0;
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
bool compareTuple(const std::tuple<int, Point> &t1, const std::tuple<int, Point> &t2)
{
    return std::get<0>(t1) < std::get<0>(t2);
}

int main()
{
    // setp1 粗拟合
    auto trajectory = read_trajectory("/home/thj/Desktop/TestProject/test_curvefitting/newfitting/trajectory.txt");

    // 分段拟合圆弧
    std::vector<Circle> circles;
    const int segmentLength = 10;                               // 每段轨迹的长度
    std::vector<std::tuple<int, Point>> shift_points = {};      // 换挡点
    std::vector<std::tuple<int, Point>> inflection_points = {}; // 折点

    // 识别换挡点
    for (int i = 0; i < trajectory.size() - 2; ++i)
    {
        if (isAngleGreaterThan90Degrees(Vector{trajectory[i].x, trajectory[i].y, trajectory[i + 1].x, trajectory[i + 1].y},
                                        Vector{trajectory[i + 1].x, trajectory[i + 1].y, trajectory[i + 2].x, trajectory[i + 2].y}))
            shift_points.emplace_back(std::make_tuple(i, Point{trajectory[i + 1].x, trajectory[i + 1].y}));
    }
    // 识别折点
    for (int i = 0; i < trajectory.size(); i += segmentLength)
    {
        inflection_points.emplace_back(std::make_tuple(i, Point{trajectory[i].x, trajectory[i].y}));
    }

    // 剔除相邻点
    for (int i = 1; i < inflection_points.size(); ++i)
    {
        for (int j = 0; j < shift_points.size(); ++j)
        {
            if (std::abs(std::get<0>(inflection_points[i]) - std::get<0>(shift_points[j])) < (segmentLength / 2)){
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
    if(trajectory.size() - std::get<0>(inflection_points.back()) > (segmentLength / 2))
        inflection_points.emplace_back(std::make_tuple(static_cast<int>(trajectory.size() - 1), trajectory.back()));
    else{
        inflection_points.erase(inflection_points.end());
        inflection_points.emplace_back(std::make_tuple(static_cast<int>(trajectory.size() - 1), trajectory.back()));
    }

    // 拟合圆弧
    for (int i = 0; i < inflection_points.size() - 1; ++i)
    {
        std::vector<Point> segmentPoints(trajectory.begin() + std::get<0>(inflection_points[i]), trajectory.begin() + std::get<0>(inflection_points[i + 1]) + 1);

        // std::for_each(segmentPoints.begin(), segmentPoints.end(), [](Point point){
        //     std::cout << point.x << " ";
        // });
        // std::cout << std::endl;
        Circle circle = fitCircleToPoints(segmentPoints);
        static Circle circle_last = {};
        std::cerr << std::sqrt(std::pow(circle.centerX - circle_last.centerX, 2) + std::pow(circle.centerY - circle_last.centerY, 2)) << " "
                  << std::abs(circle.radius - circle_last.radius) << std::endl;
        // 粗拟合，合并相似轨迹
        if (std::sqrt(std::pow(circle.centerX - circle_last.centerX, 2) + std::pow(circle.centerY - circle_last.centerY, 2)) < 0.1 &&
            std::abs(circle.radius - circle_last.radius) < 0.1)
                inflection_points.erase(inflection_points.begin() + i);
        memcpy(&circle_last, &circle, sizeof(Circle));
    }


    // 拟合圆弧
    for (int i = 0; i < inflection_points.size() - 1; ++i)
    {
        std::vector<Point> segmentPoints(trajectory.begin() + std::get<0>(inflection_points[i]), trajectory.begin() + std::get<0>(inflection_points[i + 1]) + 1);

        // std::for_each(segmentPoints.begin(), segmentPoints.end(), [](Point point){
        //     std::cout << point.x << " ";
        // });
        // std::cout << std::endl;
        Circle circle = fitCircleToPoints(segmentPoints);
        circles.emplace_back(circle);
    }



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

    for (int i = 0; i < inflection_points.size(); i++)
    {
        std::cout << std::get<0>(inflection_points[i]) << " " << std::get<1>(inflection_points[i]).x << " " << std::get<1>(inflection_points[i]).y << std::endl;
    }

    // step 2: 细拟合
    return 0;
}
