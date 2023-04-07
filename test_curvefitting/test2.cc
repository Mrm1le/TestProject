#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

// 计算两个点之间的距离
double distance(Vector2d p1, Vector2d p2)
{
    return sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
}

// 计算两个向量的夹角
double angle(Vector2d v1, Vector2d v2)
{
    return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

// 计算圆弧的半径、起始角度和终止角度
void arc_params(Vector2d p1, Vector2d p2, Vector2d p3, double &radius, double &start_angle, double &end_angle)
{
    Vector2d v1 = p1 - p2;
    Vector2d v2 = p3 - p2;
    double d1 = distance(p1, p2);
    double d2 = distance(p3, p2);
    double a = angle(v1, v2);
    if (a < 1e-6)
    {
        radius = INFINITY;
        start_angle = end_angle = 0.0;
    }
    else
    {
        radius = d1 / (2 * sin(a / 2));
        Vector2d center = p2 + (p1 - p2).rotate(-M_PI / 2).normalized() * radius;
        start_angle = atan2(p1(1) - center(1), p1(0) - center(0));
        end_angle = atan2(p3(1) - center(1), p3(0) - center(0));
        if (end_angle < start_angle)
            end_angle += 2 * M_PI;
    }
}

// 计算圆弧的三个控制点
void arc_control_points(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d &c1, Vector2d &c2)
{
    double radius, start_angle, end_angle;
    arc_params(p1, p2, p3, radius, start_angle, end_angle);
    if (radius == INFINITY)
    {
        c1 = p2;
        c2 = p2;
    }
    else
    {
        double angle = (start_angle + end_angle) / 2;
        Vector2d center = p2 + (p1 - p2).rotate(-M_PI / 2).normalized() * radius;
        c1 = center + radius * (p1 - center).rotate(-M_PI / 2).normalized();
        c2 = center + radius * (p3 - center).rotate(-M_PI / 2).normalized();
    }
}

// 残差函数
struct Residual
{
    Residual(Vector2d p1, Vector2d p2, Vector2d p3) : p1_(p1), p2_(p2), p3_(p3) {}
    template <typename T>
    booloperator()(const T *const params, T *residual) const
    {
        T cx = params[0];
        T cy = params[1];
        T radius = params[2];
        T start_angle = params[3];
        T end_angle = params[4];
        Vector2d c(cx, cy);
        Vector2<T> p1(p1_(0), p1_(1));
        Vector2<T> p2(p2_(0), p2_(1));
        Vector2<T> p3(p3_(0), p3_(1));
        Vector2<T> v1 = p1 - c;
        Vector2<T> v2 = p2 - c;
        Vector2<T> v3 = p3 - c;
        T a1 = atan2(v1.y(), v1.x());
        T a2 = atan2(v2.y(), v2.x());
        T a3 = atan2(v3.y(), v3.x());
        if (a1 > a2)
            a2 += 2 * M_PI;
        if (a2 > a3)
            a3 += 2 * M_PI;
        residual[0] = (a2 - a1) - (end_angle - start_angle);
        residual[1] = (a3 - a2) - (end_angle - start_angle);
        residual[2] = (v1.norm() - radius);
        residual[3] = (v3.norm() - radius);
        return true;
    }
    const Vector2d p1_;
    const Vector2d p2_;
    const Vector2d p3_;
};

// 多段圆弧拟合
void fit_arcs(const vector<Vector2d> &points, vector<Vector2d> &centers, vector<double> &radii, vector<double> &start_angles, vector<double> &end_angles)
{
    int n = points.size();
    if (n < 3)
        return;
    centers.clear();
    radii.clear();
    start_angles.clear();
    end_angles.clear();

    // 初始化参数
    VectorXd params(5);
    params << 0, 0, 1, 0, 0;

    // 拟合第一段圆弧
    double r1, a1, a2;
    arc_params(points[0], points[1], points[2], r1, a1, a2);
    centers.push_back((points[0] + points[1]) / 2);
    radii.push_back(r1);
    start_angles.push_back(a1);
    end_angles.push_back(a2);

    // 拟合中间的圆弧段
    for (int i = 1; i < n - 2; i++)
    {
        Vector2d c1, c2;
        arc_control_points(points[i - 1], points[i], points[i + 1], c1, c2);
        Vector2d p1 = (points[i] + c1) / 2;
        Vector2d p2 = points[i];
        Vector2d p3 = (points[i] + c2) / 2;
        double r, a3, a4;
        arc_params(p1, p2, p3, r, a3, a4);

        // 优化参数
        ceres::Problem problem;
        Residual *residual = new Residual(p1, p2, p3);
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<Residual, 4, 5>(residual), nullptr, params.data());
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        // 更新拟合结果
        centers.push_back(Vector2d(params[0], params[1]));
        radii.push_back(params[2]);
        start_angles.push_back(a3);
        end_angles.push_back(a4);
    }

    // 拟合最后一段圆弧
    double r2, a3, a4;
    arc_params(points[n - 3], points[n - 2], points[n - 1], r2, a3, a4);
    centers.push_back((points[n - 2] + points[n - 1]) / 2);
    radii.push_back(r2);
    start_angles.push_back(a3);
    end_angles.push_back(a4);
}

// 测试
int main()
{
    vector<Vector2d> points = {{0, 0}, {0.5, 0.5}, {1, 0.5}, {1, 1}, {1.5, 1}, {2, 1}, {2, 2}};
    vector<Vector2d> centers;
    vector<double> radii;
    vector<double> start_angles;
    vector<double> end_angles;
    fit_arcs(points, centers, radii, start_angles, end_angles);
    for (int i = 0; i < centers.size(); i++)
    {
        printf("center: (%f, %f), radius: %f, start_angle: %f, end_angle: %f\n", centers[i].x(), centers[i].y(), radii[i], start_angles[i], end_angles[i]);
    }
    return 0;
}
