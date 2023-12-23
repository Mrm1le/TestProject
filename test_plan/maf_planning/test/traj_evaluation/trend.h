#include <iostream>
#include <vector>
#include <chrono>
#include <bits/stdc++.h>
#include <algorithm>
#include <iomanip>
#include "common/math/math_utils.h"
#include "common/math/vec2d.h"
#include "params.h"

namespace msquare{

class Trend{
public:
    Trend(const VehicleParam* vehicle_param, Pose2D goal_pose, std::vector<planning_math::LineSegment2d> obstacles);
    ~Trend(){};

    std::vector<double> calculate_trend(const std::vector<Pose2D>& traj);
    double get_bad_rate(){
        return bad_rate_;
    }

    double get_worst_value(){
        return worst_value_;
    }

    double get_ave_value(){
        return ave_value_;
    }

    void set_bad_rate(double count, double size){
        bad_rate_ = count / size;
    }

    void set_worst_value(double value){
        worst_value_ = value;
    }

    void set_ave_value(double sum, double size){
        ave_value_ = sum / size;
    }

private:
    const VehicleParam* vehicle_param_;
    std::vector<planning_math::LineSegment2d> obstacles_;
    double safe_distance_ = 1.5;
    double threshold_distance_ = 5.0;
    double bad_rate_ = 0.0;
    double worst_value_ = 0.0;
    double ave_value_ = 0.0;

    planning_math::Vec2d getCenterBox(const Pose2D& current_pose) const{
        double center_to_geometry_center = vehicle_param_->center_to_geometry_center;
        double xx = current_pose.x + center_to_geometry_center * cos(current_pose.theta);
        double yy = current_pose.y + center_to_geometry_center * sin(current_pose.theta);
        return planning_math::Vec2d(xx, yy);
    }

    planning_math::Vec2d getCenterCircle(const Pose2D& current_pose) const{
        double center_to_geometry_center = vehicle_param_->front_edge_to_center - vehicle_param_->front_edge_to_mirror;
        double xx = current_pose.x + center_to_geometry_center * cos(current_pose.theta);
        double yy = current_pose.y + center_to_geometry_center * sin(current_pose.theta);
        return planning_math::Vec2d(xx, yy);
    }
};

}