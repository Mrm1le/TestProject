#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <bits/stdc++.h>
#include <algorithm>
#include <iomanip>
#include "common/math/box2d.h"
#include "trend.h"

namespace msquare{

using namespace planning_math;

Trend::Trend(const VehicleParam* vehicle_param, Pose2D goal_pose, std::vector<planning_math::LineSegment2d> obstacles){
    vehicle_param_ = vehicle_param;
    Box2d box_g(getCenterBox(goal_pose), goal_pose.theta, vehicle_param->length, vehicle_param_->width_wo_rearview_mirror);
    for(auto obs : obstacles){
        double e_box_dis = box_g.DistanceTo(obs);
        double e_cir_dis = obs.DistanceTo(getCenterCircle(goal_pose)) - vehicle_param_->width / 2;
        if(e_box_dis > safe_distance_  && e_cir_dis > safe_distance_){
            obstacles_.push_back(obs);
        }
    }
}

std::vector<double> Trend::calculate_trend(const std::vector<Pose2D>& traj){
    if(obstacles_.empty()){
        std::cout << "There is no obstacle in this scenario !" << std::endl;
        return std::vector<double>(traj.size());
    }

    if(traj.empty()){
        std::cout << "There is no trajectory !" << std::endl;
        return std::vector<double>(traj.size());
    }

    std::vector<double> result;
    result.emplace_back(0.0);
    Box2d box_s(getCenterBox(traj[0]), traj[0].theta, vehicle_param_->length, vehicle_param_->width_wo_rearview_mirror);
    double record_dis = DBL_MAX;
    for(auto obs : obstacles_){
        double box_dis = box_s.DistanceTo(obs);
        double cir_dis = obs.DistanceTo(getCenterCircle(traj[0])) - vehicle_param_->width / 2;
        double min_dis = std::min(box_dis, cir_dis);
        record_dis = std::min(record_dis, min_dis);
    }

    std::ofstream out("/home/ros/Downloads/trend.txt", std::ios::app);
    if(out.fail()){
        std::cout<<"fail to write trend.txt!" <<std::endl;
    }
    out << traj[0].x << " " << traj[0].y << " " << record_dis << std::endl;
    int count = 0;
    double sum = 0.0;
    double temp_worst = 0.0;
    double  bad_count = 0.0;

    for(int i = 1; i < traj.size(); ++i){
        double temp_dis = DBL_MAX;
        Box2d box_i(getCenterBox(traj[i]), traj[i].theta, vehicle_param_->length, vehicle_param_->width_wo_rearview_mirror);
        for(auto obs : obstacles_){
            double box_dis = box_i.DistanceTo(obs);
            double cir_dis = obs.DistanceTo(getCenterCircle(traj[i])) - vehicle_param_->width / 2;
            double min_dis = std::min(box_dis, cir_dis);
            temp_dis = std::min(temp_dis, min_dis);
        }

        // temp_dis or record_dis < threshold_distance; compute the trend; 
        // think other case as 0;
        double temp_value;
        if(temp_dis < threshold_distance_ || record_dis < threshold_distance_){
            temp_value = temp_dis - record_dis;
            if(temp_value < 0){
                bad_count += 1.0;
            }
            count ++;
        }else{
            temp_value = 0.0;
        }
        sum += temp_value;
        temp_worst = std::min(temp_worst, temp_value);
        result.emplace_back(temp_value);
        out << traj[i].x << " " << traj[i].y << " " << temp_value << std::endl;
        record_dis = temp_dis;
    }
    set_bad_rate(bad_count, traj.size());
    set_worst_value(temp_worst);
    if(count > 0){
        set_ave_value(sum, static_cast<double>(count));
    }
    out.close();
    return result;
}

}