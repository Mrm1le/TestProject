#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <bits/stdc++.h>
#include <algorithm>
#include <iomanip>
#include "common/math/box2d.h"
#include "diff_between_sides.h"

namespace msquare
{

using namespace planning_math;

DifBetSides::DifBetSides(const VehicleParam* vehicle_param, std::vector<Pose_v>& traj, std::vector<planning_math::LineSegment2d>& obstacles, std::vector<planning_math::Vec2d> &uss_points){
    vehicle_param_ = vehicle_param;
    radius_ = vehicle_param_->width / 2;
    // obstacles_ = obstacles;
    //broke the obstacles into small pieces
    for(auto obs : obstacles){
        if(obs.length() <= vehicle_param_->length) obstacles_.emplace_back(obs);
        else{
            double record_length = obs.length();
            double count = record_length / vehicle_param->length;
            double record_x = obs.start().x();
            double record_y = obs.start().y();
            double cos_theta = (obs.end().x() - obs.start().x()) / record_length;
            double sin_theta = (obs.end().y() - obs.start().y()) / record_length;
            while(record_length > 0){
                if(record_length > vehicle_param->length){
                    double temp_end_x = record_x + vehicle_param->length * cos_theta;
                    double temp_end_y = record_y + vehicle_param->length * sin_theta;
                    planning_math::LineSegment2d temp_obs({record_x, record_y}, {temp_end_x, temp_end_y});
                    obstacles_.emplace_back(temp_obs);
                    record_x = temp_end_x;
                    record_y = temp_end_y;
                }else{
                    planning_math::LineSegment2d temp_obs({record_x, record_y}, {obs.end().x(), obs.end().y()});
                    obstacles_.emplace_back(temp_obs);
                }
                record_length -= vehicle_param->length;
            }
        }
    }
    traj_ = traj;
    points_ = uss_points;
}

double DifBetSides::calculate_diff(const Pose_v& pose_v){
    bool v_flag = pose_v.vel >= 0 ? true : false;
    Pose2D pose = pose_v.pose;
    double cos_heading = cos(pose.theta);
    double sin_heading = sin(pose.theta);
    Box2d box(getCenterBox(pose), pose.theta, vehicle_param_->length, vehicle_param_->width_wo_rearview_mirror);
    Vec2d cir_center = getCenterCircle(pose);
    std::priority_queue<DifBetSides::obstacle_priority, std::vector<DifBetSides::obstacle_priority>, DifBetSides::cmp> obs_pq;
    for(auto obs : obstacles_){
        double box_dis = box.DistanceTo(obs);
        double cir_dis = obs.DistanceTo(getCenterCircle(pose)) - radius_;
        double min_dis = std::min(box_dis, cir_dis);
        if(min_dis > 5.0) continue;
        obs_pq.push({obs, min_dis});
    }

    std::priority_queue<DifBetSides::point_priority, std::vector<DifBetSides::point_priority>, cmp_pt> pt_l_pq;
    std::priority_queue<DifBetSides::point_priority, std::vector<DifBetSides::point_priority>, cmp_pt> pt_r_pq;
    for(auto p : points_){
      auto pr = box.DistanceTo(p, v_flag);
      double cir_dis = std::hypot(p.x() - cir_center.x(), p.y() - cir_center.y()) - radius_;
      double min_dis = std::min(pr.second, cir_dis);
      if(min_dis > 5.0) continue;
      if(pr.first == 1){
        pt_r_pq.push({p, min_dis});
      }else if(pr.first == -1){
        pt_l_pq.push({p, min_dis});
      }
    }

    double l_p_min = pt_l_pq.empty() ? 5.0 : pt_l_pq.top().dis;
    double r_p_min = pt_r_pq.empty() ? 5.0 : pt_r_pq.top().dis;
    
    double left_min = 5.0;
    bool l_flag = true;
    double right_min = 5.0;
    bool r_flag = true;
    double box_x = vehicle_param_->length / 2.0;
    double box_y = vehicle_param_->width_wo_rearview_mirror / 2.0;
    while(!obs_pq.empty()){
        if(!r_flag && !l_flag) break;
        double temp_dis = obs_pq.top().dis;
        planning_math::LineSegment2d temp_obs = obs_pq.top().obstacle;
        obs_pq.pop();
        const double ref_x1 = temp_obs.start().x() - pose.x - (vehicle_param_->center_to_geometry_center) * cos_heading;
        const double ref_y1 = temp_obs.start().y() - pose.y - (vehicle_param_->center_to_geometry_center) * sin_heading;
        double x1 = ref_x1 * cos_heading + ref_y1 * sin_heading;
        double y1 = ref_x1 * sin_heading - ref_y1 * cos_heading;
        int gx1 = (x1 >= box_x ? 1 : (x1 <= -box_x ? -1 : 0));
        int gy1 = (y1 >= box_y ? 1 : (y1 <= -box_y ? -1 : 0));
        const double ref_x2 = temp_obs.end().x() - pose.x - (vehicle_param_->center_to_geometry_center) * cos_heading;
        const double ref_y2 = temp_obs.end().y() - pose.y - (vehicle_param_->center_to_geometry_center) * sin_heading;
        double x2 = ref_x2 * cos_heading + ref_y2 * sin_heading;
        double y2 = ref_x2 * sin_heading - ref_y2 * cos_heading;
        int gx2 = (x2 >= box_x ? 1 : (x2 <= -box_x ? -1 : 0));
        int gy2 = (y2 >= box_y ? 1 : (y2 <= -box_y ? -1 : 0));

        if(v_flag && x1 + box_x < 0 && x2 + box_x < 0){
            continue;
        }else if(!v_flag && x1 - box_x > 0 && x2 - box_x > 0){
            continue;
        }

        if((l_flag) && ((gy1 == gy2 && gy1 == -1) || (gy1 == -1 && std::abs(x1) < std::abs(x2)) || (gy2 == -1 && std::abs(x2) < std::abs(x1)))){
            l_flag = false;
            left_min = temp_dis;
        }else if((r_flag) && ((gy1 == gy2 && gy1 == 1) || (gy1 == 1 && std::abs(x1) < std::abs(x2)) || (gy2 == 1 && std::abs(x2) < std::abs(x1)))){
          r_flag = false;
          right_min = temp_dis;
        }
    }
    //std::cout << "left : " << left_min << " right : " << right_min <<std::endl;
    if(left_min < safe_distance_ || right_min < safe_distance_ || 
      l_p_min < safe_distance_ || r_p_min < safe_distance_){
      double l_min_dis = std::min(l_p_min, left_min);
      double r_min_dis = std::min(r_p_min, right_min); 
        if(l_min_dis < safe_distance_){
          l_min_dis = std::max(l_min_dis, CarParams::GetInstance()->lat_inflation());
        }
        if(r_min_dis < safe_distance_){
          r_min_dis = std::max(r_min_dis, CarParams::GetInstance()->lat_inflation());
        }
        double delta_dis = std::abs(l_min_dis - r_min_dis);
        return delta_dis;
    }
    return 0.0;
}

double DifBetSides::calculate_diff(){
    if(traj_.size() == 0){
        std::cout << "The path is empty !!!!!" <<std::endl;
        return 0.0;
    }
    if(obstacles_.empty()){
        std::cout << "The obs' num is zero !" << std::endl;
        return 0.0;
    }
    double sum = 0.0;
    double count = 0.0;
    // std::ofstream out("/home/ros/Downloads/diff_between_sides.txt", std::ios::app);
    // if(out.fail()){
    //     std::cout<<"fail to write !" <<std::endl;
    // }
    for(auto p : traj_){
        double record =  calculate_diff(p);
        if(record > diffest_value_){
            diffest_value_ = record;
            diffest_point_ = p.pose;
        }
        if(record > 0) count += 1.0;
        sum += record;
        // out << p.x << " " << p.y << " " << record << std::endl;
    }
    // out.close();
    bad_rate_ = count / static_cast<double> (traj_.size());
    if(count > 0){
        return sum / count;
    }else{
        return 0.0;
    }
}

}




