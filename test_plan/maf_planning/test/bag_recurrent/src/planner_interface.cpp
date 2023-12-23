
#include "planner_interface.h"
#include "pnc/define/geometry.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/pattern_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/oblique_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rpa_straight_planner.h"
#include <experimental/filesystem>
#include <iostream>

namespace planner_interface{

namespace fs = std::experimental::filesystem;

const std::string getCarSizeParams(){
    msquare::parking::CarSizeParams car_params;
    car_params.reset();
    nlohmann::json car_param_json = car_params;
    return car_param_json.dump();
}

const std::string getAstarPlannerParams(){
    msquare::parking::AstarPlannerParams planner_params;
    planner_params.reset();
    nlohmann::json planner_params_json = planner_params;
    return planner_params_json.dump();
}

void initSingletonParams(const std::string& car_params_file, const std::string& config_param_file){
    std::cout<<"car_params_file:"<<car_params_file<<std::endl;
    msquare::VehicleParam::Instance()->loadFile(car_params_file);
    msquare::HybridAstarConfig::GetInstance()->loadFile(config_param_file);
    msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(config_param_file);
    msquare::CarParams::GetInstance()->loadFile(car_params_file);
    msquare::CarParams::GetInstance()->loadFile4Plan(config_param_file);
    msquare::StrategyParams::GetInstance()->loadFile(config_param_file);

    // reset inflation parameter
    std::cout<<"lat:"<<msquare::CarParams::GetInstance()->lat_inflation()<<std::endl;
    std::cout<<"lon:"<<msquare::CarParams::GetInstance()->lon_inflation()<<std::endl;
    std::cout<<"planning type:"<<msquare::HybridAstarConfig::GetInstance()->planning_core<<std::endl;
    std::cout<<"car type:"<<msquare::VehicleParam::Instance()->car_type<<std::endl;
    std::cout<<"min_turn_radius:"<<msquare::CarParams::GetInstance()->min_turn_radius<<std::endl;
    std::cout<<"obstacle_consider_min_v:"<<msquare::CarParams::GetInstance()->car_config.lon_config.obstacle_consider_min_v<<std::endl;
    std::cout<<"dec:"<<msquare::CarParams::GetInstance()->car_config.lon_config.dec<<std::endl;
    std::cout<<"min_radius:"<<msquare::CarParams::GetInstance()->car_config.car_only_config.min_radius<<std::endl;

    std::cout<<"car type:"<<msquare::VehicleParam::Instance()->car_type<<std::endl;
    std::cout<<"min_block_len:"<<msquare::CarParams::GetInstance()->car_config.common_config.min_block_len<<std::endl;
}

std::string planInterfaceSerializeParams(std::string odo_with_params_str){
    nlohmann::json planner_input_json = nlohmann::json::parse(odo_with_params_str);
    msquare::parking::AstarPlannerInput planner_input = planner_input_json;

    msquare::parking::OpenspaceDeciderOutput temp_odo = planner_input.odo;
    planner_input.params.update();
    return planInterfaceStr(temp_odo);
}

std::string planInterfaceSerialize(std::string ods_str){
    nlohmann::json input_json = nlohmann::json::parse(ods_str);
    msquare::parking::OpenspaceDeciderOutput odo = input_json;

    msquare::SbpResult planner_res = planInterface(odo);
    std::cout << "planning!" << std::endl;
    nlohmann::json json_result = planner_res;
    return json_result.dump();
}

std::string planInterfaceStr(msquare::parking::OpenspaceDeciderOutput odo){
    msquare::SbpResult planner_res = planInterface(odo);
    nlohmann::json json_result = planner_res;
    return json_result.dump();
}

msquare::SbpResult planInterface(msquare::parking::OpenspaceDeciderOutput odo) {
    using msquare::hybrid_a_star_2::HybridAstar2;

    msquare::SbpResult planner_res;
    std::shared_ptr<msquare::SearchBasedPlanner> solver;
    thread_local std::shared_ptr<HybridAstar2> astar2_solver;
    // if (HybridAstar2::planningCoreSupported(
    //         msquare::HybridAstarConfig::GetInstance()->planning_core)) {
      if (!astar2_solver) {
        astar2_solver = std::make_shared<HybridAstar2>();
      }
      astar2_solver->setSlotTypeByPlanningCore(9);
      solver = astar2_solver;
      solver->Update(odo.map_boundary);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 2) {
    //   solver = std::make_shared<msquare::PatternPlanner>(odo.pattern_path);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 1) {
    //   solver = std::make_shared<msquare::PerpendicularRulePlanner>(odo);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 3) {
    //   solver = std::make_shared<msquare::ParallelRulePlanner>(odo);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 4) {
    //   solver = std::make_shared<msquare::PerpendicularOutRulePlanner>(odo);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 5) {
    //   solver = std::make_shared<msquare::ParallelOutRulePlanner>(odo);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 6) {
    //   solver = std::make_shared<msquare::ObliqueRulePlanner>(odo);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 7) {
    //     solver = std::make_shared<msquare::RpaStraightPlanner>(odo);
    // } else {
    //   return planner_res;
    // }
    msquare::parking::genOpenspacePath(solver, odo, planner_res, nullptr);

    checkCollision(planner_res, odo);

    return planner_res;
}

void checkCollision(msquare::SbpResult& sbp_res, const msquare::parking::OpenspaceDeciderOutput& odo){
    bool is_para = msquare::HybridAstarConfig::GetInstance()->planning_core == 3;

    if(sbp_res.x.size()<3){
        return;
    }

    int step = 4;
    int size = sbp_res.x.size();
    double lat = msquare::CarParams::GetInstance()->lat_inflation();
    double lon = msquare::CarParams::GetInstance()->lon_inflation();
    std::vector<msquare::planning_math::Polygon2d> polygons;

    msquare::planning_math::Polygon2d end_no_lat =  getEgoPolygon(Pose2D(sbp_res.x[size-1], sbp_res.y[size-1], sbp_res.phi[size-1]), 0.0, 0.0);

    msquare::planning_math::Polygon2d start_with_lat =  getEgoPolygon(Pose2D(sbp_res.x[0], sbp_res.y[0], sbp_res.phi[0]), lat, lat);
    msquare::planning_math::Polygon2d end_with_lat =  getEgoPolygon(Pose2D(sbp_res.x[size-1], sbp_res.y[size-1], sbp_res.phi[size-1]), lat, lat);


    for(int i=(is_para? 0:1);i<size-1;i += step){
        polygons.push_back(getEgoPolygon(Pose2D(sbp_res.x[i], sbp_res.y[i], sbp_res.phi[i])));
    }

    for(const auto& p: odo.points){
        if(!is_para){
            if(start_with_lat.IsPointIn(p) || end_with_lat.IsPointIn(p)){
                continue;
            }
        }

        for(const auto& shape:polygons){
            if(shape.IsPointIn(p)){
                sbp_res.debug_string += "|collision";
                break;
            }
        }
    }

    // for(const auto& b: odo.obstacle_boxs){
    //     msquare::planning_math::Polygon2d box_polygon(b);
    //     if(!is_para){
    //         if(start_with_lat.HasOverlap(box_polygon) || end_with_lat.HasOverlap(box_polygon)){
    //             continue;
    //         }
    //     }

    //     for(const auto& shape:polygons){
    //         if(shape.HasOverlap(box_polygon)){
    //             sbp_res.debug_string += "|collision";
    //             break;
    //         }
    //     }
    // }

    for(const auto& l: odo.obstacle_lines){
        if(!is_para){
            if(start_with_lat.HasOverlap(l) || end_with_lat.HasOverlap(l)){
                continue;
            }
        }

        for(const auto& shape:polygons){
            if(shape.HasOverlap(l)){
                sbp_res.debug_string += "|collision";
                break;
            }
        }
    }

    return;
}

double getEgoMinObsDistance(const parking_scenario::Point2d& ego_pose, 
    const std::string& odo_str) {
    
    nlohmann::json json_obj = nlohmann::json::parse(odo_str);
    msquare::parking::OpenspaceDeciderOutput odo = json_obj;
    
    msquare::planning_math::Polygon2d ego_car = getEgoPolygon(
        convertPose(ego_pose), 0.0, 0.0);

    double min_distance = 1000.0;
    
    for(const auto & point: odo.points) {
        double cur_distance = ego_car.DistanceTo(point);
        min_distance = std::min(cur_distance, min_distance);
    }

    for(const auto & box: odo.obstacle_boxs) {
        double cur_distance = ego_car.DistanceTo(box);
        min_distance = std::min(cur_distance, min_distance);
    }

    for(const auto & line: odo.obstacle_lines) {
        double cur_distance = ego_car.DistanceTo(line);
        min_distance = std::min(cur_distance, min_distance);
    }

    return min_distance;
}

std::vector<parking_scenario::Point2d> getEgoCorners(const parking_scenario::Point2d& pose, double lat, double lon){
    double half_width = 0.98 + lat;
    double front_to_rear = 4.015 + lon;
    double back_to_rear = 1.083 + lon;
    double front_corner_width = 0.595 +lat;
    double front_corner_length = 3.645;

    std::vector<parking_scenario::Point2d> corners;
    corners.emplace_back(pose.x + front_to_rear * cos(pose.theta) - front_corner_width * sin(pose.theta),
                        pose.y + front_to_rear * sin(pose.theta) + front_corner_width * cos(pose.theta), 
                        0);
    corners.emplace_back(pose.x + front_corner_length * cos(pose.theta) -
                            half_width * sin(pose.theta),
                        pose.y + front_corner_length * sin(pose.theta) +
                            half_width * cos(pose.theta), 0);
    corners.emplace_back(pose.x - back_to_rear * cos(pose.theta) -
                            half_width * sin(pose.theta),
                        pose.y - back_to_rear * sin(pose.theta) +
                            half_width * cos(pose.theta), 0);
    corners.emplace_back(pose.x - back_to_rear * cos(pose.theta) +
                            half_width * sin(pose.theta),
                        pose.y - back_to_rear * sin(pose.theta) -
                            half_width * cos(pose.theta), 0);
    corners.emplace_back(pose.x + front_corner_length * cos(pose.theta) +
                            half_width * sin(pose.theta),
                        pose.y + front_corner_length * sin(pose.theta) -
                            half_width * cos(pose.theta), 0);
    corners.emplace_back(pose.x + front_to_rear * cos(pose.theta) +
                            front_corner_width * sin(pose.theta),
                        pose.y + front_to_rear * sin(pose.theta) -
                            front_corner_width * cos(pose.theta), 0);
    return corners;
}


Pose2D convertPose(const parking_scenario::Point2d& ego_pose){
    return Pose2D(ego_pose.x, ego_pose.y, ego_pose.theta);
}

parking_scenario::Point2d convertPose(const Pose2D& ego_pose){
    return parking_scenario::Point2d(ego_pose.x, ego_pose.y, ego_pose.theta);
}

msquare::planning_math::Polygon2d getEgoPolygon(const Pose2D& ego_pose, double lat, double lon){
    std::vector<parking_scenario::Point2d> corners_local = getEgoCorners(convertPose(ego_pose), lat, lon);

    std::vector<msquare::planning_math::Vec2d> corners;
    std::vector<msquare::planning_math::LineSegment2d> lines;
    for(const auto& c: corners_local){
        corners.emplace_back(c.x, c.y);
    }

    for (int i = 0; i < corners.size(); ++i) {
        lines.emplace_back(corners.at(i), corners.at((i + 1) % (corners.size())));
    }

    msquare::planning_math::Polygon2d ego_polygon;
    ego_polygon.update(corners);
    return ego_polygon;
}


}   // end namespace planner_interface