#pragma once
#include "common/planning_context.h"
#include "common/sbp_strategy.h"
#include "common/utils/yaml_utils.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"
#include <iostream>


static void initConfig(){
    std::string car_param_file = "/home/ros/catkin_ws/src/maf_planning/resource/"
                            "config/scenario_configs_json/parking/"
                            "vehicle_param.yaml";
    std::string config_file_name = "/home/ros/catkin_ws/src/maf_planning/"
                                "resource/config/scenario_configs_json/"
                                "parking/apa.yaml";

    if (!msquare::HybridAstarConfig::GetInstance()->loadFile(config_file_name)) {
        cout<<"HybridAstarConfig load file failed."<<endl;
    }
    if (!msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(
            config_file_name)) {
        cout<<"TrajectoryOptimizerConfig load file failed."<<endl;
    }
    if (!msquare::CarParams::GetInstance()->loadFile(car_param_file)) {
        cout<<"CarParams load file failed."<<endl;
    }
    if (!msquare::CarParams::GetInstance()->loadFile4Plan(config_file_name)) {
        cout<<"CarParams load file4Plan failed."<<endl;
    }
    if (!msquare::StrategyParams::GetInstance()->loadFile(config_file_name)) {
        cout<<"StrategyParams load file failed."<<endl;
    }
    if (!msquare::VehicleParam::Instance()->loadFile(car_param_file)) {
        cout<<"VehicleParam load file failed."<<endl;
    }
}


static void readConfig(
    std::vector<msquare::TrajectoryPoint>& traj_ps,
    msquare::parking::OpenspaceDeciderOutput &osd,
    const std::string& json_file
){
    using json = nlohmann::json;
    std::ifstream file_in(json_file);

    std::string line;
    if(!getline(file_in, line)){
        std::cout<<"read no data 1"<<std::endl;
    }

    json traj_json = json::parse(line);

    for(int i =0;i<traj_json.size();i++){
        msquare::TrajectoryPoint traj_p = traj_json[i];
        traj_ps.push_back(traj_p);
    }

    // std::cout<<"read traj size="<<traj_ps.size()<<std::endl;

    if(!getline(file_in, line)){
        std::cout<<"read no data 2"<<std::endl;
    }
 
    json osd_json = json::parse(line);
    osd = osd_json;
}