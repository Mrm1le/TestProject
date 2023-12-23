#include "common/config/vehicle_param.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <iostream>
#include <string>

int main(int args, char ** argv){
    std::string vehicle_param_file = "/home/yee/mm_ws/mf_system_yee/maf_planning_build/resource/config/scenario_configs_json/parking/vehicle_param_l.yaml";
    std::string scenario_config_file_dir = "/home/yee/mm_ws/mf_system_yee/maf_planning_build/resource/config/scenario_configs_json";
    if(args > 1){
        vehicle_param_file = argv[1];
    }
    if(args > 2){
        scenario_config_file_dir = argv[2];
    }
    msquare::VehicleParam::Instance()->loadFile(vehicle_param_file);
    msquare::CarParams::GetInstance()->loadFile4Car(scenario_config_file_dir);

    
    std::cout<<"obstacle_consider_max:"<<msquare::CarParams::GetInstance()->car_config.lon_config.obstacle_consider_max<<std::endl;
    std::cout<<"obstacle_consider_min_v:"<<msquare::CarParams::GetInstance()->car_config.lon_config.obstacle_consider_min_v<<std::endl;
    std::cout<<"acc:"<<msquare::CarParams::GetInstance()->car_config.lon_config.acc<<std::endl;
    std::cout<<"dec:"<<msquare::CarParams::GetInstance()->car_config.lon_config.dec<<std::endl;
    std::cout<<"min_duration_filter_time:"<<msquare::CarParams::GetInstance()->car_config.lon_config.min_duration_filter_time<<std::endl;

    std::cout<<"vel_forward:"<<msquare::CarParams::GetInstance()->car_config.kino_config.apa_parallel.vel_forward<<std::endl;
    std::cout<<"vel_backward:"<<msquare::CarParams::GetInstance()->car_config.kino_config.apa_parallel.vel_backward<<std::endl;


    return 0;
}