#include "common/config/vehicle_param.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include "planner/behavior_planner/parking/sv_speed_generator.h"
#include <iostream>
#include <string>

int main(int args, char ** argv){
    std::string vehicle_param_file = "/home/yee/mm_ws/maf_planning/test/bag_recurrent/resources/vehicle_param_l.yaml";
    std::string config_param_file = "/home/yee/mm_ws/maf_planning/test/bag_recurrent/resources/apa_parallel.yaml";

    std::string scenario_config_file_dir = "/home/yee/mm_ws/maf_planning/resource/config/scenario_configs_json";
    if(args > 1){
        vehicle_param_file = argv[1];
    }
    if(args > 2){
        scenario_config_file_dir = argv[2];
    }
    msquare::VehicleParam::Instance()->loadFile(vehicle_param_file);
    msquare::HybridAstarConfig::GetInstance()->loadFile(config_param_file);
    msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(config_param_file);
    msquare::CarParams::GetInstance()->loadFile(vehicle_param_file);
    msquare::CarParams::GetInstance()->loadFile4Plan(config_param_file);
    msquare::StrategyParams::GetInstance()->loadFile(config_param_file);
    msquare::VehicleParam::Instance()->loadFile(vehicle_param_file);
    msquare::CarParams::GetInstance()->loadFile4Car(scenario_config_file_dir);
    std::cout<<"car type:"<<msquare::VehicleParam::Instance()->car_type<<std::endl;
    std::cout<<"use_sv_speed_generator:"<<msquare::CarParams::GetInstance()->car_config.sv_config.use_sv_speed_generator<<std::endl;
    std::cout<<"min_radius_coeff:"<<msquare::CarParams::GetInstance()->car_config.sv_config.min_radius_coeff<<std::endl;
    std::cout<<"out_slot_coeff:"<<msquare::CarParams::GetInstance()->car_config.sv_config.out_slot_coeff<<std::endl;std::cout<<"large_curv_coeff:"<<msquare::CarParams::GetInstance()->car_config.sv_config.large_curv_coeff<<std::endl;std::cout<<"in_slot_coeff:"<<msquare::CarParams::GetInstance()->car_config.sv_config.in_slot_coeff<<std::endl;
    std::cout<<"out_slot_coeff:"<<msquare::CarParams::GetInstance()->car_config.sv_config.out_slot_coeff<<std::endl;std::cout<<"large_curv_coeff:"<<msquare::CarParams::GetInstance()->car_config.sv_config.large_curv_coeff<<std::endl;std::cout<<"use_new_fs_block:"<<msquare::CarParams::GetInstance()->car_config.common_config.use_new_fs_block<<std::endl;
    std::cout<<"not_use_comfortable_min_s:"<<msquare::CarParams::GetInstance()->car_config.sv_config.not_use_comfortable_min_s<<std::endl;

    const auto& sv_config = msquare::CarParams::GetInstance()->car_config.sv_config;
    msquare::parking::SvSpeedParam sv_speed_param;
    sv_speed_param.has_ever_been_inside_slot =  false;
    sv_speed_param.is_reverse = true;
    sv_speed_param.force_stop = false; 
    sv_speed_param.no_slot = true;
    sv_speed_param.out_slot_coeff = sv_config.out_slot_coeff;
    sv_speed_param.large_curv_coeff = sv_config.large_curv_coeff;
    sv_speed_param.in_slot_coeff = sv_config.in_slot_coeff;
    sv_speed_param.max_v = msquare::TrajectoryOptimizerConfig::GetInstance()->param_max_speed_forward;
    sv_speed_param.width_wo_rearview_mirror = msquare::VehicleParam::Instance()->width_wo_rearview_mirror;
    sv_speed_param.length = msquare::VehicleParam::Instance()->length;
    sv_speed_param.curv_limit = 1.0 / msquare::CarParams::GetInstance()->min_turn_radius * sv_config.min_radius_coeff;
    std::cout<<"curv_limit:"<<sv_speed_param.curv_limit<<std::endl;
    bool is_in_slot =false ;
    Pose2D  ego_pose(8.0,0, 0.0);
    std::vector<float> curv = {0.17, 0.17};
    msquare::planning_math::Box2d slot_box(
        msquare::planning_math::Vec2d(0,-2.5), M_PI_2, 5.0, 2.5);
    double v_target = msquare::parking::SvSpeedGenerator::getSpeed(slot_box, ego_pose, curv, sv_speed_param, is_in_slot);
    std::cout<<"compute_speed_for_uxe: is_in_slot = "<< is_in_slot<<std::endl;
    std::cout<<"compute_speed_for_uxe: v_target = "<< v_target<<std::endl;


    return 0;
}