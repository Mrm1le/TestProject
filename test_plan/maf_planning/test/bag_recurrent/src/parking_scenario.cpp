#include "parking_scenario/parking_scenario.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/pattern_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/oblique_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rpa_straight_planner.h"
#include "parking_scenario/threadpool.h"
#include "util.h"
#include "io.h"
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <experimental/filesystem>
#include <sys/wait.h>


namespace parking_scenario{

namespace fs = std::experimental::filesystem;

ScenarioResult scenarioPlan(const PlanParams& params, bool use_multi_process){
    msquare::parking::OpenspaceDeciderOutput odo;
    std::vector<Point2d> init_points;
    switch(params.scenario_type){
        case ScenarioType::PARALLEL:
            generateParallelScenario(params, odo, init_points);
            break;
        case ScenarioType::VERTICAL_BASE:
            generateVerticalScenario(params, odo, init_points);
            break;
        case ScenarioType::VERTICAL_SINGLE_CAR:
            generateVerticalScenario2(params, odo, init_points);
            break;
        case ScenarioType::OBLIQUE_SLOT:
            generateObliqueScenario(params, odo, init_points);
            break;
        default:
            break;
    }

    BatchResult br;
    if(use_multi_process){
        br = multiProcessPlan(odo, init_points);
    }else{
        br = singleThreadPlan(odo, init_points);
    }
    ScenarioResult sr = extractResult(br, odo);
    return sr;
}

ScenarioResult extractResult(const BatchResult &br,
                             msquare::parking::OpenspaceDeciderOutput &odo) {
  ScenarioResult sr;
  sr.br = br;
  nlohmann::json odo_json = odo;
  sr.debug_info.push_back(odo_json.dump());

  msquare::parking::CarSizeParams car_params;
  car_params.reset();
  nlohmann::json car_json = car_params;
  sr.debug_info.push_back(car_json.dump());
  return sr;
}

BatchResult singleThreadPlan(msquare::parking::OpenspaceDeciderOutput &odo,
                             const std::vector<Point2d> &init_points) {
  BatchResult batch_result;
  batch_result.reserve(init_points.size());

  for (auto point : init_points) {
    odo.target_state.path_point.x = point.x;
    odo.target_state.path_point.y = point.y;
    odo.target_state.path_point.theta = point.theta;
    msquare::SbpResult sbp_result;
    msquare::parking::SearchProcessDebug sp_debug;
    apaPlan(odo, sbp_result, nullptr);
    PlanResult plan_res;
    plan_res.point = point;
    extractSingleResult(sbp_result, nullptr, plan_res);
    batch_result.push_back(plan_res);
  }
  std::cout<<"res size():"<<batch_result.size()<<std::endl;

  return batch_result;
}

void extractSingleResult(const msquare::SbpResult &sbp_result,
             const msquare::parking::SearchProcessDebug *sp_debug,
             PlanResult &plan_res
){
    plan_res.is_success = !sbp_result.x.empty();
    nlohmann::json json_result = sbp_result;
    plan_res.debug_info = json_result.dump();
}

void apaPlan(const msquare::parking::OpenspaceDeciderOutput odo,
             msquare::SbpResult &sbp_result,
             msquare::parking::SearchProcessDebug *sp_debug) {
    using msquare::hybrid_a_star_2::HybridAstar2;
    
    std::shared_ptr<msquare::SearchBasedPlanner> solver;
    msquare::parking::OpenspaceDeciderOutput inp = odo;
    thread_local std::shared_ptr<HybridAstar2> astar2_solver;
    // if (HybridAstar2::planningCoreSupported(
    //         msquare::HybridAstarConfig::GetInstance()->planning_core)) {
      if (!astar2_solver) {
        astar2_solver = std::make_shared<HybridAstar2>();
      }
      astar2_solver->setSlotTypeByPlanningCore(
          msquare::HybridAstarConfig::GetInstance()->planning_core);
      solver = astar2_solver;
      solver->Update(inp.map_boundary);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 2) {
    //   solver = std::make_shared<msquare::PatternPlanner>(inp.pattern_path);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 1) {
    //   solver = std::make_shared<msquare::PerpendicularRulePlanner>(inp);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 3) {
    //   solver = std::make_shared<msquare::ParallelRulePlanner>(inp);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 4) {
    //     solver = std::make_shared<msquare::PerpendicularOutRulePlanner>(inp);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 5) {
    //   solver = std::make_shared<msquare::ParallelOutRulePlanner>(odo);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 6) {
    //   solver = std::make_shared<msquare::ObliqueRulePlanner>(odo);
    // } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 7) {
    //     solver = std::make_shared<msquare::RpaStraightPlanner>(odo);
    // } else {
    //   // throw std::logic_error("unsupported planner!");
    // }
    // inp.target_state = odo.init_state;
    // inp.init_state = odo.target_state;
    msquare::parking::genOpenspacePath(solver, inp, sbp_result, sp_debug);
}

void loadPlanParams(const std::string &apa_file, const std::string &vehicle_param_file){
    std::cout<<"apa:"<<apa_file<<std::endl;
    msquare::VehicleParam::Instance()->loadFile(vehicle_param_file);
    msquare::HybridAstarConfig::GetInstance()->loadFile(apa_file);
    msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(apa_file);
    msquare::CarParams::GetInstance()->loadFile(vehicle_param_file);
    msquare::CarParams::GetInstance()->loadFile4Plan(apa_file);
    msquare::StrategyParams::GetInstance()->loadFile(apa_file);
    
    fs::path car_params_path(apa_file);
    fs::path car_config_path = car_params_path.parent_path() / "../../../resource/config/scenario_configs_json/";
    if (!fs::exists(car_config_path)) 
	{
		std::cout<< "car_config_file not exist:"<< fs::canonical(car_config_path) << std::endl;
	}
    
    msquare::CarParams::GetInstance()->loadFile4Car(car_config_path.string());

    // reset inflation parameter
    msquare::CarParams::GetInstance()->setLonInflation(0.4);
    msquare::CarParams::GetInstance()->setLatInflation(0.15);
    msquare::HybridAstarConfig::GetInstance()->use_t_line_ = false;

    std::cout<<"lat:"<<msquare::CarParams::GetInstance()->lat_inflation()<<std::endl;
    std::cout<<"lon:"<<msquare::CarParams::GetInstance()->lon_inflation()<<std::endl;
    std::cout<<"planning type:"<<msquare::HybridAstarConfig::GetInstance()->planning_core<<std::endl;
    std::cout<<"min_radius:"<<msquare::CarParams::GetInstance()->car_config.car_only_config.min_radius<<std::endl;
}

/**
 * @brief vertical scenarion with a car extrude
 * 
 * @param params 
 * @param odo 
 * @param init_points 
 */
void generateVerticalScenario2(
    const PlanParams& params, 
    msquare::parking::OpenspaceDeciderOutput& odo,
    std::vector<Point2d>& init_points
){
    odo.obstacle_boxs.clear();
    odo.obstacle_lines.clear();
    odo.lines.clear();
    odo.points.clear();
    loadPlanParams(params.apa_file, params.vehicle_param_file);

    double channel_width = 5.5;
    double extrude_width = params.channel_width;

    double car_width = msquare::VehicleParam::Instance()->width;
    double car_length = msquare::VehicleParam::Instance()->length;
    double center_to_geometry_offset = msquare::VehicleParam::Instance()->center_to_geometry_center;

    double channel_low = params.BOUNDARY_MARGIN + params.SLOT_DEPTH_MARGIN + car_length;
    double channel_high = channel_low + channel_width;
    double extrude_high = channel_low + extrude_width;

    double boundary_height = std::max(channel_high, extrude_high) + params.BOUNDARY_MARGIN;

    double slot_left = params.BOUNDARY_WIDTH / 2.0 - params.slot_margin / 2.0;
    double slot_right = params.BOUNDARY_WIDTH / 2.0 + params.slot_margin / 2.0;
    double extrude_left = slot_right;
    double extrude_right = slot_right + car_width;

    Point2d target_p;
    target_p.y = channel_low - car_length / 2.0 - center_to_geometry_offset;
    target_p.x = params.BOUNDARY_WIDTH / 2.0;
    target_p.theta = M_PI / 2.0;
    odo.init_state.path_point.x = target_p.x;
    odo.init_state.path_point.y = target_p.y;
    odo.init_state.path_point.theta = target_p.theta;

    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(0, channel_low), msquare::planning_math::Vec2d(slot_left, channel_low));
    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(slot_left, channel_low), msquare::planning_math::Vec2d(slot_left, 0));
    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_low), msquare::planning_math::Vec2d(slot_right, channel_low));
    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(slot_right, channel_low), msquare::planning_math::Vec2d(slot_right, 0));
    odo.lines.emplace_back(msquare::planning_math::Vec2d(0, params.BOUNDARY_MARGIN), msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, params.BOUNDARY_MARGIN));
    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(0, channel_high), msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_high));

    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(extrude_left, extrude_high), msquare::planning_math::Vec2d(extrude_left, channel_high));
    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(extrude_right, extrude_high), msquare::planning_math::Vec2d(extrude_right, channel_high));
    odo.obstacle_lines.emplace_back(msquare::planning_math::Vec2d(extrude_left, extrude_high), msquare::planning_math::Vec2d(extrude_right, extrude_high));

    
    msquare::planning_math::Vec2d center(params.BOUNDARY_WIDTH / 2.0, boundary_height / 2.0);
    msquare::planning_math::Box2d map_boundary(center, 0, params.BOUNDARY_WIDTH, boundary_height);
    odo.map_boundary = map_boundary;
    odo.T_lines.road_upper_bound = odo.obstacle_lines.at(4);
    odo.T_lines.road_lower_left_bound = odo.obstacle_lines.at(0);
    odo.T_lines.slot_left_bound = odo.obstacle_lines.at(1);
    odo.T_lines.road_lower_right_bound = odo.obstacle_lines.at(3);
    odo.T_lines.slot_right_bound = odo.obstacle_lines.at(2);

    for (double x = params.left_space; x < params.BOUNDARY_WIDTH - params.right_space;
        x += params.step) {
        for (double y = channel_low + params.bottom_space; y < extrude_high - params.top_space; y += params.step) {
            init_points.emplace_back(x, y, params.theta);
        }
    }
}

void generateObliqueScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo,
                              std::vector<Point2d> &init_points) {
  odo.obstacle_boxs.clear();
  odo.obstacle_lines.clear();
  odo.lines.clear();
  odo.points.clear();
  loadPlanParams(params.apa_file, params.vehicle_param_file);

  double car_width = msquare::VehicleParam::Instance()->width;
  double car_length = msquare::VehicleParam::Instance()->length;
  double center_to_geometry_offset =
      msquare::VehicleParam::Instance()->center_to_geometry_center;

  // limit max oblique angle
  double oblique_angle = M_PI/2.0 - std::max(M_PI/6.0-1e-3, params.oblique_angle);
  double oblique_cos = std::cos(oblique_angle);
  double oblique_sin = std::sin(oblique_angle);
  double oblique_tan = std::tan(oblique_angle);
  
  double height_above_center = (car_length/2.0 + center_to_geometry_offset) * oblique_cos + car_width/2.0*oblique_sin;
  double channel_low = params.BOUNDARY_MARGIN + params.SLOT_DEPTH_MARGIN + car_length/2.0 - center_to_geometry_offset + height_above_center;
  double channel_high = channel_low + params.channel_width;
  double boundary_height = channel_high + params.BOUNDARY_MARGIN;



  double slot_margin_along_x = params.slot_margin / oblique_cos;
  double slot_left_upper = params.BOUNDARY_WIDTH / 2.0 - slot_margin_along_x/2.0 + height_above_center * oblique_tan;
  double slot_right_upper = slot_left_upper + slot_margin_along_x;
  double slot_left_low = slot_left_upper - channel_low * oblique_tan;
  double slot_right_low = slot_left_low + slot_margin_along_x;

  Point2d target_p;
  target_p.y = params.BOUNDARY_MARGIN + params.SLOT_DEPTH_MARGIN + car_length/2.0 - center_to_geometry_offset;
  target_p.x = params.BOUNDARY_WIDTH / 2.0;
  target_p.theta = M_PI / 2.0 - oblique_angle;
  odo.init_state.path_point.x = target_p.x;
  odo.init_state.path_point.y = target_p.y;
  odo.init_state.path_point.theta = target_p.theta;

  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(0, channel_low),
      msquare::planning_math::Vec2d(slot_left_upper, channel_low));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(slot_left_upper, channel_low),
      msquare::planning_math::Vec2d(slot_left_low, 0));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_low),
      msquare::planning_math::Vec2d(slot_right_upper, channel_low));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(slot_right_upper, channel_low),
      msquare::planning_math::Vec2d(slot_right_low, 0));
  odo.lines.emplace_back(
      msquare::planning_math::Vec2d(0, params.BOUNDARY_MARGIN),
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH,
                                    params.BOUNDARY_MARGIN));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(0, channel_high),
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_high));
  odo.T_lines.road_upper_bound = odo.obstacle_lines.at(4);
  odo.T_lines.road_lower_left_bound = odo.obstacle_lines.at(0);
  odo.T_lines.slot_left_bound = odo.obstacle_lines.at(1);
  odo.T_lines.road_lower_right_bound = odo.obstacle_lines.at(3);
  odo.T_lines.slot_right_bound = odo.obstacle_lines.at(2);

  msquare::planning_math::Vec2d center(params.BOUNDARY_WIDTH / 2.0,
                                       boundary_height / 2.0);
  msquare::planning_math::Box2d map_boundary(center, 0, params.BOUNDARY_WIDTH,
                                             boundary_height);
  odo.map_boundary = map_boundary;

  for (double x = params.left_space; x < params.BOUNDARY_WIDTH - params.right_space;
       x += params.step) {
    for (double y = channel_low + params.bottom_space; y < channel_high - params.top_space; y += params.step) {
      init_points.emplace_back(x, y, params.theta);
    }
  }
}

void generateVerticalScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo,
                              std::vector<Point2d> &init_points){
  odo.obstacle_boxs.clear();
  odo.obstacle_lines.clear();
  odo.lines.clear();
  odo.points.clear();
  loadPlanParams(params.apa_file, params.vehicle_param_file);

    double car_width = msquare::VehicleParam::Instance()->width;
  double car_length = msquare::VehicleParam::Instance()->length;
  double center_to_geometry_offset =
      msquare::VehicleParam::Instance()->center_to_geometry_center;

  double channel_low =
      params.BOUNDARY_MARGIN + params.SLOT_DEPTH_MARGIN + car_length;
  double channel_high = channel_low + params.channel_width;
  double boundary_height = channel_high + params.BOUNDARY_MARGIN;

  double slot_left = params.BOUNDARY_WIDTH / 2.0 - params.slot_margin / 2.0;
  double slot_right = params.BOUNDARY_WIDTH / 2.0 + params.slot_margin / 2.0;

  Point2d target_p;
  target_p.y = channel_low - car_length / 2.0 - center_to_geometry_offset;
  target_p.x = params.BOUNDARY_WIDTH / 2.0;
  target_p.theta = M_PI / 2.0;
  odo.init_state.path_point.x = target_p.x;
  odo.init_state.path_point.y = target_p.y;
  odo.init_state.path_point.theta = target_p.theta;

  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(0, channel_low),
      msquare::planning_math::Vec2d(slot_left, channel_low));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(slot_left, channel_low),
      msquare::planning_math::Vec2d(slot_left, 0));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_low),
      msquare::planning_math::Vec2d(slot_right, channel_low));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(slot_right, channel_low),
      msquare::planning_math::Vec2d(slot_right, 0));
  odo.lines.emplace_back(
      msquare::planning_math::Vec2d(0, params.BOUNDARY_MARGIN),
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH,
                                    params.BOUNDARY_MARGIN));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(0, channel_high),
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_high));
  odo.T_lines.road_upper_bound = odo.obstacle_lines.at(4);
  odo.T_lines.road_lower_left_bound = odo.obstacle_lines.at(0);
  odo.T_lines.slot_left_bound = odo.obstacle_lines.at(1);
  odo.T_lines.road_lower_right_bound = odo.obstacle_lines.at(3);
  odo.T_lines.slot_right_bound = odo.obstacle_lines.at(2);

  msquare::planning_math::Vec2d center(params.BOUNDARY_WIDTH / 2.0,
                                       boundary_height / 2.0);
  msquare::planning_math::Box2d map_boundary(center, 0, params.BOUNDARY_WIDTH,
                                             boundary_height);
  odo.map_boundary = map_boundary;

  for (double x = params.left_space; x < params.BOUNDARY_WIDTH - params.right_space;
       x += params.step) {
    for (double y = channel_low + params.bottom_space; y < channel_high - params.top_space; y += params.step) {
      init_points.emplace_back(x, y, params.theta);
    }
  }
}

void generateParallelScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo,
                              std::vector<Point2d> &init_points) {
  odo.obstacle_boxs.clear();
  odo.obstacle_lines.clear();
  odo.lines.clear();
  odo.points.clear();
  loadPlanParams(params.apa_file, params.vehicle_param_file);

  double car_width = msquare::VehicleParam::Instance()->width;
  double car_length = msquare::VehicleParam::Instance()->length;
  double center_to_geometry_offset =
      msquare::VehicleParam::Instance()->center_to_geometry_center;

  double channel_low =
      params.BOUNDARY_MARGIN + params.SLOT_DEPTH_MARGIN + car_width;
  double channel_high = channel_low + params.channel_width;
  double boundary_height = channel_high + params.BOUNDARY_MARGIN;

  double slot_length_half = (car_length + params.slot_margin) / 2.0;
  double slot_left = params.BOUNDARY_WIDTH / 2.0 - slot_length_half;
  double slot_right = params.BOUNDARY_WIDTH / 2.0 + slot_length_half;

  Point2d target_p;
  target_p.y = channel_low - car_width / 2.0;
  target_p.x = params.BOUNDARY_WIDTH / 2.0 - center_to_geometry_offset;
  target_p.theta = 0.0;
  odo.init_state.path_point.x = target_p.x;
  odo.init_state.path_point.y = target_p.y;
  odo.init_state.path_point.theta = target_p.theta;

  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(0, channel_low),
      msquare::planning_math::Vec2d(slot_left, channel_low));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(slot_left, channel_low),
      msquare::planning_math::Vec2d(slot_left, 0));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_low),
      msquare::planning_math::Vec2d(slot_right, channel_low));
  odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(slot_right, channel_low),
      msquare::planning_math::Vec2d(slot_right, 0));
 odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(0, channel_high),
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_high));
odo.obstacle_lines.emplace_back(
      msquare::planning_math::Vec2d(0, params.BOUNDARY_MARGIN-0.2),
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH,
                                    params.BOUNDARY_MARGIN-0.2));
odo.lines.emplace_back(
      msquare::planning_math::Vec2d(0, params.BOUNDARY_MARGIN),
      msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH,
                                    params.BOUNDARY_MARGIN));

  msquare::planning_math::Vec2d center(params.BOUNDARY_WIDTH / 2.0,
                                       boundary_height / 2.0);
  msquare::planning_math::Box2d map_boundary(center, 0, params.BOUNDARY_WIDTH,
                                             boundary_height);
  odo.map_boundary = map_boundary;
  odo.T_lines.road_upper_bound = odo.obstacle_lines.at(4);
  odo.T_lines.road_lower_left_bound = odo.obstacle_lines.at(0);
  odo.T_lines.slot_left_bound = odo.obstacle_lines.at(1);
  odo.T_lines.road_lower_right_bound = odo.obstacle_lines.at(3);
  odo.T_lines.slot_right_bound = odo.obstacle_lines.at(2);

    for (double x = params.left_space; x < params.BOUNDARY_WIDTH - params.right_space;
        x += params.step) {
        for (double y = channel_low + params.bottom_space; y < channel_high - params.top_space; y += params.step) {
            init_points.emplace_back(x, y, params.theta);
        }
    }

}

BatchResult multiThreadPlan(msquare::parking::OpenspaceDeciderOutput &odo,
                            const std::vector<Point2d> &init_points) {
  BatchResult batch_result;
  batch_result.reserve(init_points.size());

  size_t thread_num = 12;
  std::cout << "thread" << std::endl;
  ThreadPool pool(thread_num);
  std::cout << "thread2" << std::endl;
  std::vector<std::future<BatchResult>> results;
  std::vector<msquare::parking::OpenspaceDeciderOutput> odo_array;
  std::vector<std::vector<Point2d>> init_points_array;

  int quotient = init_points.size() / thread_num;
  int remainder = init_points.size() % thread_num;
  for (size_t i = 0; i < thread_num; i++) {
    msquare::parking::OpenspaceDeciderOutput odo_temp = odo;
    int si, ei;
    si = (i < remainder ? quotient * i + i : quotient * i + remainder);
    ei = (i < remainder ? quotient * (i + 1) + i + 1
                        : quotient * (i + 1) + remainder);

    std::vector<Point2d> init_points_temp(init_points.begin() + si,
                                          init_points.begin() + ei);
    results.emplace_back(
        pool.enqueue(singleThreadPlan, odo_temp, init_points_temp));

    std::cout << "create thread:" << i << " " << si << " " << ei
              << " point.size()=" << init_points_temp.size() << std::endl;
  }

  for (size_t i = 0; i < thread_num; i++) {
    BatchResult batch_result_temp = results[i].get();
    batch_result.insert(batch_result.end(), batch_result_temp.begin(),
                        batch_result_temp.end());
  }

  std::cout << "res size():" << batch_result.size() << std::endl;
  return batch_result;
}

void saveBatchRes(const BatchResult &br, const std::string &res_file_name){
    std::ofstream res_file(res_file_name);
    for(auto res : br){
        res_file << res.point.x << " "
                 << res.point.y << " "
                 << res.point.theta << " "
                 << res.is_success << " "
                 << res.debug_info <<std::endl;
    }
    res_file.close();
}

void getSingleRes(BatchResult &br, const std::string &res_file_name){
    std::ifstream res_file(res_file_name);
    if(!res_file.is_open()){
        std::cout<<"failed to open file:"<<res_file_name;
        return;
    }
    double x, y, theta;
    bool is_success;
    std::string debug_info = "";
    while (res_file >> x){      // avoid empty line
        res_file >> y >> theta >> is_success;
        getline(res_file, debug_info);

        PlanResult pr{{x,y,theta}, is_success, debug_info};
        br.push_back(pr);

    }
}

void getBatchRes(BatchResult &br, const std::string& base_folder){
    std::vector<std::string> all_files;
    getFileNames(base_folder, all_files);
    for(auto file_name: all_files){
        getSingleRes(br, file_name);
    }
}



std::string createBaseFolder(){
    auto t1=std::chrono::steady_clock::now().time_since_epoch();
    std::chrono::nanoseconds nan = std::chrono::duration_cast<std::chrono::nanoseconds>(t1);
    std::string folder_name = std::to_string(nan.count());
    while(!mkNewDir(folder_name)){
        folder_name += "_" + std::to_string(getRandNum());
    }
    return folder_name;
}

BatchResult multiProcessPlan(msquare::parking::OpenspaceDeciderOutput& odo,
    const std::vector<Point2d>& init_points){

    size_t thread_num = getCpuNum();
    pid_t pid = -1;
    int quotient = init_points.size() / thread_num;
    int remainder = init_points.size() % thread_num;

    std::string base_folder = createBaseFolder();

    int si, ei;
    std::string file_name;
    for(size_t i = 0; i < thread_num; i++){
        file_name = base_folder + "/pid_" + std::to_string(i) + ".sr";
        si = (i<remainder ? quotient * i + i : quotient * i + remainder);
        ei = (i<remainder ? quotient * (i+1) + i + 1 : quotient * (i+1) + remainder);
        pid = fork();
        if(pid == 0 || pid == -1){
            break;
        }
    }
    if(pid == -1){}
    else if(pid ==0 ){  // child
        msquare::parking::OpenspaceDeciderOutput odo_temp = odo;

        std::vector<Point2d> init_points_temp(init_points.begin()+si, init_points.begin()+ei);
        // std::cout<<"create pid:"<<getpid()<<" "<<si<<" "<<ei<<" point.size()="<<init_points_temp.size()<<std::endl;
        // std::cout<<"create pid:"<<getpid()<<" save to file:"<<file_name<<std::endl;
        BatchResult br_single = singleThreadPlan(odo_temp, init_points_temp);
        saveBatchRes(br_single, file_name);
        exit(1);

    }else{  // father  wait all child pid
        while (waitpid(-1, NULL, 0)) {
            if (errno == ECHILD) {
                break;
            }
        }
    }

    // return batch_result;
    BatchResult batch_result;
    getBatchRes(batch_result, base_folder);
    removeFolder(base_folder);

    batch_result.reserve(init_points.size());
    std::cout<<"init points size:"<<init_points.size()<<std::endl;
    std::cout<<"res size():"<<batch_result.size()<<std::endl;

    return batch_result;
    
}

} // namespace parking_scenario