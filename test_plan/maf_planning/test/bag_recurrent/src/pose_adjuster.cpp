#include "parking_scenario/pose_adjuster.h"
#include "util.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/perpendicular_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/parallel_out_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/pattern_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/oblique_rule_planner.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/rpa_straight_planner.h"

namespace parking_scenario {

PoseAdjuster::PoseAdjuster(const PlanParams& params)
:params_(params), odo_(msquare::parking::OpenspaceDeciderOutput{}){
    reset(params);
    astar2_solver_ = std::make_shared<msquare::hybrid_a_star_2::HybridAstar2>();
}

std::string PoseAdjuster::getOdo(){
    nlohmann::json json_odo = odo_;
    return json_odo.dump();
}

std::vector<std::string>
PoseAdjuster::planOnce(const Point2d &point,
                       msquare::parking::SearchProcessDebug *sp_debug) {
    using msquare::hybrid_a_star_2::HybridAstar2;
    msquare::parking::OpenspaceDeciderOutput inp = odo_;

    if (out_) {
      inp.init_state.path_point.x = point.x;
      inp.init_state.path_point.y = point.y;
      inp.init_state.path_point.theta = point.theta;
    } else {
      inp.target_state.path_point.x = point.x;
      inp.target_state.path_point.y = point.y;
      inp.target_state.path_point.theta = point.theta;
    }

    msquare::SbpResult sbp_result;
    std::shared_ptr<msquare::SearchBasedPlanner> solver;
    if (HybridAstar2::planningCoreSupported(
            msquare::HybridAstarConfig::GetInstance()->planning_core)) {
      astar2_solver_->setSlotTypeByPlanningCore(
          msquare::HybridAstarConfig::GetInstance()->planning_core);
      solver = astar2_solver_;
      solver->Update(inp.map_boundary);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 2) {
      solver = std::make_shared<msquare::PatternPlanner>(inp.pattern_path);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 1) {
      solver = std::make_shared<msquare::PerpendicularRulePlanner>(inp);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 3) {
      solver = std::make_shared<msquare::ParallelRulePlanner>(inp);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 4) {
      solver = std::make_shared<msquare::PerpendicularOutRulePlanner>(inp);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 5) {
      solver = std::make_shared<msquare::ParallelOutRulePlanner>(inp);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 6) {
      solver = std::make_shared<msquare::ObliqueRulePlanner>(inp);
    } else if (msquare::HybridAstarConfig::GetInstance()->planning_core == 7) {
        solver = std::make_shared<msquare::RpaStraightPlanner>(inp);
    } else {
      // throw std::logic_error("unsupported planner!");
    }
    msquare::parking::genOpenspacePath(solver, inp, sbp_result, sp_debug);

    nlohmann::json json_res = sbp_result;

    std::vector<std::string> res;
    res.push_back(json_res.dump());

    msquare::parking::CarSizeParams car_params;
    car_params.reset();
    nlohmann::json car_json = car_params;
    res.push_back(car_json.dump());
    return res;
}

void PoseAdjuster::reset(const PlanParams& params){
    odo_.obstacle_boxs.clear();
    odo_.obstacle_lines.clear();
    odo_.lines.clear();
    odo_.points.clear();
    loadPlanParams(params.apa_file, params.vehicle_param_file);
    out_ = false;
    switch (params.scenario_type)
    {
        case ScenarioType::PARALLEL:
            generateParallelScenario(params, odo_);
            break;
        case ScenarioType::VERTICAL_BASE:
            generateVerticalScenario(params, odo_);
            break;
        case ScenarioType::VERTICAL_SINGLE_CAR:
            generateVerticalScenario2(params, odo_);
            break;
        case ScenarioType::OBLIQUE_SLOT:
            generateObliqueScenario(params, odo_);
            break;
        case ScenarioType::PARALLEL_OUT:
            generateParallelScenario(params, odo_);
            out_ = true;
            break;
        case ScenarioType::VERTICAL_BASE_OUT:
            generateVerticalScenario(params, odo_);
            out_ = true;
            break;
        case ScenarioType::VERTICAL_SINGLE_CAR_OUT:
            generateVerticalScenario2(params, odo_);
            out_ = true;
            break;
        case ScenarioType::OBLIQUE_OUT:
            generateObliqueScenario(params, odo_);
            out_ = true;
            break;
        default:
            break;
    }
    if (out_) {
        odo_.target_state = odo_.init_state;
    }

    
}

void PoseAdjuster::generateObliqueScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo){
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

  min_x_ = 0.0;
  min_y_ = channel_low;
  max_x_ = params.BOUNDARY_WIDTH;
  max_y_ = channel_high;
}

void PoseAdjuster::generateParallelScenario(const PlanParams &params,
                              msquare::parking::OpenspaceDeciderOutput &odo){
    double car_width = msquare::VehicleParam::Instance()->width_wo_rearview_mirror;
    double car_length = msquare::VehicleParam::Instance()->length;
    double center_to_geometry_offset =
        msquare::VehicleParam::Instance()->center_to_geometry_center;

    double cut_depth = 0.2;
    double channel_low =
        params.BOUNDARY_MARGIN + params.SLOT_DEPTH_MARGIN + car_width + cut_depth;
    double channel_high = channel_low + params.channel_width;
    double boundary_height = channel_high + params.BOUNDARY_MARGIN;

    double slot_length_half = (car_length + params.slot_margin) / 2.0;
    double slot_left = params.BOUNDARY_WIDTH / 2.0 - slot_length_half;
    double slot_right = params.BOUNDARY_WIDTH / 2.0 + slot_length_half;

    Point2d target_p;
    target_p.y = channel_low - car_width / 2.0 -cut_depth;
    target_p.x = params.BOUNDARY_WIDTH / 2.0 - center_to_geometry_offset;
    target_p.theta = 0.0;
    odo.init_state.path_point.x = target_p.x;
    odo.init_state.path_point.y = target_p.y;
    odo.init_state.path_point.theta = target_p.theta;
    odo.target_state.steer = 1.0;

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
    
    odo.obstacle_lines.emplace_back(
        msquare::planning_math::Vec2d(0, params.BOUNDARY_MARGIN - 0.2),
        msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH,
                                        params.BOUNDARY_MARGIN - 0.2));

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

    min_x_ = 0.0;
    min_y_ = channel_low;
    max_x_ = params.BOUNDARY_WIDTH;
    max_y_ = channel_high;
}
void PoseAdjuster::generateVerticalScenario(const PlanParams &params,
                            msquare::parking::OpenspaceDeciderOutput &odo){
    double car_width = msquare::VehicleParam::Instance()->width;
    double car_length = msquare::VehicleParam::Instance()->length;
    double center_to_geometry_offset =
        msquare::VehicleParam::Instance()->center_to_geometry_center;

    double channel_low =
        params.BOUNDARY_MARGIN + params.SLOT_DEPTH_MARGIN + car_length;
    double channel_high = channel_low + params.channel_width;
    double boundary_height = channel_high + params.BOUNDARY_MARGIN;
    if (params.opposite_width > 0) {
        boundary_height += params.SLOT_DEPTH_MARGIN + car_length;
    }

    double slot_left = params.BOUNDARY_WIDTH / 2.0 - params.slot_margin / 2.0;
    double slot_right = params.BOUNDARY_WIDTH / 2.0 + params.slot_margin / 2.0;

    Point2d target_p;
    target_p.y = channel_low - car_length / 2.0 - center_to_geometry_offset;
    target_p.x = params.BOUNDARY_WIDTH / 2.0;
    target_p.theta = M_PI / 2.0;
    odo.init_state.path_point.x = target_p.x;
    odo.init_state.path_point.y = target_p.y;
    odo.init_state.path_point.theta = target_p.theta;

    if (params.right_blocked) {
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(0, channel_high),
            msquare::planning_math::Vec2d(slot_right - params.opposite_width, channel_high));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(slot_right - params.opposite_width, channel_high),
            msquare::planning_math::Vec2d(slot_right - params.opposite_width, boundary_height));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(0, channel_low),
            msquare::planning_math::Vec2d(slot_left, channel_low));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(slot_left, channel_low),
            msquare::planning_math::Vec2d(slot_left, 0));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(slot_right, 0),
            msquare::planning_math::Vec2d(slot_right, boundary_height));
    }
    else if (params.left_blocked) {
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_high),
            msquare::planning_math::Vec2d(slot_left + params.opposite_width, channel_high));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(slot_left + params.opposite_width, channel_high),
            msquare::planning_math::Vec2d(slot_left + params.opposite_width, boundary_height));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(params.BOUNDARY_WIDTH, channel_low),
            msquare::planning_math::Vec2d(slot_right, channel_low));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(slot_right, channel_low),
            msquare::planning_math::Vec2d(slot_right, 0));
        odo.obstacle_lines.emplace_back(
            msquare::planning_math::Vec2d(slot_left, 0),
            msquare::planning_math::Vec2d(slot_left, boundary_height));
    }
    else {
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
    }

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

    min_x_ = 0.0;
    min_y_ = channel_low;
    max_x_ = params.BOUNDARY_WIDTH;
    max_y_ = channel_high;
}
void PoseAdjuster::generateVerticalScenario2(const PlanParams &params,
                            msquare::parking::OpenspaceDeciderOutput &odo){
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
    
    min_x_ = 0.0;
    min_y_ = channel_low;
    max_x_ = params.BOUNDARY_WIDTH;
    max_y_ = extrude_high;
}

}