#include "util.h"
#include <fstream>
#include <iostream>

namespace msquare {
namespace parking {

void loadDevcarParams(){
  std::string car_param_file = "/home/ros/catkin_ws/src/maf_planning/test/bag_recurrent/resources/vehicle_param_rx5.yaml";
  msquare::CarParams::GetInstance()->loadFile(car_param_file);
  msquare::VehicleParam::Instance()->loadFile(car_param_file);
}
void loadEpcarParams(){
  std::string car_param_file = "/home/ros/catkin_ws/src/maf_planning/test/bag_recurrent/resources/vehicle_param_l.yaml";
  msquare::CarParams::GetInstance()->loadFile(car_param_file);
  msquare::VehicleParam::Instance()->loadFile(car_param_file);
}


void AstarPlannerParams::reset(){
    next_node_num = HybridAstarConfig::GetInstance()->next_node_num;
    lon_inflation = CarParams::GetInstance()->lon_inflation() + 0.1 - 0.1;
    lat_inflation = CarParams::GetInstance()->lat_inflation() + 0.1 - 0.1;
    max_zigzag_allowed = HybridAstarConfig::GetInstance() -> max_zigzag_allowd;
    xy_grid_resolution = HybridAstarConfig::GetInstance() -> xy_grid_resolution;
    step_size = HybridAstarConfig::GetInstance()->step_size;
    max_iter = HybridAstarConfig::GetInstance()->max_iter;
    traj_gear_switch_penalty = HybridAstarConfig::GetInstance()->traj_gear_switch_penalty;
    planning_core = msquare::HybridAstarConfig::GetInstance()->planning_core;
    max_iter_max = HybridAstarConfig::GetInstance()->max_iter_max;
    max_iter_base = HybridAstarConfig::GetInstance()->max_iter_base;
    traj_steer_change_penalty_gear_switch =
        HybridAstarConfig::GetInstance()->traj_steer_change_penalty_gear_switch;
    traj_obstacle_distance_1_penalty =
        HybridAstarConfig::GetInstance()->traj_obstacle_distance_1_penalty;
    traj_obstacle_distance_2_penalty =
        HybridAstarConfig::GetInstance()->traj_obstacle_distance_2_penalty;
    traj_obstacle_distance_3_penalty =
        HybridAstarConfig::GetInstance()->traj_obstacle_distance_3_penalty;
    traj_end_offset_penalty =
        HybridAstarConfig::GetInstance()->traj_end_offset_penalty;
    traj_s_turn_penalty = HybridAstarConfig::GetInstance()->traj_s_turn_penalty;
}

void AstarPlannerParams::update(){
  if(CarParams::GetInstance()-> lon_inflation_min > lon_inflation){
      CarParams::GetInstance()-> lon_inflation_min = lon_inflation;
  }
  if(CarParams::GetInstance()-> lon_inflation_max < lon_inflation){
      CarParams::GetInstance()-> lon_inflation_max = lon_inflation;
  }

  if(CarParams::GetInstance()-> lat_inflation_min > lat_inflation){
      CarParams::GetInstance()-> lat_inflation_min = lat_inflation;
  }
  if(CarParams::GetInstance()-> lat_inflation_max < lat_inflation){
      CarParams::GetInstance()-> lat_inflation_max = lat_inflation;
  }
 
  HybridAstarConfig::GetInstance()->next_node_num = next_node_num;
  CarParams::GetInstance()->setLonInflation(lon_inflation);
  CarParams::GetInstance()->setLatInflation(lat_inflation);
  HybridAstarConfig::GetInstance() -> max_zigzag_allowd =  max_zigzag_allowed;
  HybridAstarConfig::GetInstance() -> xy_grid_resolution =  xy_grid_resolution;
  HybridAstarConfig::GetInstance()->step_size = step_size;
  HybridAstarConfig::GetInstance()->max_iter = max_iter;
  HybridAstarConfig::GetInstance()->traj_gear_switch_penalty = traj_gear_switch_penalty;
  msquare::HybridAstarConfig::GetInstance()->planning_core = planning_core;
  HybridAstarConfig::GetInstance()->max_iter_max = max_iter_max;
  HybridAstarConfig::GetInstance()->max_iter_base = max_iter_base;
  HybridAstarConfig::GetInstance()->traj_steer_change_penalty_gear_switch =
      traj_steer_change_penalty_gear_switch;
  HybridAstarConfig::GetInstance()->traj_obstacle_distance_1_penalty =
      traj_obstacle_distance_1_penalty;
  HybridAstarConfig::GetInstance()->traj_obstacle_distance_2_penalty =
      traj_obstacle_distance_2_penalty;
  HybridAstarConfig::GetInstance()->traj_obstacle_distance_3_penalty =
      traj_obstacle_distance_3_penalty;
  HybridAstarConfig::GetInstance()->traj_end_offset_penalty =
      traj_end_offset_penalty;
  HybridAstarConfig::GetInstance()->traj_s_turn_penalty = traj_s_turn_penalty;
}

void CarSizeParams::reset(){
    front_edge_to_rear_real = msquare::VehicleParam::Instance() -> front_edge_to_center;
    length = msquare::VehicleParam::Instance() -> length;
    width = msquare::VehicleParam::Instance() -> width;
}


void dump_request_to_file(
  const msquare::parking::OpenspaceDeciderOutput &msg_data) {
    
  std::fstream fs;
  time_t rawtime;
  struct tm *timeinfo;

  time(&rawtime);
  timeinfo = localtime(&rawtime);
  const size_t BUF_SIZE = 128;
  char buf[BUF_SIZE];
  strftime(buf, BUF_SIZE, "%F_%H-%M-%S", timeinfo);

  std::string file_prefix = "/home/ros/Downloads/pnc_logs/SBPRequest_Info";
  std::string log_file(file_prefix + std::string(buf) +
                       std::to_string((int)clock()) + ".yaml");

  fs.open(log_file, std::fstream::app);

  YAML::Node env_node(msg_data);
  fs << env_node;
  fs << std::endl;

  YAML::Node param_node;
  param_node["x_bound"] = 28;
  param_node["y_bound"] = 28;

  YAML::Node strategy_node(*(StrategyParams::GetInstance()));
  param_node["STRATEGY_PARAM"] = strategy_node;

  YAML::Node car_node(*(CarParams::GetInstance()));
  param_node["CAR_PARAMS"] = car_node;

  YAML::Node hybrid_node(*(HybridAstarConfig::GetInstance()));
  param_node["HYBRID_ASTAR_PARAMS"] = hybrid_node;

  YAML::Node obca_node(*(TrajectoryOptimizerConfig::GetInstance()));
  param_node["OBCA_PARAMS"] = obca_node;

  fs << param_node;
  fs.close();
  // std::cout << "file storaged in " << log_file << std::endl;
}

void feedRequest(const std::string &param_string,
                 OpenspaceDeciderOutput& problem) {
    // convert params to global config
    nlohmann::json param_json_obj = nlohmann::json::parse(
      param_string);
    deserialization(param_json_obj["HybridAstarConfig"],
                    HybridAstarConfig::GetInstance());
    deserialization(param_json_obj["VehicleParam"], VehicleParam::Instance());
    deserialization(param_json_obj["CarParams"], CarParams::GetInstance());
    deserialization(param_json_obj["StrategyParams"],
                    StrategyParams::GetInstance());
    if (param_json_obj.contains("TrajectoryOptimizerConfig")) {
      deserialization(param_json_obj["TrajectoryOptimizerConfig"],
                      TrajectoryOptimizerConfig::GetInstance());
    }
    problem = param_json_obj["OpenspaceDeciderOutput"];

    if (false) {
      std::cout << "dump request file !!!" << std::endl;
      dump_request_to_file(problem);
    }
};

void convert_odo(const Pose2D& source_frame, const Pose2D& target_frame, OpenspaceDeciderOutput& source_odo) {
  source_odo.map_boundary = tf2d_inv(target_frame, tf2d(source_frame, source_odo.map_boundary));
  std::vector<planning_math::Box2d> new_obstacle_boxs;
  for(auto box : source_odo.obstacle_boxs) {
    new_obstacle_boxs.push_back(tf2d_inv(target_frame, tf2d(source_frame, box)));
  }
  source_odo.obstacle_boxs.clear();
  source_odo.obstacle_boxs = new_obstacle_boxs;

  std::vector<planning_math::LineSegment2d> new_obstacle_lines;
  for(auto line : source_odo.obstacle_lines) {
    new_obstacle_lines.push_back(tf2d_inv(target_frame, tf2d(source_frame, line)));
  }
  source_odo.obstacle_lines.clear();
  source_odo.obstacle_lines = new_obstacle_lines;

  std::vector<planning_math::LineSegment2d> new_lines;
  for(auto line : source_odo.lines) {
    new_lines.push_back(tf2d_inv(target_frame, tf2d(source_frame, line)));
  }
  source_odo.lines.clear();
  source_odo.lines = new_lines;

  std::vector<planning_math::Vec2d> new_points;
  for(auto point : source_odo.points) {
    new_points.push_back(tf2d_inv(target_frame, tf2d(source_frame, point)));
  }
  source_odo.points.clear();
  source_odo.points = new_points;

  source_odo.T_lines.road_lower_left_bound = tf2d_inv(target_frame, tf2d(source_frame, source_odo.T_lines.road_lower_left_bound));
  source_odo.T_lines.road_lower_right_bound = tf2d_inv(target_frame, tf2d(source_frame, source_odo.T_lines.road_lower_right_bound));
  source_odo.T_lines.road_upper_bound = tf2d_inv(target_frame, tf2d(source_frame, source_odo.T_lines.road_upper_bound));
  source_odo.T_lines.slot_left_bound = tf2d_inv(target_frame, tf2d(source_frame, source_odo.T_lines.slot_left_bound));
  source_odo.T_lines.slot_right_bound = tf2d_inv(target_frame, tf2d(source_frame, source_odo.T_lines.slot_right_bound));


  Pose2D init_pose{source_odo.init_state.path_point.x, source_odo.init_state.path_point.y, source_odo.init_state.path_point.theta};
  Pose2D target_pose{source_odo.target_state.path_point.x, source_odo.target_state.path_point.y, source_odo.target_state.path_point.theta};
  Pose2D init_convert_pose, target_convert_pose;
  init_convert_pose = msquare::planning_math::tf2d_inv(target_frame, msquare::planning_math::tf2d(source_frame, init_pose));
  target_convert_pose = msquare::planning_math::tf2d_inv(target_frame, msquare::planning_math::tf2d(source_frame, target_pose));
  source_odo.init_state.path_point.x = init_convert_pose.x;
  source_odo.init_state.path_point.y = init_convert_pose.y;
  source_odo.init_state.path_point.theta = init_convert_pose.theta;
  source_odo.target_state.path_point.x = target_convert_pose.x;
  source_odo.target_state.path_point.y = target_convert_pose.y;
  source_odo.target_state.path_point.theta = target_convert_pose.theta;
}

}
}