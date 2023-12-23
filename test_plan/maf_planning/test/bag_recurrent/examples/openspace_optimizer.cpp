#include <iostream>
#include <vector>

#include "planner_wrapper.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/openspace_optimizer.h"
#include "planner_wrapper_utils.h"
#include "common/math/vec2d.h"
#include "common/math/box2d.h"
#include "common/math/line_segment2d.h"
#include "common/config/vehicle_param.h"
#include "common/sbp_strategy.h"
#include "common/priority_obs.h"
#include "common/utils/yaml_utils.h"

using std::cout;
using std::endl;

extern "C"
{

msquare::SearchBasedPlannerPtr planner;
msquare::SbpResult result;
std::vector<Pose2D> search_points_pair;
msquare::planning_math::Box2d g_map_boundary;
//std::vector<msquare::Line2d> edges;

int construct_planner(const char* config_file_name, const char *arg_stra_odca_file, const char *car_param_file, TrajectoryPoint target_state, Box2d map_boundary)
{
  using namespace msquare;
  if(!msquare::HybridAstarConfig::GetInstance()->loadFile(config_file_name))
  {
    return false;
  }
  if(!msquare::TrajectoryOptimizerConfig::GetInstance()->loadFile(arg_stra_odca_file))
  {
    return false;
  }
  if(!msquare::CarParams::GetInstance()->loadFile(car_param_file))
  {
    return false;
  }
  if(!msquare::CarParams::GetInstance()->loadFile4Plan(config_file_name))
  {
    return false;
  }
  if(!msquare::StrategyParams::GetInstance()->loadFile(arg_stra_odca_file))
  {
    return false;
  }
  if(!msquare::VehicleParam::Instance()->loadFile(car_param_file))
  {
    return false;
  }

  // // std::cout << "msquare::CarParams:" << std::endl;
  // // std::cout << msquare::CarParams::GetInstance()->yaml_node_ << std::endl;
  // g_map_boundary = get_input_boxes(&map_boundary, 1)[0];
  // Pose2D target_pose(target_state.path_point_.x_, target_state.path_point_.y_, target_state.path_point_.theta_);
  // if(msquare::HybridAstarConfig::GetInstance()->planning_core == 0)
  // {
  //   planner = std::make_shared<msquare::HybridAstar>(g_map_boundary);
  // }
  // else
  // {
  //   throw std::logic_error("unsupported planner!");
  //   // planner = std::make_shared<msquare::DStarLite>(target_pose, 0, g_map_boundary);
  // }
  
  return true;
}

int destruct_planner()
{
  msquare::StrategyParams::GetInstance()->setForceTerminate(true);
  return true;
}

void clean_result() {
  result.x.clear();
  result.y.clear();
  result.phi.clear();
  result.accumulated_s.clear();
  result.steer.clear();
  result.accumulated_s.clear();
  result.v.clear();
  result.a.clear();
  result.wheel_base_offset.clear(); // unused by then 
}

double plan(const char *arg_stra_odca_file)
{
  clean_result();
  msquare::parking::OpenspaceDeciderOutput input;
  YAML::Node node = YAML::LoadFile(arg_stra_odca_file);
  input = node.as<msquare::OpenspaceDeciderOutput>();

  auto t1=std::chrono::steady_clock::now();
  nlohmann::json param_json_obj;
  using namespace msquare;
  param_json_obj["HybridAstarConfig"] = *HybridAstarConfig::GetInstance();
  param_json_obj["VehicleParam"] = *VehicleParam::Instance();
  param_json_obj["CarParams"] = *CarParams::GetInstance();
  param_json_obj["StrategyParams"] = *StrategyParams::GetInstance();
  param_json_obj["OpenspaceDeciderOutput"] = input;
  std::ofstream fs("/home/ros/Downloads/sbp.json");
  fs << param_json_obj.dump();
  if (msquare::parking::genOpenspacePath(input, result)) {
    if (!msquare::parking::GetTemporalProfile(&result)) {
      std::cout << "GetSpeedProfile from Hybrid Astar path fails" << std::endl;
    }
  }
  auto t2=std::chrono::steady_clock::now();

  search_points_pair = planner->getSearchPoints();
  double chrono_ms=std::chrono::duration<double, std::milli>(t2-t1).count();
  std::cout << "complete chrono time: "<<chrono_ms<<"ms"<<std::endl;
  return chrono_ms;
}

int get_result_size()
{
  return result.x.size();
}

int get_search_size()
{
  return search_points_pair.size();
}

void get_result(TrajectoryPoint *trajectory, int *num_segments, int *iteration_times, char *debug_string)
{
  std::cout << "get_result" << std::endl;
  for(size_t i = 0; i < result.x.size(); ++i)
  {
    trajectory[i].path_point_.x_     = result.x.at(i);
    trajectory[i].path_point_.y_     = result.y.at(i);
    trajectory[i].path_point_.theta_ = result.phi.at(i);
    trajectory[i].path_point_.s_     = result.accumulated_s.at(i);
    trajectory[i].steer_ = result.steer.at(i);
    trajectory[i].relative_time_ = result.accumulated_s.at(i);
    trajectory[i].v_ = result.v.at(i);
    trajectory[i].a_ = result.a.at(i);
    trajectory[i].wheel_base_offset_ = result.wheel_base_offset.at(i); // unused by then 
  }
  *num_segments = (int)result.num_segments;
  *iteration_times = (int)result.iteration_times;
  strcpy(debug_string, result.debug_string.c_str());
}

void get_search_progress(Pose2d *Search_points)
{
  std::cout << "get_search_progress" << std::endl;
  for(int i = 0; i < static_cast<int>(search_points_pair.size()); ++i)
  {
    Search_points[i].x = search_points_pair[i].x;
    Search_points[i].y = search_points_pair[i].y;
    Search_points[i].theta = search_points_pair[i].theta;
  }
  
}

}// extern "C"