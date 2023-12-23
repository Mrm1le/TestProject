#ifndef MSQUARE_PARKING_BIND_UTIL
#define MSQUARE_PARKING_BIND_UTIL
#include "nlohmann/json.hpp"
#include <yaml-cpp/yaml.h>
#include "common/utils/yaml_utils_parking.h"
#include "common/parking_config_deserialization.h"

namespace msquare{
namespace parking{

  /**
   * @brief planner params to be tuned
   * 
   */
  struct AstarPlannerParams{
    int next_node_num;
    double lon_inflation;
    double lat_inflation;
    double traj_gear_switch_penalty;
    int max_zigzag_allowed;
    double xy_grid_resolution;
    double step_size;
    int max_iter;
    int planning_core;
    int max_iter_max;
    int max_iter_base;
    double traj_steer_change_penalty_gear_switch;
    double traj_obstacle_distance_1_penalty;
    double traj_obstacle_distance_2_penalty;
    double traj_obstacle_distance_3_penalty;
    double traj_end_offset_penalty;
    double traj_s_turn_penalty;

    /* reset to default params  */
    void reset();
    /* overrite params  */
    void update();

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        AstarPlannerParams, next_node_num, lon_inflation, lat_inflation,
        traj_gear_switch_penalty, max_zigzag_allowed, xy_grid_resolution,
        step_size, max_iter, planning_core, max_iter_max, max_iter_base,
        traj_steer_change_penalty_gear_switch, traj_obstacle_distance_1_penalty,
        traj_obstacle_distance_2_penalty, traj_obstacle_distance_3_penalty,
        traj_end_offset_penalty, traj_s_turn_penalty);
  };

  /**
   * @brief params to get car body box
   * 
   */

  struct AstarPlannerInput{
    OpenspaceDeciderOutput odo;
    AstarPlannerParams params;
    std::string car_type = "default";
    std::string param_string;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(AstarPlannerInput, odo, params, car_type, param_string)
  };

  struct CarSizeParams{
    double front_edge_to_rear_real;
    double length;
    double width;
    /* reset to default params  */
    void reset();

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(CarSizeParams, front_edge_to_rear_real, length, width)
  };

  struct AstarPlannerOutput{
    msquare::SbpResult sbp_result;
    CarSizeParams car_params;
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(AstarPlannerOutput, sbp_result, car_params)
  };

  void feedRequest(const std::string &param_string,
                   OpenspaceDeciderOutput &problem);
  void dump_request_to_file(
      const msquare::parking::OpenspaceDeciderOutput &msg_data);

  void loadDevcarParams();
  void loadEpcarParams();

  void convert_odo(const Pose2D& source_frame, const Pose2D& target_frame, OpenspaceDeciderOutput& source_odo);
}
}

#endif  // MSQUARE_PARKING_BIND_UTIL