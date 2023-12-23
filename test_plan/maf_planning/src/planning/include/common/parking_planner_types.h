#pragma once
#include <vector>

#include "common/math/box2d.h"
#include "nlohmann/json.hpp"
#include "planner/trajectory_point.h"
#include "pnc/define/geometry.h"

namespace msquare {
namespace parking {

typedef struct {
  bool is_inited;
  planning_math::LineSegment2d road_upper_bound;
  planning_math::LineSegment2d road_lower_left_bound;
  planning_math::LineSegment2d slot_left_bound;
  planning_math::LineSegment2d road_lower_right_bound;
  planning_math::LineSegment2d slot_right_bound;
} TshapedAreaLines;
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TshapedAreaLines, is_inited,
                                   road_upper_bound, road_lower_left_bound,
                                   slot_left_bound, road_lower_right_bound,
                                   slot_right_bound)

struct APAMetaState{
  bool is_valid = false;
  double last_v = 0.0;
  double parallel_direc;
  double meta_pose_x;
  double meta_pose_y;
  double meta_pose_theta;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(APAMetaState, is_valid, last_v, parallel_direc, meta_pose_x, meta_pose_y, meta_pose_theta)
};


typedef struct {
  bool is_request_square_map = false;
  bool is_problem_ready = false;
  bool is_active_replan = false;
  planning_math::Box2d map_boundary;
  std::vector<planning_math::LineSegment2d> obstacle_lines_map_teb;
  TrajectoryPoint init_state;
  TrajectoryPoint target_state;
  std::vector<planning_math::Box2d> obstacle_boxs;
  std::vector<planning_math::LineSegment2d> obstacle_lines;
  std::vector<planning_math::LineSegment2d> lines;
  std::vector<planning_math::Vec2d> points;
  std::vector<planning_math::Vec2d> step_points;
  TshapedAreaLines T_lines;
  std::string pattern_path;
  APAMetaState apa_meta_state;                       // the init pose at fist planning in parkout
} OpenspaceDeciderOutput;
#ifdef BUILD_IN_TEST_BAG_RECURRENT
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
    OpenspaceDeciderOutput, is_request_square_map, is_problem_ready,
    map_boundary, obstacle_lines_map_teb, init_state, target_state,
    obstacle_boxs, obstacle_lines, lines, points, step_points, T_lines,
    apa_meta_state)
#else  // BUILD_IN_TEST_BAG_RECURRENT
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OpenspaceDeciderOutput,
                                   is_request_square_map, is_problem_ready,
                                   map_boundary, obstacle_lines_map_teb,
                                   init_state, target_state, obstacle_boxs,
                                   obstacle_lines, lines, points, step_points,
                                   T_lines, apa_meta_state)
#endif // BUILD_IN_TEST_BAG_RECURRENT

struct SearchDebugNode {
  double x;
  double y;
  double theta;

  double traj_cost;
  double heuristic_cost;

  SearchDebugNode(double _x, double _y, double _theta, double _traj_cost,
                  double _heuristic_cost)
      : x(_x), y(_y), theta(_theta), traj_cost(_traj_cost),
        heuristic_cost(_heuristic_cost) {}

  SearchDebugNode(const SearchDebugNode &node) {
    this->x = node.x;
    this->y = node.y;
    this->theta = node.theta;
    this->traj_cost = node.traj_cost;
    this->heuristic_cost = node.heuristic_cost;
  }
};

struct SearchDebugEdge {
  SearchDebugNode start_node;
  SearchDebugNode end_node;
  double edge_cost;

  SearchDebugEdge(const SearchDebugNode &node1, const SearchDebugNode &node2)
      : start_node(node1), end_node(node2) {
    edge_cost = end_node.traj_cost - start_node.traj_cost;
  }
};

typedef struct {
  std::vector<SearchDebugNode> searched_node;
  std::vector<std::vector<SearchDebugNode>> added_nodes;
  std::vector<std::vector<SearchDebugEdge>> added_edges;
} SearchProcessDebug;

} // namespace parking
} // namespace msquare