#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/search_node.hpp"

namespace msquare {

namespace hybrid_a_star_2 {

constexpr unsigned int SearchNode::zigzag_num_bits;
constexpr unsigned int SearchNode::zigzag_num_max;

constexpr unsigned int SearchNode::boundary_cost_bits;
constexpr float SearchNode::boundary_cost_max;
constexpr float SearchNode::boundary_cost_step;
constexpr float SearchNode::boundary_cost_inv_step;

constexpr unsigned int SearchNode::distance_to_obstacle_bits;
constexpr float SearchNode::distance_to_obstacle_max;
constexpr float SearchNode::distance_to_obstacle_step;
constexpr float SearchNode::distance_to_obstacle_inv_step;

constexpr unsigned int SearchNode::heuristic_cost_bits;
constexpr float SearchNode::heuristic_cost_max;
constexpr float SearchNode::heuristic_cost_step;
constexpr float SearchNode::heuristic_cost_inv_step;

constexpr unsigned int SearchNode::trajectory_cost_bits;
constexpr float SearchNode::trajectory_cost_max;
constexpr float SearchNode::trajectory_cost_step;
constexpr float SearchNode::trajectory_cost_inv_step;

constexpr unsigned int SearchNode::distance_from_gear_switch_bits;
constexpr float SearchNode::distance_from_gear_switch_max;
constexpr float SearchNode::distance_from_gear_switch_step;
constexpr float SearchNode::distance_from_gear_switch_inv_step;

} // namespace hybrid_a_star_2

} // namespace msquare
