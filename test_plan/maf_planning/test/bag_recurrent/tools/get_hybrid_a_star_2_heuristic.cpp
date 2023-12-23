
#include <cstdio>
#include <fstream>
#include <string>

#include <planner/motion_planner/optimizers/openspace_optimizer/config.h>
#include <planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/curve.hpp>
#include <planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/heuristic_cost.hpp>
#include <planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/hybrid_a_star_2.h>
#include <planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/multi_circle_footprint_model.h>
#include <planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/obstacle_grid.h>

#include "argparse.hpp"

using msquare::hybrid_a_star_2::HeuristicCost;
using msquare::hybrid_a_star_2::HybridAstar2;

argparse::ArgValue InitArgs(int argc, char **argv) {
  argparse::ArgumentParser argument_parser;
  argument_parser.short_name("-r")
      .long_name("--min-turn-radius")
      .help("min turn radius")
      .type(argparse::ValueType::DIGITS_DOUBLE)
      .default_value(5.0)
      .done();
  argument_parser.short_name("-g")
      .long_name("--gear-switch-cost")
      .help("gear switch cost")
      .type(argparse::ValueType::DIGITS_DOUBLE)
      .default_value(10.0)
      .done();
  argument_parser.short_name("-x")
      .long_name("--x-range")
      .help("x range of heuristic map")
      .type(argparse::ValueType::DIGITS_DOUBLE)
      .default_value(20.0)
      .done();
  argument_parser.short_name("-y")
      .long_name("--y-range")
      .help("y range of heuristic map")
      .type(argparse::ValueType::DIGITS_DOUBLE)
      .default_value(20.0)
      .done();
  argument_parser.short_name("-e")
      .long_name("--extend-range")
      .help("extend range of heuristic map calculation")
      .type(argparse::ValueType::DIGITS_DOUBLE)
      .default_value(7.0)
      .done();
  argument_parser.short_name("-xyr")
      .long_name("--xy-resolution")
      .help("x / y resolution of heuristic map")
      .type(argparse::ValueType::DIGITS_DOUBLE)
      .default_value(0.1)
      .done();
  argument_parser.short_name("-tr")
      .long_name("--theta-resolution")
      .help("theta resolution of heuristic map")
      .type(argparse::ValueType::DIGITS_DOUBLE)
      .default_value(0.0133)
      .done();
  argument_parser.short_name("-o")
      .long_name("--output")
      .help("output heuristic map file")
      .type(argparse::ValueType::STRING)
      .default_value("")
      .done();
  return argument_parser.parse_args_any_type(argc, argv);
}

int padBins(int bin_size, int count) {
  int bin_edge = (bin_size - 1) / 2;
  return (count - bin_edge + bin_size - 1) / bin_size * bin_size + bin_edge;
}

void HybridAstar2::createHeuristic(HeuristicCost &heuristic_cost,
                                   float min_turn_radius,
                                   float gear_switch_cost, float x_range,
                                   float y_range, float extend_range,
                                   float xy_resolution, float theta_resolution,
                                   float xy_resolution_state,
                                   float theta_resolution_state) {
  curve::Curve::setWheelBase(3.0f);
  curve::Curve::setMaxResolution(0.01f);

  int x_count_per_side =
      std::ceil((x_range + extend_range) / xy_resolution_state);
  float min_x = -float(x_count_per_side) * xy_resolution_state;
  int x_count = x_count_per_side * 2 + 1;
  int y_count_per_side =
      std::ceil((y_range + extend_range) / xy_resolution_state);
  float min_y = -float(y_count_per_side) * xy_resolution_state;
  int y_count = y_count_per_side * 2 + 1;
  int theta_count_per_side = std::ceil(float(M_PI) / theta_resolution_state);
  float min_theta = -float(theta_count_per_side) * theta_resolution_state;
  int theta_count = theta_count_per_side * 2 + 1;

  ResourcePool<SearchNode> nodes_pool(
      int(x_count * y_count * theta_count * 1.1));
  TrajectoryCostFunc pq_cost_func(nodes_pool);
  TrajectoryCostFunc state_map_cost_func(nodes_pool);

  float pq_bucket_size = 0.0001f;
  int pq_bucket_count = int(std::ceil(200.0f / pq_bucket_size));
  BucketPriorityQueue<int, TrajectoryCostFunc> pq(pq_cost_func, pq_bucket_size,
                                                  pq_bucket_count, 0, 0);

  StateMap<3, int, TrajectoryCostFunc> state_map(
      state_map_cost_func, nodes_pool.invalidKey(), {8, 8, 8}, 0);
  state_map.setStep(
      {xy_resolution_state, xy_resolution_state, theta_resolution_state});
  state_map.setSize({x_count, y_count, theta_count});
  state_map.setOrigin({min_x, min_y, min_theta});
  state_map.clear();

  float max_range = 1000.0f;
  VariableStepSizeStateExpansion state_expansion;
  state_expansion.init(false, min_turn_radius, xy_resolution_state,
                       theta_resolution_state, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f,
                       100000, false, 0.0f, 0.0f, 0.0f, false, 0.0f, 0.0f, 1);
  state_expansion.setLocalInfo(Pose2D(-max_range, -max_range, 0.0f),
                               max_range * 2.0f, max_range * 2.0f);
  MultiCircleFootprintModel fake_footprint_model;
  ObstacleGrid fake_obstacle_grid;

  SearchNode::Config node_config;
  node_config.traj_forward_penalty = 1.0f;
  node_config.traj_back_penalty = 1.0f;
  node_config.traj_gear_switch_penalty = gear_switch_cost;
  node_config.traj_boundary_cost_penalty = 0.0f;
  node_config.traj_steer_penalty = 0.0f;
  node_config.traj_steer_change_penalty_gear_switch = 0.0f;
  node_config.traj_steer_change_penalty = 0.0f;
  node_config.traj_s_turn_penalty = 0.0f;
  node_config.traj_obstacle_distance_1_penalty = 0.0f;
  node_config.traj_obstacle_distance_2_penalty = 0.0f;
  node_config.traj_obstacle_distance_3_penalty = 0.0f;
  node_config.traj_end_offset_penalty = 0.0f;
  node_config.traj_penalty_start_at_reverse_as_gear_switch = false;

  float max_cost = 50.0f;
  int x_count_per_side_map = std::ceil(x_range / xy_resolution);
  float min_x_map = -float(x_count_per_side_map) * xy_resolution;
  int x_count_map = x_count_per_side_map * 2 + 1;
  int y_count_map = int(std::ceil(y_range / xy_resolution)) + 1;
  int theta_count_per_side_map = std::ceil(float(M_PI) / theta_resolution);
  float min_theta_map = -float(theta_count_per_side_map) * theta_resolution;
  int theta_count_map = theta_count_per_side_map * 2 + 1;
  heuristic_cost.init(max_cost / 255.0f, min_x_map, 0.0f, min_theta_map,
                      xy_resolution, xy_resolution, theta_resolution,
                      x_count_map, y_count_map, theta_count_map, true);
  printf("min_x = %f, min_theta = %f, x_count = %d, y_count = %d, theta_count "
         "= %d\n",
         min_x_map, min_theta_map, x_count_map, y_count_map, theta_count_map);

  int start_node_key = nodes_pool.allocate().key;
  curve::Curve start_curve;
  start_curve.initAsStart(true, false);
  SearchNode &start_node = nodes_pool[start_node_key];
  start_node.init(node_config, &start_curve, 0.0f, 0.0f, 0.0f);
  start_node.setDistanceToObstacle(100.0f);
  pq.push(start_node_key);
  int counter = 0;
  int total_count = x_count * y_count * theta_count;
  while (!pq.empty()) {
    counter++;
    const SearchNode &current = nodes_pool[pq.pop()];
    if (counter % 100000 == 0) {
      printf("%d of %d, pq size: %zd, current cost: %f\n", counter, total_count,
             pq.size(), current.trajectory_cost());
    }
    heuristic_cost.update(current.x(), current.y(), current.theta(),
                          current.trajectory_cost());

    const std::vector<SearchNode *> &next = state_expansion.getNextStates(
        node_config, current, fake_footprint_model, fake_obstacle_grid);
    for (size_t i = 0; i < next.size(); i++) {
      auto state_insert_res = state_map.tryInsert(
          {next[i]->x(), next[i]->y(), next[i]->theta()}, *next[i]);
      bool override_prune =
          state_insert_res.belong && !state_insert_res.success &&
          SearchNode::isParentOrBrother(nodes_pool[state_insert_res.key],
                                        *next[i]);
      if (state_insert_res.success || override_prune) {
        auto allocate_res = nodes_pool.allocate();
        if (!allocate_res.success) {
          continue;
        }
        auto &node = nodes_pool[allocate_res.key];
        node.copy(*next[i]);
        node.setHeuristicCost(0.0f);
        node.setDistanceToObstacle(100.0f);

        pq.push(allocate_res.key);
        int replaced = nodes_pool.invalidKey();
        if (state_insert_res.belong && state_insert_res.success) {
          replaced =
              state_map.insert(state_insert_res.bucket_id,
                               state_insert_res.offset, allocate_res.key);
        }
        if (replaced != nodes_pool.invalidKey() &&
            !SearchNode::isBrother(nodes_pool[replaced],
                                   nodes_pool[allocate_res.key])) {
          if (pq.erase(replaced)) {
            nodes_pool.release(replaced);
          }
        }
      }
    }
  }
}

int main(const int argc, char **argv) {
  auto args = InitArgs(argc, argv);
  std::string output = args.get_value<std::string>("output");
  if (output == "") {
    std::fprintf(stderr, "output not set\n");
    return -1;
  }

  float min_turn_radius = args.get_value<double>("min-turn-radius");
  float gear_switch_cost = args.get_value<double>("gear-switch-cost");
  float x_range = args.get_value<double>("x-range");
  float y_range = args.get_value<double>("y-range");
  float extend_range = args.get_value<double>("extend-range");
  float xy_resolution = args.get_value<double>("xy-resolution");
  float theta_resolution = args.get_value<double>("theta-resolution");

  HeuristicCost heuristic_cost;
  HybridAstar2::createHeuristic(
      heuristic_cost, min_turn_radius, gear_switch_cost, x_range, y_range,
      extend_range, xy_resolution, theta_resolution, 0.1f, 0.0133f);
  heuristic_cost.dump(output);

  for (int it = 0; it < heuristic_cost.theta_count(); it++) {
    cv::Mat_<std::uint8_t> result((heuristic_cost.y_count() - 1) * 2 + 1,
                                  heuristic_cost.x_count());
    float theta =
        float(it - heuristic_cost.theta_count() / 2) * theta_resolution;
    for (int iy = 0; iy < result.rows; iy++) {
      for (int ix = 0; ix < result.cols; ix++) {
        float x = float(ix - result.cols / 2) * xy_resolution;
        float y = float(iy - result.rows / 2) * xy_resolution;
        float cost =
            heuristic_cost.get(true, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, x, y, theta);
        result(iy, ix) = std::round(cost / heuristic_cost.cost_scale());
      }
    }
    char buf[256];
    sprintf(buf, "%010d.png", it);
    cv::imwrite(std::string(buf), result);
  }
  return 0;
}
