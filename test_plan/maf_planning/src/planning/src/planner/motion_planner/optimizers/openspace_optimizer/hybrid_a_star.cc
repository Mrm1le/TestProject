#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star.h"
#include "planner/motion_planner/optimizers/openspace_optimizer/config.h"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace msquare {

using namespace planning_math;

HybridAstar::HybridAstar(const int max_node_num) {
  sbp_rspath_ = SbpRSPath();
  for (int i = 0; i < max_node_num; i++) {
    nodes_vec_.emplace_back(std::shared_ptr<SearchNode>(new SearchNode()));
  }
  nodes_size_ = 0;
  reeds_shepp_path_ = std::make_shared<ReedSheppPath>();
  result_.clear();
}

HybridAstar::HybridAstar(const planning_math::Box2d map_boundary,
                         const std::vector<planning_math::LineSegment2d> &map) {
  sbp_rspath_ = SbpRSPath();
  xy_grid_resolution_ = HybridAstarConfig::GetInstance()->xy_grid_resolution;
  phi_grid_resolution_ = HybridAstarConfig::GetInstance()->phi_grid_resolution;
  step_size_ = HybridAstarConfig::GetInstance()->step_size;
  delta_t_ = HybridAstarConfig::GetInstance()->delta_t;
  max_delta_angle_ = CarParams::GetInstance()->max_delta_angle;
  wheel_base_ = CarParams::GetInstance()->wheel_base;
  front_edge_to_rear_ = VehicleParam::Instance()->front_edge_to_center;
  right_edge_to_center_ = CarParams::GetInstance()->vehicle_width / 2;
  back_edge_to_rear_ = VehicleParam::Instance()->back_edge_to_center;
  left_edge_to_center_ = CarParams::GetInstance()->vehicle_width / 2;
  next_node_num_ = HybridAstarConfig::GetInstance()->next_node_num;
  // visualize
  verbose = HybridAstarConfig::GetInstance()->verbose;
  display_points = HybridAstarConfig::GetInstance()->display_points;
  // init local coordinate system
  Vec2d tmp_origin_corner = map_boundary.GetAllCorners().at(3);
  local_frame_pose_.x = tmp_origin_corner.x();
  local_frame_pose_.y = tmp_origin_corner.y();
  local_frame_pose_.theta = map_boundary.heading();

  x_bound_ = map_boundary.length();
  y_bound_ = map_boundary.width();

  nodes_size_ = 0;
  reeds_shepp_path_ = std::make_shared<ReedSheppPath>();

  reed_shepp_generator_ = std::make_unique<ReedShepp>();

  double lat_inflation_ = CarParams::GetInstance()->lat_inflation();
  double shrink_ratio_for_lines =
      CarParams::GetInstance()->shrink_ratio_for_lines_;

  FootprintModelPtr box_model = std::make_shared<BoxFootprintModel>(
      VehicleParam::Instance(), lat_inflation_, shrink_ratio_for_lines);
  FootprintModelPtr mirror_model = std::make_shared<CircleFootprintModel>(
      VehicleParam::Instance(),
      CarParams::GetInstance()->inflation_rearview_mirror,
      shrink_ratio_for_lines);
  FootprintModelPtr fast_footprint_model_ =
      std::make_shared<CompositeFootprintModel>(
          std::vector<FootprintModelPtr>({box_model, mirror_model}));

  if (HybridAstarConfig::GetInstance()->footprint_model_ == 0) {
    footprint_model_ = fast_footprint_model_;
  } else {
    footprint_model_ = std::make_shared<PolygonFootprintModel>(
        VehicleParam::Instance(),
        (EgoModelType)HybridAstarConfig::GetInstance()
            ->footprint_model_precise_,
        lat_inflation_, shrink_ratio_for_lines);
  }

  if (HybridAstarConfig::GetInstance()->footprint_model_precise_ == 0) {
    footprint_model_precise_ = fast_footprint_model_;
  } else {
    footprint_model_precise_ = std::make_shared<PolygonFootprintModel>(
        VehicleParam::Instance(),
        (EgoModelType)HybridAstarConfig::GetInstance()
            ->footprint_model_precise_,
        lat_inflation_, shrink_ratio_for_lines);
  }

  footprint_model_real_ = std::make_shared<PolygonFootprintModel>(
      VehicleParam::Instance(), EgoModelType::TETRADECAGON, 0, 0);

  double deg2rad = M_PI / 180.0;
  const double &max_delta_angle_rear =
      CarParams::GetInstance()->max_delta_angle_rear;
  if (CarParams::GetInstance()->enable_multiple_steer_modes) {
    double min_wheel_base_offset = wheel_base_ *
                                   std::tan(-max_delta_angle_rear * deg2rad) /
                                   (std::tan(max_delta_angle_ * deg2rad) -
                                    std::tan(-max_delta_angle_rear * deg2rad));
    wheel_base_offset_options_.push_back(min_wheel_base_offset);
    wheel_base_offset_options_.push_back(0);
    wheel_base_offset_options_.push_back(100.0 * wheel_base_);
  } else {
    wheel_base_offset_options_.push_back(0);
  }
  result_.clear();
  result_.status = SbpStatus::EXCEPTION;
}

void HybridAstar::Update(const planning_math::Box2d map_boundary,
                         const std::vector<planning_math::LineSegment2d> &map) {
  xy_grid_resolution_ = HybridAstarConfig::GetInstance()->xy_grid_resolution;
  phi_grid_resolution_ = HybridAstarConfig::GetInstance()->phi_grid_resolution;
  step_size_ = HybridAstarConfig::GetInstance()->step_size;
  delta_t_ = HybridAstarConfig::GetInstance()->delta_t;
  max_delta_angle_ = CarParams::GetInstance()->max_delta_angle;
  wheel_base_ = CarParams::GetInstance()->wheel_base;
  front_edge_to_rear_ = VehicleParam::Instance()->front_edge_to_center;
  right_edge_to_center_ = CarParams::GetInstance()->vehicle_width / 2;
  back_edge_to_rear_ = VehicleParam::Instance()->back_edge_to_center;
  left_edge_to_center_ = CarParams::GetInstance()->vehicle_width / 2;
  next_node_num_ = HybridAstarConfig::GetInstance()->next_node_num;
  // visualize
  verbose = HybridAstarConfig::GetInstance()->verbose;
  display_points = HybridAstarConfig::GetInstance()->display_points;
  // init local coordinate system
  Vec2d tmp_origin_corner = map_boundary.GetAllCorners().at(3);
  local_frame_pose_.x = tmp_origin_corner.x();
  local_frame_pose_.y = tmp_origin_corner.y();
  local_frame_pose_.theta = map_boundary.heading();
  x_bound_ = map_boundary.length();
  y_bound_ = map_boundary.width();

  int difference =
      2 * next_node_num_ * HybridAstarConfig::GetInstance()->max_iter -
      nodes_vec_.size();
  if (difference > 0) {
    for (int i = 0; i < difference; i++) {
      nodes_vec_.emplace_back(std::shared_ptr<SearchNode>(new SearchNode()));
    }
  }
  nodes_size_ = 0;

  reed_shepp_generator_ = std::make_unique<ReedShepp>();

  double lat_inflation_ = CarParams::GetInstance()->lat_inflation();
  double shrink_ratio_for_lines =
      CarParams::GetInstance()->shrink_ratio_for_lines_;

  FootprintModelPtr box_model = std::make_shared<BoxFootprintModel>(
      VehicleParam::Instance(), lat_inflation_, shrink_ratio_for_lines);
  FootprintModelPtr mirror_model = std::make_shared<CircleFootprintModel>(
      VehicleParam::Instance(),
      CarParams::GetInstance()->inflation_rearview_mirror,
      shrink_ratio_for_lines);
  FootprintModelPtr fast_footprint_model_ =
      std::make_shared<CompositeFootprintModel>(
          std::vector<FootprintModelPtr>({box_model, mirror_model}));

  if (HybridAstarConfig::GetInstance()->footprint_model_ == 0) {
    footprint_model_ = fast_footprint_model_;
  } else {
    footprint_model_ = std::make_shared<PolygonFootprintModel>(
        VehicleParam::Instance(),
        (EgoModelType)HybridAstarConfig::GetInstance()
            ->footprint_model_precise_,
        lat_inflation_, shrink_ratio_for_lines);
  }

  if (HybridAstarConfig::GetInstance()->footprint_model_precise_ == 0) {
    footprint_model_precise_ = fast_footprint_model_;
  } else {
    footprint_model_precise_ = std::make_shared<PolygonFootprintModel>(
        VehicleParam::Instance(),
        (EgoModelType)HybridAstarConfig::GetInstance()
            ->footprint_model_precise_,
        lat_inflation_, shrink_ratio_for_lines);
  }

  footprint_model_real_ = std::make_shared<PolygonFootprintModel>(
      VehicleParam::Instance(), EgoModelType::TETRADECAGON, 0, 0);

  double deg2rad = M_PI / 180.0;
  const double &max_delta_angle_rear =
      CarParams::GetInstance()->max_delta_angle_rear;

  wheel_base_offset_options_.clear();
  if (CarParams::GetInstance()->enable_multiple_steer_modes) {
    double min_wheel_base_offset = wheel_base_ *
                                   std::tan(-max_delta_angle_rear * deg2rad) /
                                   (std::tan(max_delta_angle_ * deg2rad) -
                                    std::tan(-max_delta_angle_rear * deg2rad));
    wheel_base_offset_options_.push_back(min_wheel_base_offset);
    wheel_base_offset_options_.push_back(0);
    wheel_base_offset_options_.push_back(100.0 * wheel_base_);
  } else {
    wheel_base_offset_options_.push_back(0);
  }
  result_.clear();
  result_.status = SbpStatus::EXCEPTION;
}

HybridAstar::~HybridAstar() {}

bool HybridAstar::AnalyticExpansion(std::shared_ptr<SearchNode> current_node,
                                    std::vector<SbpObstaclePtr> obs_ptrs) {
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_path_)) {
    // std::cout << "AnalyticExpansion failed" << std::endl;
    return false;
  }

  // prefer small curvature
  double radius = calCircularRadius(current_node, end_node_);
  radius -= 0.1;
  bool small_kappa_rs_succeed = false;
  if (radius > CarParams::GetInstance()->min_turn_radius &&
      radius < CarParams::GetInstance()->min_turn_radius * 2) {
    ReedShepp small_kappa_rs_generator(1.0 / radius, step_size_);
    small_kappa_rs_succeed = small_kappa_rs_generator.ShortestRSP(
        current_node, end_node_, reeds_shepp_path_);
  }

  if (HybridAstarConfig::GetInstance()->max_analytic_expansion_length > 0 &&
      reeds_shepp_path_->total_length >
          HybridAstarConfig::GetInstance()->max_analytic_expansion_length) {
    return false;
  }

  size_t zigzag_rs = 0;
  if ((current_node->vel > 0) != reeds_shepp_path_->gear[0]) {
    zigzag_rs += 1;
  }
  for (size_t i = 1; i < reeds_shepp_path_->gear.size(); ++i) {
    if (reeds_shepp_path_->gear[i] != reeds_shepp_path_->gear[i - 1]) {
      zigzag_rs += 1;
    }
  }
  // if(end_node_->vel * (reeds_shepp_path_->gear.back()? 1:-1) < 0) {
  //   zigzag_rs += 1;
  // }
  bool is_analytic_expantion_zigzag =
      !HybridAstarConfig::GetInstance()->enable_analytic_expansion &&
      zigzag_rs > 0;
  bool is_zigzag_too_much =
      is_analytic_expantion_zigzag ||
      zigzag_rs + current_node->zigzags >
          HybridAstarConfig::GetInstance()->max_zigzag_allowd;

  sbp_rspath_.update(current_node, reeds_shepp_path_);
  auto nodes = sbp_rspath_.getNodes();
  for (auto node : nodes) {
    if (IsNodeOutOfRange(node)) {
      return false;
    }
  }
  if (!is_zigzag_too_much &&
      !sbp_rspath_.checkCollision(step_size_, obs_ptrs, footprint_model_,
                                  footprint_model_precise_)) {
    reeds_shepp_to_end_ = *reeds_shepp_path_;
    if (small_kappa_rs_succeed) {
      // std::cout << "small_kappa_rs_succeed ";
    }
    // std::cout << "current_node->vel " << current_node->vel
    //           << ", current_node->zigzags " << current_node->zigzags
    //           << std::endl;
    // std::cout << "reeds_shepp segments len = ";
    for (const double &len : reeds_shepp_path_->segs_lengths) {
      // std::cout << len << " ";
    }
    // std::cout << std::endl;
    // rs curve
    // std::cout << "reeds_shepp segments = ";
    for (const char &type : reeds_shepp_path_->segs_types) {
      // std::cout << type << " ";
    }
    // std::cout << std::endl;
    // std::cout << "AnalyticExpansion succeed = " << zigzag_rs << std::endl;
    return true;
  }
  return false;
}

void HybridAstar::combineTrajectory(SbpResult *result,
                                    std::shared_ptr<SearchNode> current) {
  result->clear();
  double x_ref, y_ref;
  for (std::shared_ptr<SearchNode> Dummy = current; Dummy;
       Dummy = Dummy->previous) {
    x_ref = Dummy->x + Dummy->wheel_base_offset_ * std::cos(Dummy->theta);
    y_ref = Dummy->y + Dummy->wheel_base_offset_ * std::sin(Dummy->theta);
    result->x.push_back(x_ref);
    result->y.push_back(y_ref);
    result->phi.push_back(planning_math::NormalizeAngle(Dummy->theta));
    result->wheel_base_offset.push_back(Dummy->wheel_base_offset_);
  }

  std::reverse(result->x.begin(), result->x.end());
  std::reverse(result->y.begin(), result->y.end());
  std::reverse(result->phi.begin(), result->phi.end());
  std::reverse(result->wheel_base_offset.begin(),
               result->wheel_base_offset.end());

  if (verbose == VERBOSE_DEBUG) {
    // std::cout << "--------------HA* result----------------" << std::endl;
    // std::cout << "index \tx \ty \tphi \twheel_base_offset" << std::endl;
    for (int i = 0; i < result->x.size(); i++) {
      using namespace std;
      // std::cout.precision(3);
      // std::cout << setiosflags(ios::showpoint) << setiosflags(ios::fixed);
      // std::cout << i << "\t" << result->x.at(i) << "\t" << result->y.at(i)
      //           << "\t" << result->phi.at(i) << "\t"
      //           << result->wheel_base_offset.at(i) << "\t" << std::endl;
    }
  }
}

std::vector<std::shared_ptr<SearchNode>>
HybridAstar::getNextStates(const std::shared_ptr<SearchNode> &current) {
  std::vector<std::shared_ptr<SearchNode>> next;
  double next_x, next_y, next_theta;
  double alpha, beta, R;

  double x, y, theta;
  double delta_step_size = step_size_ * 2.0;
  double deg2rad = M_PI / 180.0;
  const double &max_delta_angle_rear =
      CarParams::GetInstance()->max_delta_angle_rear;
  double max_wheel_base_offset = wheel_base_ *
                                 std::tan(max_delta_angle_rear * deg2rad) /
                                 (std::tan(max_delta_angle_ * deg2rad) -
                                  std::tan(max_delta_angle_rear * deg2rad));
  for (size_t i = 0; i < wheel_base_offset_options_.size(); ++i) {
    double wheel_base_offset = wheel_base_offset_options_[i];
    if (std::fabs(current->vel) > 1e-5 && current->wheel_base_offset_ <= 0 &&
        wheel_base_offset > 0) {
      continue;
    }
    double max_delta_angle_apply = max_delta_angle_;
    if (wheel_base_offset > max_wheel_base_offset) {
      max_delta_angle_apply = max_delta_angle_rear;
    }

    if (next_node_num_ % 2 == 0 || next_node_num_ < 2) {
      // throw std::invalid_argument("invalid next_node_num!");
    }
    double initial_travel_step = -step_size_;
    double terminal_travel_step = step_size_;
    if (HybridAstarConfig::GetInstance()->step_direction == -1) {
      terminal_travel_step = 0;
    } else if (HybridAstarConfig::GetInstance()->step_direction == 1) {
      initial_travel_step = step_size_;
    }
    for (double traveled_distance = initial_travel_step;
         traveled_distance < terminal_travel_step + 0.0001;
         traveled_distance += delta_step_size) {
      if (traveled_distance * current->vel > 0) {
        if (std::fabs(wheel_base_offset - current->wheel_base_offset_) > 1e-5) {
          continue;
        }
      }

      double max_delta_rate_equivalent =
          CarParams::GetInstance()->max_delta_angle_rate *
          (wheel_base_ + wheel_base_offset) *
          std::cos(current->delta * deg2rad) *
          std::cos(current->delta * deg2rad) / wheel_base_;
      // printf("max_delta_rate_equivalent = %f, max_delta_rate = %f.\n",
      // max_delta_rate_equivalent,
      // CarParams::GetInstance()->max_delta_angle_rate);
      double max_delta_rate_apply =
          std::min(CarParams::GetInstance()->max_delta_angle_rate,
                   max_delta_rate_equivalent);
      double steer_lower = std::max(current->delta - max_delta_rate_apply,
                                    -max_delta_angle_apply);
      double steer_upper = std::min(current->delta + max_delta_rate_apply,
                                    max_delta_angle_apply);
      if (traveled_distance * current->vel < 0) {
        steer_lower = -max_delta_angle_apply;
        steer_upper = max_delta_angle_apply;
      }
      double delta_step =
          (steer_upper - steer_lower) / (double)(next_node_num_ - 1);
      if (delta_step <= 1e-6) {
        // throw std::invalid_argument("invalid steer limits!");
      }

      // transform the state w.r.t virtual wheel base
      x = current->x - wheel_base_offset * std::cos(current->theta);
      y = current->y - wheel_base_offset * std::sin(current->theta);
      theta = current->theta;
      for (alpha = steer_lower; alpha < steer_upper + 0.001;
           alpha += delta_step) {
        if (std::abs(alpha) < 1e-6) {
          alpha = alpha > 0 // parasoft-suppress AUTOSAR-M6_5_3 "f-drop"
                      ? 1e-6
                      : -1e-6;
        }
        R = (wheel_base_ + wheel_base_offset) / std::tan(alpha * deg2rad);
        beta = traveled_distance / R;
        next_theta = NormalizeAngle(theta + beta);
        next_x = x + R * (std::cos(theta) * std::sin(beta) -
                          std::sin(theta) * (1 - std::cos(beta)));
        next_y = y + R * (std::sin(theta) * std::sin(beta) +
                          std::cos(theta) * (1 - std::cos(beta)));
        // transform the state w.r.t virtual wheel base back
        next_x += wheel_base_offset * std::cos(next_theta);
        next_y += wheel_base_offset * std::sin(next_theta);

        if (nodes_size_ < nodes_vec_.size()) {
          nodes_vec_[nodes_size_]->set_node(next_x, next_y, next_theta,
                                            traveled_distance, alpha,
                                            wheel_base_offset);
        } else {
          nodes_vec_.emplace_back(std::shared_ptr<SearchNode>(
              new SearchNode(next_x, next_y, next_theta, traveled_distance,
                             alpha, wheel_base_offset)));
        }
        nodes_vec_[nodes_size_]->previous = current;
        nodes_vec_[nodes_size_]->zigzags = current->zigzags;
        if (traveled_distance * current->vel < 0) {
          nodes_vec_[nodes_size_]->zigzags = current->zigzags + 1;
        }
        // if(!next.empty() && nn->GetIndex() == next.back()->GetIndex())
        // {
        //   if(nn->trajcost < next.back()->trajcost)
        //   {
        //     next.pop_back();
        //     next.push_back(nn);
        //   }
        // }
        // else
        if (nodes_vec_[nodes_size_]->zigzags <=
                HybridAstarConfig::GetInstance()->max_zigzag_allowd &&
            !IsNodeOutOfRange(nodes_vec_[nodes_size_])) {
          next.push_back(nodes_vec_[nodes_size_]);
          nodes_size_++;
        }
      }
    }
  }
  return next;
}

bool HybridAstar::IsNodeOutOfRange(std::shared_ptr<SearchNode> current_node) {
  Pose2D cur_2D =
      tf2d(local_frame_pose_,
           Pose2D(current_node->x, current_node->y, current_node->theta));
  if (cur_2D.x < 0 || cur_2D.x >= x_bound_ || cur_2D.y < 0 ||
      cur_2D.y >= y_bound_) {
    return true;
  }
  return false;
}

bool HybridAstar::checkCollision(std::shared_ptr<SearchNode> current_state) {
  static std::shared_ptr<SearchNode> tmp_check_node =
      std::make_shared<SearchNode>();
  int gear = (current_state->vel < 0.0) ? -1 : 1;
  const SearchNode &pos = *current_state;

  if (pos.previous != nullptr) {
    const SearchNode &previous_pos = *pos.previous;
    int previous_gear = (previous_pos.vel < 0.0) ? -1 : 1;
    if (previous_gear != gear && previous_pos.previous) {
      double x_previous_safe =
          previous_pos.x + previous_gear *
                               CarParams::GetInstance()->lon_inflation() *
                               cos(previous_pos.theta);
      double y_previous_safe =
          previous_pos.y + previous_gear *
                               CarParams::GetInstance()->lon_inflation() *
                               sin(previous_pos.theta);
      tmp_check_node->set_node(x_previous_safe, y_previous_safe,
                               previous_pos.theta);
      for (const SbpObstaclePtr &obs_ptr : obs_ptrs_) {
        if (obs_ptr->checkCollision(tmp_check_node, footprint_model_precise_)) {
          return true;
        }
      }
    }
  }

  for (const SbpObstaclePtr &obs_ptr : obs_ptrs_) {
    if (obs_ptr->checkCollision(current_state, footprint_model_)) {
      return true;
    }
  }

  return false;
}

bool HybridAstar::checkCollisionReal(
    std::shared_ptr<SearchNode> current_state) {
  Pose2D cur_2D =
      tf2d(local_frame_pose_,
           Pose2D(current_state->x, current_state->y, current_state->theta));
  if (cur_2D.x < 0 || cur_2D.x >= x_bound_ || cur_2D.y < 0 ||
      cur_2D.y >= y_bound_) {
    return true;
  }
  for (const SbpObstaclePtr &obs_ptr : obs_ptrs_) {
    if (obs_ptr->checkCollision(current_state, footprint_model_real_)) {
      return true;
    }
  }
  return false;
}

bool HybridAstar::Plan(const std::vector<SbpObstaclePtr> &obs_ptrs,
                       parking::SearchProcessDebug *sp_debug) {
  int size_tmp = 0;

  obs_ptrs_ = obs_ptrs;

  if (start_node_ == nullptr || end_node_ == nullptr) {
    // std::cout << " HybridAstar::Start node or End node not set\n";
    result_.status = SbpStatus::INFEASIBLE;
    return false;
  }

  if (checkCollisionReal(start_node_)) {
    // std::cout << " HybridAstar::Plan failed for collision at start node\n";
    result_.status = SbpStatus::START_INFEASIBLE;
    return false;
  }
  if (checkCollisionReal(end_node_)) {
    // std::cout << " HybridAstar::Plan failed for collision at end node\n";
    result_.status = SbpStatus::END_INFEASIBLE;
    return false;
  }

  searchPoints_ = std::vector<Pose2D>();

  Compare::target = *end_node_.get();
  Compare::cmp_frame_pose_ = local_frame_pose_;
  Compare cmp;
  cmp.loadFrom(obs_ptrs_, x_bound_, y_bound_);
  cmp.runDijkstra(x_bound_, y_bound_);

  std::priority_queue<std::shared_ptr<SearchNode>,
                      std::vector<std::shared_ptr<SearchNode>>, Compare>
      pq; // priority queue
  pq.push(start_node_);

  std::unordered_map<std::string, std::shared_ptr<SearchNode>> close_set_;
  std::unordered_map<std::string, std::shared_ptr<SearchNode>> open_set_;
  open_set_.insert(std::make_pair(start_node_->GetIndex(), start_node_));

  // GUI display(800, 800);
  // // display.drawObs(map_);
  // display.drawCar(start);
  // display.drawCar(target);

  unsigned long iter = 0;
  while (pq.size() > 0) {
    if (iter % 2000 == 0) // about 20000 iter/s
    {
      // std::cout << "iter = " << iter << std::endl;
    }

    if (iter > HybridAstarConfig::GetInstance()->max_iter) {
      // std::cerr << "hybrid astar exceeds max iteration\n";
      result_.status = SbpStatus::TIMEOUT;
      break;
    }
    std::shared_ptr<SearchNode> current = pq.top();
    pq.pop();
    iter++;

    // for visualization
    if (verbose == VERBOSE_DEBUG) {
      if (display_points == BI_PRINT) {
        searchPoints_.emplace_back(current->x, current->y, current->theta);
      } else if (display_points == FORWARD_PRINT && current->vel > 0) {
        searchPoints_.emplace_back(current->x, current->y, current->theta);
      } else if (display_points == BACKWARD_PRINT && current->vel < 0) {
        searchPoints_.emplace_back(current->x, current->y, current->theta);
      }
    }
    if (current->previous && AnalyticExpansion(current, obs_ptrs)) {
      std::shared_ptr<SearchNode> sbp_nodes_back =
          sbp_rspath_.getNodes().back();
      combineTrajectory(&result_, sbp_nodes_back);
      // std::cout << "Hybridastar reached with cost = " << current->trajcost
      //           << ", heuristic cost = " << current->heuristic_cost_
      //           << std::endl;
      result_.status = SbpStatus::SUCCESS;
      break;
    }

    if (close_set_.find(current->GetIndex()) != close_set_.end()) {
      continue;
    }

    close_set_.insert(std::make_pair(current->GetIndex(), current));
    parking::SearchDebugNode debug_curr(current->x, current->y, current->theta,
                                        current->trajcost,
                                        current->heuristic_cost_);
    if (sp_debug != nullptr) {
      sp_debug->searched_node.push_back(debug_curr);
    }
    std::vector<parking::SearchDebugNode> debug_nodes_vec;
    std::vector<parking::SearchDebugEdge> debug_edges_vec;

    std::vector<std::shared_ptr<SearchNode>> next = getNextStates(current);
    Vec2d current_point(current->x, current->y);

    // auto grid_ptr =
    //     std::dynamic_pointer_cast<GridObsManager>(obs_ptrs_.front());
    // if (grid_ptr) {
    //   grid_ptr->rearrangeObstacleToCheck(current_point);
    // }

    for (size_t i = 0; i < next.size(); i++) {
      if (!checkCollision(next[i])) {
        if (open_set_.find(next[i]->GetIndex()) == open_set_.end()) {
          // next[i]->side_diff = CalcSideDiff(next[i]);
          next[i]->setTrajCost();
          next[i]->obs_cost =
              current->obs_cost + getObstacleCost(next[i], footprint_model_);

          if (!reed_shepp_generator_->ShortestRSP(next[i], end_node_,
                                                  reeds_shepp_path_)) {
            // std::cout << "ShortestRSP failed" << std::endl;
            continue;
          }

          sbp_rspath_.update(next[i], reeds_shepp_path_);
          next[i]->heuristic_cost_ =
              sbp_rspath_.getCost(obs_ptrs_, footprint_model_);
          // if(end_node_->vel * (reeds_shepp_path_->gear.back()? 1:-1) < 0) {
          //   next[i]->heuristic_cost_ += 5;
          // }

          if (sp_debug != nullptr) {
            parking::SearchDebugNode debug_next(
                next[i]->x, next[i]->y, next[i]->theta, next[i]->trajcost,
                next[i]->heuristic_cost_);
            debug_nodes_vec.push_back(debug_next);
            debug_edges_vec.emplace_back(debug_curr, debug_next);
          }

          pq.push(next[i]);
          open_set_.insert(std::make_pair(next[i]->GetIndex(), next[i]));
        }
      }
    }
    if (sp_debug != nullptr) {
      sp_debug->added_nodes.push_back(debug_nodes_vec);
      sp_debug->added_edges.push_back(debug_edges_vec);
    }
  }

  if (pq.empty()) {
    result_.status = SbpStatus::INFEASIBLE;
  }

  result_.iteration_times = iter;
  result_.debug_string = "iteration_times = " + std::to_string(iter);
  // double chrono_ms = 0;
  // result_.debug_string += ", time_per_iter = " +
  //                         std::to_string(chrono_ms /
  //                         result_.iteration_times);

  return result_.status == SbpStatus::SUCCESS;
}

SbpResult HybridAstar::getResult() { return result_; }

std::vector<Pose2D> HybridAstar::getSearchPoints() { return searchPoints_; }

double HybridAstar::calCircularRadius(std::shared_ptr<SearchNode> node_a,
                                      std::shared_ptr<SearchNode> node_b) {
  double turn_radius = 1e6;
  double diff_theta = NormalizeAngle(node_b->theta - node_a->theta);
  // double average_theta = std::atan2(std::sin(node_b->theta) +
  // std::sin(node_a->theta), std::cos(node_b->theta) +
  // std::cos(node_a->theta)); Vec2d diff_vec(node_b->x - node_a->x, node_b->y -
  // node_a->y);
  double denominator = std::cos(node_a->theta) * std::sin(node_b->theta) -
                       std::sin(node_a->theta) * std::cos(node_b->theta);
  // if(std::abs(std::cos(diff_vec.Angle() - average_theta)) < std::cos(1e-2))
  // {
  //   return false;
  // }

  if (std::abs(denominator) < 1e-6) {
    return turn_radius;
  }
  double length_a = ((node_b->x - node_a->x) * std::sin(node_b->theta) -
                     (node_b->y - node_a->y) * std::cos(node_b->theta)) /
                    denominator;
  double length_b = ((node_b->x - node_a->x) * std::sin(node_a->theta) -
                     (node_b->y - node_a->y) * std::cos(node_a->theta)) /
                    denominator;

  // if(std::abs(diff_theta) > M_PI_2)
  // {
  //   return false;
  // }

  if (length_a * length_b > 0) {
    return turn_radius;
  }
  // double sin_diff_2 = std::sin(diff_theta / 2);
  // double turn_radius = std::abs(sin_diff_2) < 1e-6 ? 1e6 :
  // std::hypot(node_b->x - node_a->x, node_b->y - node_a->y) / 2 / sin_diff_2;
  turn_radius = std::min(std::abs(length_a), std::abs(length_b)) *
                std::abs(std::tan(diff_theta / 2));
  return std::abs(turn_radius);
}

double HybridAstar::CalcSideDiff(const std::shared_ptr<SearchNode> &node) {

  // get ego vehicle box
  double center_to_geometry_center_ =
      VehicleParam::Instance()->center_to_geometry_center;
  double geometry_center_x =
      node->x + center_to_geometry_center_ * cos(node->theta);
  double geometry_center_y =
      node->y + center_to_geometry_center_ * sin(node->theta);
  Box2d ego_box(Vec2d(geometry_center_x, geometry_center_y), node->theta,
                VehicleParam::Instance()->length,
                VehicleParam::Instance()->width);

  double back_center_x = node->x - back_edge_to_rear_ * cos(node->theta);
  double back_center_y = node->y - back_edge_to_rear_ * sin(node->theta);
  double front_center_x = node->x + front_edge_to_rear_ * cos(node->theta);
  double front_center_y = node->y + front_edge_to_rear_ * sin(node->theta);
  LineSegment2d ego_centerline(Vec2d(back_center_x, back_center_y),
                               Vec2d(front_center_x, front_center_y));
  Vec2d back_to_front(front_center_x - back_center_x,
                      front_center_y - back_center_y);
  Vec2d front_to_back(back_center_x - front_center_x,
                      back_center_y - front_center_y);

  bool is_driving_forward = (node->vel >= 0);
  std::vector<Vec2d> nearest_pts;
  for (const SbpObstaclePtr &obs_ptr : obs_ptrs_) {
    std::vector<Vec2d> nearest_pts_tmp =
        obs_ptr->getNearestPoints(ego_centerline);
    for (auto point : nearest_pts_tmp) {
      if (point.Id() == -1 || ego_box.IsPointIn(point)) {
        continue;
      }

      // ignore points which are passed by (counter to the driving orientation)
      if (is_driving_forward) {
        Vec2d Pn_Pb(back_center_x - point.x(), back_center_y - point.y());
        if (Pn_Pb.InnerProd(back_to_front) > 0) {
          continue;
        }
      } else {
        Vec2d Pn_Pf(front_center_x - point.x(), front_center_x - point.y());
        if (Pn_Pf.InnerProd(front_to_back) > 0) {
          continue;
        }
      }
      nearest_pts.push_back(point);
    }
  }

  double min_left_dist = std::numeric_limits<double>::infinity();
  double min_right_dist = std::numeric_limits<double>::infinity();
  double dist_temp = std::numeric_limits<double>::infinity();
  for (auto point : nearest_pts) {
    Vec2d Pb_Pn(point.x() - back_center_x, point.y() - back_center_y);
    if (fabs(back_to_front.Angle() - Pb_Pn.Angle()) < 0.01) {
      continue; // skip points which are forward along driving orientation
    }
    dist_temp = ego_centerline.DistanceTo(point);
    if (back_to_front.CrossProd(Pb_Pn) >= 0) { // on the left side or colinear
      min_left_dist = std::min(min_left_dist, dist_temp);
    } else { // on the right side
      min_right_dist = std::min(min_right_dist, dist_temp);
    }
  }

  if (min_left_dist < safe_diff_distance_ &&
      min_right_dist < safe_diff_distance_) {
    if (min_left_dist < safe_diff_distance_) {
      min_left_dist =
          std::max(min_left_dist, CarParams::GetInstance()->lat_inflation());
    }
    if (min_right_dist < safe_diff_distance_) {
      min_right_dist =
          std::max(min_right_dist, CarParams::GetInstance()->lat_inflation());
    }
    return std::pow(min_left_dist - min_right_dist, 2);
  }
  return 0.0;
}

double HybridAstar::getObstacleCost(const SearchNodePtr &node,
                                    const FootprintModelPtr &footprint_model) {
  double cost = 0;
  for (const SbpObstaclePtr &obs_ptr : obs_ptrs_) {
    cost += obs_ptr->getCost(node, footprint_model);
  }

  return cost;
}

} // namespace msquare
