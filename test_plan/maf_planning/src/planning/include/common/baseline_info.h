#ifndef MSQUARE_DECISION_PLANNING_COMMON_BASELINE_INFO_H_
#define MSQUARE_DECISION_PLANNING_COMMON_BASELINE_INFO_H_

#include <limits>
#include <memory>
#include <string>

#include "common/obstacle_manager.h"
#include "common/path/path_data.h"

namespace msquare {

class WorldModel;

class BaseLineInfo {
public:
  // BaseLineInfo() = default;
  BaseLineInfo();

  int lane_id() { return lane_id_; }

  void update(const std::shared_ptr<WorldModel> &world_model, int lane_id);

  void clear() {
    lane_id_ = 0;
    is_valid_ = false;
    is_update_ = false;
    // FrenetCoordinateSystemParameters frenet_parameters_;
    // std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;
    reference_line_.Clear();
    raw_refline_points_.clear();
    // EgoStateManager ego_state_manager_;
    adc_sl_boundary_.corners.clear();
    obstacle_manager_->clear();
    hdmap_info_.clear();
    is_adc_on_reference_line_ = false;
    is_target_lane_for_map_ = true;
    is_change_lane_ = false;
    is_neighbor_lane_ = false;
    cruise_speed_ = 0.0;
    vehicle_signal_ = GREEN;
    right_of_way_status_ = PROTECTED;
    convex_hull_.clear();
    ignorable_obstacles_.clear();
    lane_type_ = 0;
    construct_failed_reason_ = "";

    car_refline_points_.clear();
    leftborder_points_.clear();
    rightborder_points_.clear();

    lidar_rb_id_vec_.clear();
  }

  const ObstacleManager &obstacle_manager() const { return *obstacle_manager_; }

  ObstacleManager &mutable_obstacle_manager() const {
    return *obstacle_manager_;
  }

  const EgoState &get_ego_state() const {
    return ego_state_manager_.get_ego_state();
  }

  const EgoStateManager &get_ego_state_manager() const {
    return ego_state_manager_;
  }

  const PathData &get_path_data() const { return reference_line_; }

  const std::shared_ptr<FrenetCoordinateSystem> &get_frenet_coord() {
    return frenet_coord_;
  }

  const std::vector<double> &get_frenet_enu_x() { return frenet_enu_x_; }

  const std::vector<double> &get_frenet_enu_y() { return frenet_enu_y_; }

  const std::vector<ReferenceLinePointDerived> &get_raw_refline_points() {
    return raw_refline_points_;
  }

  // todo: only set frenet related info here
  const FrenetState &get_planning_start_state() const {
    return ego_state_manager_.get_ego_state().planning_start_state;
  }

  const TrajectoryPoint &get_planning_start_point() const {
    return ego_state_manager_.get_ego_state().planning_init_point;
  }

  const State &get_mpc_vehicle_state() const {
    return ego_state_manager_.get_ego_state().mpc_vehicle_state;
  }

  const SLBoundary &get_adc_sl_boundary() const { return adc_sl_boundary_; }

  int lane_id() const { return lane_id_; }

  const planning_math::Polygon2d &get_convex_hull() const {
    return convex_hull_;
  }

  bool is_valid() { return is_valid_; }

  std::vector<int> get_lidar_rb_id_vec() const { return lidar_rb_id_vec_; }

  bool is_update() { return is_update_; }

  std::string invalid_reason() { return construct_failed_reason_; }

  bool is_obstacle_on_lane(const Obstacle *obstacle);

  bool is_adc_on_lane();

  bool is_obstacle_intersect_with_lane(const Obstacle *obstacle);

  typedef struct {
    std::vector<ReferenceLinePointDerived> refline_points;
    int hdmap_lane_id;
  } HdmapLaneInfo;

  typedef enum {
    UNPROTECTED,
    PROTECTED,
  } RightOfWay;

  typedef enum {
    GREEN,
    GREENBLINKING,
    YELLOW,
    RED,
    YELLOWBLINKING,
    UNKNOWN,
  } TrafficLight;

  // different types of overlaps that can be handled by different scenarios.
  typedef enum {
    CLEAR_AREA = 1,
    CROSSWALK = 2,
    OBSTACLE = 3,
    PNC_JUNCTION = 4,
    SIGNAL = 5,
    STOP_SIGN = 6,
    YIELD_SIGN = 7,
  } OverlapType;

  RightOfWay get_right_of_way_status() const { return right_of_way_status_; };

  void set_right_of_way_status(RightOfWay right_of_way) {
    right_of_way_status_ = right_of_way;
  }

  uint8_t lane_type() { return lane_type_; }

  /**
   * Check if the current reference line is a change lane reference line, i.e.,
   * ADC's current position is not on this reference line.
   */
  bool is_change_lane() const { return is_change_lane_; }

  /**
   * Check if the current reference line is the neighbor of the vehicle
   * current position
   */
  bool is_neighbor_lane() const { return is_neighbor_lane_; }

  /**
   * Check if the current reference line is the target lane for map task
   */
  bool is_target_lane() const { return is_target_lane_for_map_; }
  bool set_is_target_lane(bool is_target_lane) {
    is_target_lane_for_map_ = is_target_lane;

    return true;
  }

  void set_is_on_reference_line() { is_adc_on_reference_line_ = true; }

  std::vector<int> get_ignorable_obstacles() { return ignorable_obstacles_; }

  std::vector<Eigen::Vector3d> get_acc_refline() { return car_refline_points_; }
  std::vector<Eigen::Vector3d> get_acc_leftborder() {
    return leftborder_points_;
  }
  std::vector<Eigen::Vector3d> get_acc_rightborder() {
    return rightborder_points_;
  }

  double calc_line_y(std::vector<double> &coef, double x) {
    double y = 0;

    for (int i = (int)coef.size() - 1; i >= 0; --i) {
      y = y * x + coef[i];
    }
    return y;
  }

  double calc_line_dev(const std::vector<double> &coef, double x, int order) {
    double dev = 0;
    if (order == 1) {
      dev = coef[1] + 2 * coef[2] * x + 3 * coef[3] * std::pow(x, 2);
    } else if (order == 2) {
      dev = 2 * coef[2] + 6 * coef[3] * x;
    } else if (order == 3) {
      dev = 6 * coef[3];
    } else {
      dev = 0.0;
    }
    return dev;
  }

private:
  bool construct_reference_line(const std::shared_ptr<WorldModel> &world_model);
  bool construct_reference_line_cjacc(
      const std::shared_ptr<WorldModel> &world_model);

  bool
  construct_obstacle_manager(const std::shared_ptr<WorldModel> &world_model);

  void compute_convex_hull(double left_width, double right_width);

  ReferenceLinePointDerived
  find_point_by_x(std::vector<ReferenceLinePointDerived> &ref_line, double s);
  int find_num_by_x(std::vector<ReferenceLinePointDerived> &ref_line, double x);
  // bool update_ego_frenet_start_state();

  void generate_road_boundary_obstacles(
      const std::shared_ptr<WorldModel> &world_model);

  void road_boundary_interpolation(
      const maf_perception_interface::RoadEdge &road_edge,
      std::vector<Point2D> &road_boundary);

  bool is_edge_in_boundary(const maf_perception_interface::RoadEdge &road_edge);

private:
  int lane_id_;
  bool is_valid_{false};
  bool is_update_{false};
  FrenetCoordinateSystemParameters frenet_parameters_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_ = nullptr;
  std::shared_ptr<Obstacle> obs_ptr = nullptr;
  PathData reference_line_;
  std::vector<ReferenceLinePointDerived> raw_refline_points_;
  EgoStateManager ego_state_manager_;
  SLBoundary adc_sl_boundary_;
  std::unique_ptr<ObstacleManager> obstacle_manager_;
  std::vector<HdmapLaneInfo> hdmap_info_;
  bool is_adc_on_reference_line_ = false;
  bool is_target_lane_for_map_ = true;
  bool is_change_lane_ = false;
  bool is_neighbor_lane_ = false;
  double cruise_speed_ = 0.0;
  TrafficLight vehicle_signal_;
  RightOfWay right_of_way_status_ = PROTECTED;
  planning_math::Polygon2d convex_hull_;
  std::vector<int> ignorable_obstacles_;
  uint8_t lane_type_;
  std::string construct_failed_reason_;
  std::vector<double> frenet_enu_x_{}, frenet_enu_y_{};

  std::vector<Eigen::Vector3d> car_refline_points_;
  std::vector<Eigen::Vector3d> leftborder_points_;
  std::vector<Eigen::Vector3d> rightborder_points_;

  std::vector<int> lidar_rb_id_vec_{};

  DISALLOW_COPY_AND_ASSIGN(BaseLineInfo);
};

} // namespace msquare

#endif
