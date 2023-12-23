#pragma once

#include "common/config/obs_manager_config.h"
#include "common/parking_obstacle.h"
#include <limits>
#include <memory>
#include <string>

namespace msquare {

namespace parking {

class WorldModel;

class ObstacleManager {
public:
  // ObstacleManager();
  //~ObstacleManager() = default;

  bool init(const std::shared_ptr<WorldModel> &world_model);

  void clear();

  Obstacle *add_obstacle(const Obstacle &obstacle);
  const Obstacle *find_obstacle(int object_id) const;
  Obstacle *find_obstacle(int object_id);
  const IndexedList<int, Obstacle> &get_obstacles() const;

  // static obstacle
  Obstacle *add_static_obstacle(const Obstacle &obstacle);
  const Obstacle *find_static_obstacle(int object_id) const;
  Obstacle *find_static_obstacle(int object_id);
  const IndexedList<int, Obstacle> &get_static_obstacles() const;

  // groundline obstacle
  Obstacle *add_groundline_obstacle(const Obstacle &obstacle);
  const Obstacle *find_groundline_obstacle(int object_id) const;
  Obstacle *find_groundline_obstacle(int object_id);
  const IndexedList<int, Obstacle> &get_groundline_obstacles() const;

  // points
  Obstacle *add_point(const Obstacle &obstacle);
  const Obstacle *find_point(int object_id) const;
  Obstacle *find_point(int object_id);
  const IndexedList<int, Obstacle> &get_points() const;

  Obstacle *add_all_point(const Obstacle &obstacle);
  const Obstacle *find_all_point(int object_id) const;
  Obstacle *find_all_point(int object_id);
  const IndexedList<int, Obstacle> &get_all_points() const;

  // uss_points
  Obstacle *add_uss_point(const Obstacle &obstacle);
  const Obstacle *find_uss_point(int object_id) const;
  Obstacle *find_uss_point(int object_id);
  const IndexedList<int, Obstacle> &get_uss_points() const;

  // lines
  Obstacle *add_line(const Obstacle &obstacle);
  const Obstacle *find_line(int object_id) const;
  Obstacle *find_line(int object_id);
  const IndexedList<int, Obstacle> &get_lines() const;

  // freespace obstacle
  //   Obstacle *add_freespace_obstacle(const Obstacle &obstacle);
  const Obstacle *find_from_all_obstacle(int object_id) const;
  Obstacle *find_from_all_obstacle(int object_id);
  //   const IndexedList<int, Obstacle> &get_freespace_obstacles() const;
  const std::vector<const Obstacle *> &get_all_obstacles();

  // pillar
  Obstacle *add_pillar(const Obstacle &obstacle);
  const Obstacle *find_pillar(int object_id) const;
  Obstacle *find_pillar(int object_id);
  const IndexedList<int, Obstacle> &get_pillars() const;

  // gate
  Obstacle *add_gate(const Obstacle &obstacle);
  const Obstacle *find_gate(int object_id) const;
  Obstacle *find_gate(int object_id);
  const IndexedList<int, Obstacle> &get_gates() const;

  // road border
  Obstacle *add_road_border(const Obstacle &obstacle);
  const Obstacle *find_road_border(int object_id) const;
  Obstacle *find_road_border(int object_id);
  const IndexedList<int, Obstacle> &get_road_borders() const;

  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  bool add_lateral_decision(const std::string &tag, int id,
                            const ObjectDecisionType &decision);

  bool add_parking_lateral_decision(const std::string &tag, int id,
                                    const ObjectDecisionType &decision);

  bool add_longitudinal_decision(const std::string &tag, int id,
                                 const ObjectDecisionType &decision);

  bool add_parking_longitudinal_decision(const std::string &tag, int id,
                                         const ObjectDecisionType &decision);

  bool add_parking_static_obs_longitudinal_decision(
      const std::string &tag, int id, const ObjectDecisionType &decision);

  bool set_blocking(int id, bool blocking);
  bool set_beside_intersation(int id);
  bool set_road_status(int id);
  bool set_road_border_distance(int id, bool using_credible = true);

  bool
  set_virtual_box(int id,
                  const planning_math::Box2d &perception_bounding_box_virtual,
                  const SLBoundary &perception_sl_bounding_box_virtual);

  // void vehicle_fcp_svp_filter();
  //   void add_freespace_with_filter(
  //       const std::vector<std::shared_ptr<Obstacle>> freespace);
  // const std::unordered_map<int, const Obstacle*> &get_fcp_svp_relation()
  // const; const Obstacle *find_relative_obstacle(int object_id) const;
  // std::unordered_map<int, const Obstacle*> get_fcp_svp_relation();

  bool obj_speed_filter();
  bool obj_direction_filter();
  bool obj_intention_status_manager();
  bool obj_pseudo_prediction();
  bool obj_specific_manager();
  std::unordered_map<int, int>
  obj_static_fusion_filter(double IoU_threshold,
                           bool ralation_reverse = false) const;

  int obj_check_static(double v_thres, double obs_speed, int count,
                       const ObjectType obs_type,
                       bool for_lon_highspeed = false);
  bool is_stay_in_limited_region(double max_dist, int num_frame);
  double interp(double x, const std::vector<double> &xp,
                const std::vector<double> &fp);

  std::vector<const Obstacle *> get_static_obstacle();
  std::vector<const Obstacle *> get_dynamic_obstacle();

  std::vector<const Obstacle *> get_static_obstacle_slimit(const double s_min,
                                                           const double s_max);
  std::vector<const Obstacle *> get_dynamic_obstacle_slimit(const double s_min,
                                                            const double s_max);

private:
  std::shared_ptr<WorldModel> world_model_;
  IndexedList<int, Obstacle> obstacles_;
  IndexedList<int, Obstacle> static_obstacles_;
  IndexedList<int, Obstacle> groundline_obstacles_;
  //   IndexedList<int, Obstacle> freespace_obstacles_;
  IndexedList<int, Obstacle> pillar_obstacles_;
  IndexedList<int, Obstacle> gate_obstacles_;
  IndexedList<int, Obstacle> road_border_obstacles_;
  std::vector<const Obstacle *> all_obstacles_ptr_;
  std::unordered_map<int, bool> beside_frenet_flag;
  // std::shared_ptr<LateralObstacle> lateral_obstacle_;
  std::unordered_map<int, std::tuple<int, std::tuple<int, int, int, int>,
                                     std::tuple<int, int, int, int>>>
      obstacles_filter; //<id, <frenet_tuple, ego_tuple>>
  std::unordered_map<int,
                     std::pair<int, std::pair<std::tuple<int, int, int, int>,
                                              std::tuple<int, int, int, int>>>>
      obstacles_speed_filter; //<id, <totoal, <frenet_tuple, ego_tuple>>>
  std::unordered_map<int, std::pair<std::pair<int, int>, std::pair<int, int>>>
      obstacles_dir_count; //<id, <<ego same, ego oppo>,<frenet same, frenet
                           // oppo>>
  std::unordered_map<int, std::pair<vector<int>, vector<int>>>
      obstacles_dir_filter;

  IndexedList<int, Obstacle> points_;
  IndexedList<int, Obstacle> all_points_;
  IndexedList<int, Obstacle> uss_points_;
  IndexedList<int, Obstacle> lines_;
  std::unordered_map<int, std::pair<int, std::tuple<int, int, int>>>
      obstacles_apa_filter; //<id, <total_check_time, <count_to_side,
                            // count_cross_border, count_beside_road> > >
  std::unordered_map<int, std::pair<int, std::tuple<int, int, int>>>
      obstacles_apoa_filter; //<id, <total_check_time,<count_to_center,
                             // count_cross_border, count_in_road> > >
  std::unordered_map<int, std::pair<int, std::tuple<int, int, int>>>
      obstacles_pullover_filter; //<id, <total_check_time, <count_low_speed,
                                 // count_lat_static, count_in_road> > >
  std::unordered_map<int, std::tuple<int, int>> obstacles_cutin_filter;

  // std::unordered_map<int, const Obstacle*> fcp_svp_relation_;
};

} // namespace parking

} // namespace msquare
