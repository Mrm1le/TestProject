#ifndef MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_PARKING_REFLINE_MANAGER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_BEHAVIOR_PLANNER_PARKING_REFLINE_MANAGER_H_
// #include "common/parking_world_model.h"
#include "common/config/lateral_planning_config.h"
#include "planner/message_type.h"

namespace msquare {

namespace parking {

class WorldModel;

struct RefLineAttributeLane {
  double s_lane_start;
  int lane_start_index;
  int id_lane;
  int count_lane_points;
  double length_lane;
  int lane_attribute;
  double avg_curvature = 0.0;
  bool new_info = true;
  double via_weight_base;

  //   RefLineAttributeLane(){
  //     s_lane_start = 0.0;
  //     lane_start_index = 0;
  //     id_lane = 0;
  //     count_lane_points  = 0;
  //     length_lane = 0.0;
  //     lane_attribute = 0;
  //     new_info = true;
  //     via_weight_base = 8.0;
  //   };
};

struct RefLineAttributePoint {
  int id_lane;
  double s;
  double l;
  double x;
  double y;
  double theta;
  double left_road_border_distance;
  double right_road_border_distance;
  double left_lane_border_distance;
  double right_lane_border_distance;
  double left_obstacle_distance;
  double right_obstacle_distance;
  double lane_width;
  double curvature;
  double curvature_theta;
  bool intersection;
  bool carriage_way;
  bool interpolation;
  bool via_point = false;
  bool no_virtual_box = false;
};

class RefLineManager {
  DECLARE_SINGLETON(RefLineManager)
public:
  // RefLineManager(const std::shared_ptr<WorldModel> &world_model);
  // ~RefLineManager();
  void init(const std::shared_ptr<WorldModel> &world_model);
  void update();
  void reset();
  const std::vector<RefLineAttributeLane> &get_refline_attribute_lane() const {
    return refline_attribute_lane_;
  }
  std::vector<RefLineAttributeLane> &get_mutable_refline_attribute_lane() {
    return refline_attribute_lane_;
  }
  const std::vector<RefLineAttributePoint> &
  get_refline_attribute_point() const {
    return refline_attribute_point_;
  }
  std::vector<RefLineAttributePoint> &get_mutable_refline_attribute_point() {
    return refline_attribute_point_;
  }

  double cal_dist2virtual_box(const double s);
  void update_via_weight_base(const double via_weight_base, const int id);
  double get_via_weight_base(const double s);
  int get_lane_id(const double s);
  bool is_intersection_lane(const int id);
  bool get_attribute_point_index(double s, int &idx);
  RefLineAttributePoint interp_attribute_point(double s);

  double cal_theta(const double s);
  double cal_left_road_border_distance(double s);
  double cal_left_lane_border_distance(double s);
  double cal_right_road_border_distance(double s);
  double cal_right_lane_border_distance(double s);
  double cal_lane_width(double s);
  double cal_curvature(double s);
  double cal_curvature_theta(double s);

  bool is_no_virtual_box(const double s);
  bool is_no_virtual_box(const double s, double &idx);
  bool is_via_point(const double s);
  bool is_intersection(const double s);
  bool is_carriage_way(const double s);
  std::vector<double> get_s() { return s_vector_; };
  std::vector<double> get_lane_width() { return lane_width_vector_; };
  std::vector<double> get_left_road_border_distance() {
    return left_road_border_distance_vector_;
  };
  std::vector<double> get_right_road_border_distance() {
    return right_road_border_distance_vector_;
  };
  std::vector<double> get_left_lane_border_distance() {
    return left_lane_border_distance_vector_;
  };
  std::vector<double> get_right_lane_border_distance() {
    return right_lane_border_distance_vector_;
  };
  std::vector<double> get_curvature() { return curvature_vector_; };

private:
  bool track_id_separate(const std::string &track_id,
                         const std::string &delimiter, int &id_lane,
                         int &in_point);
  bool check_turn(const int &index_traj, const int &index_lane);
  // bool refline_info2attribute_point(const RefLineInfoFrenet & refline_info,
  // std::vector<RefLineAttributePoint> & attribute_point);
  bool
  resize_attribute_point(std::vector<RefLineAttributePoint> &attribute_point);
  bool cal_refline_attribute_point(
      std::vector<RefLineAttributePoint> &attribute_point);
  double straight_lane_length_before(const int &index);
  double straight_lane_length_after(const int &index);
  bool transform();

private:
  std::shared_ptr<WorldModel> world_model_;
  std::vector<RefLineAttributeLane> refline_attribute_lane_;
  std::vector<RefLineAttributePoint> refline_attribute_point_;
  std::vector<RefLineAttributeLane> refline_attribute_lane_last_;
  std::vector<RefLineAttributePoint> refline_attribute_point_last_;
  ViaPointDeciderConfig *via_point_decider_cfg_;
  std::shared_ptr<FrenetCoordinateSystem> frenet_coord_;

  std::vector<double> s_vector_;
  std::vector<double> lane_width_vector_;
  std::vector<double> left_road_border_distance_vector_;
  std::vector<double> right_road_border_distance_vector_;
  std::vector<double> left_lane_border_distance_vector_;
  std::vector<double> right_lane_border_distance_vector_;
  std::vector<double> curvature_vector_;

  const double WEIGHT_VIA_BASE_INTERSECTION = 2.0;
  const double WEIGHT_VIA_BASE_LANE = 8.0;
};

} // namespace parking

} // namespace msquare

#endif