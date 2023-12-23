#ifndef _APF_DECIDER_H_
#define _APF_DECIDER_H_

#include "common/config/lateral_planning_config.h"
#include "common/math/box2d.h"
#include "common/obstacle_manager.h"
#include "common/parking_world_model.h"
#include "common/utils/geometry.h"
#include "common/world_model.h"
#include <fstream>

#include "common/planning_config.h"
// #include "teb_local_planner_interface.h"

namespace msquare {
namespace parking {

using planning_math::Box2d;
using planning_math::LineSegment2d;
using planning_math::Vec2d;

class SimpleParkingLot {

public:
  SimpleParkingLot(){};
  SimpleParkingLot(mjson::Json lot_json, double z_offset);
  SimpleParkingLot(ParkingLotDetectionInfo info);
  ~SimpleParkingLot();
  Vec2d center() { return center_; };
  std::vector<LineSegment2d> lines() { return lines_; };
  std::vector<Vec2d> corners() { return corners_; };
  std::vector<Point3D> points() { return points_; };
  Box2d Box() { return box_; };
  int id() { return id_; };
  double z() { return z_; };
  double theta() { return box_.heading(); };
  double average_corner_confidence() { return average_corner_confidence_; };
  double length() { return length_; };
  double width() { return width_; };
  bool is_vertical() { return is_vertical_; };
  SimpleParkingLot &operator=(const SimpleParkingLot &copy);

private:
  std::vector<LineSegment2d> lines_;
  std::vector<Vec2d> corners_;
  std::vector<Point3D> points_;
  Box2d box_;
  double z_;
  Vec2d center_;
  double theta_;
  mjson::Json lot_json_;
  int id_;
  double length_;
  double width_;
  double average_corner_confidence_ = 0.0;
  bool is_vertical_ = true;
}; // class SimpleParkingLot

enum ApfObstacleType { none = 0, wall = 1, pillar = 2 };

class ApfDecider {

  struct ApfPoint {
    Pose2D pose;
    Point2D gradient;
    double gradient_value;
    double gradient_theta;
    ApfPoint(Pose2D a, Point2D b) : pose(a), gradient(b) {
      gradient_theta = std::atan2(b.y, b.x);
      gradient_value = std::sqrt(b.x * b.x + b.y * b.y);
    }
    ApfPoint(){};
    // ApfPoint& operator=(ApfPoint& value)
    //   {
    //       pose.x = value.pose.x;
    //       pose.y = value.pose.y;
    //       pose.theta = value.pose.theta;
    //       gradient.x = value.gradient.x;
    //       gradient.y = value.gradient.y;
    //       gradient_value = value.gradient_value;
    //       gradient_theta = value.gradient_theta;
    //       return *this;
    //   }
  };

  Pose2D operator=(const Pose2D &b) {
    Pose2D a;
    a.x = b.x;
    a.y = b.y;
    a.theta = b.theta;
    return a;
  }

  Point2D operator=(const Point2D &b) {
    Point2D a;
    a.x = b.x;
    a.y = b.y;
    return a;
  }

  using ApfTrajectory = std::vector<ApfPoint>;

  struct ApfStatus {
    bool reasonable_;
    bool changed_;
    bool empty_;
    int traj_size_;
    double traj_s_;
    bool use_apf_refline_;
    bool historical_;
    bool in_parking_lot_;

    ApfStatus() {
      empty_ = true;
      changed_ = false;
      reasonable_ = false;
      traj_size_ = 0;
      traj_s_ = 0;
      use_apf_refline_ = false;
      historical_ = false;
      in_parking_lot_ = false;
    }

    void reset() {
      empty_ = true;
      changed_ = false;
      reasonable_ = false;
      traj_size_ = 0;
      traj_s_ = 0;
      use_apf_refline_ = false;
      historical_ = false;
      in_parking_lot_ = false;
    }
    ApfStatus &operator=(const ApfStatus &b) {
      if (this == &b) {
        return *this;
      }
      reasonable_ = b.reasonable_;
      changed_ = b.changed_;
      empty_ = b.empty_;
      traj_size_ = b.traj_size_;
      traj_s_ = b.traj_s_;
      use_apf_refline_ = b.use_apf_refline_;
      historical_ = b.historical_;
      in_parking_lot_ = b.in_parking_lot_;
      return *this;
    }
  };

  struct ApfTrajectoryInfo {
    std::string tag;
    std::vector<TrajectoryPoint> traj;
    Pose2D target_pose;
    ApfStatus status;
  };

public:
  ApfDecider(const std::shared_ptr<WorldModel> &world_model);
  ApfDecider(std::string config_file);
  ~ApfDecider();
  std::vector<TrajectoryPoint> get_result() { return traj_points_; }
  void set_traj_tag(std::string traj_tag) { traj_tag_ = traj_tag; }
  bool calculate();
  void reset_all();
  ApfTrajectoryInfo traj_info() { return traj_info_; };
  ApfStatus traj_status() { return traj_status_; };
  Pose2D traj_target_pose() { return traj_target_pose_; };

private:
  // plan
  void plan();
  void replan();

  // reset and clear
  void reset_status();
  void clear_info();
  void clear_traj();
  void clear_traj_old();

  // gather
  void gather_map_info();
  void gather_obstacle_info();
  void gather_ego_state();
  void load_parking_lot_file();

  // pre calc
  bool calc_road_direction();
  bool calc_road_direction_two();
  void calc_apf_start_pose();

  // result
  bool check_traj_status();
  void print_traj_result();
  void set_status_and_info(bool historical);
  void replace_old_traj();

  void set_traj_points_from_apf();

  // gradient
  void create_gradient(Point2D &all_gradient,
                       const Point2D &directional_gradient, const Pose2D &pose);

  void create_directional_gradient(Point2D &gradient, double theta,
                                   double intensity);

  void get_apf_start_pose(Pose2D &pose);
  bool create_new_pose(Pose2D &pose, const ApfPoint &apf_point);
  bool create_apf_trajectory(Pose2D pose, double gradient_direction,
                             int max_point_num, int min_point_num,
                             ApfTrajectory &trajectory);
  void calc_apf_trajectory();

  // tool
  double distance(Pose2D pose1, Pose2D pose2);
  double distance(PathPoint point1, Pose2D pose2);
  double distance(PathPoint point1, PathPoint point2);

private:
  std::string traj_tag_;
  std::shared_ptr<WorldModel> world_model_;
  EgoState ego_state_;
  Pose2D ego_pose_;
  Pose2D apf_start_pose_;
  Box2d ego_box_;
  Vec2d ego_center_;
  ApfDeciderConfig *apf_decider_cfg_;
  double ego_z_;

  // for road direction
  double road_theta_ = 0;
  bool is_road_theta_exist_ = false;

  // segements from map
  std::vector<LineSegment2d> apf_wall_vector_;
  std::vector<LineSegment2d> apf_pillar_vector_;
  mjson::Json::array parking_lot_jsons_;
  std::unordered_map<int, SimpleParkingLot> parking_lot_map_;
  Vec2d parking_lot_circle_center_;
  std::vector<LineSegment2d> apf_parking_lot_line_vector_;

  // segements from detection
  std::vector<Vec2d> ground_point_vector_;
  std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results_;

  // for turnning
  int is_turning_ = 0; // 0: not found turning 1: pre turnning 2: turnnig
  double turning_direction_ = 0;
  double forward_direction_ = 0;

  ApfTrajectory forward_traj_;
  bool forward_traj_status_;
  ApfTrajectory left_traj_;
  bool left_traj_status_;
  ApfTrajectory right_traj_;
  bool right_traj_status_;

  // for traj
  ApfTrajectory apf_trajectory_;
  std::vector<TrajectoryPoint> traj_points_;
  ApfTrajectory apf_trajectory_old_;
  std::vector<TrajectoryPoint> traj_points_old_;

  // for output
  ApfStatus traj_status_;
  ApfStatus last_traj_status_;
  ApfTrajectoryInfo traj_info_;
  Pose2D traj_target_pose_;

  // for traj status
  bool is_traj_reasonable_;
  bool is_traj_changed_;
  double traj_s_;
  bool need_replan_ = false;

  /***********for apf show**********/
private:
  int store_count_ = 0;
  int store_num_ = 0;
  void store_scenery();

public:
  void restore_scenery(std::string path);
  ApfDeciderConfig *config() { return apf_decider_cfg_; };
  // map
  std::vector<LineSegment2d> walls() { return apf_wall_vector_; };
  std::vector<LineSegment2d> pillars() { return apf_pillar_vector_; };
  std::vector<LineSegment2d> parking_lots() {
    return apf_parking_lot_line_vector_;
  };
  std::unordered_map<int, SimpleParkingLot> parking_lot_map() {
    return parking_lot_map_;
  };
  // detection
  std::vector<Vec2d> ground_points() { return ground_point_vector_; };
  std::vector<ParkingLotDetectionInfo> parking_lots_detection_fusion_results() {
    return parking_lots_detection_fusion_results_;
  };

  // traj
  ApfTrajectory apf_trajectory() { return apf_trajectory_; };
  std::vector<TrajectoryPoint> traj_points() { return traj_points_; };
  ApfTrajectory apf_trajectory_old() { return apf_trajectory_old_; };
  std::vector<TrajectoryPoint> traj_points_old() { return traj_points_old_; };

  ApfTrajectory forward_traj() { return forward_traj_; };
  ApfTrajectory left_traj() { return left_traj_; };
  ApfTrajectory right_traj() { return right_traj_; };
  bool forward_traj_status() { return forward_traj_status_; };

  // status
  int is_turning() { return is_turning_; };
  double cal_potential_field_point(double x, double y);

  /***********for park out **********/
private:
  std::vector<TrajectoryPoint> historical_traj_points_;

  void add_historical_traj_point();
  void store_historical_traj_points();
  void restore_historical_traj_points(std::string file_path);
  bool park_out_pose_from_traj(std::vector<TrajectoryPoint> traj_candidate,
                               std::vector<TrajectoryPoint> &traj_output,
                               Pose2D &pose_output);

  bool check_traj_far_from_obstacle(std::vector<TrajectoryPoint> traj);
  bool check_traj_far_from_obstacle(Vec2d start, Vec2d end);
  // void calc_park_out_state(std::string dir = "left", double offset = 3.0);
  // bool plan_park_out_from_historical();
  bool plan_park_out_from_current(Pose2D start_pose);

  bool in_parking_lot_ = false;

public:
  // bool calculate_park_out(std::string dir = "left", double offset = 3.0);
  bool calculate_park_out(Pose2D start_pose);

}; // class ApfDecider

void mjson_demo();
void mjson_demo_2();
// void nlohmann_demo();
// void nlohmann_demo_2();

} // namespace parking
} // namespace msquare

#endif