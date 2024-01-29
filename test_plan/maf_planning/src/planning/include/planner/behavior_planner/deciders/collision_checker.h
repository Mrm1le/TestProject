#ifndef MSQUARE_DECISION_PLANNING_PLANNER_COLLISION_CHECKER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_COLLISION_CHECKER_H_

#include "common/config/vehicle_param.h"
#include "common/ego_model_manager.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/planning_context.h"
#include "planner/message_type.h"

namespace msquare {

namespace parking {

struct CollisionCheckStatus {
  bool is_collision = false;
  double s = 0; // distance to collision or distance to the traj point closest
                // to obstacle
  double min_distance = 0; // minimum distacne to obstacles (>=0)
  bool is_valid = false;
};
struct CollisionCarParams {
  double front_edge_to_center;
  double length;
  double width;
  double half_length;
  double back_comp_r;
  double back_comp_d;
  double comp_length_r;
  double comp_length_d;
  void update();
};

struct ObsPointWithId {
  double collision_threshold;
  double extra_thres;
  CollisionCheckStatus result;
  planning_math::Vec2d p;
  Pose2D ego_danger_location;

  ObsPointWithId(double _collision_threshold, double _extra_thres,
                 CollisionCheckStatus _result, planning_math::Vec2d _p,
                 double _z)
      : collision_threshold(_collision_threshold), extra_thres(_extra_thres),
        result(_result), p(_p), z_(_z) {}
  ObsPointWithId(double _collision_threshold, double _extra_thres,
                 CollisionCheckStatus _result, planning_math::Vec2d _p,
                 double _z, Pose2D _ego_danger_location)
      : collision_threshold(_collision_threshold), extra_thres(_extra_thres),
        result(_result), p(_p), z_(_z),
        ego_danger_location(_ego_danger_location) {}

public:
  const double z() const { return z_; };
  double *mutable_z() { return &z_; };

private:
  double z_ = 0.0;
};
using ObsPtsWithId = std::vector<ObsPointWithId>;
using CollisionResults = std::vector<CollisionCheckStatus>;
using PolygonsWithPose =
    std::vector<std::tuple<const planning_math::Polygon2d,
                           const planning_math::Polygon2d, Pose2D>>;

class CollisionChecker {

public:
  CollisionChecker();

  // deviation_length : the distance between trajectory point and center
  // (forward : - backward : +) cut_length : shorten the ego car length from
  // rear end reverse : if reverse, cut from the front end of ego car
  inline CollisionChecker &set_reverse(const bool reverse) {
    reverse_ = reverse;
    cut_scaler_ = reverse ? -1.0 : 1.0;
    ego_model_.set_reverse(reverse);
    return *this;
  }

  inline CollisionChecker &set_deviation_length(const double deviation_length) {
    deviation_length_ = deviation_length;
    ego_model_.set_deviation_length(deviation_length);
    return *this;
  }

  inline CollisionChecker &set_cut_length(const double cut_length) {
    cut_length_ = cut_length;
    ego_model_.set_cut_length(cut_length);
    return *this;
  }

  template <typename T>
  CollisionCheckStatus collision_check(const std::vector<PathPoint> &trajectory,
                                       const T &obs,
                                       const double collision_threshold) {
    check_type_ = CheckType::TRAJECTORY;
    CollisionCheckStatus result;
    result.min_distance = 100;
    double s = 0;
    PathPoint last_path_point;

    if (!trajectory.empty()) {
      last_path_point.x = trajectory[0].x;
      last_path_point.y = trajectory[0].y;
    }

    for (const auto &point : trajectory) {
      s += std::hypot(last_path_point.x - point.x, last_path_point.y - point.y);
      result.s = s;
      collision_check(point, obs, collision_threshold, result);
      if (result.is_collision) {
        return result;
      }

      last_path_point.x = point.x;
      last_path_point.y = point.y;
    }

    return result;
  }

  template <typename T>
  CollisionCheckStatus collision_check(const std::vector<Pose2D> &trajectory,
                                       const T &obs,
                                       const double collision_threshold) {
    check_type_ = CheckType::TRAJECTORY;
    CollisionCheckStatus result;
    result.min_distance = 100;
    double s = 0;
    PathPoint last_path_point;
    PathPoint path_point;

    if (!trajectory.empty()) {
      last_path_point.x = trajectory[0].x;
      last_path_point.y = trajectory[0].y;
    }

    for (const auto &point : trajectory) {
      s += std::hypot(last_path_point.x - point.x, last_path_point.y - point.y);
      result.s = s;
      path_point.x = point.x;
      path_point.y = point.y;
      path_point.theta = point.theta;
      collision_check(path_point, obs, collision_threshold, result);
      if (result.is_collision) {
        return result;
      }

      last_path_point.x = point.x;
      last_path_point.y = point.y;
    }

    return result;
  }

  template <typename T>
  CollisionCheckStatus collision_check(const PathPoint &trajectory,
                                       const T &obs,
                                       const double collision_threshold) {
    check_type_ = CheckType::POINT;
    CollisionCheckStatus result;
    result.min_distance = 100;
    collision_check(trajectory, obs, collision_threshold, result);
    return result;
  }

  template <typename T>
  CollisionCheckStatus collision_check(const Pose2D &trajectory, const T &obs,
                                       const double collision_threshold) {
    check_type_ = CheckType::POINT;
    CollisionCheckStatus result;
    result.min_distance = 100;

    PathPoint path_point;
    path_point.x = trajectory.x;
    path_point.y = trajectory.y;
    path_point.theta = trajectory.theta;

    collision_check(path_point, obs, collision_threshold, result);
    return result;
  }
  void remainDisCheck(
      ObsPtsWithId &pts, const Pose2DTrajectory &mpc_traj,
      const Pose2DTrajectory &plan_traj, bool is_reverse, bool is_moved,
      const Pose2D &ego_pose, FreespacePoint *const ptr_lead_point,
      std::string *const ptr_debug_str,
      std::vector<std::pair<double, double>> *const ptr_old_mpc_sl_points);

  EgoModelManager &get_ego_model() { return ego_model_; }

  // CollisionCarParams* const mutable_collision_params() {
  //   return &param_;
  // }
  CollisionCarParams *mutable_collision_params() { return &param_; }

private:
  enum CheckType {
    TRAJECTORY = 0,
    POINT,
  };

  bool set_params(const double deviation_length, const double cut_length,
                  const bool reverse);
  bool is_trajectory() const { return check_type_ == CheckType::TRAJECTORY; }
  bool is_point() const { return check_type_ == CheckType::POINT; }

  bool isCollision(ObsPtsWithId &pts, const PolygonsWithPose &polygons);

  void collision_check(const PathPoint &path_point,
                       const planning_math::Box2d &obs,
                       const double collision_threshold,
                       CollisionCheckStatus &result);
  void collision_check(const PathPoint &path_point,
                       const planning_math::Polygon2d &obs,
                       const double collision_threshold,
                       CollisionCheckStatus &result);
  void collision_check(const PathPoint &path_point,
                       const planning_math::LineSegment2d &obs,
                       const double collision_threshold,
                       CollisionCheckStatus &result);
  void collision_check(const PathPoint &path_point,
                       const planning_math::Vec2d &obs,
                       const double collision_threshold,
                       CollisionCheckStatus &result);

  bool init();
  double cut_length_;
  double deviation_length_;
  int cut_scaler_;
  CheckType check_type_;
  bool reverse_ = false;
  bool use_double_box_ = false;
  bool use_polygon_ = false;
  bool use_decagon_ = false;
  bool use_tetradecagon_ = false;
  bool use_rectangle_hexagon_ = false;
  bool use_hexadecagon_ = false;
  EgoModelManager ego_model_;
  CollisionCarParams param_;
  // planning_math::Polygon2d ego_model_;
};

} // namespace parking

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_LEADER_DECIDER_H_
