#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// #include "momenta_msgs/FusionObjectArray.h"
#include "pnc/define/parking_map_info.h"
#include "pnc/define/parking_vision_info.h"

#include "common/config/vehicle_param.h"
// #include "pnc/define/prediction_object.h"
#include "common/math/box2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/object_decision_type.h"
// #include "common/path/discretized_path.h"
// #include "common/speed/st_boundary.h"
#include "common/utils/index_list.h"
#include "planner/message_type.h"
// #include "planner/trajectory.h"
namespace msquare {

namespace parking {
/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision safety priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
struct PseudoPredictionTrajectory {
  uint64_t id;
  int coordinate = -1; // 1 for frenet, 2 for Cartesian
  ObjectType type;
  double TTC = -100.0;
  double meet_s = -100.0;
  bool enable = false;
  int direction = 0; // 0: default, 1: same direction, -1:opposite direction
  std::vector<Point2D> relative_sl;
  std::vector<double> velocity;
  std::vector<Pose2D> pose;
  std::vector<double> time_series;
};

class Obstacle {
public:
  Obstacle() = default;

  // explicit Obstacle(int id, const FusionObject &Fusion_obstacle,
  //                   bool is_static);

  // explicit Obstacle(int id, const Point3D &point,
  //                  const double v_ego, const Pose2D &ego_pose);

  explicit Obstacle(int id, const Point3D &point, const ObjectType &type);
  explicit Obstacle(int id, const Point3D &point, bool is_freespace = false,
                    const GroundLineType fusion_type =
                        GroundLineType::GROUND_LINE_TYPE_UNKNOWN);

  // explicit Obstacle(int id, const Point3D &point1, const Point3D &point2,
  //                  const double v_ego, const Pose2D &ego_pose);

  // explicit Obstacle(int id, const planning_math::LineSegment2d &line,
  //                  const double v_ego, const Pose2D &ego_pose);

  explicit Obstacle(int id, const planning_math::LineSegment2d &line,
                    const GroundLineType fusion_type =
                        GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN);

  // for parking car
  explicit Obstacle(int id, const FusionObject &object, bool is_static);

  explicit Obstacle(int id, const FusionObject &object, bool is_static,
                    const Pose2D &ego_pose);

  // explicit Obstacle(int id, const VisionCar &vision_car_obstacle,
  //                 bool is_static, const double v_ego, const Pose2D &ego_pose,
  //                 bool use_v_ego);

  // explicit Obstacle(int id, const VisionHuman &vision_human_obstacle,
  //                   bool is_static, const double v_ego, const Pose2D
  //                   &ego_pose, bool use_v_ego);

  // for ground line
  explicit Obstacle(int id, const std::vector<planning_math::Vec2d> &points);

  // for parking map obstacle
  explicit Obstacle(int id, const ObstacleItem &map_polygon_obstacle);

  explicit Obstacle(
      int id, const Point3D &point1,
      const Point3D &point2); // for parking map obstacle in enu coordinate

  void update(const Obstacle &obstacle);

  Point3D point() const { return point_; };
  Point3D *mutable_point() { return &point_; };

  int Id() const { return id_; }
  void SetId(int id) { id_ = id; }

  double speed() const { return speed_; }
  double acceleration() const { return acc_; }

  int32_t PerceptionId() const { return perception_id_; }
  bool IsInBend() const { return is_in_bend_; }
  bool IsAcrossRoadBorder() const { return is_across_road_border_; }
  bool IsInRoad() const { return is_in_road_; }
  bool IsBesideRoad() const { return is_beside_road_; }
  bool IsApproachingGate() const { return is_approaching_gate_; }
  bool IsBesideIntersection() const { return is_beside_intersation_; }
  bool isInMiddleRoad() const { return is_in_middle_road_; }
  bool isInRoadLoose() const { return is_in_road_loose_; }
  // road_side 1 right 2 left 3 unknown
  int RoadSide() const { return road_side_; }
  // road_type 1 single 2 carriage way 3 mix
  int RoadType() const { return road_type_; }

  int IsStatic() const { return is_static_; }
  int IsSFStatic() const { return is_sf_static_; }
  int IsLonStaticWrtFrenet() const { return is_lon_static_wrt_frenet_; }
  int IsLatStaticWrtFrenet() const { return is_lat_static_wrt_frenet_; }
  int IsLonHighspeedWrtFrenet() const { return is_lon_highspeed_wrt_frenet_; }
  int IsLonOppositeWrtFrenet() const { return is_lon_opposite_wrt_frenet_; }
  int IsLonStaticWrtEgo() const { return is_lon_static_wrt_ego_; }
  int IsLatStaticWrtEgo() const { return is_lat_static_wrt_ego_; }
  int IsLonHighspeedWrtEgo() const { return is_lon_highspeed_wrt_ego_; }
  int IsLonOppositeWrtEgo() const { return is_lon_opposite_wrt_ego_; }
  int IsTowardsRoadCenter() const { return is_towards_road_center_; }
  int IsTowardsRoadSide() const { return is_towards_road_side_; }
  bool IsApa() const { return is_apa_; }
  bool IsApoa() const { return is_apoa_; }
  bool IsPullover() const { return is_pullover_; }
  bool IsHighspeedDecel() const { return is_highspeed_decel_; }
  double DistAcrossRoadBoader() const { return dist_across_road_boader_; }
  int IsAlongFrenet() const { return is_along_frenet_; }
  double FilletCuttingLength() const { return fillet_cutting_length_; }
  bool IsNeedFilletCutting() const { return is_need_fillet_cutting_; }
  bool IsVirtual() const { return is_virtual_; }
  bool IsFreemove() const { return is_freemove_; }
  double CutinScore() const { return cutin_score_; }
  bool IsSigmaValid() const { return is_traj_sigma_valid_; }
  std::string Intention() const { return intention_; }
  // std::string PredictionSource() const {return prediction_source_; }
  void SetTruncatedTime(double t) { truncated_time_ = t; }
  double truncatedTime() const { return truncated_time_; }
  void SetTruncated(bool tc) { is_truncated_ = tc; }
  bool IsTruncated() const { return is_truncated_; }

  void SetLonStaticWrtFrenet(int blonstatic) {
    is_lon_static_wrt_frenet_ = blonstatic;
  }
  void SetLatStaticWrtFrenet(int blatstatic) {
    is_lat_static_wrt_frenet_ = blatstatic;
  }
  void SetLonHighspeedWrtFrenet(int blonhighspeed) {
    is_lon_highspeed_wrt_frenet_ = blonhighspeed;
  }
  void SetLonOppositeWrtFrenet(int blonopposite) {
    is_lon_opposite_wrt_frenet_ = blonopposite;
  }

  void SetLonStaticWrtEgo(int blonstatic) {
    is_lon_static_wrt_ego_ = blonstatic;
  }
  void SetLatStaticWrtEgo(int blatstatic) {
    is_lat_static_wrt_ego_ = blatstatic;
  }
  void SetLonHighspeedWrtEgo(int blonhighspeed) {
    is_lon_highspeed_wrt_ego_ = blonhighspeed;
  }
  void SetLonOppositeWrtEgo(int blonopposite) {
    is_lon_opposite_wrt_ego_ = blonopposite;
  }

  void SetStatic(bool bstatic) { is_static_ = bstatic; }
  void SetSFStatic(bool bstatic) { is_sf_static_ = bstatic; }
  void SetAcrossRoadBorder(bool bacrossroadborder) {
    is_across_road_border_ = bacrossroadborder;
  }
  void SetInRoad(bool bisinroad) { is_in_road_ = bisinroad; }
  void SetBesideRoad(bool bisbesideroad) { is_beside_road_ = bisbesideroad; }
  void SetInMiddleRoad(bool binmiddleroad) {
    is_in_middle_road_ = binmiddleroad;
  }
  // road_side 1 right 2 left 3 unknown
  void SetInRoadSide(int roadside) { road_side_ = roadside; }
  // road_type 1 single 2 carriage way 3 mix
  void SetInRoadType(int roadtype) { road_type_ = roadtype; }
  void SetInRoadLoose(int is_in_road_loose) {
    is_in_road_loose_ = is_in_road_loose;
  }
  void SetTowardRoadCenter(bool btowardroadcenter) {
    is_towards_road_center_ = btowardroadcenter;
  }
  void SetTowardRoadSide(bool btowardroadside) {
    is_towards_road_side_ = btowardroadside;
  }
  void SetIsApa(bool bisapa) { is_apa_ = bisapa; }
  void SetIsApoa(bool bisapoa) { is_apoa_ = bisapoa; }
  void SetIsPullover(bool bispullover) { is_pullover_ = bispullover; }
  void SetIsHighspeedDecel(bool bishighspeeddecel) {
    is_highspeed_decel_ = bishighspeeddecel;
  }

  void SetApproachingGate(bool bapproachinggate) {
    is_approaching_gate_ = bapproachinggate;
  }
  void SetBesideIntersection(bool bbesideintersation) {
    is_beside_intersation_ = bbesideintersation;
  };
  void SetIsAlongFrenet(bool bisalongfrenet) {
    is_along_frenet_ = bisalongfrenet;
  };
  void SetFilletCuttingLength(double fillet_cutting_length) {
    fillet_cutting_length_ = fillet_cutting_length;
  }
  void SetIsNeedFilletCutting(bool bisneedfilletcutting) {
    is_need_fillet_cutting_ = bisneedfilletcutting;
  }
  // void SetRelativeObstacleId(int relative_obstacle_id){ relative_obstacle_id_
  // = relative_obstacle_id;} int GetRelativeObstacleId() const { return
  // relative_obstacle_id_;}

  void SetDistAcrossRoadBoader(double ddistacrossroadboader) {
    dist_across_road_boader_ = ddistacrossroadboader;
  };
  Pose2D GetCartePosewrtEgo() const { return carte_pose_wrt_ego_; };
  Point2D GetCarteVelwrtEgo() const { return carte_vel_wrt_ego_; };

  int GetPoseDirection() const { return pose_direction_; }
  void SetPoseDirection(int iposedirection) {
    pose_direction_ = iposedirection;
  };
  int GetGear() const { return gear_; };
  void SetGear(int igear) { gear_ = igear; };
  PseudoPredictionTrajectory GetPseudoPredictionTraj() const {
    return pseudo_predicition_traj_;
  }
  void
  SetPseudoPredictionTraj(PseudoPredictionTrajectory pseudo_predicition_traj) {
    pseudo_predicition_traj_ = pseudo_predicition_traj;
  }

  TrajectoryPoint GetPointAtTime(const double time) const;

  planning_math::Box2d GetBoundingBox(const TrajectoryPoint &point) const;

  SLBoundary GetSLBoundary(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                           const TrajectoryPoint &point) const;

  const planning_math::Box2d &PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }

  const planning_math::Box2d &PerceptionBoundingBoxVirtual() const {
    return perception_bounding_box_virtual_;
  }

  const planning_math::Polygon2d &PerceptionPolygon() const {
    return perception_polygon_;
  }

  const planning_math::LineSegment2d &PerceptionLine() const {
    return perception_line_;
  }

  const std::vector<planning_math::Vec2d> &PerceptionPoints() const {
    return perception_points_;
  }

  const FusionObject &PerceptionObject() const { return perception_obstacle_; }

  const planning_math::Box2d &PerceptionBoundingBoxExtend() const {
    return perception_bounding_box_extend_;
  }

  double PerceptionSpeed() const { return perception_speed_; }

  ObjectType Type() const { return type_; }

  FusionObjectType SFType() const { return sf_type_; }

  GroundLineType LineFusionType() const { return line_fusion_type_; }

  ObjectType *const mutable_Type() { return &type_; }

  double Prob() const { return prob_; }

  const std::vector<TrajectoryPoint> &Trajectory() const { return trajectory_; }

  bool HasTrajectory() const { return !trajectory_.empty(); }

  inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType &LateralDecision() const;

  const ObjectDecisionType &ParkingLateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType &LongitudinalDecision() const;

  SLBoundary PerceptionSLBoundary() const;
  SLBoundary PerceptionSLBoundaryOrigin() const;
  SLBoundary PerceptionSLBoundaryPlanning() const;
  SLBoundary PerceptionSLBoundaryExtend() const;
  SLBoundary PerceptionSLBoundaryVirtual() const;

  const std::vector<std::string> &decider_tags() const;

  const std::vector<ObjectDecisionType> &decisions() const;

  void AddLongitudinalDecision(const std::string &decider_tag,
                               const ObjectDecisionType &decision);

  void AddParkingLongitudinalDecision(const std::string &decider_tag,
                                      const ObjectDecisionType &decision);

  void AddLateralDecision(const std::string &decider_tag,
                          const ObjectDecisionType &decision);

  void AddParkingLateralDecision(const std::string &decider_tag,
                                 const ObjectDecisionType &decision);

  void ClearLongitudinalDecision();

  void ClearLateralDecision();
  void ClearParkingLateralDecision();

  bool HasLateralDecision() const;

  bool HasParkingLateralDecision() const;

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void SetPerceptionSlBoundary(const SLBoundary &sl_boundary);
  void SetPerceptionSlBoundaryOrigin(const SLBoundary &sl_boundary_origin);
  void SetPerceptionSlBoundaryPlanning(const SLBoundary &sl_boundary_planning);
  void SetPerceptionSlBoundaryVirtual(const SLBoundary &sl_boundary_virtual);

  void SetPerceptionBoundingBoxVirtual(
      const planning_math::Box2d &perception_bounding_box) {
    perception_bounding_box_virtual_ = perception_bounding_box;
  }

  void
  SetPerceptionSlPolygon(const planning_math::Polygon2d &perception_polygon);
  void SetPreciseInterpFlag(bool flag) { precise_boundary_interp_ = flag; };
  bool GetPreciseInterpFlag() const { return precise_boundary_interp_; };
  void SetVirtualBoxFlag(bool flag) { using_virtual_box_ = flag; };
  bool GetVirtualBoxFlag() const { return using_virtual_box_; };

  void ComputeSlBoundary(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                         SLBoundary *const sl_boundary,
                         const bool planning_flag = false);

  static void
  ComputeSlBoundary(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                    const planning_math::Box2d &box,
                    SLBoundary *const sl_boundary);

  void
  ComputePointsSlBoundary(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                          SLBoundary *const sl_boundary);

  void ComputePolygon2dSlBoundary(
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
      planning_math::Polygon2d *const perception_polygon,
      planning_math::Box2d *const box = nullptr, double precise = 0.5);

  double InterpBounds(double s, std::string dir = "left") const;

  // void ComputeSlPoint(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
  //                        SLBoundary *sl_boundary,
  //                        const bool planning_flag = false);

  void ComputeSlLine(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                     SLBoundary *sl_boundary, const bool planning_flag = false);

  static void
  ComputeSlLine(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                const planning_math::LineSegment2d &line,
                const Pose2D &ego_pose, SLBoundary *const sl_boundary);

  void
  extendboundingbox(const double theta, const double longtitude,
                    const double latitude,
                    const std::shared_ptr<FrenetCoordinateSystem> frenet_coord);

  /**
   * @brief interfaces for optimization based speed planner
   */
  double R_frenet() const { return r_frenet_; };
  double R_planning_frenet() const { return r_planning_frenet_; };
  double Yaw_relative_frenet() const { return yaw_relative_frenet_; };
  double Yaw_relative_planning_frenet() const {
    return yaw_relative_planning_frenet_;
  };
  double Speed_yaw() const { return speed_yaw_; };
  double Speed_yaw_relative_frenet() const {
    return speed_yaw_relative_frenet_;
  };
  double Speed_yaw_relative_planning_frenet() const {
    return speed_yaw_relative_planning_frenet_;
  };
  double Speed_yaw_relative_ego() const { return speed_yaw_relative_ego_; };

  bool Is_confident() const { return is_confident_; };
  bool Is_angle_consistent() const { return is_angle_consistent_; };
  /**
   * @brief check if an ObjectDecisionType is a longitudinal decision.
   */
  static bool IsLongitudinalDecision(const ObjectDecisionType &decision);

  static bool IsParkingLongitudinalDecision(const ObjectDecisionType &decision);

  /**
   * @brief check if an ObjectDecisionType is a lateral decision.
   */
  static bool IsLateralDecision(const ObjectDecisionType &decision);

  static bool IsParkingLateralDecision(const ObjectDecisionType &decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; };
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; };

  void SetLeftSpaceLane(const double left_lane_width_start,
                        const double left_lane_width_end,
                        const SLBoundary &sl_boundary);
  double GetLeftSpaceLane() const { return left_space_lane_; };

  void SetRightSpaceLane(const double right_lane_width_start,
                         const double right_lane_width_end,
                         const SLBoundary &sl_boundary);
  double GetRightSpaceLane() const { return right_space_lane_; };

  void SetLeftSpaceBorder(const double left_road_width_start,
                          const double left_road_width_width_end,
                          const SLBoundary &sl_boundary);
  void SetLeftSpaceBorder(const double left_space_border) {
    left_space_border_ = left_space_border;
  };
  double GetLeftSpaceBorder() const { return left_space_border_; };

  void SetRightSpaceBorder(const double right_road_width_start,
                           const double right_road_width_end,
                           const SLBoundary &sl_boundary);
  void SetRightSpaceBorder(const double right_space_border) {
    right_space_border_ = right_space_border;
  };
  double GetRightSpaceBorder() const { return right_space_border_; };

private:
  static ObjectDecisionType
  MergeLongitudinalDecision(const ObjectDecisionType &lhs,
                            const ObjectDecisionType &rhs);
  static ObjectDecisionType
  MergeParkingLongitudinalDecision(const ObjectDecisionType &lhs,
                                   const ObjectDecisionType &rhs);

  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType &lhs,
                                                 const ObjectDecisionType &rhs);

  static ObjectDecisionType
  MergeParkingLateralDecision(const ObjectDecisionType &lhs,
                              const ObjectDecisionType &rhs);

private:
  int id_{};
  Point3D point_;
  int32_t perception_id_ = 0;

  int is_static_ = -1; // -1(default, unknown), 0(false), 1(true)
  int is_sf_static_ = -1;

  int is_lon_static_wrt_ego_ = -1;
  int is_lat_static_wrt_ego_ = -1;
  int is_lon_highspeed_wrt_ego_ = -1;
  int is_lon_opposite_wrt_ego_ = -1;

  int is_lon_static_wrt_frenet_ = -1;
  int is_lat_static_wrt_frenet_ = -1;
  int is_lon_highspeed_wrt_frenet_ = -1;
  int is_lon_opposite_wrt_frenet_ = -1;

  bool is_virtual_ = false;
  bool is_freemove_ = false;

  bool is_beside_road_ = false;

  bool is_in_road_ = false;
  bool is_across_road_border_ = false;
  bool is_in_bend_ = false;

  bool is_in_road_loose_ = true;

  // int relative_obstacle_id_ = -1;

  double cutin_score_ = 0.0;
  bool is_traj_sigma_valid_ = false;
  std::string intention_{"none"};
  // std::string prediction_source_;
  double speed_ = 0.0;
  double speed_yaw_ = 0.0;
  double speed_yaw_relative_frenet_ = 0.0;
  double speed_yaw_relative_planning_frenet_ = 0.0;
  double speed_yaw_relative_ego_ = 0.0;
  bool is_confident_ = true;
  bool is_angle_consistent_ = true;
  int is_along_frenet_ = -1;
  int pose_direction_ = 0;
  bool is_approaching_gate_ = false;
  int is_towards_road_center_ =
      -1; // check if obs is parking out(especially for vectical parking lot)
  int is_towards_road_side_ = -1; // check if obs is parking in
  bool is_apa_ = false;
  bool is_apoa_ = false;
  bool is_pullover_ = false;
  bool is_highspeed_decel_ = false;
  bool is_beside_intersation_ = false;
  double dist_across_road_boader_ = 0.0;
  bool is_in_middle_road_ = false;
  // road_side 1 right 2 left 3 unknown
  int road_side_;
  // road_type 1 single 2 carriage way 3 mix
  int road_type_;
  double acc_ = 0.0;
  double prob_;
  int gear_ = 0;
  ObjectType type_;
  FusionObjectType sf_type_;
  GroundLineType line_fusion_type_ =
      GroundLineType::GROUND_LINE_USS_TYPE_UNKNOWN;

  std::vector<TrajectoryPoint> trajectory_;
  FusionObject perception_obstacle_;
  planning_math::Box2d perception_bounding_box_;
  planning_math::Box2d perception_bounding_box_virtual_;
  planning_math::Polygon2d perception_polygon_;
  planning_math::Polygon2d fillet_cutting_polygon_;
  double fillet_cutting_length_ = 0.3;
  bool is_need_fillet_cutting_ = false;
  planning_math::LineSegment2d perception_line_;
  std::vector<planning_math::Vec2d> perception_points_;
  double perception_speed_{};
  double perception_speed_yaw_{};
  planning_math::Box2d perception_bounding_box_extend_;

  SLBoundary sl_boundary_{};
  SLBoundary sl_boundary_origin_{};
  SLBoundary sl_boundary_planning_{};
  SLBoundary sl_boundary_extend_{};
  SLBoundary sl_boundary_virtual_{};

  planning_math::Polygon2d perception_sl_polygon_;
  bool precise_boundary_interp_ = false;
  bool using_virtual_box_ = false;
  int s_max_index;
  int s_min_index;

  double r_frenet_{};
  double yaw_relative_frenet_{};
  double r_planning_frenet_{};
  double yaw_relative_planning_frenet_{};
  double s_min{};
  double s_max{};

  double left_space_lane_;
  double right_space_lane_;
  double left_space_border_;
  double right_space_border_;

  // STBoundary reference_line_st_boundary_;
  // STBoundary path_st_boundary_;

  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  double box_theta_ = 0.0;
  double box_length_ = 0.0;
  double box_width_ = 0.0;
  planning_math::Vec2d box_base_center_;
  planning_math::Vec2d box_base_center_sl_;
  Pose2D carte_pose_wrt_ego_;
  Point2D carte_vel_wrt_ego_;
  PseudoPredictionTrajectory pseudo_predicition_traj_;

  std::vector<planning_math::Box2d> sub_box_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;

  // indicate whether the st boundary has already been constructed and
  // up-to-date
  bool is_st_boundary_constructed_ = false;

  // info about revision of prediction trajectory
  bool is_truncated_ = false;
  double truncated_time_ = std::numeric_limits<double>::infinity();

  // info about st graph zoom manipulation
  bool is_zoomed_ = false;

  struct ObjectTagCaseHash {
    size_t operator()(const ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  };

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;
};

typedef IndexedList<int, Obstacle> IndexedObstacles;
// typedef ThreadSafeIndexedList<int, Obstacle> ThreadSafeIndexedObstacles;

} // namespace parking

} // namespace msquare