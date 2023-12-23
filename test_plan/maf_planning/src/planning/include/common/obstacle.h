#ifndef MODULES_PLANNING_OPTIMIZERS_COMMON_OBSTACLE_H_
#define MODULES_PLANNING_OPTIMIZERS_COMMON_OBSTACLE_H_

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "common/object_decision_type.h"
#include "common/path/discretized_path.h"
#include "common/prediction_object.h"
#include "common/speed/sl_polygon_seq.h"
#include "common/speed/st_boundary.h"
#include "common/utils/index_list.h"
#include "planner/message_type.h"

namespace msquare {

using namespace msd_planning;

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

class Obstacle {
public:
  Obstacle() = default;
  explicit Obstacle(int id, std::size_t traj_index,
                    const PredictionObject &prediction_object, bool is_static,
                    double start_timestamp);

  explicit Obstacle(int id,
                    const maf_perception_interface::PerceptionFusionObjectData
                        &perception_obstacle,
                    bool is_static);

  int Id() const { return id_; }
  void SetId(int id) { id_ = id; }

  double speed() const { return speed_; }
  double acceleration() const { return acc_; }

  int32_t PerceptionId() const { return perception_id_; }

  bool IsStatic() const { return is_static_; }
  bool IsFrenetInvalid() const { return is_frenet_invalid_; }
  bool IsPredictFrenetInvalid() const { return is_pred_frenet_invalid_; }
  bool IsVirtual() const { return is_virtual_; }
  bool IsFreemove() const { return is_freemove_; }
  bool IsMinorModal() const { return b_minor_modal_; }
  double CutinScore() const { return cutin_score_; }
  double MaxSigma() const { return max_sigma_; }
  bool IsSigmaValid() const { return is_traj_sigma_valid_; }
  std::string Intention() const { return intention_; }
  std::string PredictionSource() const { return prediction_source_; }
  bool IsCutin() const { return is_cutin_; }
  bool IsDistinguishCutin() const { return is_distinguish_cutin_; }
  bool IsEgoLaneOverlap() const { return is_ego_lane_overlap_; }

  void SetTruncatedTime(double t) { truncated_time_ = t; }
  double truncatedTime() const { return truncated_time_; }
  double PredSLBoundaryRelativeTime() const {
    return pred_sl_boundary_relative_time_;
  }
  void SetTruncated(bool tc) { is_truncated_ = tc; }
  bool IsTruncated() const { return is_truncated_; }

  void SetTrajectory(std::vector<TrajectoryPoint> traj) { trajectory_ = traj; }

  void SetAvdDisBUffer(double s) { avd_dis_buffer_ = s; }
  double avdDisBuffer() const { return avd_dis_buffer_; }

  TrajectoryPoint GetPointAtTime(const double time) const;

  planning_math::Box2d GetBoundingBox(const TrajectoryPoint &point) const;

  planning_math::Polygon2d
  GetPolygonAtPoint(const TrajectoryPoint &point) const;

  SLBoundary GetSLBoundary(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                           const TrajectoryPoint &point) const;

  const planning_math::Box2d &PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }

  const planning_math::Box2d &PredictionBoundingBox() const {
    return prediction_bounding_box_;
  }

  const planning_math::Polygon2d &PerceptionPolygon() const {
    return perception_polygon_;
  }

  const planning_math::Polygon2d &CarEgoPolygon() const {
    return car_ego_polygon_;
  }

  double PerceptionSpeed() const { return perception_speed_; }

  const maf_perception_interface::PerceptionFusionObjectData &
  PerceptionInfo() const {
    return perception_obstacle_;
  }

  ObjectType Type() const { return type_; }

  double Prob() const { return prob_; }

  const std::vector<TrajectoryPoint> &Trajectory() const { return trajectory_; }

  bool HasTrajectory() const { return !trajectory_.empty(); }

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>>
  CreateObstacles(const std::vector<PredictionObject> &predictions);

  static std::unique_ptr<Obstacle>
  CreateStaticVirtualObstacles(const int &id,
                               const planning_math::Box2d &obstacle_box);

  static bool IsValidPredictionObject(const PredictionObject &object);

  static bool IsValidTrajectoryPoint(const PredictionTrajectoryPoint &point);

  inline bool IsCautionLevelObstacle() const {
    return is_caution_level_obstacle_;
  }

  bool IsIrrelevant(const DiscretizedPath &reference_line,
                    const SLBoundary &adc_sl);

  SLBoundary PerceptionSLBoundary() const;

  SLBoundary PredictionSLBoundary() const;

  const STBoundary &reference_line_st_boundary() const;

  const STBoundary &path_st_boundary() const;

  const SLPolygonSeq &sl_polygon_seq() const;

  bool has_sl_polygon_seq() const;

  void SetSLPolygonSequenceInvalid(bool is_invalid);

  bool is_sl_polygon_seq_invalid() const;

  void set_path_st_boundary(const STBoundary &boundary);

  void SetStBoundaryType(const STBoundary::BoundaryType &type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const STBoundary &boundary);

  void SetReferenceLineStBoundaryType(const STBoundary::BoundaryType &type);

  void EraseReferenceLineStBoundary();

  void SetSLPolygonSequence(const SLPolygonSeq &sl_polygon_seq);

  void SetCutinProperty(const bool is_cutin) {
    is_distinguish_cutin_ = is_cutin;
  }

  void BuildReferenceLineStBoundary(
      const DiscretizedPath &reference_line,
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord, double adc_start_s);

  void SetPerceptionSlBoundary(const SLBoundary &sl_boundary);

  void ComputeSlBoundary(std::shared_ptr<FrenetCoordinateSystem> frenet_coord,
                         SLBoundary *sl_boundary);
  void ComputePredictSlBoundary(
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord);
  /**
   * @brief interfaces for optimization based speed planner
   */
  double R_frenet() const { return r_frenet_; };
  double S_frenet() const { return s_frenet_; };
  double Yaw_relative_frenet() const { return yaw_relative_frenet_; };

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

  /**
   * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
   */
  bool IsLaneBlocking() const { return is_lane_blocking_; }
  void CheckLaneBlocking(const DiscretizedPath &reference_line);
  bool IsLaneChangeBlocking() const { return is_lane_change_blocking_; }
  void SetLaneChangeBlocking(bool is_distance_clear);

  /**
   * @brief TruncateByTime is only called after revisePrediction()
   * when STboundary overtake other's
   */
  void TruncateByTime();

  void SetStBoudanrySolid(bool solid) { is_st_boundary_constructed_ = solid; };

  bool IsStBoundarySolid() { return is_st_boundary_constructed_; };

  void clear();

  void update(int id,
              const maf_perception_interface::PerceptionFusionObjectData
                  &perception_obstacle,
              const bool is_static);

  void update(int id, std::size_t traj_index,
              const PredictionObject &prediction_object, const bool is_static,
              double start_timestamp);

  void update(const Obstacle &obstacle);

private:
  bool BuildTrajectoryStBoundary(
      const DiscretizedPath &reference_line,
      std::shared_ptr<FrenetCoordinateSystem> frenet_coord, double adc_start_s,
      STBoundary *st_boundary);
  bool
  IsValidObstacle(const maf_perception_interface::PerceptionFusionObjectData
                      &perception_obstacle);
  void
  ExtractPointAtSpecifiedResolution(std::vector<planning_math::Vec2d> &points);

private:
  int id_{};
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_frenet_invalid_ = false;
  bool is_pred_frenet_invalid_ = true;
  bool is_virtual_ = false;
  bool is_freemove_ = false;
  bool b_minor_modal_ = false;
  double cutin_score_ = 0.0;
  double max_sigma_ = 0.0;
  bool is_traj_sigma_valid_ = false;
  std::string intention_{"none"};
  std::string prediction_source_;
  double speed_ = 0.0;
  double speed_direction_ = 0.0;
  double acc_ = 0.0;
  double prob_;
  ObjectType type_;
  bool is_cutin_ = false;
  bool is_distinguish_cutin_ = false;
  bool is_ego_lane_overlap_ = false;

  std::vector<TrajectoryPoint> trajectory_;
  maf_perception_interface::PerceptionFusionObjectData perception_obstacle_;
  planning_math::Box2d perception_bounding_box_;
  planning_math::Box2d prediction_bounding_box_;
  planning_math::Polygon2d perception_polygon_;
  planning_math::Polygon2d car_ego_polygon_;
  double perception_speed_{};

  SLBoundary sl_boundary_{};
  SLBoundary pred_sl_boundary_{};
  double r_frenet_{};
  double s_frenet_{};
  double yaw_relative_frenet_{};
  double s_min{};
  double s_max{};
  double pred_sl_boundary_relative_time_{};

  STBoundary reference_line_st_boundary_;
  STBoundary path_st_boundary_;

  SLPolygonSeq sl_polygon_seq_;
  // to be delete
  bool has_sl_polygon_seq_{false};
  bool invalid_sl_polygon_seq_{false};

  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  std::vector<std::string> lat_decider_tags_;
  std::vector<std::string> lon_decider_tags_;
  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  bool is_lane_change_blocking_ = false;

  bool is_caution_level_obstacle_ = false;

  double min_radius_stop_distance_ = -1.0;

  // avoidance distance buffer
  double avd_dis_buffer_ = 0.2;

  // indicate whether the st boundary has already been constructed and
  // up-to-date
  bool is_st_boundary_constructed_ = false;

  // info about revision of prediction trajectory
  bool is_truncated_ = false;
  double truncated_time_ = std::numeric_limits<double>::infinity();

  // info about st graph zoom manipulation
  bool is_zoomed_ = false;
};

typedef IndexedList<int, Obstacle> IndexedObstacles;
} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_COMMON_OBSTACLE_H_ */
