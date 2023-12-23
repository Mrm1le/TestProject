#ifndef MSQUARE_DECISION_PLANNING_COMMON_DISCRETIZED_TRAJECTORY_H_
#define MSQUARE_DECISION_PLANNING_COMMON_DISCRETIZED_TRAJECTORY_H_

#include "mph_assert.h"
#include <vector>

#include "common/math/vec2d.h"
#include "planner/message_type.h"

namespace msquare {

class DiscretizedTrajectory : public std::vector<TrajectoryPoint> {
public:
  DiscretizedTrajectory() = default;

  /**
   * Create a DiscretizedTrajectory based on ros message
   */
  explicit DiscretizedTrajectory(const PlanningResult &msg);

  /**
   * generate a publishable trajectory ros message
   * call after SetHeaderTime()
   */
  //   void PopulateTrajectoryProtobuf(momenta_msgs::Planning* trajectory_pb);

  static void
  CompensateTrajectory(std::vector<TrajectoryPoint> &trajectory_points,
                       double max_time);

  explicit DiscretizedTrajectory(
      const std::vector<TrajectoryPoint> &trajectory_points);

  void
  SetTrajectoryPoints(const std::vector<TrajectoryPoint> &trajectory_points);

  virtual ~DiscretizedTrajectory() = default;

  virtual TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSpatialLength() const;

  virtual TrajectoryPoint Evaluate(const double relative_time) const;

  virtual size_t QueryLowerBoundPoint(const double relative_time,
                                      const double epsilon = 1.0e-5) const;

  virtual size_t QueryNearestPoint(const planning_math::Vec2d &position) const;

  size_t QueryNearestPointWithBuffer(const planning_math::Vec2d &position,
                                     const double buffer) const;

  size_t QueryNearestPointWithBuffer(double s, const double buffer) const;

  virtual void AppendTrajectoryPoint(const TrajectoryPoint &trajectory_point);

  void PrependTrajectoryPoints(
      const std::vector<TrajectoryPoint> &trajectory_points) {
    if (!empty() && trajectory_points.size() > 1) {
      mph_assert(trajectory_points.back().relative_time <
                 front().relative_time);
    }
    insert(begin(), trajectory_points.begin(), trajectory_points.end());
  }

  const TrajectoryPoint &TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const;

  virtual void Clear();

  double header_time() const { return header_time_; };

  void set_header_time(double header_time) { header_time_ = header_time; }

  // void SetHeaderTime(double header_time);

private:
  double header_time_ = 0.0;
};

inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

inline void DiscretizedTrajectory::Clear() { clear(); }

} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_COMMON_DISCRETIZED_TRAJECTORY_H_
