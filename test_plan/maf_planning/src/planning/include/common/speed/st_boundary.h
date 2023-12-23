#ifndef MODULES_PLANNING_OPTIMIZERS_STBOUNDARY_H_
#define MODULES_PLANNING_OPTIMIZERS_STBOUNDARY_H_

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/config/dp_st_speed_config.h"
#include "common/math/box2d.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/speed/st_point.h"

namespace msquare {

class STBoundary : public planning_math::Polygon2d {
public:
  STBoundary() = default;

  explicit STBoundary(
      const std::vector<std::pair<STPoint, STPoint>> &point_pairs);

  explicit STBoundary(const planning_math::Box2d &box) = delete;

  explicit STBoundary(std::vector<planning_math::Vec2d> points) = delete;

  static STBoundary CreateInstance(const std::vector<STPoint> &lower_points,
                                   const std::vector<STPoint> &upper_points);

  static std::unique_ptr<STBoundary>
  CreateInstance(const std::vector<std::pair<STPoint, STPoint>> &point_pairs);

  ~STBoundary() = default;

  bool IsEmpty() const { return lower_points_.empty(); }
  bool IsPointInBoundary(const STPoint &st_point) const;

  bool GetTimestampBeginWithS(double lower_s, double &t) const;

  STPoint upper_left_point() const;
  STPoint upper_right_point() const;

  STPoint bottom_left_point() const;
  STPoint bottom_right_point() const;

  void set_upper_left_point(STPoint st_point);
  void set_upper_right_point(STPoint st_point);
  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);

  STBoundary ExpandByS(const double s) const;
  STBoundary ExpandByT(const double t) const;
  STBoundary ShiftByS(const double s) const;

  STBoundary ShrinkByDs(const double ds, const double begin_t) const;

  // if you need to add boundary type, make sure you modify
  // GetUnblockSRange accordingly.
  enum class BoundaryType {
    UNKNOWN,
    STOP,
    FOLLOW,
    YIELD,
    OVERTAKE,
    KEEP_CLEAR,
  };

  static std::string TypeName(BoundaryType type);

  BoundaryType boundary_type() const;
  int id() const;
  double characteristic_length() const;

  void set_id(const int &id);
  void SetBoundaryType(const BoundaryType &boundary_type);
  void SetCharacteristicLength(const double characteristic_length);

  bool GetUnblockSRange(const double curr_time, double *s_upper,
                        double *s_lower) const;

  bool GetBoundarySRange(const double curr_time, double *s_upper,
                         double *s_lower) const;

  double min_s() const;
  double min_t() const;
  double max_s() const;
  double max_t() const;

  std::vector<STPoint> upper_points() const { return upper_points_; }
  std::vector<STPoint> lower_points() const { return lower_points_; }

  STBoundary CutOffByT(const double t) const;

  void set_invalid_time_sections(
      const std::vector<std::pair<double, double>> &secs) {
    invalid_time_sections_ = secs;
  }

  void clear() {
    BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

    upper_points_.clear();
    lower_points_.clear();
    invalid_time_sections_.clear();
    // static planning_math::IntervalMethodSolution<double> interval_methods_;

    id_ = 0;
    characteristic_length_ = 1.0;
    min_s_ = std::numeric_limits<double>::max();
    max_s_ = std::numeric_limits<double>::lowest();
    min_t_ = std::numeric_limits<double>::max();
    max_t_ = std::numeric_limits<double>::lowest();

    // STPoint bottom_left_point_;
    // STPoint bottom_right_point_;
    // STPoint upper_left_point_;
    // STPoint upper_right_point_;
  }

private:
  bool
  IsValid(const std::vector<std::pair<STPoint, STPoint>> &point_pairs) const;

  bool IsPointNear(const planning_math::LineSegment2d &seg,
                   const planning_math::Vec2d &point, const double max_dist);

  void
  RemoveRedundantPoints(std::vector<std::pair<STPoint, STPoint>> *point_pairs);

  bool GetIndexRange(const std::vector<STPoint> &points, const double t,
                     size_t *left, size_t *right) const;

private:
  BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

  std::vector<STPoint> upper_points_;
  std::vector<STPoint> lower_points_;
  std::vector<std::pair<double, double>> invalid_time_sections_;
  static planning_math::IntervalMethodSolution<double> interval_methods_;

  int id_;
  double characteristic_length_ = 1.0;
  double min_s_ = std::numeric_limits<double>::max();
  double max_s_ = std::numeric_limits<double>::lowest();
  double min_t_ = std::numeric_limits<double>::max();
  double max_t_ = std::numeric_limits<double>::lowest();

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
};

} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_STBOUNDARY_H_ */