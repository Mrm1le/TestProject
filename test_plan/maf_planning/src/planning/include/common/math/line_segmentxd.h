#pragma once

#include "common/math/line_segment2d.h"
#include "common/utils/frenet_coordinate_system.h"
#include "common/utils/pose2d_utils.hpp"
#include "pnc/define/geometry.h"

namespace msquare {

namespace planning_math {

class LineSegmentXd {
public:
  /**
   * @brief constructor.
   */
  LineSegmentXd();

  LineSegmentXd(const Vec2d &start, const Vec2d &end, const double max_length);

  LineSegmentXd(const Point3D &start, const Point3D &end,
                const double max_length);

  // LineSegmentXd(const std::shared_ptr<WorldModel> &world_model);

  void Init(const std::shared_ptr<FrenetCoordinateSystem> &frenet_coord,
            const double ego_frenet_s, const Pose2D &ego_pose);

  void Init(Vec2d start, Vec2d end);

  const std::vector<LineSegment2d> &GetSubSegment2d() const;

  int Size() const;

  double TotalLength() const;

  void SetBoard(double left_board, double right_board);

  double DistanceTo(double x, double y) const;

  bool IsInRoad(double x, double y) const;

private:
  Vec2d start_;
  Vec2d end_;
  double length_;
  double subSegment_max_length_ = 0.2;
  std::vector<LineSegment2d> subSegment2d_;

  double left_board_;
  double right_board_;
};

} // namespace planning_math

} // namespace msquare