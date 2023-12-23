#ifndef MSQUARE_DECISION_PLANNING_PLANNER_GROUND_LINE_DECIDER_H_
#define MSQUARE_DECISION_PLANNING_PLANNER_GROUND_LINE_DECIDER_H_

#include "common/math/vec2d.h"
#include "common/utils/geometry.h"
#include "pnc/define/parking_vision_info.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace msquare {
namespace parking {

struct GroundLinePoint {
  enum Status {
    UNCLASSIFIED = 0,
    CLASSIFIED = 1,
    NOISE = 2,
  };

  planning_math::Vec2d point;
  Status status;

  bool operator==(const GroundLinePoint &input) {
    return (this->point == input.point && this->status == input.status);
  }

  bool operator!=(const GroundLinePoint &input) {
    return (!(this->point == input.point) || this->status != input.status);
  }
};

class GroundLineDecider {

public:
  GroundLineDecider(const int min_pts, const double eps) {
    min_pts_ = min_pts;
    eps_ = eps;
  }

  ~GroundLineDecider() {}

  bool update_params(const int min_pts, const double eps);
  std::vector<std::vector<planning_math::Vec2d>>
  execute(const std::vector<FusionFreespacePoint> &ground_line_points);

private:
  bool
  update_points(const std::vector<FusionFreespacePoint> &ground_line_points);
  std::vector<int> calc_cluster(GroundLinePoint &point);
  std::vector<planning_math::Vec2d> expand_cluster(GroundLinePoint &point);
  inline double calc_distance(const GroundLinePoint &point1,
                              const GroundLinePoint &point2);

  int min_pts_;
  double eps_;
  std::vector<GroundLinePoint> points_;
};
} // namespace parking
} // namespace msquare

#endif // MSQUARE_DECISION_PLANNING_PLANNER_GROUND_LINE_DECIDER_H_
