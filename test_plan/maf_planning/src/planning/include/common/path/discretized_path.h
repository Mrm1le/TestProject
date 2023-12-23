#ifndef MODULES_PLANNING_COMMON_DISCRETIZED_PATH_H_
#define MODULES_PLANNING_COMMON_DISCRETIZED_PATH_H_

#include <vector>

#include "common/math/linear_interpolation.h"
#include "planner/message_type.h"

namespace msquare {

class DiscretizedPath : public std::vector<PathPoint> {
public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(const std::vector<PathPoint> &path_points);

  double Length() const;

  PathPoint Evaluate(const double path_s) const;

  PathPoint EvaluateReverse(const double path_s) const;

  double QueryMatchedS(const PathPoint &path_point) const;

protected:
  std::vector<PathPoint>::const_iterator
  QueryLowerBound(const double path_s) const;
  std::vector<PathPoint>::const_iterator
  QueryUpperBound(const double path_s) const;
};

} // namespace msquare

#endif /* MODULES_PLANNING_COMMON_DISCRETIZED_PATH_H_ */
