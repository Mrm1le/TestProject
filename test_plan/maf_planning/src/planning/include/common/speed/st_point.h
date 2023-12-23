#ifndef MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_
#define MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_

#include "common/math/vec2d.h"
#include <string>

namespace msquare {

typedef std::pair<double, double> Point;

class STPoint : public planning_math::Vec2d {
public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const planning_math::Vec2d &vec2d_point);
  STPoint(const STPoint &point);

  double x() const = delete;
  double y() const = delete;

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);
};

} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_ */