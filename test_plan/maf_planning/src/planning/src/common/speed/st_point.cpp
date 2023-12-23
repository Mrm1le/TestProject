#include "common/speed/st_point.h"

namespace msquare {

STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const planning_math::Vec2d &vec2d_point)
    : Vec2d(vec2d_point) {}

STPoint::STPoint(const STPoint &point) : Vec2d(point.t(), point.s()) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { y_ = s; }

void STPoint::set_t(const double t) { x_ = t; }

} // namespace msquare
