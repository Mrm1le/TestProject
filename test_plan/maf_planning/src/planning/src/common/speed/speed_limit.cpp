#include "common/speed/speed_limit.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace msquare {

void SpeedLimit::AppendSpeedLimit(const double t, const double v) {
  if (!speed_limit_points_.empty()) {
    mph_assert(t >= speed_limit_points_.back().first);
  }
  speed_limit_points_.emplace_back(t, v);
}

const std::vector<std::pair<double, double>> &
SpeedLimit::speed_limit_points() const {
  return speed_limit_points_;
}

double SpeedLimit::GetSpeedLimitByT(const double t) const {
  mph_assert(int(speed_limit_points_.size()) >= 2);
  mph_assert(t >= speed_limit_points_.front().first);

  auto compare_t = [](const std::pair<double, double> &point, const double t) {
    return point.first < t;
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), t, compare_t);

  if (it_lower == speed_limit_points_.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

void SpeedLimit::Clear() { speed_limit_points_.clear(); }

} // namespace msquare
