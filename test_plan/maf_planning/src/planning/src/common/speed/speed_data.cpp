#include "mph_assert.h"
#include <algorithm>
#include <utility>

#include "common/math/linear_interpolation.h"
#include "common/speed/speed_data.h"

namespace msquare {

SpeedData::SpeedData(std::vector<SpeedPoint> speed_points)
    : std::vector<SpeedPoint>(std::move(speed_points)) {
  std::sort(begin(), end(), [](const SpeedPoint &p1, const SpeedPoint &p2) {
    return p1.t < p2.t;
  });
}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
  if (!empty()) {
    mph_assert(back().t < time);
  }
  SpeedPoint speed_point;
  speed_point.s = s;
  speed_point.t = time;
  speed_point.v = v;
  speed_point.a = a;
  speed_point.da = da;
  emplace_back(speed_point);
}

bool SpeedData::EvaluateByTime(const double t,
                               SpeedPoint *const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t < t + 1.0e-6 && t - 1.0e-6 < back().t)) {
    return false;
  }

  auto comp = [](const SpeedPoint &sp, const double t) { return sp.t < t; };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto &p0 = *(it_lower - 1);
    const auto &p1 = *it_lower;
    double t0 = p0.t;
    double t1 = p1.t;

    SpeedPoint res;
    res.t = t;

    double s = planning_math::lerp(p0.s, t0, p1.s, t1, t);
    res.s = s;

    if (true) {
      double v = planning_math::lerp(p0.v, t0, p1.v, t1, t);
      res.v = v;
    }

    if (true) {
      double a = planning_math::lerp(p0.a, t0, p1.a, t1, t);
      res.a = a;
    }

    if (true) {
      double da = planning_math::lerp(p0.da, t0, p1.da, t1, t);
      res.da = da;
    }

    *speed_point = res;
  }
  return true;
}

// attention!
// to this function must make sure that the speed profile is generated according
// to const acc model and for each segment(begin and end speed point pair) the
// acceleration is stored in the end speed point by default
bool SpeedData::EvaluateByTimeWithConstAcc(
    const double t, SpeedPoint *const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t < t + 1.0e-6 && t - 1.0e-6 < back().t)) {
    return false;
  }

  auto comp = [](const SpeedPoint &sp, const double t) { return sp.t < t; };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto &p0 = *(it_lower - 1);
    const auto &p1 = *it_lower;
    double t0 = p0.t;
    double t1 = p1.t;

    SpeedPoint res;
    res.t = t;
    // res.a = p1.a;
    res.a = (p1.v - p0.v) / (t1 - t0);
    res.v = p0.v + res.a * (t - t0);
    res.s = p0.s + p0.v * (t - t0) + 0.5 * res.a * std::pow(t - t0, 2);

    res.da = 0.0;

    *speed_point = res;
  }
  return true;
}

// attention!
// to this function must make sure that the speed profile is generated according
// to const jerk model and for each segment(begin and end speed point pair) the
// jerk is stored in the end speed point by default
bool SpeedData::EvaluateByTimeWithConstJerk(
    const double t, SpeedPoint *const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t < t + 1.0e-6 && t - 1.0e-6 < back().t)) {
    return false;
  }

  auto comp = [](const SpeedPoint &sp, const double t) { return sp.t < t; };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto &p0 = *(it_lower - 1);
    const auto &p1 = *it_lower;
    double t0 = p0.t;
    //    double t1 = p1.t;

    SpeedPoint res;
    res.t = t;
    // res.da = p1.da;
    res.da = (p1.a - p0.a) / (t - t0);
    res.a = p0.a + res.da * (t - t0);
    res.v = p0.v + p0.a * (t - t0) + 0.5 * res.da * std::pow(t - t0, 2);
    res.s = p0.s + p0.v * (t - t0) + 0.5 * p0.a * std::pow(t - t0, 2) +
            1 / 6 * res.da * std::pow(t - t0, 3);
    *speed_point = res;
  }
  return true;
}

double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t - front().t;
}

} // namespace msquare
