#ifndef MODULES_PLANNING_OPTIMIZERS_SPEED_DATA_H_
#define MODULES_PLANNING_OPTIMIZERS_SPEED_DATA_H_

#include <string>
#include <vector>

#include "planner/message_type.h"

namespace msquare {

class SpeedData : public std::vector<SpeedPoint> {
public:
  SpeedData() = default;

  virtual ~SpeedData() = default;

  explicit SpeedData(std::vector<SpeedPoint> speed_points);

  void AppendSpeedPoint(const double s, const double time, const double v,
                        const double a, const double da);

  bool EvaluateByTime(const double time, SpeedPoint *const speed_point) const;

  bool EvaluateByTimeWithConstAcc(const double time,
                                  SpeedPoint *const speed_point) const;

  bool EvaluateByTimeWithConstJerk(const double time,
                                   SpeedPoint *const speed_point) const;

  double TotalTime() const;
};

} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_SPEED_DATA_H_ */
