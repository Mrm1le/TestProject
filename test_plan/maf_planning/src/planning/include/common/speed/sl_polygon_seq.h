#ifndef MODULES_PLANNING_OPTIMIZERS_SL_POLYGON_SEQ_H_
#define MODULES_PLANNING_OPTIMIZERS_SL_POLYGON_SEQ_H_

#include <string>
#include <vector>

#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"

namespace msquare {

using PolygonWithT = std::pair<double, planning_math::Polygon2d>;
class SLPolygonSeq : public std::vector<PolygonWithT> {
public:
  SLPolygonSeq() = default;

  virtual ~SLPolygonSeq() = default;

  explicit SLPolygonSeq(std::vector<PolygonWithT> sl_polygon_points);

  void SetTimeStep(double time_step);

  bool EvaluateByTime(const double t, PolygonWithT *const polygon_t) const;

  void
  set_invalid_time_sections(const std::vector<std::pair<double, double>> &secs);

  double TotalTime() const;

  const double GetTimeStep() const { return time_step_; }

  const bool GetIsUniformTimeStep() const { return is_uniform_time_step_; }

  const std::vector<std::pair<double, double>> &GetInvalidTimeSections() const {
    return invalid_time_sections_;
  }

  void unset() {
    clear();

    time_step_ = 0;
    is_uniform_time_step_ = false;
    invalid_time_sections_.clear();
    // static planning_math::IntervalMethodSolution<double> interval_methods_;
  }

  void
  update(const double time_step, const bool is_uniform_time_step,
         const std::vector<std::pair<double, double>> &invalid_time_sections) {
    time_step_ = time_step;
    is_uniform_time_step_ = is_uniform_time_step;
    invalid_time_sections_.assign(invalid_time_sections.begin(),
                                  invalid_time_sections.end());
  }

private:
  double time_step_;
  bool is_uniform_time_step_{false};
  std::vector<std::pair<double, double>> invalid_time_sections_;
  static planning_math::IntervalMethodSolution<double> interval_methods_;
};

} // namespace msquare

#endif /* MODULES_PLANNING_OPTIMIZERS_SL_POLYGON_SEQ_H_ */
