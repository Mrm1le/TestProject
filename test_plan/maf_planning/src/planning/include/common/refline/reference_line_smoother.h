#pragma once

#include <vector>

#include "common/refline/reference_line_smoother_config.h"
#include "planner/message_type.h"

namespace msquare {
class ReferenceLineSmoother {
public:
  explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig &config)
      : config_(config) {}

  /**
   * Smoothing constraints
   */
  virtual void
  SetAnchorPoints(const std::vector<AnchorPoint> &achor_points) = 0;

  /**
   * Smooth a given reference line
   */
  virtual bool Smooth() = 0;

  virtual ~ReferenceLineSmoother() = default;

protected:
  ReferenceLineSmootherConfig config_;
};

} // namespace msquare
