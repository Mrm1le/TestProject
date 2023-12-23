#pragma once

#include <vector>

#include "common/refline/refline_smoother/spline_2d.h"
#include "common/refline/refline_smoother/spline_2d_constraint.h"
#include "common/refline/refline_smoother/spline_2d_kernel.h"

namespace msquare {

class Spline2dSolver {
public:
  Spline2dSolver(const std::vector<double> &t_knots, const uint32_t order)
      : spline_(t_knots, order), kernel_(t_knots, order),
        constraint_(t_knots, order) {}

  virtual ~Spline2dSolver() = default;

  virtual void Reset(const std::vector<double> &t_knots,
                     const uint32_t order) = 0;

  void clear() { solve_status = -2; }

  // customize setup
  virtual Spline2dConstraint *mutable_constraint() = 0;
  virtual Spline2dKernel *mutable_kernel() = 0;
  virtual Spline2d *mutable_spline() = 0;

  // solve
  virtual bool Solve() = 0;

  // get solve status
  int GetSolveStatus() { return solve_status; }

  // extract
  virtual const Spline2d &spline() const = 0;

protected:
  Spline2d spline_;
  Spline2dKernel kernel_;
  Spline2dConstraint constraint_;

  int solve_status = -2; // solve status @zyl
};

} // namespace msquare