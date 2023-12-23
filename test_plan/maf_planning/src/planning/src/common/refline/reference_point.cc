#include "common/refline/reference_point.h"
#include "common/utils/util.h"

namespace msquare {

namespace {
// Minimum distance to remove duplicated points.
const double kDuplicatedPointsEpsilon = 1e-7;
} // namespace

ReferencePoint::ReferencePoint(const MapPathPoint &map_path_point,
                               const double kappa, const double dkappa)
    : MapPathPoint(map_path_point), kappa_(kappa), dkappa_(dkappa) {}

PathPoint ReferencePoint::ToPathPoint(double s) const {
  PathPoint path_point =
      MakePathPoint(x(), y(), 0.0, heading(), kappa_, dkappa_, 0.0);
  path_point.set_s(s);
  return path_point;
}

double ReferencePoint::kappa() const { return kappa_; }

double ReferencePoint::dkappa() const { return dkappa_; }

} // namespace msquare