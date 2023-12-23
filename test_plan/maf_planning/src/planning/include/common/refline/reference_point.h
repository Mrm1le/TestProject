#pragma once

#include <string>
#include <vector>

#include "common/refline/map_pnc/path.h"
#include "planner/message_type.h"

namespace msquare {

class ReferencePoint : public MapPathPoint {
public:
  ReferencePoint() = default;

  ReferencePoint(const MapPathPoint &map_path_point, const double kappa,
                 const double dkappa);

  PathPoint ToPathPoint(double s) const;

  double kappa() const;
  double dkappa() const;

  // std::string DebugString() const;

  // static void RemoveDuplicates(std::vector<ReferencePoint>* points);

private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

} // namespace msquare
