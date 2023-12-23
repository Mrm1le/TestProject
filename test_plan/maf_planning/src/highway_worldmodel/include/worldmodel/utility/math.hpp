#pragma once

#include <cmath>
#include <limits>

namespace msd_worldmodel {
namespace worldmodel_v1 {

template <typename T> bool almostEqual(T lhs, T rhs) {
  return std::abs(lhs - rhs) < std::numeric_limits<T>::epsilon();
}

} // namespace worldmodel_v1
} // namespace msd_worldmodel
