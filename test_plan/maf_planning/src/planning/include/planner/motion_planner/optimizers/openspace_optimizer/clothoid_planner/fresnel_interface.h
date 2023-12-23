#ifndef FRESNEL_INTERFACE_HEADER
#define FRESNEL_INTERFACE_HEADER

#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/fresnel.h"

#include <vector>

namespace clothoid {

void fresnelSingle(double a, double *sf, double *cf);
void fresnelBatch(const std::vector<double> &aas, std::vector<double> &sfs,
                  std::vector<double> &cfs);

} // namespace clothoid

#endif