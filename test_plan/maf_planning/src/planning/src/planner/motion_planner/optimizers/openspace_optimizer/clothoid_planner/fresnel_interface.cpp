#include "planner/motion_planner/optimizers/openspace_optimizer/clothoid_planner/fresnel_interface.h"

namespace clothoid {

void fresnelSingle(double a, double *sf, double *cf) { fresnel(a, sf, cf); }

void fresnelBatch(const std::vector<double> &aas, std::vector<double> &sfs,
                  std::vector<double> &cfs) {
  if (sfs.size() < aas.size()) {
    sfs.resize(aas.size());
  }
  if (cfs.size() < aas.size()) {
    cfs.resize(aas.size());
  }
  for (unsigned int i = 0; i < aas.size(); i++) {
    fresnel(aas[i], &sfs[i], &cfs[i]);
  }
}

} // namespace clothoid