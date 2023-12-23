#include <vector>

#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/curve.hpp"

namespace msquare {

namespace hybrid_a_star_2 {

namespace curve {

std::vector<State> Curve::temp_interpolated_;
float Curve::wheel_base_ = 0.0f;
float Curve::max_resolution_ = 0.0f;
detail::Fresnel detail::fresnel(-5.0f, 5.0f, 0.001f);

} // namespace curve

} // namespace hybrid_a_star_2

} // namespace msquare
