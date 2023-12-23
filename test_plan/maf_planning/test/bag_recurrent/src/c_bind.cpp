#include "parking_scenario/parking_scenario.h"
#include "parking_scenario/pose_adjuster.h"
#include "perfect_scene.h"
#include "planner_interface.h"
#include "sbp_planner_wrapper.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

void c_get_apa_metas(void *out) {
  std::string metas = "suppoted_apa_features:parkin_vertical,parkin_vertical_astar,parkin_horizontal,parkin_oblique,parkin_oblique_astar,parkout_vertical_front,parkout_horizontal_front;";
  *((std::string *)out) = std::move(metas);
}

void c_apa_initSingletonParams(void *car_params_file, void *config_param_file) {
  planner_interface::initSingletonParams(*((std::string *)car_params_file),
                                         *((std::string *)config_param_file));
}

void c_apa_planInterfaceSerialize(void *ods_str, void *out) {
  std::string result =
      planner_interface::planInterfaceSerialize(*((std::string *)ods_str));
  *((std::string *)out) = std::move(result);
}

double c_apa_getEgoMinObsDistance(double x, double y, double theta,
                                  void *odo_str) {
  parking_scenario::Point2d ego_pose(x, y, theta);
  return planner_interface::getEgoMinObsDistance(ego_pose,
                                                 *((std::string *)odo_str));
}

#ifdef __cplusplus
}
#endif
