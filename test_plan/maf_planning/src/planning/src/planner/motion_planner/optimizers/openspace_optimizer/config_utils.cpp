#include "planner/motion_planner/optimizers/openspace_optimizer/config_utils.h"
#include "planning/common/common.h"
namespace msquare {

using namespace planning_math;

void modify_inflation_for_obstacle(CarParams *car_params, Box2d ego_box,
                                   LineSegment2d obs_line, double lat_margin) {
  if (ego_box.HasOverlap(obs_line)) {
    MSD_LOG(INFO, "Can not avoid such line\n");
  }
  // MSD_LOG(INFO, "modify inflate for line.\n");

  double lat_dist = ego_box.LateralDistanceTo(obs_line);
  double lon_dist = ego_box.LongitudinalDistanceTo(obs_line);
  // MSD_LOG(INFO, "hzmdebug: inflate [lat_dist:%f, lon_dist:%f]\n", lat_dist,
  // lon_dist);

  if (lat_dist > 0 && lon_dist <= 0) {
    (void)car_params->shrinkLatInflation(lat_dist - lat_margin);
  } else if (lon_dist > 0 && lat_dist <= 0) {
    (void)car_params->shrinkLonInflation(lon_dist);
  } else if (lat_dist > 0 && lon_dist > 0) {
    (void)car_params->shrinkLatInflation(lat_dist -
                                         lat_margin); // prefer lateral shrink
  }

  // MSD_LOG(INFO, "hzmdebug: inflate vehicle [width:%f, length:%f]\n",
  // VehicleParam::Instance()->width, car_params->vehicle_length);
}

void modify_inflation_for_obstacle(CarParams *car_params, Box2d ego_box,
                                   Box2d obs_box, double lat_margin) {
  if (ego_box.HasOverlap(obs_box)) {
    MSD_LOG(INFO, "Can not avoid such box\n");
  }

  double lat_dist = ego_box.LateralDistanceTo(obs_box);
  double lon_dist = ego_box.LongitudinalDistanceTo(obs_box);
  // MSD_LOG(INFO, "hzmdebug: inflate distance[lat: %lf, lon: %lf].\n",
  // lat_dist, lon_dist);

  if (lat_dist > 0 && lon_dist <= 0) {
    (void)car_params->shrinkLatInflation(lat_dist - lat_margin);
  } else if (lon_dist > 0 && lat_dist <= 0) {
    (void)car_params->shrinkLonInflation(lon_dist);
  } else if (lat_dist > 0 && lon_dist > 0) {
    (void)car_params->shrinkLatInflation(lat_dist -
                                         lat_margin); // prefer lateral shrink
  }

  // MSD_LOG(INFO, "hzmdebug: inflate vehicle after[width:%f, length:%f]\n",
  // VehicleParam::Instance()->width, car_params->vehicle_length);
}

void modify_inflation_for_obstacle(const CarParams *car_params, Box2d ego_box,
                                   Vec2d obs_point, double lat_margin) {
  if (ego_box.IsPointIn(obs_point)) {
    MSD_LOG(INFO, "Can not avoid such box\n");
  }
  return;

  // double lat_dist = ego_box.LateralDistanceTo(obs_point);
  // double lon_dist = ego_box.LongitudinalDistanceTo(obs_point);
  // // MSD_LOG(INFO, "hzmdebug: inflate distance[lat: %lf, lon: %lf].\n",
  // lat_dist, lon_dist);

  // if(lat_dist > 0 && lon_dist <= 0)
  // {
  //   car_params->shrinkLatInflation(lat_dist - lat_margin);
  // }
  // else if(lon_dist > 0 && lat_dist <= 0)
  // {
  //   car_params->shrinkLonInflation(lon_dist);
  // }
  // else if(lat_dist > 0 && lon_dist > 0)
  // {
  //   car_params->shrinkLatInflation(lat_dist - lat_margin);//prefer lateral
  //   shrink
  // }

  // MSD_LOG(INFO, "hzmdebug: inflate vehicle after[width:%f, length:%f]\n",
  // VehicleParam::Instance()->width, car_params->vehicle_length);
}

} // namespace msquare