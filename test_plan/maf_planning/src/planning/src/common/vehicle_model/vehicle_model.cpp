#include "common/vehicle_model/vehicle_model.h"
#include "common/config_context.h"
#include "common/planning_context.h"
#include "mjson/mjson.hpp"
#include "planning/common/common.h"
#include <fstream>
#include <iterator>

namespace msquare {
namespace common {

VehicleModelConfig VehicleModel::vehicle_model_config_;

void VehicleModel::RearCenteredKinematicBicycleModel(
    const VehicleModelConfig &vehicle_model_config,
    const double predicted_time_horizon, const VehicleState &cur_vehicle_state,
    VehicleState *predicted_vehicle_state) {
  // Kinematic bicycle model centered at rear axis center by Euler forward
  // discretization
  // Assume constant control command and constant z axis position
  mph_assert(predicted_time_horizon > 0.0);
  double dt = vehicle_model_config.rc_kinematic_bicycle_model.dt;
  double cur_x = cur_vehicle_state.x;
  double cur_y = cur_vehicle_state.y;
  double cur_z = cur_vehicle_state.z;
  double cur_phi = cur_vehicle_state.heading;
  double cur_v = cur_vehicle_state.linear_velocity;
  double cur_a = cur_vehicle_state.linear_acceleration;
  double next_x = cur_x;
  double next_y = cur_y;
  double next_phi = cur_phi;
  double next_v = cur_v;
  if (dt >= predicted_time_horizon) {
    dt = predicted_time_horizon;
  }

  double countdown_time = predicted_time_horizon;
  bool finish_flag = false;
  static constexpr double kepsilon = 1e-8;
  while (countdown_time > kepsilon && !finish_flag) {
    countdown_time -= dt;
    if (countdown_time < kepsilon) {
      dt = countdown_time + dt;
      finish_flag = true;
    }
    double intermidiate_phi =
        cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa;
    next_phi =
        cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_vehicle_state.kappa;
    next_x =
        cur_x + dt * (cur_v + 0.5 * dt * cur_a) * std::cos(intermidiate_phi);
    next_y =
        cur_y + dt * (cur_v + 0.5 * dt * cur_a) * std::sin(intermidiate_phi);

    next_v = cur_v + dt * cur_a;
    cur_x = next_x;
    cur_y = next_y;
    cur_phi = next_phi;
    cur_v = next_v;
  }

  predicted_vehicle_state->x = next_x;
  predicted_vehicle_state->y = next_y;
  predicted_vehicle_state->z = cur_z;
  predicted_vehicle_state->heading = next_phi;
  predicted_vehicle_state->yaw = predicted_vehicle_state->heading;
  predicted_vehicle_state->kappa = cur_vehicle_state.kappa;
  predicted_vehicle_state->linear_velocity = next_v;
  predicted_vehicle_state->linear_acceleration =
      cur_vehicle_state.linear_acceleration;
}

VehicleState VehicleModel::Predict(const double predicted_time_horizon,
                                   const VehicleState &cur_vehicle_state) {
  VehicleModelConfig vehicle_model_config;

  // Some models not supported for now
  mph_assert(vehicle_model_config.model_type !=
             VehicleModelConfig::COM_CENTERED_DYNAMIC_BICYCLE_MODEL);
  mph_assert(vehicle_model_config.model_type != VehicleModelConfig::MLP_MODEL);

  VehicleState predicted_vehicle_state;
  if (vehicle_model_config.model_type ==
      VehicleModelConfig::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL) {
    auto rear_center_state = cur_vehicle_state;
    rear_center_state.x -=
        std::cos(rear_center_state.yaw) *
        (ConfigurationContext::Instance()->get_vehicle_param().length / 2.0 -
         ConfigurationContext::Instance()
             ->get_vehicle_param()
             .back_edge_to_center);
    rear_center_state.y -=
        std::sin(rear_center_state.yaw) *
        (ConfigurationContext::Instance()->get_vehicle_param().length / 2.0 -
         ConfigurationContext::Instance()
             ->get_vehicle_param()
             .back_edge_to_center);
    RearCenteredKinematicBicycleModel(vehicle_model_config,
                                      predicted_time_horizon, rear_center_state,
                                      &predicted_vehicle_state);
    predicted_vehicle_state.x +=
        std::cos(predicted_vehicle_state.yaw) *
        (ConfigurationContext::Instance()->get_vehicle_param().length / 2.0 -
         ConfigurationContext::Instance()
             ->get_vehicle_param()
             .back_edge_to_center);
    predicted_vehicle_state.y +=
        std::sin(predicted_vehicle_state.yaw) *
        (ConfigurationContext::Instance()->get_vehicle_param().length / 2.0 -
         ConfigurationContext::Instance()
             ->get_vehicle_param()
             .back_edge_to_center);
  }

  return predicted_vehicle_state;
}

bool VehicleModel::LoadVehicleModelConfig() {
  auto config_file_dir = PlanningContext::Instance()->get_config_file_dir();
  std::string config_path = config_file_dir + "/vehicle_model.json";

  std::ifstream fjson(config_path);
  std::string json_str((std::istreambuf_iterator<char>(fjson)),
                       std::istreambuf_iterator<char>());
  mjson::Reader reader(json_str);
  auto model_type = reader.get<mjson::Json>("model_type").string_value();
  if (model_type == "REAR_CENTERED_KINEMATIC_BICYCLE_MODEL") {
    vehicle_model_config_.model_type =
        VehicleModelConfig::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL;
    vehicle_model_config_.rc_kinematic_bicycle_model.dt =
        reader.get<mjson::Json>("rc_kinematic_bicycle_model")
            .object_value()["dt"]
            .number_value();
  } else if (model_type == "COM_CENTERED_DYNAMIC_BICYCLE_MODEL") {
    vehicle_model_config_.model_type =
        VehicleModelConfig::COM_CENTERED_DYNAMIC_BICYCLE_MODEL;
    vehicle_model_config_.comc_dynamic_bicycle_model.dt =
        reader.get<mjson::Json>("comc_dynamic_bicycle_model")
            .object_value()["dt"]
            .number_value();
  } else if (model_type == "MLP_MODEL") {
    vehicle_model_config_.model_type = VehicleModelConfig::MLP_MODEL;
    vehicle_model_config_.mlp_model.dt = reader.get<mjson::Json>("mlp_model")
                                             .object_value()["dt"]
                                             .number_value();
  } else {
    vehicle_model_config_.model_type =
        VehicleModelConfig::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL;
    MSD_LOG(INFO, "Failed to load vehicle model config file %s",
            config_path.c_str());
    return false;
  }
  return true;
}

} // namespace common
} // namespace msquare
