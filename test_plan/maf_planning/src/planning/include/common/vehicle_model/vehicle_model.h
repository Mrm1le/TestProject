#pragma once

#include "planner/message_type.h"

namespace msquare {
namespace common {

typedef struct {
  typedef enum {
    REAR_CENTERED_KINEMATIC_BICYCLE_MODEL = 0,
    COM_CENTERED_DYNAMIC_BICYCLE_MODEL = 1,
    MLP_MODEL = 2,
  } ModelType;

  typedef struct {
    double dt{0.1};
  } RearCenteredKinematicBicycleModelConfig;

  typedef struct {
    double dt{0.1};
  } ComCenteredDynamicBicycleModelConfig;

  typedef struct {
    double dt{0.1};
  } MlpModelConfig;

  ModelType model_type = ModelType::REAR_CENTERED_KINEMATIC_BICYCLE_MODEL;
  RearCenteredKinematicBicycleModelConfig rc_kinematic_bicycle_model;
  ComCenteredDynamicBicycleModelConfig comc_dynamic_bicycle_model;
  MlpModelConfig mlp_model;
} VehicleModelConfig;

class VehicleModel {
public:
  VehicleModel() = delete;

  static bool LoadVehicleModelConfig();

  static VehicleState Predict(const double predicted_time_horizon,
                              const VehicleState &cur_vehicle_state);

private:
  static void RearCenteredKinematicBicycleModel(
      const VehicleModelConfig &vehicle_model_config,
      const double predicted_time_horizon,
      const VehicleState &cur_vehicle_state,
      VehicleState *predicted_vehicle_state);

private:
  static VehicleModelConfig vehicle_model_config_;
};

} // namespace common
} // namespace msquare
