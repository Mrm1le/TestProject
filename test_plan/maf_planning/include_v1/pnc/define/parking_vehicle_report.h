#ifndef CP_MSQUARE_DECISION_PLANNING_COMMON_VEHICLE_REPORT_H
#define CP_MSQUARE_DECISION_PLANNING_COMMON_VEHICLE_REPORT_H

namespace cp {
namespace parking {

enum class TurnSignalType : unsigned char {
  NONE = 0,
  LEFT,
  RIGHT,
  EMERGENCY_FLASHER,
};

struct MiscReport {
  TurnSignalType turn_signal_value = TurnSignalType::NONE;
};

enum class GearType : unsigned char {
  NONE = 0,
  PARK,
  REVERSE,
  NEUTRAL,
  DRIVE,
  LOW,
};

enum class GearRejectType : unsigned char {
  NONE = 0,
  SHIFT_IN_PROGRESS,
  OVERRIDE,
  ROTARY_LOW,
  ROTARY_PARK,
  VEHICLE,
};

struct GearReport {
  GearType gear_state = GearType::NONE;
  GearType gear_cmd = GearType::NONE;
  GearRejectType reject_value = GearRejectType::NONE;
  bool override = false;
  bool fault_bus = false;
};

struct WheelSpeedReport {
  float front_left = 0.0f;
  float front_right = 0.0f;
  float rear_left = 0.0f;
  float rear_right = 0.0f;
};

} // namespace parking
} // namespace cp

#endif
