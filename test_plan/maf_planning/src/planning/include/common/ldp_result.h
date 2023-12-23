#ifndef LDP_RESULT_H
#define LDP_RESULT_H

#include "pnc.h"
#include <string>
#include <vector>

namespace msquare {
typedef struct {
  enum LdpStateMachineStatus {
    LDP_STATE_MACHINE_OFF = 0,
    LDP_STATE_MACHINE_STANDBY = 1,
    LDP_STATE_MACHINE_ACTIVE = 2,
    LDP_STATE_MACHINE_OVERRIDE = 3,
    LDP_STATE_MACHINE_ERROR = 4,
  };

  LdpStateMachineStatus ldp_state_machine_status{LDP_STATE_MACHINE_OFF};

} LdpStateMachineInfo;

typedef struct {
  enum LdpSensitiveDegree {
    LDP_SENSITIVE_DEGREE_LOW = 1,
    LDP_SENSITIVE_DEGREE_MIDDLE = 2,
    LDP_SENSITIVE_DEGREE_HIGH = 3,
  };

  LdpSensitiveDegree ldp_sensitive_degree{LDP_SENSITIVE_DEGREE_MIDDLE};

} LdpSensitiveInfo;

typedef struct {
  enum LdwStatus {
    LDW_STATUS_NONE = 0,
    LDW_STATUS_LEFT_TRIGGERED = 1,
    LDW_STATUS_RIGHT_TRIGGERED = 2,
  };

  bool reset_flag{false}; // CJ: if next warning is available
  long trigger_time{0};   // CJ: current warning time
  LdwStatus ldw_status{LDW_STATUS_NONE};
  LdwStatus last_warning{LDW_STATUS_NONE}; // CJ: last triggering status
  double ldw_dlc;
  double ldw_tlc;
  long ldw_dlc_dec_time{0};
} LdwInfo;

typedef struct {
  enum LkaStatus {
    LKA_STATUS_NONE = 0,
    LKA_STATUS_LEFT_TRIGGERED = 1,
    LKA_STATUS_RIGHT_TRIGGERED = 2,
  };
  bool reset_flag{false}; // CJ: if next warning is available
  long trigger_time{0};   // CJ: current warning time
  long straight_time{0};  // CJ: time vehicle has been keeping straight
  LkaStatus lka_status{LKA_STATUS_NONE};
  LkaStatus last_warning{LKA_STATUS_NONE}; // CJ: last triggering status
  std::vector<double> d_poly_lka{0, 0, 0, 0};
  int lka_left_id{99};
  int lka_right_id{99};
  double ldp_dlc;
  double ldp_tlc;
  long ldp_dlc_dec_time{0};
  double center_offset = 0.0;
} LkaInfo;

struct LdpResult {
  LdwInfo ldw_info;
  LkaInfo lka_info;
};

struct ElkResult {
  LkaInfo elk_info;
};

typedef struct {
  enum ElkStateMachineStatus {
    ELK_STATE_MACHINE_DISABLE = 0,
    ELK_STATE_MACHINE_ENABLE = 1,
  };

  ElkStateMachineStatus re_status{ELK_STATE_MACHINE_DISABLE};
  ElkStateMachineStatus ot_status{ELK_STATE_MACHINE_DISABLE};
  ElkStateMachineStatus oc_status{ELK_STATE_MACHINE_DISABLE};
  ElkStateMachineStatus absm_status{ELK_STATE_MACHINE_DISABLE};

} ElkStateMachineInfo;

typedef struct {
  int track_id;
  maf_perception_interface::Vector3d relative_velocity;
  maf_perception_interface::Point3d relative_position;
  double oncomine_ttc{10.0};
  double overtaking_ttc{10.0};
} ElkTarget;

typedef struct {
  enum EssStatus {
    ESS_STATUS_NONE = 0,
    ESS_STATUS_LEFT_TRIGGERED = 1,
    ESS_STATUS_RIGHT_TRIGGERED = 2,
  };

  enum EssDriverIntention {
    ESS_DRIVER_INTENTION_NONE = 0,
    ESS_DRIVER_INTENTION_LEFT = 1,
    ESS_DRIVER_INTENTION_RIGHT = 2,
    ESS_DRIVER_INTENTION_LEFT_OVERRIDE = 3,
    ESS_DRIVER_INTENTION_RIGHT_OVERRIDE = 4,
  };

  EssStatus ess_status{ESS_STATUS_NONE};
  EssStatus last_ess_status{ESS_STATUS_NONE};
  EssDriverIntention ess_driver_intention{ESS_DRIVER_INTENTION_NONE};
  std::vector<double> d_poly{};
  double end_x = -1;

  double trigger_timestamp_start_sec{0.0};
  double trigger_timestamp_sec{0.0};
  uint64_t aeb_object_track_id{0};

} EssInfo;

struct EssResult {
  EssInfo ess_info;
};

typedef struct {
  enum EssStateMachineStatus {
    ESS_STATE_MACHINE_DISABLE = 0,
    ESS_STATE_MACHINE_ENABLE = 1,
  };

  EssStateMachineStatus ess_status{ESS_STATE_MACHINE_DISABLE};
} EssStateMachineInfo;

} // namespace msquare

#endif