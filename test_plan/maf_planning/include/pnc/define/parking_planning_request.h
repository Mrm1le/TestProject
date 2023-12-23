#ifndef MSQUARE_DECISION_PLANNING_COMMON_PARKING_PLANNING_REQUEST_H
#define MSQUARE_DECISION_PLANNING_COMMON_PARKING_PLANNING_REQUEST_H

#include <string>

namespace msquare {

struct ParkingCommand {
  uint8_t value;

  enum : uint8_t {
    NONE = 0,
    RANDOM_PARKIN = 1,
    DESIGNATE_PARKIN = 2,
    PARK_OUT = 3,
    PICK_UP = 4,
    EXIT = 5,
    STOP = 6,
    RE_PARKIN = 7,
    PARK_OUT_STANDBY = 8,
    RPA_STRAIGHT_STANDBY = 9,
    RPA_STRAIGHT = 10
  };
};

struct ParkOutDirectionType {
  uint32_t value;

  enum : uint32_t {
    AUTO_SELECT = 0,               //!< auto select park out direction
    PERPENDICULAR_FRONT = 1,       //!< perpendicular slot front side
    PERPENDICULAR_FRONT_LEFT = 2,  //!< perpendicular slot front-left side
    PERPENDICULAR_FRONT_RIGHT = 4, //!< perpendicular slot front-right side
    PERPENDICULAR_REAR = 8,        //!< perpendicular slot rear side
    PERPENDICULAR_REAR_LEFT = 16,  //!< perpendicular slot rear-left side
    PERPENDICULAR_REAR_RIGHT = 32, //!< perpendicular slot rear-right side
    PARALLEL_LEFT = 64,            //!< parallel slot left side
    PARALLEL_LEFT_FRONT = 128,     //!< parallel slot left-front side
    PARALLEL_RIGHT = 256,          //!< parallel slot right side
    PARALLEL_RIGHT_FRONT = 512,    //!< parallel slot right-front side
  };
};

struct RpaStraightDirectionType {
  uint32_t value;

  enum : uint32_t {
    RPA_STRAIGHT_FORWARD = 1,
    RPA_STRAIGHT_BACKWARD = 2,
  };
};

struct PlanningRequest {
  ParkingCommand cmd;
  int id = 0;
  bool wireless_charge;
  ParkOutDirectionType park_out_direction;
  RpaStraightDirectionType rpa_straight_direction;
};

// struct ParkingUiRequest {
//   int64_t time_stamp = 0;
//   std::string action;
//   int id = 0;
//   std::string map_dir;
// };

// struct ParkingUiResponse {
//   bool send_flag = false;
//   int64_t time_stamp = 0;
//   bool success = false;
// };

// struct ParkingUiReport {
//   std::string task;
//   std::string scenario;
//   int64_t time_stamp = 0;
// };

} // namespace msquare

#endif
