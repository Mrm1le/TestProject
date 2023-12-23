#pragma once

#include "logging.h"
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

namespace msd_worldmodel {

enum MSDStatus {
  MSD_OK = 0,
  MSD_FAILURE = 1,
};

struct CanFrame {
  uint64_t timestamp_us;
  uint32_t message_id;
  bool is_remote;   // frame is a remote transfer request
  bool is_extended; // frame uses 29 bit CAN identifier (not support)
  bool is_error;    // marks an error frame (only used internally)
  uint8_t data_size;
  uint8_t data[8];
};
} // namespace msd_worldmodel
