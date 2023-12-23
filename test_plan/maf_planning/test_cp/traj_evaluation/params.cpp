#include "params.h"

namespace msquare
{

CarParams::CarParams() {}
CarParams::~CarParams() {}
CarParams *CarParams::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new CarParams;
  }
  return instance_;
}

// Init static member before use
CarParams *CarParams::instance_ = nullptr;

VehicleParam::VehicleParam() {}
VehicleParam::~VehicleParam() {}
VehicleParam *VehicleParam::Instance() {
  if (instance_ == nullptr) {
    instance_ = new VehicleParam;
  }
  return instance_;
}

// Init static member before use
VehicleParam *VehicleParam::instance_ = nullptr;

} // namespace msquare

