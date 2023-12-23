#pragma once

#include "common/hfsm/machine_single.hpp"
#include "common/planning_context.h"
#include <iostream>

namespace msquare {
namespace parking {

#ifdef HFSM_ENABLE_LOG_INTERFACE
struct HfsmContextLogger : hfsm::LoggerInterface {

  static HfsmContextLogger *Instance() {
    static HfsmContextLogger instance;
    return &instance;
  }
  // hfsm::LoggerInterface
  void record(const std::type_index & /*state*/, const char *const stateName,
              const Method /*method*/, const char *const methodName) override {
    // std::cout << stateName << "::" << methodName << "()\n";
    PlanningContext::Instance()
            ->mutable_planning_status()
            ->statemachine_stringstream
        << stateName << "::" << methodName << "()\n";
  }
};
#endif

} // namespace parking
} // namespace msquare
