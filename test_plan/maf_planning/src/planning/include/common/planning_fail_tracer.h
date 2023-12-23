#pragma once

#include "mlog_core/mlog_data_stream.h"
#include <sstream>
#include <string.h>
#include <string>
#include <utility>
#include <vector>

#ifndef __FILENAME__
#define __FILENAME__                                                           \
  (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

#define PLANNING_FAIL_TRACE(...)                                               \
  PlanningFailTracer::Instance()->trace(                                       \
      __FILENAME__, ":", std::to_string(__LINE__), "|", __VA_ARGS__, "\n");

class PlanningFailTracer {
public:
  static PlanningFailTracer *Instance() {
    static PlanningFailTracer instance{};
    return &instance;
  }

  template <typename T, typename... T_ARGS>
  void trace(T &&info, T_ARGS &&... args) {
    trace(std::forward<T>(info));
    trace(std::forward<T_ARGS>(args)...);
  }

  template <typename T> void trace(T &&info) {
    planning_fail_infos.emplace_back(std::forward<T>(info));
  }

  std::string dump() {
    if (planning_fail_msg.empty()) {
      mlog::MLogDataStream ss;
      for (const auto &info : planning_fail_infos) {
        ss << info;
      }
      planning_fail_msg = ss.str();
    }
    return planning_fail_msg;
  }

  void reset() {
    planning_fail_infos.clear();
    planning_fail_msg.clear();
  }

private:
  enum : size_t {
    DEFAULT_PLANNING_FAIL_INFOS_SIZE = 100,
  };

  PlanningFailTracer() { planning_fail_infos.reserve(100); }

  std::vector<std::string> planning_fail_infos;
  std::string planning_fail_msg{};
};
