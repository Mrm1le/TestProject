#include "common/planning_context.h"
#include <stdarg.h>
#include <stdio.h>

#define LOG_TO_CONTEXT(...) msquare::parking::recordLogToContext(__VA_ARGS__)

namespace msquare {
namespace parking {

inline void recordLogToContext(const char *const fmt, ...) {
  std::vector<char> buffer_;
  const size_t DATA_SIZE = 128;
  buffer_.reserve(DATA_SIZE);
  buffer_.clear();
  size_t steady_timestamp = 0;
  va_list list;
  va_start(list, fmt);
  size_t actual_size = vsnprintf(buffer_.data(), DATA_SIZE, fmt, list);
  va_end(list);
  if (actual_size >= DATA_SIZE) {
    //
  }

  PlanningContext::Instance()
      ->mutable_parking_behavior_planner_output()
      ->behavior = buffer_.data();
}

} // namespace parking

} // namespace msquare