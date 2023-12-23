#ifndef CP_OPTIMAL_PLANNER_TRIGGER_H
#define CP_OPTIMAL_PLANNER_TRIGGER_H

#include <string>
#include <vector>

namespace cp {

struct AvdMsg {
  int id;
  std::string property;
  bool ignore;
  std::string avd_direction;
  int avd_priority;
  float blocked_time_begin;
  float blocked_time_end;
};

} // namespace cp
#endif
