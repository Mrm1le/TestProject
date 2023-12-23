#ifndef COMMON_TRIGGER_STATUS_
#define COMMON_TRIGGER_STATUS_
#include <string>
#include <vector>
namespace msquare {

class TriggerStatus {
public:
  bool dbw_status;
  size_t start_time;
  size_t end_time;
  std::string scenario_name;
  std::string detail;

  TriggerStatus(bool dbw = false, size_t start = 0, size_t end = 0,
                std::string scenario = "", std::string detail = "")
      : dbw_status(dbw), start_time(start), end_time(end),
        scenario_name(scenario), detail(detail) {}
};
} // namespace msquare
#endif
