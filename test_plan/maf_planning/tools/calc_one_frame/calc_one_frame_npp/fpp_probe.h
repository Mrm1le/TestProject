#include <chrono>
#include <limits>
#include <vector>

#include "core/common/log.h"
#include "core/framework/interceptor.h"

namespace msquare {

struct TimeStaticstics {
  std::chrono::time_point<std::chrono::steady_clock> clock{};
  std::vector<long> durations{};
  long min_duration = std::numeric_limits<long>::max();
  long max_duration = 0.0;
  long sum_duration = 0.0;
};

class FppProbe : public npp::Probe {
public:
  FppProbe() : npp::Probe("fpp") {}
  ~FppProbe() {}

  void on_module_begin(npp::framework::Frame *frame,
                       const std::string &module_name) override {
    if (time_statistics_.find(module_name) == time_statistics_.end()) {
      time_statistics_[module_name] = {};
    }

    NLOGI("on_module_begin name=%s", module_name.c_str());
    time_statistics_[module_name].clock = std::chrono::steady_clock::now();
  }

  void on_module_end(npp::framework::Frame *frame,
                     const std::string &module_name) override {
    if (time_statistics_.find(module_name) == time_statistics_.end()) {
      NLOGI("module not begin name=%s", module_name.c_str());
      return;
    }

    TimeStaticstics &statistics = time_statistics_[module_name];
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - statistics.clock)
                        .count();
    statistics.durations.push_back(duration);
    NLOGI("on_module_end name=%s, duration=%u", module_name.c_str(),
          static_cast<uint32_t>(duration));

    // distribution of duration can be obtained from durations_
    statistics.min_duration = std::min(statistics.min_duration, duration);
    statistics.max_duration = std::max(statistics.max_duration, duration);
    statistics.sum_duration += duration;
    NLOGI("[duration] max=%u ms, min=%u ms, avg=%u ms",
          static_cast<uint32_t>(statistics.max_duration),
          static_cast<uint32_t>(statistics.min_duration),
          static_cast<uint32_t>(statistics.sum_duration /
                                statistics.durations.size()));
  }

private:
  std::map<std::string, TimeStaticstics> time_statistics_{};
};
} // namespace msquare