#pragma once
#include <string>
#include <unordered_map>

// #define STAT_DEBUG

#ifndef STAT_DEBUG
#define STAT(stat_name)
#define STAT_END(stat_name)
#define STAT_DISPLAY()
#else
#error STAT_DEBUG can not upload to devops
#define STAT(stat_name) msd_planning::Statistic _##stat_name(#stat_name)
#define STAT_END(stat_name) _##stat_name##.end()
#define STAT_DISPLAY() msd_planning::StatisticManager::display_stats()

namespace msd_planning {
class Statistic {
public:
  Statistic(const std::string &name);
  ~Statistic();

  void begin();
  void end();

private:
  std::string name_;
  int index_;
  bool ended_{false};
};

class StatisticManager {
public:
  struct stat_value {
    uint32_t count;
    double total_cost;
  };

  static void stat(const std::string &name);

  static void display_stats();

public:
  static std::unordered_map<std::string, stat_value> stats_;
};
} // namespace msd_planning
#endif