#include "planning/common/statistic.h"
#include "planning/common/logging.h"

#ifdef STAT_DEBUG
namespace msd_planning {

std::unordered_map<std::string, StatisticManager::stat_value>
    StatisticManager::stats_;

Statistic::Statistic(const std::string &name) : name_(name) { begin(); }

Statistic::~Statistic() {
  if (!ended_) {
    end();
  }
}

void Statistic::begin() {}

void Statistic::end() {
  auto itor = StatisticManager::stats_.find(name_);
  if (itor == StatisticManager::stats_.end()) {
    StatisticManager::stat_value stat_value;
    stat_value.count = 1;
    index_ = 0;
    StatisticManager::stats_[name_] = stat_value;
  } else {
    auto &stat_value = StatisticManager::stats_[name_];
    index_ = stat_value.count;
    stat_value.count += 1;
  }
  ended_ = true;
}

void StatisticManager::display_stats() {
  for (const auto &itor : stats_) {
    MSD_LOG(ERROR, "statistic %s invoked %d times", itor.first.c_str(),
            itor.second.count);
  }
}
} // namespace msd_planning
#endif