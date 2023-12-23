#pragma once

#include "msd/worldmodel/common/logging.h"
#include <memory>
#include <mutex>

namespace msd_worldmodel {

class MSDWorldModelLogger {
public:
  bool set_config(const std::string &config);
  void set_callback(mlog::MLogEndpointCallback outer_callback);

  static MSDWorldModelLogger &get_logger_instance();

  mlog::MLogManager *get_logger_manager_instance();

private:
  bool enable_callback_{false};
  mlog::MLogSettings settings_{};
  std::shared_ptr<mlog::MLogManager> log_manager_ = nullptr;

  MSDWorldModelLogger(){};
  ~MSDWorldModelLogger() {}
  MSDWorldModelLogger(const MSDWorldModelLogger &);
  MSDWorldModelLogger &operator=(const MSDWorldModelLogger &);
};

} // namespace msd_worldmodel

#define MSD_LOG(level, ...)                                                    \
  msd_worldmodel::MSDWorldModelLogger::get_logger_instance()                   \
      .get_logger_manager_instance()                                           \
      ->record_log(mlog::MLOG_LEVEL_##level, __VA_ARGS__)

#define MSD_LOG_EVERY_N(level, N, ...)                                         \
  do {                                                                         \
    static int n = N;                                                          \
    if (!(--n)) {                                                              \
      n = N;                                                                   \
      MSD_LOG(mlog::MLOG_LEVEL_##level, __VA_ARGS__);                          \
    }                                                                          \
  } while (false)

#define MSD_LOG_FIRST_N(level, N, ...)                                         \
  do {                                                                         \
    static int n = 0;                                                          \
    if (n++ < N) {                                                             \
      MSD_LOG(mlog::MLOG_LEVEL_##level, __VA_ARGS__);                          \
    }                                                                          \
  } while (false)
