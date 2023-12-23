#ifndef __LOG_UTIL_HPP
#define __LOG_UTIL_HPP

#if defined(FEAT_LOG_UTIL)

#include "mjson/mjson.hpp"
#include "mlog_core/mlog.h"
#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

namespace mpilot_planning {

class LogUtil {
public:
  static LogUtil &get_instance() {
    static LogUtil instance;
    return instance;
  }

  ~LogUtil() {}
  LogUtil(const LogUtil &) = delete;
  LogUtil &operator=(const LogUtil &) = delete;

private:
  LogUtil() {}

public:
  void config(const mjson::Reader &config_reader) {
    if (!log_manager_) {
      static std::once_flag init_flag;
      std::call_once(init_flag, [&]() {
        mlog::MLogSettings settings;
        settings.level = get_log_level(config_reader);
        settings.endpoint = get_log_endpoint(config_reader);
        if (settings.endpoint != 0) {
            settings.log_file = get_log_file(config_reader);
          if (!settings.log_file.empty()) {
            settings.file_endpoint_type =
                get_log_file_endpoint_type(config_reader);
          } else {
            settings.endpoint &= ~mlog::MLogEndpoint::MLOG_ENDPOINT_FILE;
          }
          settings.enable_background_dump =
              get_enable_background_dump(config_reader);
          log_manager_ = mlog::MLogManager::make(settings);
        }
      });
    }
  }

  template <typename... Args>
  void log(const mlog::MLogLevel level, const char *format, Args... args) {
    if (log_manager_) {
      log_manager_->record_log(level, format, args...);
      log_manager_->flush_log();
    }
  }

private:
  mlog::MLogLevel get_log_level(const mjson::Reader &config_reader) {
    auto str_level = config_reader.get<std::string>("level", false, "info");
    to_lower(str_level);
    if (str_level == "debug") {
      return mlog::MLogLevel::MLOG_LEVEL_DEBUG;
    } else if (str_level == "info") {
      return mlog::MLogLevel::MLOG_LEVEL_INFO;
    } else if (str_level == "warn") {
      return mlog::MLogLevel::MLOG_LEVEL_WARN;
    } else if (str_level == "error") {
      return mlog::MLogLevel::MLOG_LEVEL_ERROR;
    } else if (str_level == "fatal") {
      return mlog::MLogLevel::MLOG_LEVEL_FATAL;
    }
    return mlog::MLogLevel::MLOG_LEVEL_DEBUG;
  }

  size_t get_log_endpoint(const mjson::Reader &config_reader) {
    auto str_endpoint =
        config_reader.get<std::string>("endpoint", false, "default");
    to_lower(str_endpoint);
    size_t log_endpoint = 0U;
    std::string item;
    std::istringstream sstr(str_endpoint);
    while (std::getline(sstr, item, '|')) {
      if (item == "stderr" || item == "default") {
        log_endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_STDERR;
      } else if (item == "file") {
        log_endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_FILE;
      }
    }
    return log_endpoint;
  }

  std::string get_log_file(const mjson::Reader &config_reader) {
    return config_reader.get<std::string>("file", false, "");
  }

  mlog::MLogFileEndpointType
  get_log_file_endpoint_type(const mjson::Reader &config_reader) {
    auto str_log_file_endpoint_type =
        config_reader.get<std::string>("file_endpoint_type", false, "write");
    to_lower(str_log_file_endpoint_type);
    if (str_log_file_endpoint_type == "write") {
      return mlog::MLogFileEndpointType::MLOG_FILE_ENDPOINT_TYPE_WRITE;
    } else if (str_log_file_endpoint_type == "append") {
      return mlog::MLogFileEndpointType::MLOG_FILE_ENDPOINT_TYPE_APPEND;
    }
    return mlog::MLogFileEndpointType::MLOG_FILE_ENDPOINT_TYPE_WRITE;
  }

  bool get_enable_background_dump(const mjson::Reader &config_reader) {
    return config_reader.get<bool>("enable_background_dump", false, false);
  }

  void to_lower(std::string &str) {
    std::transform(str.begin(), str.end(), str.begin(),
                   [](const uint8_t c) -> uint8_t { return std::tolower(c); });
  }

private:
  std::shared_ptr<mlog::MLogManager> log_manager_;
};

} // namespace mpilot_planning

#ifndef LOG_PREFIX
#define LOG_PREFIX ""
#endif // !LOG_PREFIX

#define SETUP_LOG_UTIL(...)                                                    \
  mpilot_planning::LogUtil::get_instance().config(__VA_ARGS__)
#define LOGT(level, format, ...)                                               \
  mpilot_planning::LogUtil::get_instance().log(level, LOG_PREFIX " " format,   \
                                               ##__VA_ARGS__)
#define LOGD(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOGI(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOGW(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_WARN, format, ##__VA_ARGS__)
#define LOGE(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOGF(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_FATAL, format, ##__VA_ARGS__)

#else

#define SETUP_LOG_UTIL(...)
#define LOGT(level, format, ...)
#define LOGD(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOGI(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define LOGW(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_WARN, format, ##__VA_ARGS__)
#define LOGE(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_ERROR, format, ##__VA_ARGS__)
#define LOGF(format, ...)                                                      \
  LOGT(mlog::MLogLevel::MLOG_LEVEL_FATAL, format, ##__VA_ARGS__)

#endif // FEAT_LOG_UTIL

#endif // __LOG_UTIL_HPP
