#pragma once

#include "mjson/mjson.hpp"
#include "mlog_core/mlog.h"
#include "mph_assert.h"
#include "msd/planning/logging.h"
#include <memory>
#include <mutex>

namespace msd_planning {

class MSDPlanningLogger {
public:
  void set_config(const std::string &config, const void *node_handle = nullptr);
  void set_callback(mlog::MLogEndpointCallback outer_callback);

  static MSDPlanningLogger &get_logger_instance();

  mlog::MLogManager *get_logger_manager_instance();

private:
  static void inner_callback(const std::vector<mlog::MLogEntry> &data);

  MSDPlanningLogCallback outer_callback_;
  mlog::MLogSettings settings_{};
  std::shared_ptr<mlog::MLogManager> log_manager_ = nullptr;

  MSDPlanningLogger(){};
  ~MSDPlanningLogger() {}
  MSDPlanningLogger(const MSDPlanningLogger &);
  MSDPlanningLogger &operator=(const MSDPlanningLogger &);
};

} // namespace msd_planning

#define MSD_LOG(level, ...)                                                    \
  msd_planning::MSDPlanningLogger::get_logger_instance()                       \
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

#define MSD_DBG_TOPIC_STR(topic_name, var_name, str)                           \
  MSD_LOG(DEBUG, "[msd_planning][%s][%s][%s][%s]", topic_name,                 \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name, str)

#define MSD_INF_TOPIC_STR(topic_name, var_name, str)                           \
  MSD_LOG(INFO, "[msd_planning][%s][%s][%s][%s]", topic_name,                  \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name, str)

#define MSD_WRN_TOPIC_STR(topic_name, var_name, str)                           \
  MSD_LOG(WARN, "[msd_planning][%s][%s][%s][%s]", topic_name,                  \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name, str)

#define MSD_ERR_TOPIC_STR(topic_name, var_name, str)                           \
  MSD_LOG(ERROR, "[msd_planning][%s][%s][%s][%s]", topic_name,                 \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name, str)

#define MSD_DBG_TOPIC_VALUE(topic_name, var_name, var_value)                   \
  MSD_LOG(DEBUG, "[msd_planning][%s][%s][%s][%s]", topic_name,                 \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          std::to_string(var_value).c_str())

#define MSD_INF_TOPIC_VALUE(topic_name, var_name, var_value)                   \
  MSD_LOG(INFO, "[msd_planning][%s][%s][%s][%s]", topic_name,                  \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          std::to_string(var_value).c_str())

#define MSD_WRN_TOPIC_VALUE(topic_name, var_name, var_value)                   \
  MSD_LOG(WARN, "[msd_planning][%s][%s][%s][%s]", topic_name,                  \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          std::to_string(var_value).c_str())

#define MSD_ERR_TOPIC_VALUE(topic_name, var_name, var_value)                   \
  MSD_LOG(ERROR, "[msd_planning][%s][%s][%s][%s]", topic_name,                 \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          std::to_string(var_value).c_str())

#define MSD_DBG_TOPIC_LIST(topic_name, var_name, vector_value)                 \
  MSD_LOG(DEBUG, "[msd_planning][%s][%s][%s][%s]", topic_name,                 \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          msd_planning::utils::vec_to_string(vector_value).c_str())

#define MSD_INF_TOPIC_LIST(topic_name, var_name, vector_value)                 \
  MSD_LOG(INFO, "[msd_planning][%s][%s][%s][%s]", topic_name,                  \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          msd_planning::utils::vec_to_string(vector_value).c_str())

#define MSD_WRN_TOPIC_LIST(topic_name, var_name, vector_value)                 \
  MSD_LOG(WARN, "[msd_planning][%s][%s][%s][%s]", topic_name,                  \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          msd_planning::utils::vec_to_string(vector_value).c_str())

#define MSD_ERR_TOPIC_LIST(topic_name, var_name, vector_value)                 \
  MSD_LOG(ERROR, "[msd_planning][%s][%s][%s][%s]", topic_name,                 \
          std::to_string(MTIME()->timestamp().us()).c_str(), var_name,         \
          msd_planning::utils::vec_to_string(vector_value).c_str())

#define MSD_DBG_TOPIC_ARRAY(topic_name, var_name, array_value)                 \
  MSD_LOG(                                                                     \
      DEBUG, "[msd_planning][%s][%s][%s][%s]", topic_name,                     \
      std::to_string(MTIME()->timestamp().us()).c_str(), var_name,             \
      msd_planning::utils::list_to_string<sizeof(array_value) /                \
                                          sizeof(array_value[0])>(array_value) \
          .c_str())

#define MSD_INF_TOPIC_ARRAY(topic_name, var_name, array_value)                 \
  MSD_LOG(                                                                     \
      INFO, "[msd_planning][%s][%s][%s][%s]", topic_name,                      \
      std::to_string(MTIME()->timestamp().us()).c_str(), var_name,             \
      msd_planning::utils::list_to_string<sizeof(array_value) /                \
                                          sizeof(array_value[0])>(array_value) \
          .c_str())

#define MSD_WRN_TOPIC_ARRAY(topic_name, var_name, array_value)                 \
  MSD_LOG(                                                                     \
      WARN, "[msd_planning][%s][%s][%s][%s]", topic_name,                      \
      std::to_string(MTIME()->timestamp().us()).c_str(), var_name,             \
      msd_planning::utils::list_to_string<sizeof(array_value) /                \
                                          sizeof(array_value[0])>(array_value) \
          .c_str())

#define MSD_ERR_TOPIC_ARRAY(topic_name, var_name, array_value)                 \
  MSD_LOG(                                                                     \
      ERROR, "[msd_planning][%s][%s][%s][%s]", topic_name,                     \
      std::to_string(MTIME()->timestamp().us()).c_str(), var_name,             \
      msd_planning::utils::list_to_string<sizeof(array_value) /                \
                                          sizeof(array_value[0])>(array_value) \
          .c_str())
