#include "logging.h"
#include <string>
#include <typeinfo>

#define MSD_FILENAME(x) strrchr(x, '/') ? strrchr(x, '/') + 1 : x
#define MSD_DBG_TAG(tag)                                                       \
  MSD_LOG(DEBUG, "[%s(%d)][%s][%s]", MSD_FILENAME(__FILE__), __LINE__,         \
          __FUNCTION__, #tag)
#define MSD_DBG_LOOK_AT(val_name)                                              \
  MSD_LOG(DEBUG, "[%s(%d)][%s][%s][%s][%s][%s]", MSD_FILENAME(__FILE__),       \
          __LINE__, __FUNCTION__,                                              \
          std::to_string(MTIME()->timestamp().us()).c_str(), #val_name,        \
          typeid(val_name).name(), std::to_string(val_name).c_str())
#define MSD_DBG_TAG_LOOK_AT(tag, val_name)                                     \
  MSD_LOG(DEBUG, "[%s(%d)][%s][%s][%s][%s][%s]", MSD_FILENAME(__FILE__),       \
          __LINE__, __FUNCTION__, #tag, #val_name, typeid(val_name).name(),    \
          std::to_string(val_name).c_str())

#define MSD_LOG_INSTANCE(level, ...)                                           \
  msd_planning::MSDPlanningLogger::get_logger_instance()                       \
      .get_logger_manager_instance()                                           \
      ->record_log(mlog::MLOG_LEVEL_##level, __VA_ARGS__)

#define MSD_DBG_TAG_INSTANCE(tag)                                              \
  MSD_LOG_INSTANCE(DEBUG, "[%s(%d)][%s][%s]", MSD_FILENAME(__FILE__),          \
                   __LINE__, __FUNCTION__, #tag)
#define MSD_DBG_LOOK_AT_INSTANCE(val_name)                                     \
  MSD_LOG_INSTANCE(                                                            \
      DEBUG, "[%s(%d)][%s][%s][%s][%s][%s]", MSD_FILENAME(__FILE__), __LINE__, \
      __FUNCTION__, std::to_string(MTIME()->timestamp().us()).c_str(),         \
      #val_name, typeid(val_name).name(), std::to_string(val_name).c_str())
#define MSD_DBG_TAG_LOOK_AT_INSTANCE(tag, val_name)                            \
  MSD_LOG_INSTANCE(DEBUG, "[%s(%d)][%s][%s][%s][%s][%s]",                      \
                   MSD_FILENAME(__FILE__), __LINE__, __FUNCTION__, #tag,       \
                   #val_name, typeid(val_name).name(),                         \
                   std::to_string(val_name).c_str())

#define MSD_DBG_TAG_VALUE(tag, var_name, var_value)                            \
  MSD_LOG_INSTANCE(DEBUG, "[%s][%s][%s]", #tag, #var_name,                     \
                   std::to_string(var_value).c_str())

#define MSD_DBG_TAG_STR(tag, var_name, str)                                    \
  MSD_LOG_INSTANCE(DEBUG, "[%s][%s][%s]", #tag, #var_name, str)

#define MSD_WRN_TAG(tag)                                                       \
  MSD_LOG(WARN, "[%s(%d)][%s][%s]", MSD_FILENAME(__FILE__), __LINE__,          \
          __FUNCTION__, #tag)
#define MSD_WRN_LOOK_AT(val_name)                                              \
  MSD_LOG(WARN, "[%s(%d)][%s][%s][%s][%s][%s]", MSD_FILENAME(__FILE__),        \
          __LINE__, __FUNCTION__,                                              \
          std::to_string(MTIME()->timestamp().us()).c_str(), #val_name,        \
          typeid(val_name).name(), std::to_string(val_name).c_str())
#define MSD_WRN_TAG_LOOK_AT(tag, val_name)                                     \
  MSD_LOG(WARN, "[%s(%d)][%s][%s][%s][%s][%s]", MSD_FILENAME(__FILE__),        \
          __LINE__, __FUNCTION__, #tag, #val_name, typeid(val_name).name(),    \
          std::to_string(val_name).c_str())

#define MSD_LOG_INSTANCE(level, ...)                                           \
  msd_planning::MSDPlanningLogger::get_logger_instance()                       \
      .get_logger_manager_instance()                                           \
      ->record_log(mlog::MLOG_LEVEL_##level, __VA_ARGS__)

#define MSD_WRN_TAG_INSTANCE(tag)                                              \
  MSD_LOG_INSTANCE(WARN, "[%s(%d)][%s][%s]", MSD_FILENAME(__FILE__), __LINE__, \
                   __FUNCTION__, #tag)
#define MSD_WRN_LOOK_AT_INSTANCE(val_name)                                     \
  MSD_LOG_INSTANCE(                                                            \
      WARN, "[%s(%d)][%s][%s][%s][%s][%s]", MSD_FILENAME(__FILE__), __LINE__,  \
      __FUNCTION__, std::to_string(MTIME()->timestamp().us()).c_str(),         \
      #val_name, typeid(val_name).name(), std::to_string(val_name).c_str())
#define MSD_WRN_TAG_LOOK_AT_INSTANCE(tag, val_name)                            \
  MSD_LOG_INSTANCE(WARN, "[%s(%d)][%s][%s][%s][%s][%s]",                       \
                   MSD_FILENAME(__FILE__), __LINE__, __FUNCTION__, #tag,       \
                   #val_name, typeid(val_name).name(),                         \
                   std::to_string(val_name).c_str())

#define MSD_WRN_TAG_VALUE(tag, var_name, var_value)                            \
  MSD_LOG_INSTANCE(WARN, "[%s][%s][%s]", #tag, #var_name,                      \
                   std::to_string(var_value).c_str())

#define MSD_WRN_TAG_STR(tag, var_name, str)                                    \
  MSD_LOG_INSTANCE(WARN, "[%s][%s][%s]", #tag, #var_name, str)
