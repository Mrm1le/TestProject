#include "planning/common/logging.h"
#include "mjson/mjson.hpp"
#include "mlog_core/mlog.h"
#include "mtime_core/mtime.h"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <string>

namespace msd_planning {

namespace {
mlog::MLogLevel get_mlog_level(std::string level) {
  std::transform(level.begin(), level.end(), level.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (level == "debug") {
    return mlog::MLogLevel::MLOG_LEVEL_DEBUG;
  } else if (level == "info") {
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  } else if (level == "warn") {
    return mlog::MLogLevel::MLOG_LEVEL_WARN;
  } else if (level == "error") {
    return mlog::MLogLevel::MLOG_LEVEL_ERROR;
  } else if (level == "fatal") {
    return mlog::MLogLevel::MLOG_LEVEL_FATAL;
  } else {
    // use default level
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  }
}

// MSDPlanningLogDataType convert(mlog::MLogDataType type) {
//   switch (type) {
//   case mlog::MLogDataType::MLOG_DATA_TYPE_NORMAL:
//     return MSDPlanningLogDataType::MSD_PLANNING_LOG_DATA_TYPE_NORMAL;
//   case mlog::MLogDataType::MLOG_DATA_TYPE_MSG_RELATION:
//     return MSDPlanningLogDataType::MSD_PLANNING_LOG_DATA_TYPE_MSG_RELATION;
//   case mlog::MLogDataType::MLOG_DATA_TYPE_MSG_TIMESTAMP:
//     return MSDPlanningLogDataType::MSD_PLANNING_LOG_DATA_TYPE_MSG_TIMESTAMP;
//   }
// }

// MSDPlanningLogEntry convert(const mlog::MLogEntry &data) {
//   MSDPlanningLogEntry ret{};
//   ret.type = convert(data.type);
//   ret.data = data.data;
//   ret.timestamp_ns = data.timestamp_ns;
//   return ret;
// }

// std::vector<MSDPlanningLogEntry>
// convert_log_entry(const std::vector<mlog::MLogEntry> &data) {
//   std::vector<MSDPlanningLogEntry> ret(data.size());
//   for (size_t i = 0; i < data.size(); ++i) {
//     ret[i] = convert(data[i]);
//   }
//   return ret;
// }
} // namespace

void MSDPlanningLogger::set_config(const std::string &config,
                                   const void *node_handle) {

  constexpr auto PARAM_KEY_LOG_LEVEL = "level";
  constexpr auto PARAM_KEY_LOG_FILE = "file";
  constexpr auto PARAM_KEY_LOG_FILE_TYPE = "file_type";
  constexpr auto PARAM_KEY_LOG_MAX_FILE_SIZE = "max_file_size";
  constexpr auto PARAM_KEY_LOG_MAX_FILE_NUM = "max_file_num";
  constexpr auto PARAM_KEY_LOG_MAX_BUFFER_SIZE = "max_buffer_size_KB";
  constexpr auto PARAM_KEY_LOG_ENABLE_STDERR = "enable_stderr";
  constexpr auto PARAM_KEY_LOG_ENABLE_BACKGROUND_DUMP =
      "enable_background_dump";
  constexpr auto PARAM_KEY_LOG_EXPORT_FREQUENCE = "export_frequence";
  // constexpr auto PARAM_KEY_LOG_ENABLE_ROS_DUMP = "enable_ros_dump";
  // constexpr auto PARAM_KEY_LOG_ENABLE_MFR_DUMP = "enable_mfr_dump";
  constexpr auto PARAM_KEY_LOG_ROSTOPIC_NAME = "rostopic_name";
  // constexpr auto PARAM_KEY_LOG_MFRTOPIC_NAME = "mfrtopic_name";
  constexpr auto MANAGER_LOGGER = "logger";

  auto reader = mjson::Reader(config);
  reader.expand_environment_varialbe();
  auto logger_reader = mjson::Reader(reader.get<mjson::Json>(MANAGER_LOGGER));

  std::string log_level_str =
      logger_reader.get<std::string>(PARAM_KEY_LOG_LEVEL, false, "info");
  std::string log_file =
      logger_reader.get<std::string>(PARAM_KEY_LOG_FILE, false, "");
  std::string log_file_type =
      logger_reader.get<std::string>(PARAM_KEY_LOG_FILE_TYPE, false, "append");
  std::uint32_t max_file_size =
      logger_reader.get<std::uint32_t>(PARAM_KEY_LOG_MAX_FILE_SIZE, false, 0U);
  std::uint32_t max_file_num =
      logger_reader.get<std::uint32_t>(PARAM_KEY_LOG_MAX_FILE_NUM, false, 1U);
  std::uint32_t max_buffer_size_KB = logger_reader.get<std::uint32_t>(
      PARAM_KEY_LOG_MAX_BUFFER_SIZE, false, 8U);
  bool enable_stderr =
      logger_reader.get<bool>(PARAM_KEY_LOG_ENABLE_STDERR, false, true);
  bool enable_background_dump = logger_reader.get<bool>(
      PARAM_KEY_LOG_ENABLE_BACKGROUND_DUMP, false, false);
  int export_frequence =
      logger_reader.get<int>(PARAM_KEY_LOG_EXPORT_FREQUENCE, false, 1);
  // bool enable_ros_dump =
  //     logger_reader.get<bool>(PARAM_KEY_LOG_ENABLE_ROS_DUMP, false, false);
  std::string rostopic_name = logger_reader.get<std::string>(
      PARAM_KEY_LOG_ROSTOPIC_NAME, false, "/mlog/logs/apa_planning");
  // bool enable_mfr_dump =
  //     logger_reader.get<bool>(PARAM_KEY_LOG_ENABLE_MFR_DUMP, false, false);
  std::string mfrtopic_name = logger_reader.get<std::string>(
      PARAM_KEY_LOG_ROSTOPIC_NAME, false, "/mlog/logs/apa_planning");

  mlog::MLogLevel log_level = get_mlog_level(log_level_str);
  settings_.level = log_level;
  settings_.export_frequence = export_frequence;
  if (enable_stderr) {
    settings_.endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_STDERR;
  } else {
    settings_.endpoint &= ~mlog::MLogEndpoint::MLOG_ENDPOINT_STDERR;
  }
  if (enable_background_dump) {
    settings_.enable_background_dump = true;
  } else {
    settings_.enable_background_dump = false;
  }
  if (!log_file.empty()) {
    settings_.endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_FILE;
    settings_.log_file = log_file;
    if (log_file_type == "write") {
      settings_.file_endpoint_type = mlog::MLOG_FILE_ENDPOINT_TYPE_WRITE;
    } else if (log_file_type == "append") {
      settings_.file_endpoint_type = mlog::MLOG_FILE_ENDPOINT_TYPE_APPEND;
    } else {
      mph_assert(0);
    }
  } else {
    settings_.endpoint &= ~mlog::MLogEndpoint::MLOG_ENDPOINT_FILE;
  }
  settings_.max_file_size = max_file_size;
  settings_.max_file_num = max_file_num;
  settings_.max_buffer_size_KB = max_buffer_size_KB;
  // if (enable_ros_dump) {
  //   settings_.endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_ROS_TOPIC;
  //   settings_.rostopic_name = rostopic_name;
  // } else if (enable_mfr_dump) {
  //   settings_.endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_MFR_TOPIC;
  //   settings_.mfrtopic_name = mfrtopic_name;
  //   settings_.mfr_settings.enable_mfr_publisher_lazy_init = false;
  //   settings_.mfr_settings.node_handle = node_handle;
  // } else {
  //   settings_.endpoint &= ~mlog::MLogEndpoint::MLOG_ENDPOINT_ROS_TOPIC;
  // }
}

void MSDPlanningLogger::set_callback(
    mlog::MLogEndpointCallback outer_callback) {
  settings_.endpoint_callback = outer_callback;
  // outer_callback_ = outer_callback;
  // settings_.endpoint_callback = &MSDPlanningLogger::inner_callback;
  settings_.endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_CUSTOM_CALLBACK;
}

MSDPlanningLogger &MSDPlanningLogger::get_logger_instance() {
  static MSDPlanningLogger instance{};
  return instance;
}

// void MSDPlanningLogger::inner_callback(
//     const std::vector<mlog::MLogEntry> &data) {
//   std::vector<MSDPlanningLogEntry> a = convert_log_entry(data);
//   MSDPlanningLogger::get_logger_instance().outer_callback_(a);
// }

mlog::MLogManager *MSDPlanningLogger::get_logger_manager_instance() {
  if (log_manager_ == nullptr) {
    static std::once_flag flag;
    std::call_once(flag, [&]() {
      mlog::MLogManager::set_settings(settings_);
      log_manager_ = mlog::MLogManager::make();
    });
  }
  return log_manager_.get();
}

void MSDPlanning_set_log_config( // parasoft-suppress AUTOSAR-M7_1_2-b-2
    const std::string &config,
    void *node_handle) { // parasoft-suppress AUTOSAR-M7_1_2-b-2
  MSDPlanningLogger::get_logger_instance().set_config(config, node_handle);
}

void MSDPlanning_init_log_manager() {
  MSDPlanningLogger::get_logger_instance().get_logger_manager_instance();
}

void MSDPlanning_set_log_callback(mlog::MLogEndpointCallback callback) {
  MSDPlanningLogger::get_logger_instance().set_callback(callback);
}

#ifndef __QNX__
static std::shared_ptr<mlog::MLogManager> get_static_log_manager() {
  static std::shared_ptr<mlog::MLogManager> static_log_manager{};

  static std::once_flag flag;
  std::call_once(flag, [&]() {
    auto manager = msd_planning::MSDPlanningLogger::get_logger_instance()
                       .get_logger_manager_instance();
    if (nullptr == manager) {
      return;
    }

    // set new settings
    auto settings = manager->get_settings();
    mlog::MLogSettings new_settings(settings);
    new_settings.endpoint &= ~mlog::MLOG_ENDPOINT_STDERR;
    mlog::MLogManager::set_settings(new_settings);

    // construct static manager
    static_log_manager = mlog::MLogManager::make();

    // recover old settings
    mlog::MLogManager::set_settings(settings);
  });

  return static_log_manager;
}
#endif

void MSDPlanning_record_timestamp(const std::string &msg_id,
                                  const std::string &tag,
                                  uint64_t timestamp_ns) {
#ifndef __QNX__
  auto static_log_manager = get_static_log_manager();
  if (nullptr == static_log_manager) {
    MSD_LOG(ERROR, "(%s)static log manager null", __FUNCTION__);
    return;
  }

  if (!timestamp_ns) {
    timestamp_ns = MTIME()->timestamp().ns();
  }
  static_log_manager->record_timestamp(msg_id, tag, timestamp_ns);
#endif
}

void MSDPlanning_record_relation(const std::string &current_msg_id,
                                 const std::string &relevant_msg_id) {
#ifndef __QNX__
  auto static_log_manager = get_static_log_manager();
  if (nullptr == static_log_manager) {
    MSD_LOG(ERROR, "(%s)static log manager null", __FUNCTION__);
    return;
  }
  static_log_manager->record_relation(current_msg_id, relevant_msg_id);
#endif
}

} // namespace msd_planning
